"""Latest-frame QR worker; shares the official camera, never commands motion."""
import queue
import threading
import time
from .config import MissionConfig
from .qr_scan import LayoutGate, QRReader, StartGate, classify_layout, emit
from .sequence_selection import SideQrGate, sequence_for

# States in which camera frames are decoded.
SCANNING = ('WAIT_QR', 'WAIT_LAYOUT', 'WAIT_SIDE_QR')


class MissionRuntime:
    def __init__(self, config=None, reader_factory=None, clock=time.monotonic):
        self.config = config or MissionConfig()
        self.clock = clock
        self.reader_factory = reader_factory
        self.gate = StartGate(self.config.required_frames, self.config.hold_seconds, self.config.max_gap_seconds)
        self.layout_gate = LayoutGate(self.config.layout_frames, self.config.layout_min_gap)
        self.side_gate = SideQrGate(self.config.layout_frames, self.config.layout_min_gap, self.config.max_gap_seconds)
        self.frames = queue.Queue(maxsize=1)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.worker = None
        self.last_sequence = -1
        self.scan_epoch = 0
        self.scan_after = float('-inf')
        self.data = {
            'state': 'WAIT_QR' if self.config.qr_enabled else 'DISABLED',
            'first_direction': None,
            # layout is latched only after request_layout(); layout_preview is
            # the last decoded frame's reading, for checking the camera view.
            'layout': None, 'layout_preview': None,
            'sequence_selection': None, 'side_direction': None,
            'side_candidate': None, 'side_frames': 0,
            'target_distance_cm': self.config.target_distance_cm,
            'distance_offset_cm': self.config.distance_offset_cm,
            'automatic_motion_enabled': False,
            'implemented_stages': ['qr_recognition'],
            'pending_stages': ['approach_to_target', 'arm_transfer'],
            'detections': [], 'candidate': None, 'consecutive_frames': 0,
            'frames_processed': 0, 'last_frame_time': None, 'last_camera_time': None,
            # Time the last frame took to decode, for checking a slow camera view.
            'decode_ms': None,
            'error': None,
        }

    def snapshot(self):
        import copy
        with self.lock:
            result = copy.deepcopy(self.data)
        stamp = result.pop('last_frame_time')
        result['frame_age_seconds'] = None if stamp is None else round(max(0., self.clock()-stamp), 3)
        camera_stamp = result.pop('last_camera_time')
        result['camera_fresh'] = camera_stamp is not None and self.clock()-camera_stamp <= self.config.max_frame_age_seconds
        return result

    def submit_frame(self, frame, captured_at=None):
        if not self.config.qr_enabled or self.stop_event.is_set():
            return
        with self.lock:
            self.data['last_camera_time'] = self.clock() if captured_at is None else captured_at
            if self.data['state'] not in SCANNING:
                return
        item = (frame.copy(), self.clock() if captured_at is None else captured_at)
        # Keep at most one unprocessed frame. Producer never waits for decoding.
        try:
            self.frames.get_nowait()
        except queue.Empty:
            pass
        try:
            self.frames.put_nowait(item)
        except queue.Full:
            pass

    def request_layout(self):
        """After the stop: decode frames again until the box layout latches."""
        with self.lock:
            if self.data['state'] != 'DIRECTION_CONFIRMED':
                return False
            self.layout_gate = LayoutGate(self.config.layout_frames, self.config.layout_min_gap)
            self.scan_epoch += 1
            self.data.update(state='WAIT_LAYOUT', layout=None)
        emit('layout_requested')
        return True

    def _discard_frames(self):
        while True:
            try:
                self.frames.get_nowait()
            except queue.Empty:
                return

    def wait_for_arm_reset(self):
        """Frames captured before the initial pose is restored cannot start driving."""
        with self.lock:
            self.scan_epoch += 1
            self.data.update(state='WAIT_ARM_RESET', first_direction=None, sequence_selection=None,
                side_direction=None, side_candidate=None, side_frames=0, detections=[])
            self._discard_frames()

    def begin_start_scan(self):
        with self.lock:
            if self.stop_event.is_set() or self.data['state'] != 'WAIT_ARM_RESET':
                return False
            self.scan_epoch += 1
            self.scan_after = self.clock()
            self.gate = StartGate(self.config.required_frames, self.config.hold_seconds, self.config.max_gap_seconds)
            self.data.update(state='WAIT_QR', candidate=None, consecutive_frames=0)
            self._discard_frames()
            return True

    def request_side_qr(self, direction, not_before):
        with self.lock:
            if self.stop_event.is_set() or self.data['state'] != 'DIRECTION_CONFIRMED' or direction != self.data['first_direction']:
                return False
            self.scan_epoch += 1
            self.scan_after = max(self.clock(), not_before)+self.config.side_qr_settle_seconds
            self.side_gate = SideQrGate(self.config.layout_frames, self.config.layout_min_gap, self.config.max_gap_seconds)
            self.data.update(state='WAIT_SIDE_QR', side_direction=direction,
                side_candidate=None, side_frames=0, sequence_selection=None, detections=[])
            self._discard_frames()
            return True

    def _stale(self, captured_at, decoded=False):
        # A stale frame breaks the consecutive-frame streak of either gate.
        # PICK/PLACE order does not age: a slow decode of a frame taken after
        # the scan began still counts, or a slow Pi would discard every one.
        fresh = decoded and self.data['state'] == 'WAIT_SIDE_QR' or             self.clock()-captured_at <= self.config.max_frame_age_seconds
        if captured_at >= self.scan_after and fresh:
            return False
        if self.data['state'] == 'WAIT_QR':
            self.gate.update([], captured_at)
            self.data.update(candidate=None, consecutive_frames=0)
        elif self.data['state'] == 'WAIT_LAYOUT':
            self.layout_gate.update([])
        else:
            self.side_gate.update([], self.data['side_direction'], captured_at)
            self.data.update(side_candidate=None, side_frames=0)
        return True

    def process_frame(self, frame, captured_at, sequence, reader):
        """Also callable synchronously for deterministic integration tests."""
        with self.lock:
            if self.data['state'] not in SCANNING or sequence <= self.last_sequence:
                return None
            self.last_sequence = sequence
            if self._stale(captured_at):
                return None
            epoch, state = self.scan_epoch, self.data['state']
        side = state == 'WAIT_SIDE_QR' and hasattr(reader, 'detect_side')
        began = time.monotonic()
        detections = reader.detect_side(frame) if side else reader.detect(frame)
        decode_ms = round(1000*(time.monotonic()-began))
        direction = layout = start = None
        with self.lock:
            if self.data['state'] not in SCANNING or self.stop_event.is_set() or epoch != self.scan_epoch:
                return None
            if self._stale(captured_at, decoded=True):
                return None
            self.data.update(detections=detections,decode_ms=decode_ms,
                             layout_preview=classify_layout(detections, self.config.layout_min_gap),
                             frames_processed=self.data['frames_processed']+1,
                             last_frame_time=captured_at)
            if self.data['state'] == 'WAIT_QR':
                direction = self.gate.update([x['text'] for x in detections], captured_at)
                self.data.update(candidate=self.gate.candidate, consecutive_frames=self.gate.count)
                if direction:
                    self.data.update(state='DIRECTION_CONFIRMED', first_direction=direction)
            elif self.data['state'] == 'WAIT_LAYOUT':
                layout = self.layout_gate.update(detections)
                if layout:
                    self.data.update(state='LAYOUT_CONFIRMED', layout=layout)
            else:
                start = self.side_gate.update(detections, self.data['side_direction'], captured_at)
                self.data.update(side_candidate=self.side_gate.candidate, side_frames=self.side_gate.count)
                if start:
                    self.data.update(state='SEQUENCE_IDENTIFIED', sequence_selection=sequence_for(start))
                    emit('sequence_identified', start_position=start)
        if direction:
            emit('direction_confirmed', direction=direction,
                 target_distance_cm=self.config.target_distance_cm,
                 distance_offset_cm=self.config.distance_offset_cm,
                 automatic_motion_enabled=False)
        if layout:
            emit('layout_confirmed', layout=layout)
        return direction or layout or start

    def start(self):
        if not self.config.qr_enabled or self.worker is not None:
            return
        self.worker = threading.Thread(target=self._run, name='mission-qr', daemon=True)
        self.worker.start()

    def _run(self):
        try:
            if self.reader_factory:
                reader = self.reader_factory()
            else:
                import cv2
                reader = QRReader(cv2)
            sequence = 0
            while not self.stop_event.is_set():
                try:
                    frame, stamp = self.frames.get(timeout=.2)
                except queue.Empty:
                    continue
                sequence += 1
                self.process_frame(frame, stamp, sequence, reader)
        except Exception as exc:
            self.fail(str(exc))

    def fail(self, message):
        with self.lock:
            self.data.update(state='ERROR', error=message, automatic_motion_enabled=False)
        emit('mission_error', message=message, automatic_motion_enabled=False)

    def stop(self):
        self.stop_event.set()
        if self.worker and self.worker is not threading.current_thread():
            self.worker.join(timeout=2.)
