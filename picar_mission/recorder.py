"""Manual arm demonstration recorder: commanded servo angles + sensors to CSV.

Hardware-free: angle and distance readers are injected by mission_bridge.
Servo values are the last commanded targets (no encoders), never measured
joint positions. Never commands motors or servos.
"""
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import re
import threading
import time

JOINTS = ('base', 'shoulder', 'elbow', 'gripper')
SIDES = ('LEFT', 'RIGHT')
TASKS = ('pick', 'place')
# Manual web arm buttons (Server/WebServer.py robotCtrl). Releases are stop commands.
ARM_COMMANDS = frozenset({'lookleft', 'lookright', 'armup', 'armdown',
                          'handup', 'handdown', 'grab', 'loose', 'home'})
LABEL_PATTERN = re.compile(r'^[a-z][a-z0-9_]{0,31}$')

EPISODE_FIELDS = ['episode_id', 'started_at', 'ended_at', 'duration_s', 'side', 'success',
                  'stop_distance_cm', 'sensor_distance_cm', 'keyframes', 'trajectory_rows', 'note']
KEYFRAME_FIELDS = ['episode_id', 'index', 'time', 't', 'side', 'task', 'label', 'ok',
                   *JOINTS, 'distance_cm', 'distance_age_s',
                   'pick_cx', 'pick_cy', 'pick_size', 'place_cx', 'place_cy', 'place_size',
                   'qr_age_s', 'command', 'frame_file']
TRAJECTORY_FIELDS = ['episode_id', 't', *JOINTS, 'distance_cm', 'command']


def _round(value, digits):
    return '' if value is None else round(value, digits)


def qr_features(detections, shape):
    """Largest PICK and PLACE code: center and mean side length, normalized by width."""
    height, width = shape[:2]
    best = {}
    for item in detections:
        label = item.get('label')
        if label not in ('PICK', 'PLACE'):
            continue
        pts = item['corners_px']
        side = sum(math.dist(pts[i], pts[(i+1) % 4]) for i in range(4)) / 4
        if label not in best or side > best[label][2]:
            cx, cy = item['center_px']
            best[label] = (cx/width, cy/height, side)
    result = {}
    for label in ('PICK', 'PLACE'):
        cx, cy, side = best.get(label, (None, None, None))
        key = label.lower()
        result.update({key+'_cx': cx, key+'_cy': cy,
                       key+'_size': None if side is None else side/width})
    return result


class DemoRecorder:
    def __init__(self, directory, read_angles, read_distance, reader_factory=None,
                 sample_hz=20., qr_hz=5., threaded=True, clock=time.monotonic, wall=datetime.now):
        self.directory = Path(directory)
        self.read_angles, self.read_distance = read_angles, read_distance
        self.reader_factory = reader_factory
        self.sample_period, self.qr_period = 1./sample_hz, 1./qr_hz
        self.threaded = threaded
        self.clock, self.wall = clock, wall
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.threads = []
        self.reader = None
        self.qr_error = None
        self.episode = None
        self.command = None
        self.frame = self.frame_time = None
        self.qr = qr_features([], (1, 1))
        self.qr_time = None
        self.episodes_total = self._count_rows('episodes.csv')

    # -- inputs from the server ------------------------------------------
    def note_command(self, command):
        """Active manual arm command, or None after a release/stop command."""
        with self.lock:
            self.command = command

    def submit_frame(self, frame):
        # Copy at most 10 times per second; the camera loop draws overlays in place.
        if self.episode is None:
            return
        now = self.clock()
        with self.lock:
            if self.frame_time is not None and now-self.frame_time < .1:
                return
            self.frame, self.frame_time = frame.copy(), now

    # -- episode lifecycle -----------------------------------------------
    def start(self, side, note=''):
        if side not in SIDES:
            raise ValueError('side must be LEFT or RIGHT')
        with self.lock:
            if self.episode is not None:
                raise ValueError('already recording ' + self.episode['id'])
            started = self.wall()
            episode_id = started.strftime('%Y%m%d-%H%M%S')
            (self.directory/'frames'/episode_id).mkdir(parents=True, exist_ok=True)
            trajectory = self._open('trajectory.csv', TRAJECTORY_FIELDS)
            distance, _ = self._distance()
            self.episode = {'id': episode_id, 'side': side, 'note': str(note)[:200],
                            'started': started, 't0': self.clock(), 'keyframes': 0,
                            'rows': 0, 'sensor_distance': distance, 'last_mark': None,
                            'trajectory': trajectory}
            self.frame = self.frame_time = self.qr_time = None
            self.qr = qr_features([], (1, 1))
            self.stop_event.clear()
        if self.threaded:
            self.threads = [threading.Thread(target=self._loop, args=(self.sample_period, self.sample_once),
                                             name='recorder-sample', daemon=True),
                            threading.Thread(target=self._loop, args=(self.qr_period, self.decode_once),
                                             name='recorder-qr', daemon=True)]
            for thread in self.threads:
                thread.start()
        return self.status()

    def mark(self, task, label, ok=True):
        if task not in TASKS:
            raise ValueError('task must be pick or place')
        if not LABEL_PATTERN.match(str(label)):
            raise ValueError('label must be lowercase letters, digits or _')
        with self.lock:
            episode = self._require_episode()
            index = episode['keyframes'] + 1
            angles = self._angles()
            distance, age = self._distance()
            now = self.clock()
            qr_age = None if self.qr_time is None else now-self.qr_time
            frame_file = ''
            if self.frame is not None:
                name = f'{index:03d}_{task}_{label}.jpg'
                if self._save_jpeg(self.frame, self.directory/'frames'/episode['id']/name):
                    frame_file = f"frames/{episode['id']}/{name}"
            row = {'episode_id': episode['id'], 'index': index,
                   'time': self.wall().isoformat(timespec='milliseconds'),
                   't': round(now-episode['t0'], 3), 'side': episode['side'],
                   'task': task, 'label': label, 'ok': int(bool(ok)),
                   **{j: _round(angles.get(j), 1) for j in JOINTS},
                   'distance_cm': _round(distance, 1), 'distance_age_s': _round(age, 3),
                   **{k: _round(v, 4) for k, v in self.qr.items()},
                   'qr_age_s': _round(qr_age, 3), 'command': self.command or '',
                   'frame_file': frame_file}
            with self._open('keyframes.csv', KEYFRAME_FIELDS) as handle:
                csv.DictWriter(handle, KEYFRAME_FIELDS).writerow(row)
            episode['keyframes'] = index
            episode['last_mark'] = {k: row[k] for k in ('index', 'task', 'label', 'ok', *JOINTS)}
            return row

    def stop(self, success=None, stop_distance_cm=None, note=None):
        if success not in (True, False, None):
            raise ValueError('success must be true, false or null')
        if stop_distance_cm is not None:
            if isinstance(stop_distance_cm, bool) or not isinstance(stop_distance_cm, (int, float)) \
                    or not math.isfinite(stop_distance_cm) or stop_distance_cm <= 0:
                raise ValueError('stop_distance_cm must be a positive number')
        with self.lock:
            self._require_episode()
            self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=2.)
        self.threads = []
        with self.lock:
            episode, self.episode = self.episode, None
            episode['trajectory'].close()
            ended = self.wall()
            row = {'episode_id': episode['id'],
                   'started_at': episode['started'].isoformat(timespec='seconds'),
                   'ended_at': ended.isoformat(timespec='seconds'),
                   'duration_s': round(self.clock()-episode['t0'], 1), 'side': episode['side'],
                   'success': '' if success is None else int(success),
                   'stop_distance_cm': _round(stop_distance_cm, 1),
                   'sensor_distance_cm': _round(episode['sensor_distance'], 1),
                   'keyframes': episode['keyframes'], 'trajectory_rows': episode['rows'],
                   'note': episode['note'] if note is None else str(note)[:200]}
            with self._open('episodes.csv', EPISODE_FIELDS) as handle:
                csv.DictWriter(handle, EPISODE_FIELDS).writerow(row)
            self.episodes_total += 1
            self.frame = None
            return row

    def close(self):
        if self.episode is not None:
            self.stop(note='server_shutdown')

    # -- periodic work (threads, or called directly in tests) ------------
    def sample_once(self):
        with self.lock:
            episode = self.episode
            if episode is None:
                return
            angles = self._angles()
            distance, _ = self._distance()
            row = [episode['id'], round(self.clock()-episode['t0'], 3),
                   *(_round(angles.get(j), 1) for j in JOINTS),
                   _round(distance, 1), self.command or '']
            csv.writer(episode['trajectory']).writerow(row)
            episode['trajectory'].flush()
            episode['rows'] += 1

    def decode_once(self):
        with self.lock:
            frame, stamp = self.frame, self.frame_time
        if frame is None or stamp == self.qr_time:
            return
        try:
            if self.reader is None:
                if self.reader_factory:
                    self.reader = self.reader_factory()
                else:
                    import cv2
                    from .qr_scan import QRReader
                    self.reader = QRReader(cv2)
            features = qr_features(self.reader.detect(frame), frame.shape)
        except Exception as exc:  # QR is optional; keep recording angles.
            self.qr_error = str(exc)
            return
        with self.lock:
            if self.episode is not None:
                self.qr, self.qr_time = features, stamp

    def status(self):
        with self.lock:
            episode = self.episode
            distance, age = self._distance()
            now = self.clock()
            return {'recording': episode is not None,
                    'episode_id': None if episode is None else episode['id'],
                    'side': None if episode is None else episode['side'],
                    'elapsed_s': None if episode is None else round(now-episode['t0'], 1),
                    'keyframes': 0 if episode is None else episode['keyframes'],
                    'trajectory_rows': 0 if episode is None else episode['rows'],
                    'last_mark': None if episode is None else episode['last_mark'],
                    'angles_deg': self._angles(), 'distance_cm': _round(distance, 1),
                    'distance_age_s': _round(age, 3), 'command': self.command,
                    'qr': {k: _round(v, 4) for k, v in self.qr.items()},
                    'qr_age_s': None if self.qr_time is None else round(now-self.qr_time, 2),
                    'qr_error': self.qr_error, 'episodes_total': self.episodes_total,
                    'directory': str(self.directory)}

    # -- helpers ---------------------------------------------------------
    def _loop(self, period, work):
        while not self.stop_event.wait(period):
            try:
                work()
            except Exception as exc:
                print(json.dumps({'event': 'recorder_error', 'message': str(exc)}), flush=True)

    def _require_episode(self):
        if self.episode is None:
            raise ValueError('not recording')
        return self.episode

    def _angles(self):
        try:
            angles = self.read_angles()
        except (AttributeError, TypeError, IndexError, ValueError):
            angles = None
        return dict(angles or {})

    def _distance(self):
        try:
            sample = self.read_distance()
        except Exception:
            sample = None
        if not sample or sample.get('distance_cm') is None:
            return None, None
        return float(sample['distance_cm']), max(0., self.clock()-sample['captured_at'])

    def _open(self, name, fields):
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory/name
        fresh = not path.exists() or path.stat().st_size == 0
        if not fresh:
            with path.open(newline='', encoding='utf-8') as handle:
                header = next(csv.reader(handle), None)
            if header != fields:
                raise ValueError(f'{path} has a different header; move it aside before recording')
        handle = path.open('a', newline='', encoding='utf-8')
        if fresh:
            csv.writer(handle).writerow(fields)
        return handle

    def _count_rows(self, name):
        path = self.directory/name
        if not path.exists():
            return 0
        with path.open(newline='', encoding='utf-8') as handle:
            return max(0, sum(1 for _ in csv.reader(handle)) - 1)

    @staticmethod
    def _save_jpeg(frame, path):
        try:
            import cv2
            ok, encoded = cv2.imencode('.jpg', frame)
        except Exception:
            return False
        if ok:
            path.write_bytes(encoded.tobytes())  # Also handles non-ASCII paths.
        return bool(ok)
