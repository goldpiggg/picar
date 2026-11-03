#!/usr/bin/env python3
"""QR-only PiCar prototype. Does not import or command motors/servos.

Default input is the official controller's video feed; it does not open the
USB camera a second time. Only OpenCV and Python's standard library required.
"""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time

if __package__:
    from .config import load_config
else:
    from config import load_config

DEFAULT_CONFIG = load_config()
TARGET_DISTANCE_CM = DEFAULT_CONFIG.target_distance_cm
DISTANCE_OFFSET_CM = DEFAULT_CONFIG.distance_offset_cm
KNOWN_CODES = frozenset({'LEFT', 'RIGHT', 'PICK', 'PLACE'})
START_CODES = frozenset({'LEFT', 'RIGHT'})


def emit(event, **fields):
    print(json.dumps({'event': event, **fields}, ensure_ascii=True), flush=True)


def normalize(text):
    """Only exact mission tokens, allowing case and surrounding whitespace."""
    value = text.strip().upper()
    return value if value in KNOWN_CODES else None


@dataclass
class StartGate:
    required_frames: int = 3
    hold_seconds: float = 0.35
    max_gap_seconds: float = 0.8
    candidate: str = None
    count: int = 0
    first_seen: float = 0.0
    last_seen: float = None
    confirmed: str = None

    def update(self, decoded_texts, now):
        """One call per captured frame; emit the direction once, then latch.

        Missing/unknown/ambiguous frames break the consecutive sequence.
        A different direction starts a new sequence. Multiple start symbols
        are ambiguous even if they contain the same text.
        """
        if self.confirmed is not None:
            return None
        starts = [v for t in decoded_texts if (v := normalize(t)) in START_CODES]
        if len(starts) != 1:
            self.candidate = None
            self.count = 0
            self.last_seen = None
            return None
        direction = starts[0]
        gap = self.last_seen is not None and now - self.last_seen > self.max_gap_seconds
        if direction != self.candidate or gap:
            self.candidate, self.count, self.first_seen = direction, 1, now
        else:
            self.count += 1
        self.last_seen = now
        if self.count >= self.required_frames and now - self.first_seen >= self.hold_seconds:
            self.confirmed = direction
            return direction
        return None


# (higher code, lower code) pairs that hold in the LEFT_TOP layout; LEFT_BOTTOM
# reverses every one. PICK marks an object, PLACE an empty box. Pairs in the
# same row (left PICK vs right PLACE) carry no information and are not used.
LEFT_TOP_PAIRS = ((('left', 'PICK'), ('left', 'PLACE')),
                  (('right', 'PLACE'), ('right', 'PICK')),
                  (('left', 'PICK'), ('right', 'PICK')),
                  (('right', 'PLACE'), ('left', 'PLACE')))


def classify_layout(detections, min_gap=0.05):
    """LEFT_TOP / LEFT_BOTTOM from PICK/PLACE codes in one frame, else None.

    A side is the image half a code's center falls in. Every visible pair
    must agree and differ in height by at least min_gap (fraction of image
    height), so one code hidden under a sponge still leaves a decision, while
    a duplicate code, a level pair or a contradiction gives none.
    """
    heights = {}
    for item in detections:
        if item.get('label') not in ('PICK', 'PLACE'):
            continue
        x, y = item['center_normalized']
        key = ('left' if x < .5 else 'right', item['label'])
        if key in heights:
            return None
        heights[key] = y
    votes = set()
    for upper, lower in LEFT_TOP_PAIRS:
        if upper in heights and lower in heights:
            gap = heights[lower] - heights[upper]
            if abs(gap) < min_gap:
                return None
            votes.add('LEFT_TOP' if gap > 0 else 'LEFT_BOTTOM')
    return votes.pop() if len(votes) == 1 else None


@dataclass
class LayoutGate:
    """Latch a layout after required_frames consecutive frames agree."""
    required_frames: int = 3
    min_gap: float = 0.05
    candidate: str = None
    count: int = 0
    confirmed: str = None

    def update(self, detections):
        if self.confirmed is not None:
            return None
        layout = classify_layout(detections, self.min_gap)
        if layout is None or layout != self.candidate:
            self.candidate, self.count = layout, int(layout is not None)
        else:
            self.count += 1
        if layout is not None and self.count >= self.required_frames:
            self.confirmed = layout
            return layout
        return None


class QRReader:
    def __init__(self, cv2):
        self.cv2 = cv2
        self.detector = cv2.QRCodeDetector()
        if not hasattr(self.detector, 'detectAndDecodeMulti'):
            raise RuntimeError('OpenCV lacks detectAndDecodeMulti; OpenCV 4.5+ is required.')
        # OpenCV 4.8+ only; reads bent codes the classic detector misses.
        self.aruco = cv2.QRCodeDetectorAruco() if hasattr(cv2, 'QRCodeDetectorAruco') else None

    def detect(self, frame):
        # Multi detection is mandatory: a single-code fallback could conceal
        # a conflicting second direction. There is intentionally no such fallback.
        ok, texts, points, _ = self.detector.detectAndDecodeMulti(frame)
        if not ok or points is None:
            return []
        return self._results(frame, texts, points)

    def detect_side(self, frame, one_is_enough=False):
        """PICK and PLACE seen from the side, small and tilted in the frame.

        At 640x480 the plain multi decoder misses codes under about 90 px; a
        2x upscale recovers them, and decoding each half alone recovers a
        pair the multi decoder splits. The fallbacks only run while a code is
        still missing, so a clear view costs the same as detect(). After the
        turn one code clearly off center decides (one_is_enough), so the
        search stops there instead of hunting for a code it cannot read.

        A code found but not decoded (a curled sheet) is returned as an
        `undecoded` entry with its position: after the turn the one decoded
        code and that box are the two codes of this side.
        """
        done = self._decided if one_is_enough else self._pair
        located = []
        ok, texts, points, _ = self.detector.detectAndDecodeMulti(frame)
        found = []
        if ok and points is not None:
            found = self._results(frame, texts, points)
            located += self._boxes(frame, texts, points)
        if done(self._with_located(found, located)):
            return self._with_located(found, located)
        cv2 = self.cv2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        big = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        mid = cv2.resize(gray, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
        sharp = cv2.addWeighted(mid, 1.8, cv2.GaussianBlur(mid, (0, 0), 2), -.8, 0)
        # Cheapest first; each try only adds codes still missing. A curled
        # sheet bends its code, and which try reads it varies frame to frame.
        tries = [('halves', gray, 1), ('multi', big, 2), ('halves', sharp, 1.5), ('halves', big, 2)]
        if self.aruco is not None:
            tries.insert(0, ('aruco', gray, 1))
        for kind, image, scale in tries:
            if kind == 'halves':
                width = image.shape[1]
                for x0, x1 in ((0, width*6//10), (width*4//10, width)):
                    text, corners, _ = self.detector.detectAndDecode(image[:, x0:x1])
                    if corners is not None:
                        corners = corners.reshape(1, 4, 2)
                        found = self._merge(found, self._results(image, [text], corners, scale=scale, x_offset=x0))
                        located += self._boxes(image, [text], corners, scale=scale, x_offset=x0)
            else:
                detector = self.aruco if kind == 'aruco' else self.detector
                ok, texts, points, _ = detector.detectAndDecodeMulti(image)
                if ok and points is not None:
                    found = self._merge(found, self._results(image, texts, points, scale=scale))
                    located += self._boxes(image, texts, points, scale=scale)
            if done(self._with_located(found, located)):
                break
        return self._with_located(found, located)

    def _boxes(self, image, texts, points, scale=1, x_offset=0):
        """Codes located but not decoded, in the original frame's coordinates."""
        boxes = []
        for text, corners in zip(texts, points):
            if text:
                continue
            item, = self._results(image, ['?'], corners.reshape(1, 4, 2), scale=scale, x_offset=x_offset)
            boxes.append(dict(text='', label=None, undecoded=True, corners_px=item['corners_px'],
                              center_px=item['center_px'], center_normalized=item['center_normalized']))
        return boxes

    @staticmethod
    def _with_located(found, located):
        # A box is a separate code only if no decoded code or earlier box sits there.
        from .sequence_selection import SAME_CODE_GAP
        kept = list(found)
        for box in located:
            if all(abs(box['center_normalized'][0]-item['center_normalized'][0]) >= SAME_CODE_GAP for item in kept):
                kept.append(box)
        return kept

    @staticmethod
    def _pair(found):
        return {'PICK', 'PLACE'} <= {item['label'] for item in found}

    @classmethod
    def _decided(cls, found):
        from .sequence_selection import classify_side_start
        # Either side gives the same answer for "is this decidable".
        return cls._pair(found) or classify_side_start(found, 'LEFT', settled=True) is not None

    @staticmethod
    def _merge(found, more):
        # Keep the first reading of each code; another try may see it again.
        labels = {item['label'] for item in found}
        return found+[item for item in more if item['label'] not in labels]

    def _results(self, image, texts, points, scale=1, x_offset=0):
        """Detections in the original frame's pixels, whatever image was decoded."""
        height, width = image.shape[:2]
        results = []
        for text, corners in zip(texts, points):
            if not text:
                continue
            polygon = [[(float(x)+x_offset)/scale, float(y)/scale] for x, y in corners.reshape(4, 2)]
            cx = sum(p[0] for p in polygon) / 4
            cy = sum(p[1] for p in polygon) / 4
            results.append({'text': text, 'label': normalize(text),
                            'corners_px': polygon,
                            'center_px': [cx, cy],
                            'center_normalized': [cx*scale / width, cy*scale / height]})
        return results


def annotate(cv2, frame, detections, gate):
    for item in detections:
        pts = item['corners_px']
        color = (0, 200, 0) if item['label'] else (0, 180, 255)
        for i in range(4):
            cv2.line(frame, tuple(map(round, pts[i])), tuple(map(round, pts[(i + 1) % 4])), color, 2)
        x, y = map(round, pts[0])
        cv2.putText(frame, item['label'] or 'UNKNOWN', (max(0, x), max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
    message = 'CONFIRMED ' + gate.confirmed if gate.confirmed else f'WAIT {gate.candidate or "-"} {gate.count}'
    cv2.putText(frame, message, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 200, 255), 2)


def open_capture(cv2, source):
    if source.startswith(('http://', 'https://')):
        # FFmpeg supports open/read timeouts. No unbounded automatic fallback.
        if not all(hasattr(cv2, name) for name in ('CAP_PROP_OPEN_TIMEOUT_MSEC', 'CAP_PROP_READ_TIMEOUT_MSEC')):
            raise RuntimeError('This OpenCV build lacks HTTP timeout support; use a newer OpenCV build.')
        params = [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                  cv2.CAP_PROP_READ_TIMEOUT_MSEC, 3000]
        cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG, params)
    else:
        value = int(source) if source.isdecimal() else source
        cap = cv2.VideoCapture(value)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError('Cannot open source. Check /video_feed in a browser, camera ownership, and OpenCV FFmpeg support.')
    return cap


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default='http://127.0.0.1:5000/video_feed',
                        help='Official video URL (default), camera index, or video path')
    parser.add_argument('--image', type=Path, help='Decode one saved image; never confirms a live start')
    parser.add_argument('--mode', choices=['start', 'inspect'], default='start',
                        help='start: latch LEFT/RIGHT then exit; inspect: list all four QR types')
    parser.add_argument('--preview', action='store_true', help='Local GUI only; leave off over normal SSH')
    parser.add_argument('--timeout', type=float, default=60., help='Live test duration in seconds')
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be a positive finite number')
    try:
        import cv2
    except ImportError:
        print('OpenCV missing. On Raspberry Pi: sudo apt install python3-opencv', file=sys.stderr)
        return 1
    cap = None
    try:
        reader = QRReader(cv2)
        if args.image:
            # imdecode supports non-ASCII Windows paths too.
            import numpy as np
            frame = cv2.imdecode(np.frombuffer(args.image.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise RuntimeError('Cannot decode image file')
            items = reader.detect(frame)
            emit('image_result', detections=items, live_start_confirmed=False)
            return 0 if any(x['label'] for x in items) else 2
        emit('waiting', mode=args.mode, source=args.source, motor_control=False,
             planned_target_cm=TARGET_DISTANCE_CM, planned_offset_cm=DISTANCE_OFFSET_CM)
        cap = open_capture(cv2, args.source)
        gate = StartGate(DEFAULT_CONFIG.required_frames, DEFAULT_CONFIG.hold_seconds,
                         DEFAULT_CONFIG.max_gap_seconds)
        deadline = time.monotonic() + args.timeout
        previous = None
        frames = 0
        while time.monotonic() < deadline:
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError('Frame read failed / stream ended; start not authorized')
            now = time.monotonic()
            if now >= deadline:
                break
            frames += 1
            items = reader.detect(frame)
            signature = tuple(sorted(x['text'] for x in items))
            if signature != previous:
                emit('qr_changed', detections=items)
                previous = signature
            direction = gate.update([x['text'] for x in items], now) if args.mode == 'start' else None
            if args.preview:
                annotate(cv2, frame, items, gate)
                cv2.imshow('PiCar QR - q to quit', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    emit('cancelled')
                    return 130
            if direction:
                emit('start_confirmed', direction=direction, frames=frames,
                     consecutive_frames=gate.count, hold_seconds=round(now-gate.first_seen, 3),
                     target_distance_cm=TARGET_DISTANCE_CM, distance_offset_cm=DISTANCE_OFFSET_CM,
                     motor_control=False)
                return 0
        emit('inspection_complete' if args.mode == 'inspect' else 'timeout',
             frames=frames, start_confirmed=False)
        return 0 if args.mode == 'inspect' else 2
    except KeyboardInterrupt:
        emit('cancelled')
        return 130
    except (RuntimeError, OSError, ValueError, cv2.error) as exc:
        emit('error', message=str(exc), start_confirmed=False)
        return 1
    finally:
        if cap is not None:
            cap.release()
        if args.preview:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass  # Headless OpenCV can reject GUI cleanup as well as imshow.


if __name__ == '__main__':
    sys.exit(main())
