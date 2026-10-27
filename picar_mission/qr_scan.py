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


class QRReader:
    def __init__(self, cv2):
        self.cv2 = cv2
        self.detector = cv2.QRCodeDetector()
        if not hasattr(self.detector, 'detectAndDecodeMulti'):
            raise RuntimeError('OpenCV lacks detectAndDecodeMulti; OpenCV 4.5+ is required.')

    def detect(self, frame):
        # Multi detection is mandatory: a single-code fallback could conceal
        # a conflicting second direction. There is intentionally no such fallback.
        ok, texts, points, _ = self.detector.detectAndDecodeMulti(frame)
        if not ok or points is None:
            return []
        height, width = frame.shape[:2]
        results = []
        for text, corners in zip(texts, points):
            if not text:
                continue
            polygon = [[float(x), float(y)] for x, y in corners.reshape(4, 2)]
            cx = sum(p[0] for p in polygon) / 4
            cy = sum(p[1] for p in polygon) / 4
            results.append({'text': text, 'label': normalize(text),
                            'corners_px': polygon,
                            'center_px': [cx, cy],
                            'center_normalized': [cx / width, cy / height]})
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
