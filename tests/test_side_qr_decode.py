"""Side PICK/PLACE decoding: small codes in a robot-sized frame, and slow decodes."""
from pathlib import Path
from types import SimpleNamespace
import unittest

from picar_mission.config import MissionConfig
from picar_mission.runtime import MissionRuntime
from picar_mission.sequence_selection import classify_side_start

FIXTURES = Path(__file__).resolve().parent/'fixtures'
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None


def side_frame(size):
    """The printed codes (tests/fixtures/qr.png) side by side in a 640x480 frame,
    PICK left of PLACE, each `size` px including its white margin."""
    source = cv2.imread(str(FIXTURES/'qr.png'))
    codes = {'PICK': source[52:318, 165:430], 'PLACE': source[475:740, 165:430]}
    rng = np.random.default_rng(1)
    frame = cv2.GaussianBlur(np.clip(rng.normal(128, 25, (480, 640, 3)), 0, 255).astype(np.uint8), (0, 0), 2)
    for label, cx in (('PICK', 200), ('PLACE', 440)):
        code = cv2.resize(codes[label], (size, size), interpolation=cv2.INTER_AREA)
        y0, x0 = 240-size//2, cx-size//2
        frame[y0:y0+size, x0:x0+size] = code
    return frame


@unittest.skipIf(cv2 is None, 'OpenCV is not installed')
class SmallCodeTests(unittest.TestCase):
    def setUp(self):
        from picar_mission.qr_scan import QRReader
        self.reader = QRReader(cv2)

    def test_small_codes_missed_by_plain_decoding_are_found_from_the_side(self):
        frame = cv2.GaussianBlur(side_frame(80), (0, 0), .5)
        self.assertNotEqual({d['label'] for d in self.reader.detect(frame)}, {'PICK', 'PLACE'})
        found = self.reader.detect_side(frame)
        self.assertEqual(sorted(d['label'] for d in found), ['PICK', 'PLACE'])
        # Positions are in the original frame, so the side order still holds.
        self.assertEqual(classify_side_start(found, 'LEFT'), 'LEFT-B')
        for item in found:
            x, y = item['center_normalized']
            self.assertAlmostEqual(y, .5, delta=.05)
            self.assertAlmostEqual(x, (200 if item['label'] == 'PICK' else 440)/640, delta=.05)

    def test_clear_codes_need_no_fallback(self):
        frame = side_frame(150)
        calls = []
        resize = cv2.resize
        try:
            cv2.resize = lambda *args, **kwargs: calls.append(1) or resize(*args, **kwargs)
            found = self.reader.detect_side(frame)
        finally:
            cv2.resize = resize
        self.assertEqual(sorted(d['label'] for d in found), ['PICK', 'PLACE'])
        self.assertEqual(calls, [])  # No upscaled retry for a clear view.


@unittest.skipIf(cv2 is None, 'OpenCV is not installed')
class CrumpledSheetTests(unittest.TestCase):
    def test_robot_frame_with_a_curled_place_sheet_still_selects_after_the_turn(self):
        # A real camera frame: PICK screen-left, PLACE on a curled sheet that
        # OpenCV 4.6 cannot decode at all.
        from picar_mission.qr_scan import QRReader
        frame = cv2.imread(str(FIXTURES/'side_crumpled.png'))
        found = QRReader(cv2).detect_side(frame, one_is_enough=True)
        # 4.6 reads only PICK; 4.8+ may stop at PLACE from its Aruco decoder.
        self.assertTrue({d['label'] for d in found} & {'PICK', 'PLACE'})
        self.assertEqual(classify_side_start(found, 'LEFT', settled=True), 'LEFT-B')
        self.assertEqual(classify_side_start(found, 'RIGHT', settled=True), 'RIGHT-A')


class SingleCodeTests(unittest.TestCase):
    def code(self, label, x):
        return [dict(text=label, label=label, center_normalized=[x, .5])]

    def test_one_code_decides_only_after_the_turn_and_off_center(self):
        for codes, left in [(self.code('PICK', .3), 'LEFT-B'), (self.code('PICK', .7), 'LEFT-A'),
                            (self.code('PLACE', .7), 'LEFT-B'), (self.code('PLACE', .3), 'LEFT-A')]:
            with self.subTest(codes=codes):
                self.assertIsNone(classify_side_start(codes, 'LEFT'))
                self.assertEqual(classify_side_start(codes, 'LEFT', settled=True), left)
        self.assertIsNone(classify_side_start(self.code('PICK', .42), 'LEFT', settled=True))

    def runtime(self, **cfg):
        now = [1.]
        runtime = MissionRuntime(MissionConfig(layout_frames=1, **cfg), clock=lambda: now[0])
        runtime.data.update(state='DIRECTION_CONFIRMED', first_direction='LEFT')
        runtime.request_side_qr('LEFT', now[0])
        return runtime, now

    def test_runtime_waits_for_the_turn_and_its_settle_time(self):
        runtime, now = self.runtime()
        reader = SimpleNamespace(detect=lambda frame: frame, detect_side=lambda frame, **_: frame)
        now[0] = 1.5
        self.assertIsNone(runtime.process_frame(self.code('PICK', .3), now[0], 1, reader))  # Still turning.
        self.assertTrue(runtime.side_view_settled(now[0]))
        self.assertIsNone(runtime.process_frame(self.code('PICK', .3), now[0]+.1, 2, reader))  # Before settle.
        now[0] = 2.
        self.assertEqual(runtime.process_frame(self.code('PICK', .3), now[0], 3, reader), 'LEFT-B')

    def test_single_code_can_be_turned_off(self):
        runtime, now = self.runtime(side_single_code=False)
        self.assertFalse(runtime.side_view_settled(now[0]))
        now[0] = 3.
        reader = SimpleNamespace(detect=lambda frame: frame, detect_side=lambda frame, **_: frame)
        self.assertIsNone(runtime.process_frame(self.code('PICK', .3), now[0], 1, reader))


class SlowDecodeTests(unittest.TestCase):
    def runtime(self):
        now = [1.]
        runtime = MissionRuntime(MissionConfig(layout_frames=1), clock=lambda: now[0])
        runtime.data.update(state='DIRECTION_CONFIRMED', first_direction='LEFT')
        return runtime, now

    def test_side_frame_decoded_slower_than_frame_age_still_selects(self):
        runtime, now = self.runtime()
        runtime.request_side_qr('LEFT', now[0])
        captured = now[0]+.4
        def slow(frame, **_):
            now[0] = captured+3.  # Twice max_frame_age_seconds on a slow Pi.
            return [dict(text='PICK', label='PICK', center_normalized=[.3, .5]),
                    dict(text='PLACE', label='PLACE', center_normalized=[.7, .5])]
        now[0] = captured
        reader = SimpleNamespace(detect=slow, detect_side=slow)
        self.assertEqual(runtime.process_frame(None, captured, 1, reader), 'LEFT-B')
        self.assertIsNotNone(runtime.snapshot()['decode_ms'])

    def test_start_qr_decoded_too_slowly_is_still_discarded(self):
        runtime, now = self.runtime()
        runtime.data['state'] = 'WAIT_ARM_RESET'
        runtime.begin_start_scan()
        def slow(frame):
            now[0] += 3.
            return [{'text': 'LEFT'}]
        self.assertIsNone(runtime.process_frame(None, now[0], 1, SimpleNamespace(detect=slow)))


if __name__ == '__main__': unittest.main()
