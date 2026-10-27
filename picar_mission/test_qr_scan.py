import importlib.util
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from qr_scan import StartGate, QRReader, normalize


class GateTests(unittest.TestCase):
    def test_needs_time_and_consecutive_frames(self):
        g = StartGate()
        self.assertIsNone(g.update(['LEFT'], 0))
        self.assertIsNone(g.update(['LEFT'], .1))
        self.assertIsNone(g.update(['LEFT'], .2))
        self.assertEqual(g.update(['LEFT'], .4), 'LEFT')
        self.assertIsNone(g.update(['RIGHT'], .8))
        self.assertEqual(g.confirmed, 'LEFT')

    def test_missing_resets(self):
        g = StartGate()
        for t in [0, .2]: g.update(['LEFT'], t)
        g.update([], .3)
        self.assertIsNone(g.update(['LEFT'], .5))
        self.assertEqual(g.count, 1)

    def test_conflict_resets(self):
        g = StartGate()
        g.update(['LEFT'], 0)
        self.assertIsNone(g.update(['LEFT', 'RIGHT'], .3))
        self.assertEqual(g.count, 0)

    def test_duplicate_start_symbols_are_ambiguous(self):
        g = StartGate()
        self.assertIsNone(g.update(['LEFT', 'LEFT'], 0))
        self.assertEqual(g.count, 0)

    def test_object_codes_do_not_start(self):
        g = StartGate()
        for t in [0, .2, .5]: self.assertIsNone(g.update(['PICK', 'PLACE'], t))

    def test_direction_change_restarts_confirmation(self):
        g = StartGate()
        g.update(['LEFT'], 0)
        g.update(['LEFT'], .2)
        self.assertIsNone(g.update(['RIGHT'], .4))
        self.assertIsNone(g.update(['RIGHT'], .6))
        self.assertEqual(g.update(['RIGHT'], .8), 'RIGHT')

    def test_long_gap_restarts(self):
        g = StartGate()
        g.update(['RIGHT'], 0)
        g.update(['RIGHT'], .2)
        self.assertIsNone(g.update(['RIGHT'], 1.5))
        self.assertEqual(g.count, 1)

    def test_payload_allowlist(self):
        self.assertEqual(normalize(' left\n'), 'LEFT')
        self.assertIsNone(normalize('https://LEFT'))
        self.assertIsNone(normalize('LEFT;shutdown'))


@unittest.skipUnless(importlib.util.find_spec('cv2'), 'OpenCV unavailable')
class AttachedQRTests(unittest.TestCase):
    def setUp(self):
        import cv2
        self.cv2 = cv2
        self.reader = QRReader(cv2)

    def load(self, name):
        import numpy as np
        p = HERE.parent / 'tmp' / 'pdfs' / name
        if not p.exists(): self.skipTest('Original attachment image not available')
        return self.cv2.imdecode(np.frombuffer(p.read_bytes(), dtype=np.uint8), self.cv2.IMREAD_COLOR)

    def test_attached_left_right(self):
        for file, expected in [('Left.png', 'LEFT'), ('Right.png', 'RIGHT')]:
            with self.subTest(file=file):
                items = self.reader.detect(self.load(file))
                self.assertEqual([x['label'] for x in items], [expected])
                g = StartGate()
                for t in [0., .2, .4]: result = g.update([x['text'] for x in items], t)
                self.assertEqual(result, expected)

    def test_attached_pick_place(self):
        items = self.reader.detect(self.load('qr.png'))
        self.assertEqual({x['label'] for x in items}, {'PICK', 'PLACE'})

    def test_blank_image(self):
        import numpy as np
        self.assertEqual(self.reader.detect(np.full((480, 640, 3), 255, dtype=np.uint8)), [])

    def test_two_directions_in_one_frame(self):
        import numpy as np
        left, right = self.load('Left.png'), self.load('Right.png')
        a = self.cv2.resize(left, (350, 350))
        b = self.cv2.resize(right, (350, 350))
        image = np.full((450, 850, 3), 255, dtype=np.uint8)
        image[50:400, 25:375] = a
        image[50:400, 475:825] = b
        items = self.reader.detect(image)
        self.assertEqual({x['label'] for x in items}, {'LEFT', 'RIGHT'})
        g = StartGate()
        for t in [0, .2, .4]: self.assertIsNone(g.update([x['text'] for x in items], t))


if __name__ == '__main__': unittest.main()
