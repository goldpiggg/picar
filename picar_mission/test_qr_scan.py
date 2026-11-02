import importlib.util
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from qr_scan import LayoutGate, StartGate, QRReader, classify_layout, normalize


def code(label, x, y):
    return {'text': label, 'label': label, 'center_normalized': [x, y]}


# LEFT_TOP: objects (PICK) at left-top and right-bottom, empty boxes (PLACE) elsewhere.
LEFT_TOP = [code('PICK', .2, .3), code('PLACE', .2, .7), code('PLACE', .8, .3), code('PICK', .8, .7)]
LEFT_BOTTOM = [code('PLACE', .2, .3), code('PICK', .2, .7), code('PICK', .8, .3), code('PLACE', .8, .7)]


class LayoutTests(unittest.TestCase):
    def test_both_diagonals(self):
        self.assertEqual(classify_layout(LEFT_TOP), 'LEFT_TOP')
        self.assertEqual(classify_layout(LEFT_BOTTOM), 'LEFT_BOTTOM')

    def test_any_hidden_code_still_decides(self):
        for layout, items in [('LEFT_TOP', LEFT_TOP), ('LEFT_BOTTOM', LEFT_BOTTOM)]:
            for hidden in range(4):
                with self.subTest(layout=layout, hidden=hidden):
                    self.assertEqual(classify_layout(items[:hidden]+items[hidden+1:]), layout)

    def test_only_place_codes_decide(self):
        # Sponges may cover both PICK codes; the empty boxes stay visible.
        self.assertEqual(classify_layout([x for x in LEFT_TOP if x['label'] == 'PLACE']), 'LEFT_TOP')

    def test_unclear_frames_give_no_layout(self):
        cases = {
            'single code': [code('PICK', .2, .3)],
            'same row only': [code('PICK', .2, .3), code('PLACE', .8, .3)],
            'level pair': [code('PICK', .2, .50), code('PLACE', .2, .53)],
            'contradiction': [code('PICK', .2, .3), code('PLACE', .2, .7), code('PICK', .8, .2)],
            'duplicate': LEFT_TOP+[code('PICK', .3, .4)],
            'start codes only': [code('LEFT', .2, .3), code('RIGHT', .8, .7)],
        }
        for name, items in cases.items():
            with self.subTest(name):
                self.assertIsNone(classify_layout(items))

    def test_start_sign_does_not_disturb_layout(self):
        self.assertEqual(classify_layout(LEFT_TOP+[code('LEFT', .5, .1)]), 'LEFT_TOP')

    def test_gate_needs_consecutive_agreeing_frames_then_latches(self):
        g = LayoutGate(required_frames=3)
        self.assertIsNone(g.update(LEFT_TOP))
        self.assertIsNone(g.update([]))  # An unclear frame restarts the count.
        self.assertIsNone(g.update(LEFT_TOP))
        self.assertIsNone(g.update(LEFT_BOTTOM))  # So does a different layout.
        self.assertIsNone(g.update(LEFT_BOTTOM))
        self.assertEqual(g.update(LEFT_BOTTOM), 'LEFT_BOTTOM')
        self.assertIsNone(g.update(LEFT_TOP))
        self.assertEqual(g.confirmed, 'LEFT_BOTTOM')


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
        p = HERE.parent / 'tests' / 'fixtures' / name
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

    def test_four_boxes_layout_from_a_real_decode(self):
        import numpy as np
        encoder = self.cv2.QRCodeEncoder.create()
        def tile(text):
            return self.cv2.cvtColor(self.cv2.resize(encoder.encode(text), (150, 150),
                                     interpolation=self.cv2.INTER_NEAREST), self.cv2.COLOR_GRAY2BGR)
        for layout, labels in [('LEFT_TOP', ('PICK', 'PLACE', 'PLACE', 'PICK')),
                               ('LEFT_BOTTOM', ('PLACE', 'PICK', 'PICK', 'PLACE'))]:
            with self.subTest(layout=layout):
                image = np.full((480, 640, 3), 255, dtype=np.uint8)
                # left-top, left-bottom, right-top, right-bottom
                for label, (y, x) in zip(labels, [(40, 60), (290, 60), (40, 430), (290, 430)]):
                    image[y:y+150, x:x+150] = tile(label)
                items = self.reader.detect(image)
                self.assertEqual(sorted(x['label'] for x in items), ['PICK', 'PICK', 'PLACE', 'PLACE'])
                self.assertEqual(classify_layout(items), layout)


if __name__ == '__main__': unittest.main()
