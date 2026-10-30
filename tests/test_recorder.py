"""Demonstration recorder writes CSVs from injected readers; no GPIO or servos."""
import csv
from datetime import datetime, timedelta
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from picar_mission.recorder import (DemoRecorder, EPISODE_FIELDS, KEYFRAME_FIELDS,
                                    TRAJECTORY_FIELDS, qr_features)


def square(label, x, y, side):
    pts = [[x, y], [x+side, y], [x+side, y+side], [x, y+side]]
    return {'text': label, 'label': label, 'corners_px': pts,
            'center_px': [x+side/2, y+side/2]}


class FakeReader:
    def __init__(self, detections):
        self.detections = detections

    def detect(self, frame):
        return self.detections


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.clock = Clock()
        self.angles = {'base': 90., 'shoulder': 85., 'elbow': 100., 'gripper': 110.}
        self.distance = {'distance_cm': 37.26, 'captured_at': 99.9}
        self.wall = datetime(2026, 10, 30, 14, 0, 0)
        self.rec = self.make()

    def tearDown(self):
        self.tmp.cleanup()

    def make(self, detections=()):
        return DemoRecorder(self.dir, lambda: self.angles, lambda: self.distance,
                            reader_factory=lambda: FakeReader(list(detections)),
                            threaded=False, clock=self.clock, wall=lambda: self.wall)

    def rows(self, name):
        with (self.dir/name).open(newline='', encoding='utf-8') as handle:
            return list(csv.DictReader(handle))

    def test_full_episode_writes_three_csvs(self):
        self.rec = self.make([square('PICK', 320, 120, 64), square('PICK', 10, 10, 20)])
        self.rec.start('LEFT', 'stop 37cm')
        self.rec.submit_frame(np.zeros((480, 640, 3), np.uint8))
        self.rec.decode_once()
        self.rec.note_command('armdown')
        self.clock.now += .05
        self.rec.sample_once()
        self.angles = {**self.angles, 'shoulder': 104.}
        self.clock.now += .05
        self.rec.sample_once()
        row = self.rec.mark('pick', 'grasp')
        self.rec.note_command(None)
        self.rec.mark('pick', 'close', ok=False)
        self.rec.stop(True, 36.5)

        self.assertEqual(row['shoulder'], 104.)
        self.assertEqual(row['command'], 'armdown')
        self.assertTrue((self.dir/row['frame_file']).exists())
        traj = self.rows('trajectory.csv')
        self.assertEqual(list(traj[0]), TRAJECTORY_FIELDS)
        self.assertEqual([r['shoulder'] for r in traj], ['85.0', '104.0'])
        keys = self.rows('keyframes.csv')
        self.assertEqual(list(keys[0]), KEYFRAME_FIELDS)
        self.assertEqual([(k['label'], k['ok']) for k in keys], [('grasp', '1'), ('close', '0')])
        # Largest PICK code wins, normalized by frame width.
        self.assertEqual((keys[0]['pick_cx'], keys[0]['pick_size']), ('0.55', '0.1'))
        self.assertEqual(keys[0]['place_cx'], '')
        self.assertEqual(keys[0]['distance_cm'], '37.3')
        episodes = self.rows('episodes.csv')
        self.assertEqual(list(episodes[0]), EPISODE_FIELDS)
        self.assertEqual(episodes[0]['episode_id'], '20261030-140000')
        self.assertEqual((episodes[0]['success'], episodes[0]['stop_distance_cm']), ('1', '36.5'))
        self.assertEqual((episodes[0]['keyframes'], episodes[0]['trajectory_rows']), ('2', '2'))
        self.assertEqual(episodes[0]['note'], 'stop 37cm')

    def test_appends_episodes_and_counts_existing(self):
        self.rec.start('LEFT')
        self.rec.stop()
        self.wall += timedelta(seconds=5)
        again = self.make()
        self.assertEqual(again.status()['episodes_total'], 1)
        again.start('RIGHT')
        again.stop(False, note='dropped')
        episodes = self.rows('episodes.csv')
        self.assertEqual([e['side'] for e in episodes], ['LEFT', 'RIGHT'])
        self.assertEqual(episodes[1]['success'], '0')

    def test_invalid_requests_rejected(self):
        with self.assertRaises(ValueError):
            self.rec.start('UP')
        with self.assertRaises(ValueError):
            self.rec.mark('pick', 'grasp')
        with self.assertRaises(ValueError):
            self.rec.stop()
        self.rec.start('LEFT')
        with self.assertRaises(ValueError):
            self.rec.start('LEFT')
        for task, label in [('lift', 'grasp'), ('pick', 'Bad Label'), ('pick', '../x')]:
            with self.subTest(task=task, label=label), self.assertRaises(ValueError):
                self.rec.mark(task, label)
        for value in [-1, float('nan'), True, '36']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.rec.stop(stop_distance_cm=value)
        self.rec.stop()

    def test_missing_sensors_leave_blank_cells(self):
        self.angles, self.distance = None, None
        self.rec.start('RIGHT')
        row = self.rec.mark('place', 'open')
        self.assertEqual((row['base'], row['distance_cm'], row['frame_file']), ('', '', ''))
        self.rec.stop()

    def test_header_mismatch_refuses_to_mix_files(self):
        (self.dir/'trajectory.csv').write_text('old,header\n', encoding='utf-8')
        with self.assertRaises(ValueError):
            self.rec.start('LEFT')
        self.assertFalse(self.rec.status()['recording'])

    def test_frames_ignored_when_not_recording(self):
        self.rec.submit_frame(np.zeros((4, 4, 3), np.uint8))
        self.assertIsNone(self.rec.frame)

    def test_qr_features_without_codes(self):
        self.assertEqual(set(qr_features([], (480, 640)).values()), {None})

    def test_close_saves_open_episode(self):
        self.rec.start('LEFT')
        self.rec.close()
        self.assertEqual(self.rows('episodes.csv')[0]['note'], 'server_shutdown')


if __name__ == '__main__':
    unittest.main()
