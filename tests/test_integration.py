"""Exercise actual QR worker and camera bridge without GPIO or a real webcam."""
import ast
import json
import importlib.util
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from picar_mission.config import MissionConfig, load_config
from picar_mission.runtime import MissionRuntime


class ConfigTests(unittest.TestCase):
    def test_repository_defaults(self):
        c = load_config()
        self.assertEqual((c.target_distance_cm, c.distance_offset_cm), (36., 0.))
        self.assertEqual(c.camera_backend, 'usb')

    def test_bad_settings_rejected(self):
        for content in ['{"required_frames":0}', '{"hold_seconds":-1}',
                        '{"qr_enabled":"yes"}', '{"camera_backend":"other"}',
                        '{"target_distance_cm":NaN}',
                        '{"arm_enabled":true,"arm_start_delay_seconds":-1}']:
            with self.subTest(content=content), tempfile.TemporaryDirectory() as d:
                p = Path(d) / 'config.json'
                p.write_text(content)
                with self.assertRaises(ValueError): load_config(p)

    def test_service_uses_existing_entry_and_quotes_spaces(self):
        from scripts.register_service import unit_text
        text = unit_text('/home/pi/picar kit')
        self.assertIn('ExecStart=/usr/bin/python3 "/home/pi/picar kit/Server/WebServer.py"', text)
        self.assertNotIn('main.py', text)

    def test_unused_precision_settings_cannot_block_direct_drive(self):
        from picar_mission.direct_mission_controller import DirectMissionController
        from test_approach import Drive
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json'
            path.write_text(json.dumps(dict(target_distance_cm=60.,slow_distance_cm=5.,
                distance_filter_size='unused',stop_samples=None,stop_timeout_seconds=0.,
                arm_start_delay_seconds='unused',
                direct_steer_preload_seconds=0.,direct_steer_settle_seconds=0.)))
            cfg=load_config(path)
        drive=Drive()
        controller=DirectMissionController(cfg,drive,
            lambda:dict(distance_cm=80.,captured_at=1.),
            lambda:dict(first_direction='LEFT'),clock=lambda:1.)
        controller.step()
        self.assertIn(('forward',40),drive.commands)

    def test_disabled_arm_settings_are_not_loaded_and_angles_remain_visible(self):
        from dataclasses import replace
        import test_approach
        bridge=test_approach.OfficialConnectionTests().load_bridge()
        bridge.config=replace(bridge.config,auto_approach_enabled=False)
        calls=[]
        servo=SimpleNamespace(nowPos=[90.]*8,stopWiggle=lambda:calls.append('hold'))
        move=SimpleNamespace(motorStop=lambda:calls.append('stop'))
        with patch.object(bridge,'load_profile',side_effect=AssertionError('disabled arm must not load')):
            bridge.configure(move,servo,SimpleNamespace(read_sample=lambda:None))
        self.assertIsNone(bridge.controller.arm)
        self.assertEqual(bridge.status()['commanded_servo_angles_deg'],
                         dict(base=90.,shoulder=90.,elbow=90.,gripper=90.))
        bridge.cancel()
        self.assertIn('hold',calls)


class FakeReader:
    def detect(self, frame):
        return [{'text': frame}]


class RuntimeTests(unittest.TestCase):
    def test_direction_reaches_shared_state_without_motion(self):
        runtime = MissionRuntime(MissionConfig(required_frames=3,hold_seconds=.35),clock=lambda: .5)
        for i, t in enumerate([.05, .25, .45]):
            direction = runtime.process_frame('RIGHT', t, i, FakeReader())
        self.assertEqual(direction, 'RIGHT')
        state = runtime.snapshot()
        self.assertEqual(state['first_direction'], 'RIGHT')
        self.assertEqual(state['state'], 'DIRECTION_CONFIRMED')
        self.assertFalse(state['automatic_motion_enabled'])
        self.assertEqual(state['target_distance_cm'], 36.)
        runtime.process_frame('LEFT', .5, 4, FakeReader())
        self.assertEqual(runtime.snapshot()['first_direction'], 'RIGHT')

    def test_stale_frames_and_duplicate_sequence_do_not_confirm(self):
        runtime = MissionRuntime(MissionConfig(required_frames=3,hold_seconds=.35),clock=lambda: 10.)
        for i in range(5): runtime.process_frame('LEFT', 0, i, FakeReader())
        runtime.process_frame('LEFT', 9.9, 9, FakeReader())
        for _ in range(5): runtime.process_frame('LEFT', 9.9, 9, FakeReader())
        self.assertIsNone(runtime.snapshot()['first_direction'])
        self.assertEqual(runtime.snapshot()['consecutive_frames'], 1)

    def test_slow_decode_cannot_confirm_old_frame(self):
        now = [0.]
        class SlowReader:
            def detect(self, frame):
                now[0] += 2.
                return [{'text':'LEFT'}]
        r = MissionRuntime(clock=lambda: now[0])
        r.process_frame(None, 0., 0, SlowReader())
        self.assertEqual(r.snapshot()['consecutive_frames'], 0)

    def test_decoder_exception_is_visible(self):
        def bad_factory(): raise RuntimeError('test decoder failure')
        r = MissionRuntime(reader_factory=bad_factory)
        r.start()
        r.worker.join(timeout=2)
        self.assertEqual(r.snapshot()['state'], 'ERROR')
        self.assertIn('test decoder failure', r.snapshot()['error'])
        r.stop()

    def test_disabled_starts_no_thread(self):
        r = MissionRuntime(MissionConfig(qr_enabled=False))
        r.start()
        self.assertIsNone(r.worker)
        self.assertEqual(r.snapshot()['state'], 'DISABLED')


@unittest.skipUnless(importlib.util.find_spec('cv2'), 'OpenCV unavailable')
class CameraConnectionTests(unittest.TestCase):
    def test_official_camera_passes_raw_frame_to_worker_and_releases_device(self):
        import cv2
        import numpy as np
        # Compile only the actual Camera class, excluding upstream module-level
        # imports and class-level servo initialization in CVThread.
        tree = ast.parse((ROOT/'Server/camera_opencv.py').read_text(encoding='utf-8'))
        node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name=='Camera')
        raw = np.full((480,640,3),255,dtype=np.uint8)
        class Capture:
            released=False
            def set(self,*args): pass
            def isOpened(self): return True
            def read(self): return True,raw
            def release(self): self.released=True
        class CVWorker:
            CVThreading=0
            def start(self): pass
            def pause(self): pass
        capture=Capture(); received=[]
        bridge=SimpleNamespace(config=MissionConfig(),submit_frame=received.append,
                               runtime=SimpleNamespace(fail=lambda message: self.fail(message)))
        namespace=dict(BaseCamera=object,CVThread=CVWorker,cv2=cv2,mission_bridge=bridge,hflip=0,vflip=0,ImgIsNone=0)
        exec(compile(ast.Module(body=[node],type_ignores=[]),'camera_opencv.py','exec'),namespace)
        with patch.object(cv2,'VideoCapture',return_value=capture) as opened:
            frames=namespace['Camera'].frames()
            self.assertTrue(next(frames).startswith(b'\xff\xd8'))
            frames.close()
        opened.assert_called_once_with(0)
        self.assertEqual(len(received),1)
        self.assertIs(received[0],raw)
        self.assertTrue(capture.released)

    def test_copied_latest_frame_queue_is_bounded(self):
        import numpy as np
        r = MissionRuntime()
        frame=np.zeros((4,4,3),dtype=np.uint8)
        r.submit_frame(frame,1.)
        frame[:]=255
        self.assertEqual(int(r.frames.get_nowait()[0].max()),0)
        for i in range(10): r.submit_frame(frame,float(i))
        self.assertEqual(r.frames.qsize(),1)
        self.assertEqual(r.frames.get_nowait()[1],9.)

    def test_actual_fixture_through_async_runtime(self):
        import cv2
        import numpy as np
        fixture=ROOT/'tests/fixtures/Left.png'
        frame=cv2.imdecode(np.frombuffer(fixture.read_bytes(),dtype=np.uint8),cv2.IMREAD_COLOR)
        r=MissionRuntime(MissionConfig(hold_seconds=.1,max_frame_age_seconds=2.))
        r.start()
        deadline=time.monotonic()+4.
        while time.monotonic()<deadline and r.snapshot()['first_direction'] is None:
            r.submit_frame(frame)
            time.sleep(.06)
        r.stop()
        self.assertEqual(r.snapshot()['first_direction'],'LEFT')
        self.assertFalse(r.snapshot()['automatic_motion_enabled'])


if __name__=='__main__': unittest.main()
