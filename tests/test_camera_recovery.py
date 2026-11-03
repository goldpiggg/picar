"""Transient camera failures recover without importing or restarting robot drivers."""
import ast
import importlib.util
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from picar_mission.config import MissionConfig
from picar_mission.runtime import MissionRuntime

ROOT = Path(__file__).resolve().parent.parent


class CaptureRecoveryTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('recovering_base_camera',ROOT/'Server/base_camera.py')
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.base = self.module.BaseCamera
        self.base.RECONNECT_DELAY_SECONDS = .01
        self.base.STARTUP_TIMEOUT_SECONDS = .3
        self.base.FRAME_TIMEOUT_SECONDS = .1
        self.release = threading.Event()

    def tearDown(self):
        self.release.set()
        self.base.capture_stop.set()
        worker = self.base.thread
        if worker is not None:
            worker.join(1.)
            self.assertFalse(worker.is_alive())

    def camera(self,frames):
        return type('RecoveringCamera',(self.base,),{'AUTO_RECONNECT':True,'frames':staticmethod(frames)})

    def test_retry_closes_failed_attempt_and_reuses_one_capture_worker_for_all_clients(self):
        attempts, active, closed = [],[],[]
        recovered = threading.Event()
        def frames():
            index = len(attempts)+1
            attempts.append(index)
            active.append(index)
            self.assertEqual(len(active),1)
            try:
                if index==1: raise RuntimeError('USB camera frame read failed')
                yield b'fresh frame'
                recovered.set()
                while not self.release.is_set():
                    time.sleep(.002)
                    yield b'fresh frame'
            finally:
                active.remove(index)
                closed.append(index)
        cls = self.camera(frames)
        camera = cls(wait_for_first_frame=False)
        worker = self.base.thread
        for _ in range(12): cls(wait_for_first_frame=False)
        self.assertTrue(recovered.wait(1.))
        self.assertEqual(attempts,[1,2])
        self.assertEqual(closed,[1])
        self.assertIs(self.base.thread,worker)
        self.assertIsNone(self.base.error)
        self.assertEqual(camera.get_frame(),b'fresh frame')
        camera.close()
        self.assertEqual(closed,[1,2])

    def test_initial_camera_failure_does_not_block_nonwaiting_web_startup(self):
        attempted = threading.Event()
        def frames():
            attempted.set()
            raise RuntimeError('USB camera unavailable')
            yield
        started = time.monotonic()
        camera = self.camera(frames)(wait_for_first_frame=False)
        self.assertLess(time.monotonic()-started,.25)
        self.assertTrue(attempted.wait(.3))
        self.assertTrue(self.base.ready.wait(.3))
        with self.assertRaisesRegex(RuntimeError,'USB camera unavailable'): camera.get_frame()
        self.assertIsNone(self.base.frame)
        camera.close()

    def test_old_image_is_removed_on_failure_and_fresh_frame_clears_error(self):
        old_published, fault, recover, recovered = (threading.Event() for _ in range(4))
        attempts = []
        def frames():
            attempts.append(1)
            if len(attempts)==1:
                yield b'old image'
                old_published.set()
                fault.wait(.5)
                raise RuntimeError('disconnected')
            recover.wait(.5)
            yield b'new image'
            recovered.set()
            while not self.release.is_set():
                time.sleep(.002)
                yield b'new image'
        camera = self.camera(frames)(wait_for_first_frame=False)
        self.assertTrue(old_published.wait(.5))
        fault.set()
        with self.assertRaisesRegex(RuntimeError,'disconnected'): camera.get_frame()
        self.assertIsNone(self.base.frame)
        recover.set()
        self.assertTrue(recovered.wait(1.))
        self.assertIsNone(self.base.error)
        self.assertEqual(camera.get_frame(),b'new image')
        camera.close()

    def test_pruned_slow_client_can_clear_its_event_without_crashing(self):
        event = self.module.CameraEvent()
        with patch.object(self.module,'get_ident',return_value=42):
            event.wait(timeout=0.)
            event.set()
            event.events[42][1] -= 10
            event.set()
            self.assertNotIn(42,event.events)
            event.clear()


class USBRecoveryTests(unittest.TestCase):
    def camera(self,captures):
        import cv2
        node = next(n for n in ast.parse((ROOT/'Server/camera_opencv.py').read_text(encoding='utf-8')).body
            if isinstance(n,ast.ClassDef) and n.name=='Camera')
        workers, received, failures, starts = [],[],[],[]
        class CVWorker:
            CVThreading = 0
            def __init__(self): workers.append(self)
            def start(self): pass
            def is_alive(self): return True
            def pause(self): pass
        bridge = SimpleNamespace(config=MissionConfig(),submit_frame=received.append,
            camera_capture_started=lambda:starts.append(1),camera_failed=failures.append)
        namespace = dict(BaseCamera=object,CVThread=CVWorker,cv2=cv2,mission_bridge=bridge,
            time=SimpleNamespace(sleep=lambda seconds:None),hflip=0,vflip=0,ImgIsNone=0)
        exec(compile(ast.Module(body=[node],type_ignores=[]),'camera_opencv.py','exec'),namespace)
        return namespace['Camera'],workers,received,failures,starts

    def capture(self,results):
        class Capture:
            released = False
            reads = 0
            def __init__(self):self.settings=[]
            def set(self,*args):self.settings.append(args)
            def isOpened(self): return True
            def read(self):
                self.reads += 1
                return results[min(self.reads-1,len(results)-1)]
            def release(self): self.released = True
        return Capture()

    def test_single_bad_read_is_tolerated_without_failing_qr(self):
        import cv2,numpy as np
        raw = np.zeros((480,640,3),dtype=np.uint8)
        capture = self.capture([(False,None),(True,raw)])
        camera,workers,received,failures,starts = self.camera([capture])
        with patch.object(cv2,'VideoCapture',return_value=capture):
            stream = camera.frames()
            self.assertTrue(next(stream).startswith(b'\xff\xd8'))
            stream.close()
        self.assertEqual(capture.reads,2)
        self.assertTrue(capture.released)
        self.assertEqual(failures,[])
        self.assertEqual(len(received),1)

    def test_repeated_bad_reads_release_device_and_retry_reuses_cv_worker(self):
        import cv2,numpy as np
        bad = self.capture([(False,None)])
        good = self.capture([(True,np.zeros((480,640,3),dtype=np.uint8))])
        camera,workers,received,failures,starts = self.camera([bad,good])
        with patch.object(cv2,'VideoCapture',side_effect=[bad,good]):
            with self.assertRaisesRegex(RuntimeError,'USB camera frame read failed'): next(camera.frames())
            self.assertEqual(bad.reads,3)
            self.assertTrue(bad.released)
            stream = camera.frames()
            self.assertTrue(next(stream).startswith(b'\xff\xd8'))
            stream.close()
        self.assertEqual(len(workers),1)
        self.assertEqual(len(starts),2)
        self.assertEqual(len(failures),1)
        self.assertTrue(good.released)

    def test_low_resolution_negotiates_yuyv_and_only_publishes_full_size(self):
        import cv2,numpy as np
        small=np.zeros((120,160,3),dtype=np.uint8)
        full=np.zeros((480,640,3),dtype=np.uint8)
        capture=self.capture([(True,small)]*3+[(True,full)])
        camera,_,received,failures,_=self.camera([capture])
        with patch.object(cv2,'VideoCapture',return_value=capture):
            stream=camera.frames()
            jpg=next(stream)
            stream.close()
        self.assertEqual(capture.settings[:3],[(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG')),
            (cv2.CAP_PROP_FRAME_WIDTH,640),(cv2.CAP_PROP_FRAME_HEIGHT,480)])
        self.assertIn((cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'YUYV')),capture.settings)
        self.assertEqual(cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR).shape,(480,640,3))
        self.assertEqual(len(received),1)
        self.assertIs(received[0],full)
        self.assertEqual(failures,[])

    def test_persistent_low_resolution_reports_error_and_releases_capture(self):
        import cv2,numpy as np
        capture=self.capture([(True,np.zeros((120,160,3),dtype=np.uint8))])
        camera,_,received,failures,_=self.camera([capture])
        with patch.object(cv2,'VideoCapture',return_value=capture):
            with self.assertRaisesRegex(RuntimeError,'resolution mismatch.*160x120'):
                next(camera.frames())
        self.assertEqual(capture.reads,6)
        self.assertTrue(capture.released)
        self.assertEqual(received,[])
        self.assertEqual(len(failures),1)


class QRFailureRaceTests(unittest.TestCase):
    def test_decode_already_in_progress_cannot_confirm_direction_after_camera_failed(self):
        runtime = MissionRuntime()
        entered,release = threading.Event(),threading.Event()
        class Reader:
            def detect(self,frame):
                entered.set()
                release.wait(.5)
                return [dict(text='LEFT')]
        worker = threading.Thread(target=lambda:runtime.process_frame(object(),time.monotonic(),0,Reader()))
        worker.start()
        self.assertTrue(entered.wait(.5))
        runtime.fail('camera disconnected')
        release.set()
        worker.join(1.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(runtime.snapshot()['state'],'ERROR')
        self.assertIsNone(runtime.snapshot()['first_direction'])


if __name__ == '__main__': unittest.main()
