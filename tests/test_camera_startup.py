"""Exercise the real camera startup/thread lifecycle without any robot drivers."""
import importlib.util
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent


class StartupTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('startup_base_camera', ROOT/'Server/base_camera.py')
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.base = self.module.BaseCamera
        self.base.STARTUP_TIMEOUT_SECONDS = .5
        self.base.FRAME_TIMEOUT_SECONDS = .02
        self.release = threading.Event()
        self.worker = None

    def tearDown(self):
        self.release.set()
        worker = self.worker or self.base.thread
        if worker is not None:
            worker.join(timeout=1.)
            self.assertFalse(worker.is_alive())

    def camera(self, frames):
        return type('FakeCamera', (self.base,), {'frames': staticmethod(frames)})

    def test_slow_first_frame_uses_startup_budget(self):
        release = self.release
        def frames():
            time.sleep(.06)  # Exceeds the normal frame budget, not startup.
            yield b'first frame'
            release.wait()
        normal_wait=self.module.CameraEvent.wait
        def short_frame_wait(event,timeout=.02): return normal_wait(event,timeout)
        # The old constructor used this short per-frame wait for startup too.
        with patch.object(self.module.CameraEvent,'wait',short_frame_wait):
            self.camera(frames)()
        self.assertEqual(self.base.frame, b'first frame')
        self.assertIsNone(self.base.error)

    def test_immediate_capture_error_is_not_lost(self):
        def frames():
            raise RuntimeError('USB camera unavailable in test')
            yield
        with self.assertRaisesRegex(RuntimeError, 'USB camera unavailable in test'):
            self.camera(frames)()

    def test_empty_stream_reports_reason_instead_of_timeout(self):
        def frames():
            return
            yield
        with self.assertRaisesRegex(RuntimeError, 'before the first frame'):
            self.camera(frames)()

    def test_startup_timeout_stays_bounded(self):
        release = self.release
        self.base.STARTUP_TIMEOUT_SECONDS = .02
        def frames():
            release.wait()
            yield b'late frame'
        with self.assertRaisesRegex(RuntimeError, 'Camera startup timeout'):
            self.camera(frames)()
        self.worker = self.base.thread

    def test_normal_frame_timeout_is_not_extended(self):
        release = self.release
        def frames():
            yield b'first frame'
            release.wait()
        camera = self.camera(frames)()
        with self.assertRaisesRegex(RuntimeError, 'Camera frame timeout'):
            camera.get_frame()


if __name__ == '__main__':
    unittest.main()
