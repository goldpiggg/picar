#!/usr/bin/env/python
# File name   : base_camera.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date		  : 2025/03/12
import time
import threading
import cv2
try:
    from greenlet import getcurrent as get_ident
except ImportError:
    try:
        from thread import get_ident
    except ImportError:
        from _thread import get_ident


class CameraEvent(object):
    def __init__(self):
        self.events = {}

    def wait(self, timeout=5):
        ident = get_ident()
        if ident not in self.events:
            self.events[ident] = [threading.Event(), time.time()]
        return self.events[ident][0].wait(timeout)

    def set(self):
        now = time.time()
        remove = None
        for ident, event in list(self.events.items()):
            if not event[0].is_set():
                event[0].set()
                event[1] = now
            else:
                if now - event[1] > 5:
                    remove = ident
        if remove:
            del self.events[remove]

    def clear(self):
        self.events[get_ident()][0].clear()


class BaseCamera(object):
    STARTUP_TIMEOUT_SECONDS = 15
    FRAME_TIMEOUT_SECONDS = 5
    error = None
    thread = None
    frame = None  
    last_access = 0 
    event = CameraEvent()
    ready = threading.Event()
    thread_lock = threading.Lock()

    def __init__(self):
        with BaseCamera.thread_lock:
            if BaseCamera.thread is None:
                BaseCamera.error = None
                BaseCamera.frame = None
                BaseCamera.last_access = time.time()
                BaseCamera.event = CameraEvent()
                BaseCamera.ready = threading.Event()
                BaseCamera.thread = threading.Thread(target=self._thread, daemon=True)
                BaseCamera.thread.start()
            ready = BaseCamera.ready
        # Device opening/format negotiation can take longer than normal frame
        # delivery. A separate event also preserves errors raised immediately,
        # before a browser's per-client frame event has been registered.
        if not ready.wait(self.STARTUP_TIMEOUT_SECONDS):
            raise RuntimeError(
                'Camera startup timeout (%g seconds); check camera_source and camera ownership'
                % self.STARTUP_TIMEOUT_SECONDS)
        if BaseCamera.error is not None:
            raise RuntimeError('Camera capture failed: ' + str(BaseCamera.error))

    def get_frame(self):
        BaseCamera.last_access = time.time()
        if BaseCamera.error is not None:
            raise RuntimeError('Camera capture failed: ' + str(BaseCamera.error))
        if not BaseCamera.event.wait(self.FRAME_TIMEOUT_SECONDS):
            raise RuntimeError('Camera frame timeout (%g seconds)' % self.FRAME_TIMEOUT_SECONDS)
        BaseCamera.event.clear()
        if BaseCamera.error:
            raise RuntimeError('Camera capture failed: ' + str(BaseCamera.error))

        return BaseCamera.frame

    @staticmethod
    def frames():
        raise RuntimeError('Must be implemented by subclasses.')

    @classmethod
    def _thread(cls):
        try:
            frames_iterator = cls.frames()
            for frame in frames_iterator:
                if frame is None:
                    continue
                BaseCamera.frame = frame
                BaseCamera.event.set()
                BaseCamera.ready.set()
                time.sleep(0)
        except Exception as exc:
            BaseCamera.error = exc
        finally:
            if BaseCamera.frame is None and BaseCamera.error is None:
                BaseCamera.error = RuntimeError('Camera stream ended before the first frame')
            BaseCamera.event.set()
            BaseCamera.ready.set()
            with BaseCamera.thread_lock:
                BaseCamera.thread = None
