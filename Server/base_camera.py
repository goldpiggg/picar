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
        self.lock = threading.RLock()

    def wait(self, timeout=5):
        ident = get_ident()
        with self.lock:
            if ident not in self.events:
                self.events[ident] = [threading.Event(), time.time()]
            event = self.events[ident][0]
        return event.wait(timeout)

    def set(self):
        now = time.time()
        with self.lock:
            for ident, event in list(self.events.items()):
                if not event[0].is_set():
                    event[0].set()
                    event[1] = now
                elif now - event[1] > 5:
                    del self.events[ident]

    def clear(self):
        with self.lock:
            event = self.events.get(get_ident())
            if event is not None:
                event[0].clear()


class BaseCamera(object):
    AUTO_RECONNECT = False
    RECONNECT_DELAY_SECONDS = 1.
    STARTUP_TIMEOUT_SECONDS = 15
    FRAME_TIMEOUT_SECONDS = 5
    error = None
    thread = None
    frame = None  
    last_access = 0 
    event = CameraEvent()
    ready = threading.Event()
    thread_lock = threading.Lock()
    capture_stop = threading.Event()

    def __init__(self, wait_for_first_frame=True):
        with BaseCamera.thread_lock:
            if BaseCamera.thread is None:
                BaseCamera.error = None
                BaseCamera.frame = None
                BaseCamera.last_access = time.time()
                BaseCamera.event = CameraEvent()
                BaseCamera.ready = threading.Event()
                BaseCamera.capture_stop = threading.Event()
                BaseCamera.thread = threading.Thread(target=self._thread, daemon=True)
                BaseCamera.thread.start()
            ready = BaseCamera.ready
        if not wait_for_first_frame:
            return
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
        if BaseCamera.capture_stop.is_set():
            raise RuntimeError('Camera capture stopped')
        if BaseCamera.error is not None:
            raise RuntimeError('Camera capture failed: ' + str(BaseCamera.error))
        if not BaseCamera.event.wait(self.FRAME_TIMEOUT_SECONDS):
            if BaseCamera.error is not None:
                raise RuntimeError('Camera capture failed: ' + str(BaseCamera.error))
            raise RuntimeError('Camera frame timeout (%g seconds)' % self.FRAME_TIMEOUT_SECONDS)
        BaseCamera.event.clear()
        if BaseCamera.capture_stop.is_set():
            raise RuntimeError('Camera capture stopped')
        if BaseCamera.error:
            raise RuntimeError('Camera capture failed: ' + str(BaseCamera.error))

        return BaseCamera.frame

    @staticmethod
    def frames():
        raise RuntimeError('Must be implemented by subclasses.')

    def close(self):
        BaseCamera.capture_stop.set()
        BaseCamera.event.set()
        BaseCamera.ready.set()
        worker = BaseCamera.thread
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=2.)

    @classmethod
    def _thread(cls):
        try:
            while not BaseCamera.capture_stop.is_set():
                frames_iterator = None
                received_frame = False
                try:
                    frames_iterator = cls.frames()
                    for frame in frames_iterator:
                        if BaseCamera.capture_stop.is_set():
                            break
                        if frame is None:
                            continue
                        received_frame = True
                        BaseCamera.error = None
                        BaseCamera.frame = frame
                        BaseCamera.event.set()
                        BaseCamera.ready.set()
                        time.sleep(0)
                    if not BaseCamera.capture_stop.is_set():
                        raise RuntimeError('Camera stream ended' if received_frame else 'Camera stream ended before the first frame')
                except Exception as exc:
                    BaseCamera.frame = None
                    BaseCamera.error = exc
                finally:
                    try:
                        if frames_iterator is not None and hasattr(frames_iterator,'close'):
                            frames_iterator.close()
                    except Exception as exc:
                        BaseCamera.frame = None
                        BaseCamera.error = exc
                    BaseCamera.event.set()
                    BaseCamera.ready.set()
                if not cls.AUTO_RECONNECT or BaseCamera.capture_stop.wait(cls.RECONNECT_DELAY_SECONDS):
                    break
        finally:
            if BaseCamera.frame is None and BaseCamera.error is None and not BaseCamera.capture_stop.is_set():
                BaseCamera.error = RuntimeError('Camera stream ended before the first frame')
            BaseCamera.event.set()
            BaseCamera.ready.set()
            with BaseCamera.thread_lock:
                BaseCamera.thread = None
