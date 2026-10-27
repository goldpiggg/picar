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
            if not event[0].isSet():
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
    error = None
    thread = None
    frame = None  
    last_access = 0 
    event = CameraEvent()

    def __init__(self):
        if BaseCamera.thread is None:
            BaseCamera.error = None
            BaseCamera.last_access = time.time()

            BaseCamera.thread = threading.Thread(target=self._thread)
            BaseCamera.thread.daemon = True
            BaseCamera.thread.start()
            while self.get_frame() is None:
                time.sleep(0)

    def get_frame(self):
        BaseCamera.last_access = time.time()
        if not BaseCamera.event.wait():
            raise RuntimeError('Camera frame timeout (5 seconds)')
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
                BaseCamera.frame = frame
                BaseCamera.event.set()
                time.sleep(0)
        except Exception as exc:
            BaseCamera.error = exc
        finally:
            BaseCamera.event.set()
            BaseCamera.thread = None
