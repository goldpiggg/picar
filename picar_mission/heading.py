"""Gyro yaw integrator for straight-line heading hold. Never commands motion.

The yaw-rate reader is injected (MPU6050 z axis in deg/s on the robot). Bias is
averaged while the robot is still after server start, then integrated. Yaw is
positive counterclockwise (a left turn) once gyro_z_sign is set correctly.
"""
import json
import math
import threading
import time


class GyroHeading:
    def __init__(self, read_rate_dps, z_sign=1, calibration_seconds=1.5,
                 sample_hz=100., clock=time.monotonic):
        self.read_rate_dps = read_rate_dps
        self.z_sign = z_sign
        self.calibration_seconds = calibration_seconds
        self.period = 1./sample_hz
        self.clock = clock
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.worker = None
        self.started_at = None
        self.bias_sum, self.bias_count, self.bias = 0., 0, None
        self.yaw, self.rate, self.last_time = 0., None, None
        self.error = None

    def step(self, now=None):
        """One gyro read; also callable directly from tests."""
        now = self.clock() if now is None else now
        try:
            rate = self.z_sign*float(self.read_rate_dps())
            if not math.isfinite(rate):
                raise ValueError('non-finite gyro rate')
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
                self.last_time = None  # Do not integrate across a gap.
            return
        with self.lock:
            self.error = None
            if self.started_at is None:
                self.started_at = now
            if self.bias is None:
                self.bias_sum += rate
                self.bias_count += 1
                if now-self.started_at >= self.calibration_seconds:
                    self.bias = self.bias_sum/self.bias_count
                    self.last_time = now
                return
            self.rate = rate-self.bias
            if self.last_time is not None:
                self.yaw += self.rate*(now-self.last_time)
            self.last_time = now

    def read_sample(self):
        """Latest integrated yaw, or None before calibration or after a read failure."""
        with self.lock:
            if self.bias is None or self.error is not None or self.last_time is None:
                return None
            return {'yaw_deg': self.yaw, 'rate_dps': self.rate, 'captured_at': self.last_time}

    def snapshot(self):
        with self.lock:
            return {'calibrated': self.bias is not None,
                    'bias_dps': None if self.bias is None else round(self.bias, 3),
                    'yaw_deg': round(self.yaw, 2), 'error': self.error}

    def start(self):
        if self.worker is not None:
            return
        self.worker = threading.Thread(target=self._run, name='gyro-heading', daemon=True)
        self.worker.start()

    def _run(self):
        while not self.stop_event.wait(self.period):
            self.step()
            if self.error is not None:
                print(json.dumps({'event': 'gyro_error', 'message': self.error}), flush=True)
                self.stop_event.wait(1.)

    def stop(self):
        self.stop_event.set()
        if self.worker and self.worker is not threading.current_thread():
            self.worker.join(timeout=1.)
