#!/usr/bin/python3
# File name   : Ultrasonic.py
# Description : Detection distance and tracking with ultrasonic
# Website     : www.adeept.com
# Author      : Adeept
# Date        : 2025/03/12

from gpiozero import DistanceSensor
from time import sleep, monotonic
from threading import Lock


class TimestampedDistanceSensor(DistanceSensor):
    """Observe gpiozero's actual echo acquisition, including failed echoes.

    gpiozero 2.x _read() is used deliberately; tests cover this integration.
    Reading .distance alone cannot tell a stale value from a fresh echo.
    """
    def __init__(self, *args, **kwargs):
        self._sample_lock = Lock()
        self._latest_sample = None
        self._sample_sequence = 0
        super().__init__(*args, **kwargs)

    def _read(self):
        stamp = monotonic()
        try:
            value = super()._read()
        except Exception:
            value = None
        with self._sample_lock:
            self._sample_sequence += 1
            self._latest_sample = {
                'distance_cm': None if value is None else value*self.max_distance*100,
                'captured_at': stamp,
                'sequence': self._sample_sequence,
            }
        return value

    def latest_sample(self):
        with self._sample_lock:
            return None if self._latest_sample is None else dict(self._latest_sample)

Tr = 23
Ec = 24
sensor = TimestampedDistanceSensor(echo=Ec, trigger=Tr, max_distance=2,
                                   queue_len=1, partial=True)


def read_sample():
    """Non-blocking raw-echo snapshot for the automatic approach controller."""
    return sensor.latest_sample()

def checkdist():
    value = (sensor.distance) * 100  # Unit: cm
    return round(value, 2)

if __name__ == "__main__":
    while True:
        distance = checkdist() 
        print("%.2f cm" %distance)
        sleep(0.05)
