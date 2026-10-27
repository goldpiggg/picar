"""Validate *new echo* samples, with a short median filter (units: cm)."""
from collections import deque
from dataclasses import dataclass
import math
from statistics import median


class SensorFault(RuntimeError):
    pass


@dataclass(frozen=True)
class DistanceReading:
    raw_cm: float
    filtered_cm: float
    captured_at: float
    sequence: int
    is_new: bool
    ready: bool


class DistanceFilter:
    def __init__(self, config):
        self.cfg = config
        self.values = deque(maxlen=config.distance_filter_size)
        self.previous = None

    def update(self, sample, now):
        if sample is None:
            raise SensorFault('NO_DISTANCE_SAMPLE')
        try:
            value = sample['distance_cm']
            stamp = sample['captured_at']
            sequence = sample['sequence']
            if any(isinstance(x, bool) or not isinstance(x, (int,float)) or not math.isfinite(x)
                   for x in (value,stamp)):
                raise ValueError('invalid distance/time')
            if type(sequence) is not int or sequence < 1:
                raise ValueError('invalid sequence')
            raw = float(value) + self.cfg.distance_offset_cm
        except (KeyError, TypeError, ValueError) as exc:
            raise SensorFault('INVALID_DISTANCE_SAMPLE') from exc
        if not 2 <= raw < 199:
            raise SensorFault('DISTANCE_OUT_OF_RANGE')
        if stamp > now or now-stamp > self.cfg.distance_max_age_seconds:
            raise SensorFault('STALE_DISTANCE_SAMPLE')
        if self.previous:
            old_raw, old_stamp, old_sequence = self.previous
            if sequence < old_sequence or stamp < old_stamp:
                raise SensorFault('DISTANCE_SEQUENCE_REVERSED')
            if sequence == old_sequence:
                if stamp != old_stamp or raw != old_raw:
                    raise SensorFault('DISTANCE_SEQUENCE_REUSED')
                return DistanceReading(raw, median(self.values), stamp, sequence, False,
                                       len(self.values)==self.values.maxlen)
            allowed = self.cfg.distance_jump_slack_cm + self.cfg.max_closing_speed_cm_s*(stamp-old_stamp)
            if abs(raw-old_raw) > allowed:
                raise SensorFault('IMPLAUSIBLE_DISTANCE_JUMP')
        self.previous = raw,stamp,sequence
        self.values.append(raw)
        return DistanceReading(raw, median(self.values), stamp, sequence, True,
                               len(self.values)==self.values.maxlen)
