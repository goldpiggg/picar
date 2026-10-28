"""One QR -> constant forward PWM -> stop at the configured distance."""
import math
from .mission_controller import MissionController, TERMINAL
from .distance import SensorFault
from .qr_scan import emit


class DirectMissionController(MissionController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.data.update(drive_mode='direct', configured_speed=self.cfg.direct_speed,
                         implemented_stages=['qr_recognition','direct_approach'],
                         stop_reason=None)
        if self.arm is not None:
            self.data['implemented_stages'].append('arm_transfer_sequence')

    def fail(self, reason):
        super().fail(reason)
        emit('drive_error',reason=str(reason),command_speed=0)

    def _read_distance(self, now):
        sample = self.read_sample()
        # A sensor thread may publish during the snapshot call. Compare time
        # after acquiring the sample, not before, so a fresh echo is not future.
        now = max(now,self.clock())
        if sample is None:
            raise SensorFault('NO_DISTANCE_SAMPLE')
        try:
            value, stamp = sample['distance_cm'], sample['captured_at']
            if any(type(x) not in (int,float) or not math.isfinite(x) for x in (value,stamp)):
                raise ValueError('invalid echo')
        except (KeyError,TypeError,ValueError) as exc:
            raise SensorFault('INVALID_DISTANCE_SAMPLE') from exc
        if not 2 <= value < 199:
            raise SensorFault('DISTANCE_OUT_OF_RANGE')
        if stamp > now or now-stamp > self.cfg.direct_distance_max_age_seconds:
            raise SensorFault('STALE_DISTANCE_SAMPLE')
        distance = float(value)+self.cfg.distance_offset_cm
        if distance <= 0:
            raise SensorFault('DISTANCE_OUT_OF_RANGE')
        self.data.update(distance_cm=round(distance,3),raw_distance_cm=round(distance,3),
                         distance_samples=1,distance_wait_reason=None)
        return distance

    def _step(self, now=None):
        now = self.clock() if now is None else now
        with self.lock:
            state = self.data['state']
            if state in TERMINAL or self.stop_event.is_set():
                return
            # The default arm flag is false. If an explicitly enabled, calibrated
            # arm is used later, retain its existing stationary hand-off checks.
            if state in ('VERIFY_STOP','ARM_TRANSFER'):
                return super()._step(now)
            if state=='WAIT_QR':
                qr = self.qr_snapshot()
                direction = qr.get('first_direction')
                if direction not in ('LEFT','RIGHT'):
                    if qr.get('state')=='ERROR':
                        raise SensorFault('QR_CAMERA_ERROR: '+str(qr.get('error')))
                    return
                # The QR worker already checks frame age before publishing this
                # one-shot direction. There is no second camera/startup gate.
                self.data['first_direction'] = direction
                self._stop()
                self.drive.straight()
                self.started_at = now
                self._state('DRIVING',now)

            distance = self._read_distance(now)
            if distance <= self.cfg.target_distance_cm:
                self._stop()
                self.data['stop_reason'] = 'TARGET_DISTANCE_REACHED'
                emit('drive_command',action='stop',pwm=0,distance_cm=round(distance,3))
                if self.cfg.arm_enabled and self.arm is not None and self.arm.profile.calibrated:
                    self.stop_at = now
                    self.verify_values.clear()
                    self.stable_since = None
                    self._state('VERIFY_STOP',now)
                else:
                    self.data.update(approach_complete=True,automatic_motion_enabled=False,
                                     drive_locked=True,
                                     arm_blocked_reason='DISABLED' if not self.cfg.arm_enabled else 'CALIBRATION_REQUIRED')
                    self._state('STOPPED_WAIT',now)
                return
            if now-self.started_at > self.cfg.approach_timeout_seconds:
                raise SensorFault('APPROACH_TIMEOUT')
            if self.data['command_speed'] != self.cfg.direct_speed:
                self.drive.forward(self.cfg.direct_speed)
                self.data['command_speed'] = self.cfg.direct_speed
                emit('drive_command',action='forward',pwm=self.cfg.direct_speed,
                     distance_cm=round(distance,3))
