"""One QR -> constant forward PWM -> stop at the configured distance."""
import math
from .mission_controller import MissionController, TERMINAL
from .distance import SensorFault
from .qr_scan import emit


class DirectMissionController(MissionController):
    def __init__(self, *args, **kwargs):
        kwargs['_precision'] = False
        super().__init__(*args, **kwargs)
        self.target_stop_latched = False
        self.distance_retry_at = None
        self.data.update(drive_mode='direct', configured_speed=self.cfg.direct_speed,
                         implemented_stages=['qr_recognition','direct_approach'],
                         stop_reason=None,target_stop_latched=False)
        if self.arm is not None:
            self.data['implemented_stages'].append('arm_transfer_sequence')

    def fail(self, reason):
        super().fail(reason)
        emit('drive_error',reason=str(reason),command_speed=0)

    def _stop_at_target(self, now, distance):
        # This latch is never cleared during this mission, even if the sensor
        # later points elsewhere or reports a different distance.
        self.target_stop_latched = True
        self.data.update(target_stop_latched=True,stop_reason='TARGET_DISTANCE_REACHED',
                         drive_locked=True)
        lock = getattr(self.drive,'lock',None)
        if lock is not None:
            lock()
        self._stop()
        self.data['approach_complete'] = True
        self.stop_at = max(now,self.clock())
        emit('drive_command',action='stop',pwm=0,distance_cm=round(distance,3))
        if self.cfg.arm_enabled and self.arm is not None and self.arm.profile.calibrated:
            self.data.update(automatic_motion_enabled=True,arm_blocked_reason=None,
                             pending_stages=['arm_transfer'])
            self._state('ARM_WAIT',now)
        else:
            self.data.update(automatic_motion_enabled=False,
                             arm_blocked_reason='DISABLED' if not self.cfg.arm_enabled else 'CALIBRATION_REQUIRED')
            self._state('STOPPED_WAIT',now)

    def _after_stop(self, now):
        # Never sample front distance, recenter steering, or send forward here.
        # Arm movement can rotate the ultrasonic sensor away from the monitor.
        if self.data['command_speed']!=0:
            raise RuntimeError('ARM_REQUIRES_STOPPED_DRIVE')
        if self.data['state']=='ARM_WAIT':
            if now-self.stop_at >= self.cfg.arm_start_delay_seconds:
                self.arm.start(self.data['first_direction'],now)
                self.data['arm_motion_enabled'] = True
                self._state('ARM_TRANSFER',now)
        elif self.data['state']=='ARM_TRANSFER':
            if self.arm.tick(now)=='COMPLETE':
                self._stop()
                self.data.update(arm_motion_enabled=False,automatic_motion_enabled=False,
                                 pending_stages=[])
                self._state('COMPLETE',now)

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
            if self.target_stop_latched:
                self._after_stop(now)
                return
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

            if now-self.started_at > self.cfg.approach_timeout_seconds:
                raise SensorFault('APPROACH_TIMEOUT')
            try:
                distance = self._read_distance(now)
            except SensorFault as exc:
                # A temporary missed echo pauses the wheels, not the mission.
                # Recovery is allowed only before the target-stop latch is set.
                self._stop()
                if self.distance_retry_at is None:
                    self.distance_retry_at = max(now,self.clock())
                self.data.update(distance_wait_reason=str(exc),distance_samples=0,
                                 distance_cm=None,raw_distance_cm=None)
                if now-self.distance_retry_at > self.cfg.direct_distance_retry_seconds:
                    raise SensorFault('DISTANCE_RECOVERY_TIMEOUT: '+str(exc))
                if state!='WAIT_DISTANCE':
                    self._state('WAIT_DISTANCE',now)
                    emit('distance_retry',reason=str(exc),command_speed=0)
                return
            self.distance_retry_at = None
            if distance <= self.cfg.target_distance_cm:
                self._stop_at_target(now,distance)
                return
            if state=='WAIT_DISTANCE':
                self._state('DRIVING',now)
            if self.data['command_speed'] != self.cfg.direct_speed:
                self.drive.forward(self.cfg.direct_speed)
                self.data['command_speed'] = self.cfg.direct_speed
                emit('drive_command',action='forward',pwm=self.cfg.direct_speed,
                     distance_cm=round(distance,3))
