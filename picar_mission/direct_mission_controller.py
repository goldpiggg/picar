"""One QR -> settle steering -> slow forward with gyro heading hold -> pulses to target.

Motor power is set in volts and sent as percent of Move.REFERENCE_VOLTS (5 V);
Move.py rescales that by the measured battery, so any charge drives the same.
Inside pulse_zone_cm the wheels stop; each cycle then lets the car settle,
takes the median of fresh echoes, and either finishes within the tolerance or
sends one short forward/backward pulse sized from the movement seen so far.
"""
from collections import deque
import math
from statistics import median
from .mission_controller import MissionController, TERMINAL
from .distance import SensorFault
from .qr_scan import emit

ZONE_SAMPLES = 3          # Fresh echoes whose median must be inside pulse_zone_cm.
ZONE_EXIT_MARGIN_CM = 2.  # Settled this far beyond the zone: it was a false entry.
REFERENCE_VOLTS = 5.     # Must match Server/Move.py REFERENCE_VOLTS.


class DirectMissionController(MissionController):
    def __init__(self, *args, read_heading=None, read_battery=None, **kwargs):
        kwargs['_precision'] = False
        super().__init__(*args, **kwargs)
        self.read_heading = read_heading
        self.read_battery = read_battery
        self.heading_ref = None
        self.target_stop_latched = False
        self.distance_retry_at = None
        self.distance_stamp = None
        self.cruise_pwm = self.pulse_pwm = None
        self.pulse_until = None
        self.pulse_direction = 0
        self.pulse_duration = None
        self.pulse_from_cm = None
        self.cm_per_second = None
        self.next_pulse_seconds = self.cfg.pulse_seconds
        self.measure_values, self.measure_last = [], None
        self.zone_values, self.zone_last = deque(maxlen=ZONE_SAMPLES), None
        self.data.update(drive_mode='direct', configured_speed=None,
                         implemented_stages=['qr_recognition','direct_approach','pulse_approach'],
                         stop_reason=None,target_stop_latched=False,steer_deg=0,
                         heading_status='OFF' if not self._heading_enabled() else 'WAITING',
                         heading_error_deg=None,battery_volts=None,battery_source=None,
                         cruise_pwm=None,pulse_pwm=None,pulse_count=0,last_pulse_seconds=None,
                         pulse_cm_per_second=None,final_distance_cm=None)
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
                         drive_locked=True,final_distance_cm=round(distance,3))
        if self.data['steer_deg'] != 0:
            self._center()  # Last steering command before the lock blocks it.
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
        self.distance_stamp = stamp
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
                self._set_power()
                self.started_at = now
                # Approach center from one side so linkage backlash settles the
                # same way every run, then let the servo arrive before moving.
                if self.cfg.direct_steer_preload_deg:
                    self._steer(self.cfg.direct_steer_preload_deg)
                    state = 'STEER_PRELOAD'
                else:
                    self._center()
                    state = 'STEER_SETTLE'
                self._state(state,now)
            if state=='STEER_PRELOAD':
                if now-self.state_at < self.cfg.direct_steer_preload_seconds:
                    return
                self._center()
                state = 'STEER_SETTLE'
                self._state(state,now)
            if state=='STEER_SETTLE':
                if now-self.state_at < self.cfg.direct_steer_settle_seconds:
                    return
                self._lock_heading_reference(now)
                self._state('DRIVING',now)

            if now-self.started_at > self.cfg.approach_timeout_seconds:
                raise SensorFault('APPROACH_TIMEOUT')
            if state in ('PULSE_SETTLE','PULSING'):
                self._pulse_step(state,now)
                return
            try:
                distance = self._read_distance(now)
            except SensorFault as exc:
                # A temporary missed echo pauses the wheels, not the mission.
                # Recovery is allowed only before the target-stop latch is set.
                self._stop()
                self.zone_values.clear()
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
            if self.distance_stamp != self.zone_last:
                self.zone_last = self.distance_stamp
                self.zone_values.append(distance)
            if len(self.zone_values)==ZONE_SAMPLES and median(self.zone_values) <= self.cfg.pulse_zone_cm:
                # Brake here and finish with measured pulses, never by coasting.
                # A median, not one echo, so a stray short echo far from the
                # target cannot switch the whole approach to pulses.
                self._begin_settle(now)
                emit('drive_command',action='stop',pwm=0,distance_cm=round(median(self.zone_values),3),
                     reason='PULSE_ZONE')
                return
            if distance <= self.cfg.pulse_zone_cm and self.data['command_speed']==0:
                return  # Possibly already near: confirm before the first forward.
            if state=='WAIT_DISTANCE':
                self._state('DRIVING',now)
            self._correct_heading(now)
            if self.data['command_speed'] != self.cruise_pwm:
                self.drive.forward(self.cruise_pwm)
                self.data['command_speed'] = self.cruise_pwm
                emit('drive_command',action='forward',pwm=self.cruise_pwm,
                     distance_cm=round(distance,3))

    # -- battery-compensated power and pulse approach ------------------------
    def _set_power(self):
        """Fix the 5 V-referenced PWM; the battery reading is kept for the status."""
        volts, source = None, 'FALLBACK'
        if self.read_battery is not None:
            try:
                volts = self.read_battery()
            except Exception:
                volts = None
        if type(volts) in (int,float) and math.isfinite(volts) and 4. <= volts <= 12.:
            source = 'MEASURED'
        else:
            volts = self.cfg.battery_fallback_volts
            emit('battery_fallback',battery_volts=volts)
        self.cruise_pwm = self._pwm(self.cfg.cruise_motor_volts)
        self.pulse_pwm = self._pwm(self.cfg.pulse_motor_volts)
        self.data.update(battery_volts=round(volts,2),battery_source=source,
                         cruise_pwm=self.cruise_pwm,pulse_pwm=self.pulse_pwm,
                         configured_speed=self.cruise_pwm)

    @staticmethod
    def _pwm(motor_volts):
        # Move.py treats 0..100 as percent of 5 V and compensates the battery.
        return round(min(100.,100.*motor_volts/REFERENCE_VOLTS),1)

    def _begin_settle(self, now):
        self._stop()
        if self.data['steer_deg'] != 0:
            self._center()  # Short pulses go straight; heading hold would only wobble.
        self.measure_values, self.measure_last = [], None
        self._state('PULSE_SETTLE',now)

    def _pulse_step(self, state, now):
        if state=='PULSING':
            if now >= self.pulse_until:
                self.pulse_until = None
                self._begin_settle(now)
            return
        settled_at = self.state_at+self.cfg.pulse_settle_seconds
        if now < settled_at:
            return
        try:
            distance = self._read_distance(now)
        except SensorFault as exc:
            distance = None
            reason = str(exc)
        if distance is not None and self.distance_stamp >= settled_at and \
                self.distance_stamp != self.measure_last:
            self.measure_last = self.distance_stamp
            self.measure_values.append(distance)
        elif now-settled_at > self.cfg.direct_distance_retry_seconds:
            raise SensorFault('STOP_DISTANCE_TIMEOUT'+('' if distance is not None else ': '+reason))
        if len(self.measure_values) < self.cfg.pulse_samples:
            return
        distance = median(self.measure_values)
        self.data['distance_cm'] = round(distance,3)
        self._learn_pulse(distance)
        if distance > self.cfg.pulse_zone_cm+ZONE_EXIT_MARGIN_CM:
            # Still far from the target: drive there instead of spending pulses.
            self.zone_values.clear()
            emit('pulse_zone_exit',distance_cm=round(distance,2))
            self._state('DRIVING',now)
            return
        error = distance-self.cfg.target_distance_cm
        if abs(error) <= self.cfg.stop_tolerance_cm:
            self._stop_at_target(now,distance)
            return
        if self.data['pulse_count'] >= self.cfg.max_pulses:
            raise SensorFault('PULSE_LIMIT_REACHED')
        self._send_pulse(1 if error > 0 else -1,abs(error),distance,now)

    def _learn_pulse(self, distance):
        if self.pulse_from_cm is None:
            return
        moved = (self.pulse_from_cm-distance)*self.pulse_direction
        if moved >= .3:
            self.cm_per_second = moved/self.pulse_duration
        else:
            # Did not beat static friction: push longer next time.
            self.cm_per_second = None
            self.next_pulse_seconds = self.pulse_duration*1.3
        self.data['pulse_cm_per_second'] = None if self.cm_per_second is None else round(self.cm_per_second,2)
        emit('pulse_result',moved_cm=round(moved,2),seconds=round(self.pulse_duration,3),
             distance_cm=round(distance,2))
        self.pulse_from_cm = None

    def _send_pulse(self, direction, remaining_cm, distance, now):
        seconds = remaining_cm/self.cm_per_second if self.cm_per_second else self.next_pulse_seconds
        seconds = max(self.cfg.pulse_min_seconds,min(self.cfg.pulse_max_seconds,seconds))
        (self.drive.forward if direction > 0 else self.drive.backward)(self.pulse_pwm)
        self.data['command_speed'] = self.pulse_pwm
        self.pulse_direction, self.pulse_duration, self.pulse_from_cm = direction, seconds, distance
        self.pulse_until = now+seconds
        self.data.update(pulse_count=self.data['pulse_count']+1,last_pulse_seconds=round(seconds,3))
        emit('pulse_command',action='forward' if direction > 0 else 'backward',
             pwm=self.pulse_pwm,seconds=round(seconds,3),distance_cm=round(distance,2))
        self._state('PULSING',now)

    def _wait_seconds(self):
        # End a pulse on time instead of up to one control period late.
        until = self.pulse_until
        if until is None:
            return self.cfg.control_period_seconds
        return max(0.,min(self.cfg.control_period_seconds,until-self.clock()))

    # -- steering and gyro heading hold ------------------------------------
    def _heading_enabled(self):
        return self.read_heading is not None and self.cfg.heading_correction_enabled

    def _steer(self, angle):
        self.drive.steer(angle)
        self.data['steer_deg'] = angle

    def _center(self):
        self.drive.straight()
        self.data['steer_deg'] = 0

    def _heading_yaw(self, now):
        try:
            sample = self.read_heading()
            yaw, stamp = sample['yaw_deg'], sample['captured_at']
            if any(type(x) not in (int,float) or not math.isfinite(x) for x in (yaw,stamp)):
                return None
        except (KeyError,TypeError,ValueError):
            return None
        now = max(now,self.clock())
        if stamp > now or now-stamp > self.cfg.gyro_max_age_seconds:
            return None
        return float(yaw)

    def _lock_heading_reference(self, now):
        if not self._heading_enabled():
            return
        self.heading_ref = self._heading_yaw(now)
        self.data['heading_status'] = 'ACTIVE' if self.heading_ref is not None else 'GYRO_UNAVAILABLE'

    def _correct_heading(self, now):
        # Without a gyro this keeps the centered steering from the settle step.
        if not self._heading_enabled():
            return
        yaw = self._heading_yaw(now)
        if yaw is None:
            self.data['heading_status'] = 'GYRO_UNAVAILABLE'
            if self.data['steer_deg'] != 0:
                self._center()
            return
        if self.heading_ref is None:
            self.heading_ref = yaw  # Gyro became ready after the settle step.
        # Yaw is counterclockwise-positive: drifting left gives a positive error
        # and a positive (manufacturer "right") steering angle.
        error = yaw-self.heading_ref
        limit = self.cfg.heading_max_steer_deg
        steer = 0 if abs(error) < self.cfg.heading_deadband_deg else \
            int(round(max(-limit,min(limit,self.cfg.heading_gain*error))))
        self.data.update(heading_status='ACTIVE',heading_error_deg=round(error,2))
        if steer != self.data['steer_deg']:
            self._steer(steer) if steer else self._center()
            emit('steer_command',steer_deg=steer,heading_error_deg=round(error,2))
