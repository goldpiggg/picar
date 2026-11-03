"""One QR -> forward/pulses to target -> BASE toward that side -> side QR
sequence -> taught-position transfer when enabled, otherwise wait.

Motor power is set in volts and sent as percent of Move.REFERENCE_VOLTS (7 V);
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
from .sequence_selection import START_POSITIONS, sequence_for

ZONE_SAMPLES = 3          # Fresh echoes whose median must be inside pulse_zone_cm.
ZONE_EXIT_MARGIN_CM = 2.  # Settled this far beyond the zone: it was a false entry.
REFERENCE_VOLTS = 7.     # Must match Server/Move.py REFERENCE_VOLTS.
QR_DECODE_GRACE_SECONDS = .6  # Final stationary view: finish a slow Pi decode.
MAX_REVERSE_PULSES = 3
MAX_REVERSE_SECONDS = .15


class DirectMissionController(MissionController):
    def __init__(self, *args, read_heading=None, read_battery=None, qr_preparation=None,
                 arm_reset=None, on_reset_complete=None, request_side_qr=None,
                 transfer_blocked_reason=None, settle_side_qr=None, view_scan_enabled=False, **kwargs):
        kwargs['_precision'] = False
        super().__init__(*args, **kwargs)
        # Why arm_sequence is None, e.g. untaught positions; shown on selection.
        self.transfer_blocked_reason = transfer_blocked_reason
        # Tells the QR worker the turn has finished (one code may then do).
        self.settle_side_qr = settle_side_qr
        self.view_scan_enabled = view_scan_enabled
        self.view_scan_started = None
        self.view_scan_next = None
        self.read_heading = read_heading
        self.read_battery = read_battery
        self.qr_preparation = qr_preparation
        self.arm_reset = arm_reset
        self.on_reset_complete = on_reset_complete
        self.request_side_qr = request_side_qr
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
                         implemented_stages=['arm_reset','qr_recognition','direct_approach','pulse_approach','qr_view_preparation','sequence_selection','arm_transfer'],
                         pending_stages=['qr_view_preparation','pick_place_qr','arm_transfer'],
                         qr_view_ready=False,sequence_selection=None,arm_requested=True,
                         arm_blocked_reason=None if qr_preparation is not None else 'QR_PREPARATION_UNAVAILABLE',
                         stop_reason=None,target_stop_latched=False,steer_deg=0,
                         heading_status='OFF' if not self._heading_enabled() else 'WAITING',
                         heading_error_deg=None,battery_volts=None,battery_source=None,
                         cruise_pwm=None,pulse_pwm=None,pulse_count=0,reverse_pulse_count=0,last_pulse_seconds=None,
                         pulse_cm_per_second=None,final_distance_cm=None)

    def snapshot(self):
        with self.lock:
            result = super().snapshot()
            result['qr_preparation'] = None if self.qr_preparation is None else self.qr_preparation.snapshot()
            result['arm_reset'] = None if self.arm_reset is None else self.arm_reset.snapshot()
            return result

    def start(self):
        with self.lock:
            if not self.cfg.auto_approach_enabled or self.worker is not None:
                return
            if self.arm_reset is not None:
                self._stop()
                try:
                    self.arm_reset.start(self.clock())
                    self.data['arm_motion_enabled'] = True
                    self._state('ARM_RESET',self.clock())
                except Exception as exc:
                    self.fail(str(exc))
                    return
            super().start()

    def cancel(self, reason='USER_STOP'):
        with self.lock:
            if self.arm_reset is not None:
                self.arm_reset.cancel()
            if self.qr_preparation is not None:
                self.qr_preparation.cancel()
            super().cancel(reason)

    def fail(self, reason):
        with self.lock:
            try:
                if self.arm_reset is not None:
                    self.arm_reset.cancel()
                if self.qr_preparation is not None:
                    self.qr_preparation.cancel()
            finally:
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
        if self.qr_preparation is not None:
            self.data.update(automatic_motion_enabled=True,arm_blocked_reason=None,
                             pending_stages=['qr_view_preparation','pick_place_qr','arm_transfer'])
            self._state('ARM_WAIT',now)
        else:
            self.data.update(automatic_motion_enabled=False,
                             arm_blocked_reason='QR_PREPARATION_UNAVAILABLE')
            self._state('STOPPED_WAIT',now)

    def _after_stop(self, now):
        # Never sample front distance, recenter steering, or send forward here.
        # Arm movement can rotate the ultrasonic sensor away from the monitor.
        if self.data['command_speed']!=0:
            raise RuntimeError('ARM_REQUIRES_STOPPED_DRIVE')
        if self.data['state']=='ARM_WAIT':
            delay = min(self.cfg.arm_start_delay_seconds,.05) if self.cfg.fast_mission_enabled else self.cfg.arm_start_delay_seconds
            if now-self.stop_at >= delay:
                self.qr_preparation.start(self.data['first_direction'],now)
                # Read PICK/PLACE while the camera is still turning toward that
                # side: their left/right order does not change during the turn.
                if self.request_side_qr is not None and not self.request_side_qr(self.data['first_direction'],now):
                    raise RuntimeError('SIDE_QR_SCAN_NOT_STARTED')
                self.data['arm_motion_enabled'] = True
                self._state('QR_TURNING',now)
        elif self.data['state'] in ('QR_TURNING','QR_SCAN_MOVING'):
            position = self._side_selection() if self.request_side_qr is not None else None
            if position is not None and self._transfer_blocked_reason() is None:
                # Recognised mid-turn: stop turning where the arm is and go.
                self.qr_preparation.interrupt()
                self._accept_side_selection(position,now)
            elif position is None and self._view_scan_expired(now):
                raise RuntimeError('QR_VIEW_NOT_FOUND: QR 시야 확보 실패')
            elif self.qr_preparation.tick(now)=='READY':
                if self.view_scan_started is None:
                    self.view_scan_started = now
                self.view_scan_next = now+(.55 if self.data['state']=='QR_TURNING' else .30)
                self._stop()
                self.data.update(arm_motion_enabled=False,automatic_motion_enabled=False,
                                 qr_view_ready=True,pending_stages=['pick_place_qr','arm_transfer'])
                self._state('QR_VIEW_READY' if self.request_side_qr is None else 'SIDE_QR_WAIT',now)
                if self.request_side_qr is not None and self.settle_side_qr is not None:
                    self.settle_side_qr(now)
        elif self.data['state']=='SIDE_QR_WAIT':
            position = self._side_selection()
            if position is not None:
                self._accept_side_selection(position,now)
            elif self.view_scan_enabled:
                if self._view_scan_expired(now):
                    raise RuntimeError('QR_VIEW_NOT_FOUND: QR 시야 확보 실패')
                if now-self.view_scan_started >= self.cfg.qr_scan_timeout_seconds:
                    return  # Keep the final view and scan epoch while decoding.
                if now >= self.view_scan_next:
                    pitch = self.qr_preparation.next_pitch_target(self.qr_snapshot().get('detections',[]))
                    if pitch is None and self.qr_preparation.next_scan_target() is None:
                        return
                    # A decoded result may arrive during this check. Never erase
                    # a confirmed selection to move the camera again.
                    if self.request_side_qr(self.data['first_direction'],now):
                        if pitch is not None:
                            self.qr_preparation.scan_pitch(now)
                        else:
                            self.qr_preparation.scan(now)
                        self.data.update(arm_motion_enabled=True,automatic_motion_enabled=True)
                        self._state('QR_SCAN_MOVING',now)
        elif self.data['state']=='ARM_TRANSFER':
            if self.arm.tick(now)=='COMPLETE':
                self.data.update(arm_motion_enabled=False,automatic_motion_enabled=False,pending_stages=[])
                self._state('COMPLETE',now)

    def _view_scan_expired(self, now):
        return self.view_scan_started is not None and now-self.view_scan_started >= self.cfg.qr_scan_timeout_seconds+QR_DECODE_GRACE_SECONDS

    def _accept_side_selection(self, position, now):
        reason = self._transfer_blocked_reason()
        if reason is not None:
            self.data.update(sequence_selection=sequence_for(position),
                             pending_stages=['arm_transfer'],arm_blocked_reason=reason)
            self._state('SEQUENCE_READY',now)
        else:
            self._start_transfer(position,now)

    def _side_selection(self):
        qr = self.qr_snapshot()
        if qr.get('state')=='ERROR':
            raise SensorFault('QR_CAMERA_ERROR: '+str(qr.get('error')))
        position = (qr.get('sequence_selection') or {}).get('start_position')
        if position in START_POSITIONS and position.startswith(self.data['first_direction']+'-'):
            return position
        return None

    def _start_transfer(self, position, now):
        # The selection is fixed from here: QR codes hidden by the moving arm
        # can no longer change the plan.
        self.data['sequence_selection'] = sequence_for(position)
        self.arm.start(self.data['first_direction'],position,now)
        self.data.update(arm_motion_enabled=True,automatic_motion_enabled=True,
                         arm_blocked_reason=None,qr_view_ready=True,pending_stages=['arm_transfer'])
        self._state('ARM_TRANSFER',now)

    def _transfer_blocked_reason(self):
        if not self.cfg.arm_enabled:
            return 'DISABLED'
        if self.arm is None or not self.arm.profile.calibrated:
            return self.transfer_blocked_reason or 'ARM_TRANSFER_UNAVAILABLE'
        return None

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
            if state=='ARM_RESET':
                if self.arm_reset.tick(now)=='READY':
                    self.data['arm_motion_enabled'] = False
                    if self.on_reset_complete is not None and not self.on_reset_complete():
                        raise RuntimeError('START_QR_SCAN_NOT_STARTED')
                    self._state('WAIT_QR',now)
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
        """Fix the 7 V-referenced PWM; the battery reading is kept for the status."""
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
        # Move.py treats 0..100 as percent of 7 V and compensates the battery.
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
            if self.data['reverse_pulse_count']:
                raise SensorFault('REVERSE_DISTANCE_JUMP: refusing to resume cruise after reverse correction')
            # Still far from the target: drive there instead of spending pulses.
            self.zone_values.clear()
            emit('pulse_zone_exit',distance_cm=round(distance,2))
            self._state('DRIVING',now)
            return
        error = distance-self.cfg.target_distance_cm
        if abs(error) <= self.cfg.stop_tolerance_cm:
            self._stop_at_target(now,distance)
            return
        if error < 0 and not self.cfg.reverse_correction_enabled:
            raise SensorFault('TARGET_OVERSHOOT: automatic reverse correction is disabled')
        if self.data['pulse_count'] >= self.cfg.max_pulses:
            raise SensorFault('PULSE_LIMIT_REACHED')
        self._send_pulse(1 if error > 0 else -1,abs(error),distance,now)

    def _learn_pulse(self, distance):
        if self.pulse_from_cm is None:
            return
        moved = (self.pulse_from_cm-distance)*self.pulse_direction
        if self.pulse_direction < 0:
            if moved < .3 and abs(distance-self.cfg.target_distance_cm) > self.cfg.stop_tolerance_cm:
                raise SensorFault('REVERSE_NO_PROGRESS: stopped distance did not increase after reverse pulse')
            if moved > self.cfg.max_closing_speed_cm_s*self.pulse_duration+self.cfg.distance_jump_slack_cm:
                raise SensorFault('REVERSE_DISTANCE_JUMP: implausible distance change after reverse pulse')
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
        if direction < 0:
            if self.data['reverse_pulse_count'] >= MAX_REVERSE_PULSES:
                raise SensorFault('REVERSE_PULSE_LIMIT_REACHED')
            seconds = min(seconds,MAX_REVERSE_SECONDS)
        timed_pulse = getattr(self.drive,'pulse',None)
        if timed_pulse is not None:
            timed_pulse(direction,self.pulse_pwm,seconds)
        else:
            (self.drive.forward if direction > 0 else self.drive.backward)(self.pulse_pwm)
        self.data['command_speed'] = self.pulse_pwm
        self.pulse_direction, self.pulse_duration, self.pulse_from_cm = direction, seconds, distance
        self.pulse_until = now+seconds
        self.data.update(pulse_count=self.data['pulse_count']+1,last_pulse_seconds=round(seconds,3))
        if direction < 0:
            self.data['reverse_pulse_count'] += 1
        emit('pulse_command',action='forward' if direction > 0 else 'backward',
             pwm=self.pulse_pwm,seconds=round(seconds,3),distance_cm=round(distance,2))
        self._state('PULSING',now)

    def _wait_seconds(self):
        # End a pulse on time instead of up to one control period late.
        if self.data['state'] in TERMINAL:
            return self.cfg.control_period_seconds
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
