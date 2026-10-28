"""QR -> verified stop -> optional calibrated arm transfers; wheels stay locked."""
from collections import deque
from statistics import median
import threading
import time
from .approach import Approach
from .distance import DistanceFilter, SensorFault
from .qr_scan import emit


TERMINAL = frozenset({'STOPPED_WAIT','COMPLETE','ERROR','CANCELLED','DISABLED'})


class MissionController:
    def __init__(self, config, drive, read_sample, qr_snapshot, clock=time.monotonic, arm_sequence=None, _precision=True):
        self.cfg,self.drive,self.read_sample,self.qr_snapshot = config,drive,read_sample,qr_snapshot
        self.clock = clock
        self.arm = arm_sequence
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.worker = None
        self.filter = DistanceFilter(config) if _precision else None
        self.policy = Approach(config) if _precision else None
        self.started_at = None
        self.stop_at = None
        self.progress_at = None
        self.progress_distance = None
        self.verify_values = deque(maxlen=config.stop_samples) if _precision else None
        self.stable_since = None
        self.state_at = clock()
        self.data = dict(state='WAIT_QR' if config.auto_approach_enabled else 'DISABLED',
                         first_direction=None, target_distance_cm=config.target_distance_cm,
                         distance_offset_cm=config.distance_offset_cm, command_speed=0,
                         distance_cm=None, raw_distance_cm=None, error=None,
                         distance_wait_reason=None, distance_samples=0,
                         automatic_motion_enabled=config.auto_approach_enabled,
                         approach_complete=False, drive_locked=True, arm_motion_enabled=False,
                         implemented_stages=['qr_recognition','approach_30cm'],
                         pending_stages=['arm_calibration'],
                         arm_requested=config.arm_enabled,
                         arm_blocked_reason='DISABLED' if not config.arm_enabled else 'CALIBRATION_REQUIRED')
        if self.arm is not None:
            self.data['implemented_stages'].append('arm_transfer_sequence')

    def snapshot(self):
        with self.lock:
            result = dict(self.data)
            result['arm'] = None if self.arm is None else self.arm.snapshot()
            return result

    def _state(self, state, now):
        self.data['state'] = state
        self.state_at = now
        emit('mission_state', state=state, first_direction=self.data['first_direction'])

    def _stop(self):
        self.drive.stop()
        self.data['command_speed'] = 0

    def cancel(self, reason='USER_STOP'):
        with self.lock:
            self._stop()
            if self.arm is not None:
                self.arm.cancel()
            if self.data['state'] not in TERMINAL:
                self.data.update(error=reason, automatic_motion_enabled=False,arm_motion_enabled=False)
                self._state('CANCELLED',self.clock())

    def fail(self, reason):
        with self.lock:
            try:
                self._stop()
                if self.arm is not None and self.arm.state=='MOVING':
                    self.arm.cancel()
            finally:
                self.data.update(error=reason, automatic_motion_enabled=False,
                                 drive_locked=True,arm_motion_enabled=False)
                self._state('ERROR',self.clock())

    def step(self, now=None):
        try:
            self._step(now)
        except Exception as exc:
            self.fail(str(exc))

    def _step(self, now=None):
        now = self.clock() if now is None else now
        with self.lock:
            state = self.data['state']
            if state in TERMINAL or self.stop_event.is_set():
                return
            qr = self.qr_snapshot()
            if state=='WAIT_QR' and qr['state']=='ERROR':
                raise SensorFault('QR_CAMERA_ERROR: '+str(qr.get('error')))
            # Once the direction is latched, this fixed-pose mission navigates
            # by ultrasound. A video interruption must not cancel that motion.
            if state=='ARM_TRANSFER':
                # The head/sensor may rotate with the arm: do not use side
                # echoes as monitor distance. All wheel commands remain locked.
                if self.data['command_speed']!=0:
                    raise RuntimeError('ARM_REQUIRES_STOPPED_DRIVE')
                if self.arm.tick(now)=='COMPLETE':
                    self._stop()
                    self.data.update(arm_motion_enabled=False,automatic_motion_enabled=False,
                                     arm_blocked_reason=None,pending_stages=[])
                    self._state('COMPLETE',now)
                return
            if state=='WAIT_QR':
                direction = qr.get('first_direction')
                if direction not in ('LEFT','RIGHT'):
                    return
                if not qr.get('camera_fresh',False):
                    raise SensorFault('STALE_QR_CAMERA')
                self.data['first_direction'] = direction
                self._stop()
                self.drive.straight()
                self._state('CHECK_DISTANCE',now)
                state = 'CHECK_DISTANCE'
            if state=='CHECK_DISTANCE' and now-self.state_at > self.cfg.sensor_warmup_seconds:
                raise SensorFault('SENSOR_WARMUP_TIMEOUT')
            sample = self.read_sample()  # Non-blocking snapshot, not a new GPIO ping.
            try:
                reading = self.filter.update(sample,max(now,self.clock()))
            except SensorFault as exc:
                if state!='CHECK_DISTANCE':
                    raise  # Never drive on failed/stale echoes.
                # Before moving, allow the sensor to recover within its startup
                # budget. Only distinct valid echoes may fill the filter again.
                reason = str(exc)
                if self.data['distance_wait_reason']!=reason:
                    emit('waiting_for_distance',reason=reason)
                self.filter = DistanceFilter(self.cfg)
                self.data.update(distance_wait_reason=reason,distance_samples=0,
                                 distance_cm=None,raw_distance_cm=None)
                return
            self.data.update(distance_cm=round(reading.filtered_cm,3),
                             raw_distance_cm=round(reading.raw_cm,3),
                             distance_wait_reason=None,distance_samples=len(self.filter.values))
            if reading.raw_cm < self.cfg.target_distance_cm-self.cfg.stop_tolerance_cm:
                raise SensorFault('PASSED_STOP_BAND')
            if state=='CHECK_DISTANCE':
                if not reading.ready or now-self.state_at < self.cfg.steering_settle_seconds:
                    return
                self.started_at = now
                self.progress_at,self.progress_distance = now,reading.filtered_cm
                self._state('APPROACH',now)
                state = 'APPROACH'
            if state in ('APPROACH','SLOW_APPROACH'):
                if now-self.started_at > self.cfg.approach_timeout_seconds:
                    raise SensorFault('APPROACH_TIMEOUT')
                speed = self.policy.command(reading)
                if speed==0:
                    self._stop()
                    self.stop_at = now
                    self.verify_values.clear()
                    self._state('VERIFY_STOP',now)
                    return
                if reading.filtered_cm <= self.progress_distance-self.cfg.progress_min_cm:
                    self.progress_at,self.progress_distance = now,reading.filtered_cm
                elif now-self.progress_at > self.cfg.progress_timeout_seconds:
                    raise SensorFault('NO_FORWARD_PROGRESS')
                if self.policy.slow_latched and state!='SLOW_APPROACH':
                    self._state('SLOW_APPROACH',now)
                if speed != self.data['command_speed']:
                    self.drive.forward(speed)
                    self.data['command_speed'] = speed
            elif state=='VERIFY_STOP':
                # Require distinct post-stop echoes, elapsed settling time, a
                # stable distance window and final position within tolerance.
                if reading.is_new and reading.captured_at >= self.stop_at:
                    self.verify_values.append(reading.raw_cm)
                if now-self.stop_at > self.cfg.stop_timeout_seconds:
                    raise SensorFault('STOP_NOT_CONFIRMED')
                stable = (len(self.verify_values)==self.cfg.stop_samples and
                          max(self.verify_values)-min(self.verify_values) <= self.cfg.stop_stability_cm and
                          abs(median(self.verify_values)-self.cfg.target_distance_cm) <= self.cfg.stop_tolerance_cm)
                if reading.is_new:
                    if not stable:
                        self.stable_since = None
                    elif self.stable_since is None:
                        self.stable_since = now
                if (reading.is_new and stable and self.stable_since is not None and
                    now-self.stable_since >= self.cfg.stop_hold_seconds):
                    self._stop()
                    self.data.update(approach_complete=True, automatic_motion_enabled=False,drive_locked=True)
                    if self.cfg.arm_enabled and self.arm is not None and self.arm.profile.calibrated:
                        self.arm.start(self.data['first_direction'],now)
                        self.data.update(arm_motion_enabled=True,automatic_motion_enabled=True,
                                         arm_blocked_reason=None)
                        self._state('ARM_TRANSFER',now)
                    else:
                        self.data['arm_blocked_reason'] = 'DISABLED' if not self.cfg.arm_enabled else 'CALIBRATION_REQUIRED'
                        self._state('STOPPED_WAIT',now)

    def start(self):
        if not self.cfg.auto_approach_enabled or self.worker is not None:
            return
        self.drive.stop()
        self.worker = threading.Thread(target=self._run,name='mission-drive',daemon=True)
        self.worker.start()

    def _run(self):
        try:
            while not self.stop_event.is_set():
                self.step()
                self.stop_event.wait(self.cfg.control_period_seconds)
        except Exception as exc:
            self.fail(str(exc))
        finally:
            with self.lock:
                self._stop()

    def stop(self):
        self.stop_event.set()
        self.cancel('SERVER_SHUTDOWN')
        if self.worker and self.worker is not threading.current_thread():
            self.worker.join(timeout=2.)
