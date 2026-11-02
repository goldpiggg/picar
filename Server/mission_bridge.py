"""Bridge between existing Server scripts and the repository's QR package."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from picar_mission.config import load_config
from picar_mission.runtime import MissionRuntime
from picar_mission.direct_mission_controller import DirectMissionController
from picar_mission.arm_sequence import MAX_SERVO_ANGLE, ArmSequence, PoseMove, load_profile, pose_targets, save_pose as save_arm_pose
from picar_mission.arm_positions import PositionStore
from picar_mission.recorder import ARM_COMMANDS, DemoRecorder
from picar_mission.heading import GyroHeading
from picar_mission.qr_scan import emit
from threading import Event, RLock, Thread
import time
import math
import uuid
from picar_mission.mission_controller import TERMINAL

config = load_config()
runtime = MissionRuntime(config)
controller = None
_lock = RLock()
_cancelled = False
_servo = None
_ultrasound = None
gyro = None
_pose_move = None
_move = None
_mission_started = False
_mission_started_at = None
_camera_seen_at = None
_camera_error = None
_camera_generation = 0
_attempt = 0
_control_revision = 0
_position_teaching = False
SESSION_ID = uuid.uuid4().hex


def _make_controller():
    arm = ArmSequence(ArmAdapter(_servo),load_profile()) if config.arm_enabled else None
    return DirectMissionController(config,DriveAdapter(_move,_servo),
        _ultrasound.read_sample,runtime.snapshot,arm_sequence=arm,
        read_heading=None if gyro is None else gyro.read_sample,
        read_battery=_read_battery_volts)


class DriveAdapter:
    """Reuse already initialized official objects; channel 0 is steering only."""
    def __init__(self, move, steering):
        self.move,self.steering = move,steering
        self.locked = False

    def stop(self):
        self.move.motorStop()

    def straight(self):
        if self.locked:
            raise RuntimeError('DRIVE_LOCKED_AFTER_STOP')
        self.steering.moveAngle(0,0)

    def steer(self, angle):
        # Relative to the servo center; positive is the manufacturer's "right".
        if self.locked:
            raise RuntimeError('DRIVE_LOCKED_AFTER_STOP')
        self.steering.moveAngle(0,angle)

    def forward(self, speed):
        if self.locked:
            raise RuntimeError('DRIVE_LOCKED_AFTER_STOP')
        self.move.move(speed,1,'mid')

    def backward(self, speed):
        if self.locked:
            raise RuntimeError('DRIVE_LOCKED_AFTER_STOP')
        self.move.move(speed,-1,'mid')

    def lock(self):
        self.locked = True
        self.stop()


class ArmAdapter:
    """Absolute servo degrees. Steering channel 0 is never accepted here."""
    channels = {'base':1,'shoulder':2,'elbow':3,'gripper':4}

    def __init__(self, servo):
        self.servo = servo

    def read_angles(self):
        # RPIservo.moveWiggle stores nowPos before clamping, so holding a manual
        # button past the end reads 190/-1 although the servo never got it.
        return {name:min(float(MAX_SERVO_ANGLE),max(0.,float(self.servo.nowPos[channel])))
                for name,channel in self.channels.items()}

    def set_joint(self, name, angle):
        self.servo.setPWM(self.channels[name],angle)

    def hold(self):
        # Pause interpolation/wiggle; maintain PWM so a held object is not dropped.
        self.servo.stopWiggle()


def _read_arm_angles():
    return ArmAdapter(_servo).read_angles() if _servo is not None else None


def _read_distance():
    return _ultrasound.read_sample() if _ultrasound is not None else None


# Demonstration CSVs for arm learning; reads commanded angles/sensors only.
recorder = DemoRecorder(ROOT / 'data' / 'demos', _read_arm_angles, _read_distance)


def _read_battery_volts():
    # WebServer already runs Voltage.BatteryLevelMonitor; reuse its filtered
    # average instead of opening the ADC a second time. 0.0 means not ready.
    volts = getattr(sys.modules.get('Voltage'),'average_voltage',0.)
    return volts if isinstance(volts,(int,float)) and volts > 0 else None


def automatic_enabled():
    return config.auto_approach_enabled


def _start_gyro():
    """MPU6050 yaw for heading hold; drive without correction if it is missing."""
    try:
        from mpu6050 import mpu6050
        sensor = mpu6050(0x68)
        heading = GyroHeading(lambda: sensor.get_gyro_data()['z'], z_sign=config.gyro_z_sign,
                              calibration_seconds=config.gyro_calibration_seconds)
    except Exception as exc:
        emit('gyro_unavailable', message=str(exc))
        return None
    heading.start()
    return heading


def configure(move, steering, ultrasound):
    global controller, _servo, _ultrasound, gyro, _move, _mission_started, _mission_started_at
    with _lock:
        if controller is not None:
            return
        move.motorStop()
        _servo, _ultrasound = steering, ultrasound
        _move = move
        if config.auto_approach_enabled and config.heading_correction_enabled:
            gyro = _start_gyro()
        controller = _make_controller()
        if _cancelled:
            cancel()
        elif not config.web_start_required and config.auto_approach_enabled:
            _mission_started = True
            _mission_started_at = time.monotonic()
            controller.start()


def start_mission(expected_revision=None):
    """Start a fresh mission using initialized drivers, never a shell command.

    The lifecycle lock makes concurrent starts idempotent and serializes stop
    against replacement. Retire every old worker before clearing its QR latch.
    """
    global controller, runtime, _cancelled, _mission_started, _mission_started_at, _attempt, _control_revision
    with _lock:
        if expected_revision is not None and expected_revision != _control_revision:
            raise ValueError('제어 상태가 바뀌었습니다. 현재 상태를 확인하고 다시 시작하세요')
        if controller is None:
            raise ValueError('서버가 아직 준비되지 않았습니다')
        if not config.auto_approach_enabled:
            raise ValueError('수동 모드입니다. 자동 미션 설정을 켜고 서버를 재시작하세요')
        if _position_teaching:
            raise ValueError('팔 수동 조절을 종료한 뒤 미션을 시작하세요')
        if _mission_started and controller.snapshot()['state'] not in TERMINAL:
            return status()  # Double click or retry must not start a second run.
        if _camera_seen_at is None or time.monotonic()-_camera_seen_at > config.max_frame_age_seconds:
            raise ValueError('카메라 영상이 연결된 뒤 시작할 수 있습니다')
        controller.stop()
        if controller.worker is not None and controller.worker.is_alive():
            raise RuntimeError('이전 주행 작업을 중지하지 못했습니다')
        runtime.stop()
        if runtime.worker is not None and runtime.worker.is_alive():
            raise RuntimeError('이전 QR 작업을 중지하지 못했습니다')
        stop_pose_move()
        runtime = MissionRuntime(config)
        # Keep a stopped controller installed if profile validation fails.
        replacement = _make_controller()
        _cancelled = False
        _mission_started = True
        _mission_started_at = time.monotonic()
        _attempt += 1
        _control_revision += 1
        controller = replacement
        runtime.start()
        controller.start()
        emit('web_mission_start',attempt=_attempt)
        return status()


def status():
    with _lock:
        result = _status()
        result['dashboard'] = _dashboard_status(result)
        result['state'] = result['dashboard']['state']
        if result['state'] in ('READY','ARM_TEACHING'):
            result['automatic_motion_enabled'] = False
        return result


def _dashboard_status(result):
    now = time.monotonic()
    state = result.get('state')
    if not config.auto_approach_enabled:
        state = 'DISABLED'
    ready = controller is not None
    if ready and config.auto_approach_enabled and not _mission_started and not _cancelled:
        state = 'READY'
    if _position_teaching:
        state = 'ARM_TEACHING'
    finished = _mission_started and result.get('state') in TERMINAL
    end = max(_mission_started_at or now,controller.state_at) if finished else now
    elapsed = None if _mission_started_at is None else round(max(0.,end-_mission_started_at),2)
    distance, age, source = None, None, None
    if result.get('target_stop_latched'):
        distance, source = result.get('final_distance_cm'), 'FINAL'
    else:
        try:
            sample = _read_distance()
            value, stamp = sample['distance_cm'], sample['captured_at']
            now = max(now,time.monotonic())
            if all(type(v) in (int,float) and math.isfinite(v) for v in (value,stamp)):
                age = max(0.,now-stamp)
                if 2 <= value < 199 and stamp <= now and age <= config.direct_distance_max_age_seconds:
                    distance, source = round(value+config.distance_offset_cm,1), 'LIVE'
        except (KeyError,TypeError,ValueError,OSError):
            pass
    camera_online = _camera_seen_at is not None and now-_camera_seen_at <= config.max_frame_age_seconds
    return dict(state=state,attempt=_attempt,started=_mission_started,finished=finished,
        can_start=ready and camera_online and config.auto_approach_enabled and not _position_teaching and (not _mission_started or finished),
        elapsed_seconds=elapsed,distance_cm=distance,distance_age_seconds=age,distance_source=source,
        remaining_cm=0. if result.get('target_stop_latched') else None if distance is None else round(max(0.,distance-config.target_distance_cm),1),
        target_distance_cm=config.target_distance_cm,
        camera_online=camera_online,
        camera_generation=_camera_generation,
        camera_error=_camera_error,
        arm_enabled=config.arm_enabled,
        control_revision=_control_revision,
        session_id=SESSION_ID,
        arm_calibrated=controller is not None and controller.arm is not None and controller.arm.profile.calibrated)


def _status():
    qr = runtime.snapshot()
    if controller is None:
        return {**qr,'state':'INITIALIZING' if config.auto_approach_enabled else qr['state'],
                'control_ready':False,'qr':qr}
    # Memory-only readback of commanded servo angles, useful when teaching poses
    # with the existing manual web controls. This is not physical position feedback.
    try:
        angles = ArmAdapter(_servo).read_angles() if _servo is not None else None
    except (AttributeError,TypeError,IndexError):
        angles = None
    if not config.auto_approach_enabled:
        return {**qr,'control_ready':True,'qr':qr,'approach':controller.snapshot(),
                'commanded_servo_angles_deg':angles}
    return {**controller.snapshot(),'control_ready':True,'qr':qr,
            'commanded_servo_angles_deg':angles,
            'gyro':None if gyro is None else gyro.snapshot()}


def save_pose(name):
    """Teach mode only: store the current commanded arm angles in arm_poses.json."""
    if config.auto_approach_enabled:
        raise ValueError('auto_approach_enabled=false 수동 모드에서만 저장할 수 있습니다')
    try:
        angles = ArmAdapter(_servo).read_angles() if _servo is not None else None
    except (AttributeError,TypeError,IndexError):
        angles = None
    if angles is None:
        raise ValueError('서보 명령각을 아직 읽을 수 없습니다')
    return save_arm_pose(name,angles)


def move_to_pose(name):
    """Teach mode only: move the arm to a saved pose at the profile speed."""
    if config.auto_approach_enabled:
        raise ValueError('auto_approach_enabled=false 수동 모드에서만 이동할 수 있습니다')
    if _servo is None:
        raise ValueError('서보가 아직 준비되지 않았습니다')
    profile = load_profile()
    targets = pose_targets(profile,name)
    return _move_arm_targets(name, targets, profile)


def _move_arm_targets(name, targets, profile, speed=None):
    global _pose_move
    stop_pose_move()
    if _pose_move is not None and _pose_move['thread'].is_alive():
        raise RuntimeError('이전 팔 이동이 멈추지 않았습니다')
    driver = ArmAdapter(_servo)
    driver.hold()
    speed = profile.move_speed(profile.holding(driver.read_angles()['gripper'])) if speed is None else speed
    move = PoseMove(driver,targets,speed)
    stop = Event()
    info = dict(name=name,state='MOVING',targets=targets,error=None,speed_deg_per_second=speed,
                seconds=round(move.motion.duration,1))

    def run():
        try:
            while not stop.is_set():
                if move.tick():
                    info['state'] = 'DONE'
                    return
                time.sleep(.02)
            info['state'] = 'STOPPED'
        except Exception as exc:
            info.update(state='ERROR',error=str(exc))

    thread = Thread(target=run,daemon=True)
    _pose_move = dict(info=info,stop=stop,thread=thread)
    thread.start()
    return pose_move_status()


def _position_store():
    return PositionStore(ROOT/'arm_positions.json', limits=load_profile().limits)


def position_data():
    with _lock:
        return dict(profile=_position_store().read(), manual=_position_teaching or not config.auto_approach_enabled,
            angles=_read_arm_angles(), move=pose_move_status(), preview=False,
            legacy_available=True, control_revision=_control_revision, session_id=SESSION_ID,
            teaching_active=_position_teaching, can_switch_teaching=config.auto_approach_enabled and controller is not None,
            joint_limits=load_profile().limits, supports_joint_control=True)


def _require_position_teaching():
    if config.auto_approach_enabled and not _position_teaching:
        raise ValueError('수동 저장 모드가 필요합니다. 팔 수동 조절 시작을 누르세요')
    if _servo is None:
        raise ValueError('서보가 아직 준비되지 않았습니다')


def save_position(position, pose):
    with _lock:
        _require_position_teaching()
        if _pose_move is not None and _pose_move['thread'].is_alive():
            raise ValueError('팔이 도착하거나 정지한 뒤 저장하세요')
        return _position_store().save(position, pose, _read_arm_angles())


def move_to_position(position, pose, expected_revision=None, session_id=None):
    global _control_revision
    with _lock:
        _require_position_teaching()
        if type(expected_revision) is not int or expected_revision != _control_revision or session_id != SESSION_ID:
            raise ValueError('제어 상태가 바뀌었습니다. 현재 상태를 확인하고 다시 이동하세요')
        targets = _position_store().targets(position, pose)
        _control_revision += 1
        return _move_arm_targets(position+' / '+pose, targets, load_profile())


def stop_position_move():
    global _control_revision
    with _lock:
        _control_revision += 1
        stop_pose_move()
        if _servo is not None:
            ArmAdapter(_servo).hold()
        return pose_move_status()


def _check_position_revision(expected_revision, session_id):
    if type(expected_revision) is not int or expected_revision != _control_revision or session_id != SESSION_ID:
        raise ValueError('제어 상태가 바뀌었습니다. 현재 상태를 확인하고 다시 조절하세요')


def set_position_teaching(enabled, expected_revision=None, session_id=None):
    """Temporarily reserve control for the arm page without changing settings."""
    global _position_teaching, _control_revision, _mission_started, _mission_started_at, _cancelled, runtime, controller
    with _lock:
        _check_position_revision(expected_revision, session_id)
        if type(enabled) is not bool:
            raise ValueError('팔 수동 조절 모드를 선택하세요')
        if controller is None or _servo is None:
            raise ValueError('로봇이 아직 준비되지 않았습니다')
        _control_revision += 1
        controller.stop()
        if controller.worker is not None and controller.worker.is_alive():
            raise RuntimeError('이전 주행 작업이 멈추지 않았습니다')
        runtime.stop()
        if runtime.worker is not None and runtime.worker.is_alive():
            raise RuntimeError('이전 QR 작업이 멈추지 않았습니다')
        stop_pose_move()
        if _pose_move is not None and _pose_move['thread'].is_alive():
            raise RuntimeError('이전 팔 이동이 멈추지 않았습니다')
        _move.motorStop()
        ArmAdapter(_servo).hold()
        if not enabled:
            # Prepare only: never resume driving on leaving the teaching page.
            replacement_runtime = MissionRuntime(config)
            previous_runtime = runtime
            runtime = replacement_runtime
            try:
                replacement = _make_controller()
            except Exception:
                runtime = previous_runtime
                raise
            controller = replacement
        _position_teaching = enabled
        _mission_started = False
        _mission_started_at = None
        _cancelled = False
        return position_data()


def set_joint_angle(joint, angle, expected_revision=None, session_id=None):
    """Rate-limited single-joint targets; does not save a taught pose."""
    global _control_revision
    with _lock:
        _require_position_teaching()
        _check_position_revision(expected_revision, session_id)
        profile = load_profile()
        if joint not in ('base','shoulder','elbow','gripper'):
            raise ValueError('팔 관절 이름을 확인하세요')
        low, high = profile.limits[joint]
        if type(angle) not in (int,float) or not math.isfinite(angle) or not low <= angle <= high:
            raise ValueError('허용 각도 범위를 확인하세요')
        holding = profile.holding(_read_arm_angles()['gripper'])
        speed = min(config.arm_jog_speed, profile.move_speed(holding))
        _control_revision += 1
        _move_arm_targets('JOINT / '+joint, {joint:float(angle)}, profile, speed=speed)
        return position_data()


def rehearse(direction):
    """Teach mode only: run the automatic arm plan in place, without driving."""
    global _pose_move
    if config.auto_approach_enabled:
        raise ValueError('auto_approach_enabled=false 수동 모드에서만 미리 실행할 수 있습니다')
    if _servo is None:
        raise ValueError('서보가 아직 준비되지 않았습니다')
    if direction not in ('LEFT','RIGHT'):
        raise ValueError('direction must be LEFT or RIGHT')
    stop_pose_move()
    sequence = ArmSequence(ArmAdapter(_servo),load_profile())
    sequence.start(direction,require_calibration=False)
    stop = Event()
    info = dict(name=direction+' 순서',state='MOVING',error=None,step_index=0,
                step_count=len(sequence.steps),action=None,pose=None)

    def run():
        try:
            while not stop.is_set():
                state = sequence.tick()
                step = sequence.steps[min(sequence.index,len(sequence.steps)-1)]
                info.update(step_index=sequence.index,action=step['action'],pose=step['pose'])
                if state=='COMPLETE':
                    info['state'] = 'DONE'
                    return
                time.sleep(.02)
            sequence.cancel()  # Holds the current pose and grip; no release.
            info['state'] = 'STOPPED'
        except Exception as exc:
            info.update(state='ERROR',error=str(exc))

    thread = Thread(target=run,daemon=True)
    _pose_move = dict(info=info,stop=stop,thread=thread)
    thread.start()
    return pose_move_status()


def stop_pose_move():
    if _pose_move is not None and _pose_move['thread'].is_alive():
        _pose_move['stop'].set()
        _pose_move['thread'].join(1.)


def pose_move_status():
    return None if _pose_move is None else dict(_pose_move['info'])


STOP_COMMANDS = frozenset({'stop','DS','TS','LRstop','armstop','HAstop',
                           'stopCV','automaticOff','trackLineOff','steadyCameraOff',
                           'keepDistanceOff'})
BUTTON_RELEASE_COMMANDS = frozenset({'DS','TS','LRstop','armstop','HAstop'})


def cancel(reason='USER_STOP'):
    global _cancelled, _control_revision
    with _lock:
        _cancelled = True
        _control_revision += 1
        stop_pose_move()
        if controller is not None:
            controller.cancel(reason)
        if _servo is not None and getattr(controller,'arm',None) is None:
            ArmAdapter(_servo).hold()


def allow_command(command, response=None):
    """Reserve all motion sources, including QR wait and stopped wait states."""
    if not config.auto_approach_enabled:
        if command in ARM_COMMANDS or command in STOP_COMMANDS:
            stop_pose_move()  # A manual button always wins over a recall move.
        if command in ARM_COMMANDS:
            recorder.note_command(command)
        elif command in STOP_COMMANDS:
            recorder.note_command(None)
        return True
    if command in STOP_COMMANDS:
        # The old UI sends these on releasing even a blocked manual button.
        # Before QR start, releasing an arrow must not cancel the future mission.
        # Explicit stop/HTTP stop still cancel; all stop commands cancel in motion.
        waiting = controller is None or controller.snapshot()['state']=='WAIT_QR'
        if command not in BUTTON_RELEASE_COMMANDS or not waiting:
            cancel()
    if command == 'get_info':
        return True
    if response is not None:
        response.update(title='missionLocked',data={'state':status()['state'],
                        'message':'Automatic approach owns motor/servo control; stop commands cancel it.'})
    return False


def shutdown():
    stop_pose_move()
    recorder.close()
    if gyro is not None:
        gyro.stop()
    if controller is not None:
        controller.stop()
    if _servo is not None and getattr(controller,'arm',None) is None:
        ArmAdapter(_servo).hold()
    runtime.stop()


def camera_capture_started():
    global _camera_seen_at, _camera_error, _camera_generation
    with _lock:
        _camera_generation += 1
        _camera_seen_at = None
        _camera_error = None


def camera_failed(message):
    global _camera_seen_at, _camera_error
    with _lock:
        _camera_seen_at = None
        _camera_error = message
        runtime.fail(message)


def submit_frame(frame):
    global _camera_seen_at, _camera_error
    with _lock:
        _camera_seen_at = time.monotonic()
        _camera_error = None
        if not _position_teaching and (not config.web_start_required or _mission_started):
            runtime.submit_frame(frame)
    recorder.submit_frame(frame)
