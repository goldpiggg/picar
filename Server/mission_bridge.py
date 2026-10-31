"""Bridge between existing Server scripts and the repository's QR package."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from picar_mission.config import load_config
from picar_mission.runtime import MissionRuntime
from picar_mission.direct_mission_controller import DirectMissionController
from picar_mission.arm_sequence import ArmSequence, load_profile
from picar_mission.recorder import ARM_COMMANDS, DemoRecorder
from picar_mission.heading import GyroHeading
from picar_mission.qr_scan import emit
from threading import RLock

config = load_config()
runtime = MissionRuntime(config)
controller = None
_lock = RLock()
_cancelled = False
_servo = None
_ultrasound = None
gyro = None


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
        return {name:float(self.servo.nowPos[channel]) for name,channel in self.channels.items()}

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
    global controller, _servo, _ultrasound, gyro
    with _lock:
        if controller is not None:
            return
        move.motorStop()
        _servo, _ultrasound = steering, ultrasound
        if config.auto_approach_enabled and config.heading_correction_enabled:
            gyro = _start_gyro()
        arm = ArmSequence(ArmAdapter(steering),load_profile()) if config.arm_enabled else None
        controller = DirectMissionController(config,DriveAdapter(move,steering),
                                             ultrasound.read_sample,runtime.snapshot,arm_sequence=arm,
                                             read_heading=None if gyro is None else gyro.read_sample,
                                             read_battery=_read_battery_volts)
        if _cancelled:
            cancel()
        else:
            controller.start()


def status():
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


STOP_COMMANDS = frozenset({'stop','DS','TS','LRstop','armstop','HAstop',
                           'stopCV','automaticOff','trackLineOff','steadyCameraOff',
                           'keepDistanceOff'})
BUTTON_RELEASE_COMMANDS = frozenset({'DS','TS','LRstop','armstop','HAstop'})


def cancel(reason='USER_STOP'):
    global _cancelled
    with _lock:
        _cancelled = True
        if controller is not None:
            controller.cancel(reason)
        if _servo is not None and getattr(controller,'arm',None) is None:
            ArmAdapter(_servo).hold()


def allow_command(command, response=None):
    """Reserve all motion sources, including QR wait and stopped wait states."""
    if not config.auto_approach_enabled:
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
    recorder.close()
    if gyro is not None:
        gyro.stop()
    if controller is not None:
        controller.stop()
    if _servo is not None and getattr(controller,'arm',None) is None:
        ArmAdapter(_servo).hold()
    runtime.stop()


def submit_frame(frame):
    runtime.submit_frame(frame)
    recorder.submit_frame(frame)
