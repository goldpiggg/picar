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
from threading import RLock

config = load_config()
runtime = MissionRuntime(config)
controller = None
_lock = RLock()
_cancelled = False


class DriveAdapter:
    """Reuse already initialized official objects; channel 0 is steering only."""
    def __init__(self, move, steering):
        self.move,self.steering = move,steering

    def stop(self):
        self.move.motorStop()

    def straight(self):
        self.steering.moveAngle(0,0)

    def forward(self, speed):
        self.move.move(speed,1,'mid')


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


def automatic_enabled():
    return config.auto_approach_enabled


def configure(move, steering, ultrasound):
    global controller
    with _lock:
        if controller is not None:
            return
        move.motorStop()
        arm = ArmSequence(ArmAdapter(steering),load_profile())
        controller = DirectMissionController(config,DriveAdapter(move,steering),
                                             ultrasound.read_sample,runtime.snapshot,arm_sequence=arm)
        if _cancelled:
            controller.cancel()
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
        angles = controller.arm.driver.read_angles()
    except (AttributeError,TypeError,IndexError):
        angles = None
    if not config.auto_approach_enabled:
        return {**qr,'control_ready':True,'qr':qr,'approach':controller.snapshot(),
                'commanded_servo_angles_deg':angles}
    return {**controller.snapshot(),'control_ready':True,'qr':qr,
            'commanded_servo_angles_deg':angles}


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


def allow_command(command, response=None):
    """Reserve all motion sources, including QR wait and stopped wait states."""
    if not config.auto_approach_enabled:
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
    if controller is not None:
        controller.stop()
    runtime.stop()


def submit_frame(frame):
    runtime.submit_frame(frame)
