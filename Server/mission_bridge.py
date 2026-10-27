"""Bridge between existing Server scripts and the repository's QR package."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from picar_mission.config import load_config
from picar_mission.runtime import MissionRuntime
from picar_mission.mission_controller import MissionController
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


def automatic_enabled():
    return config.auto_approach_enabled


def configure(move, steering, ultrasound):
    global controller
    with _lock:
        if controller is not None:
            return
        controller = MissionController(config,DriveAdapter(move,steering),
                                       ultrasound.read_sample,runtime.snapshot)
        if _cancelled:
            controller.cancel()
        else:
            controller.start()


def status():
    qr = runtime.snapshot()
    if controller is None:
        return {**qr,'state':'INITIALIZING' if config.auto_approach_enabled else qr['state'],
                'control_ready':False,'qr':qr}
    if not config.auto_approach_enabled:
        return {**qr,'control_ready':True,'qr':qr,'approach':controller.snapshot()}
    return {**controller.snapshot(),'control_ready':True,'qr':qr}


STOP_COMMANDS = frozenset({'stop','DS','TS','LRstop','armstop','HAstop',
                           'stopCV','automaticOff','trackLineOff','steadyCameraOff',
                           'keepDistanceOff'})


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
