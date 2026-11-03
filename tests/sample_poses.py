"""Keep tests off the robot's taught arm files committed in the repository.

The default arm_poses.json becomes the sample profile the tests assert on,
and arm_positions.json starts untaught in a temporary folder, so no test
reads the real positions or saves over them.
"""
import atexit
from pathlib import Path
import tempfile
from picar_mission import arm_positions, arm_sequence

SAMPLE_POSES = Path(__file__).resolve().parent/'fixtures'/'sample_arm_poses.json'
arm_sequence.POSES_FILE = SAMPLE_POSES
_positions = tempfile.TemporaryDirectory()
atexit.register(_positions.cleanup)
arm_positions.POSITIONS_FILE = Path(_positions.name)/'arm_positions.json'
