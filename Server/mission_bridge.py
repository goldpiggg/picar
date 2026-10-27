"""Bridge between existing Server scripts and the repository's QR package."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from picar_mission.config import load_config
from picar_mission.runtime import MissionRuntime

config = load_config()
runtime = MissionRuntime(config)


def submit_frame(frame):
    runtime.submit_frame(frame)
