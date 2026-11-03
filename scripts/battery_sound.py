#!/usr/bin/env python3
"""Set the battery sound preference before starting the robot web service."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from picar_mission.battery_alarm import BatteryAlarm


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--mute', action='store_true')
    mode.add_argument('--enable', action='store_true')
    args = parser.parse_args()
    BatteryAlarm(ROOT/'data'/'battery_alarm.json').set_sound_enabled(args.enable)
    print('Battery warning sound: '+('enabled' if args.enable else 'muted'))
    print('No sound is played by this command. Restart an already running service to load this preference.')
