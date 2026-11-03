#!/usr/bin/env python3
"""Check the taught /poses positions and print every automatic transfer plan.

Reads arm_positions.json and arm_poses.json only. Never imports servo or
motor drivers and never moves hardware. Exit code 1 when transfer is blocked.
"""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from picar_mission.arm_positions import PositionStore
from picar_mission.arm_sequence import load_motion_profile
from picar_mission.config import load_config
from picar_mission.position_transfer import PositionArmProfile, fast_motion_values
from picar_mission.sequence_selection import START_POSITIONS


def main():
    parser = argparse.ArgumentParser(description='Check taught positions; never moves hardware')
    parser.add_argument('--positions', type=Path, default=ROOT/'arm_positions.json')
    parser.add_argument('--poses', type=Path, help='arm_poses.json for limits and speeds')
    args = parser.parse_args()
    if not args.positions.exists():
        print('자세 파일이 없습니다: %s' % args.positions)
        return 1
    motion = load_motion_profile(args.poses)
    cfg = load_config()
    values = fast_motion_values(motion.values) if cfg.fast_mission_enabled else motion.values
    try:
        profile = PositionArmProfile(values, PositionStore(args.positions, motion.limits).read())
        profile.speed_scale = 4. if cfg.fast_mission_enabled else 1.
    except ValueError as exc:
        print('자동 운반 불가: %s' % exc)
        return 1
    print('집게 열기 %g° / 닫기 %g°' % (profile.gripper['open'], profile.gripper['closed']))
    for start in START_POSITIONS:
        steps = profile.plan(start.split('-')[0], start)
        print('\n[%s 시작] %d단계' % (start, len(steps)))
        for n, step in enumerate(steps, 1):
            angles = step['angles_deg']
            grip = '유지' if angles['gripper'] is None else '%g' % angles['gripper']
            print('  %2d. %-14s %-20s base %6.1f  shoulder %6.1f  elbow %6.1f  gripper %s' % (
                n, step['action'], step['pose'], angles['base'], angles['shoulder'], angles['elbow'], grip))
    enabled = load_config().arm_enabled
    print('\n모든 자세 확인 완료. mission_config.json arm_enabled=%s%s' % (
        str(enabled).lower(), '' if enabled else ' → true로 바꿔야 자동 운반합니다'))
    return 0 if enabled else 1


if __name__ == '__main__':
    sys.exit(main())
