"""Fast motion preserves the taught closing angle, dwell and physical limits."""
import copy
import unittest
from dataclasses import replace
from picar_mission.arm_sequence import ArmSequence,MAX_ARM_SPEED
from picar_mission.position_transfer import fast_motion_values,PositionArmProfile
from picar_mission.arm_positions import PositionStore
from picar_mission.sequence_selection import START_POSITIONS
from test_arm import FakeArm,load_profile
from test_dashboard import load_bridge
from pathlib import Path

ROOT=Path(__file__).resolve().parent.parent


class FastMissionTests(unittest.TestCase):
    def test_profile_is_a_copy_and_grip_strength_commands_and_dwell_are_preserved(self):
        original=load_profile().values
        before=copy.deepcopy(original)
        fast=fast_motion_values(original)
        self.assertEqual(original,before)
        self.assertEqual(fast['gripper_deg'],original['gripper_deg'])
        self.assertEqual(fast['grip_settle_seconds'],original['grip_settle_seconds'])
        self.assertEqual(fast['carry_speed_deg_per_second'],original['speed_deg_per_second'])
        self.assertLessEqual(fast['settle_seconds'],.05)

    def test_fast_transfer_holds_closed_through_every_loaded_move(self):
        motion=load_profile()
        profile=PositionArmProfile(fast_motion_values(motion.values),
            PositionStore(ROOT/'arm_positions.json',motion.limits).read(),
            carry_clearance={'shoulder':153.,'elbow':156.})
        profile.speed_scale=4.
        self.assertLessEqual(profile.move_speed(True),MAX_ARM_SPEED)
        for start in START_POSITIONS:
            driver=FakeArm()
            sequence=ArmSequence(driver,profile,clock=lambda:0.)
            sequence.start(start.split('-')[0],start,0.)
            self.assertEqual(sequence.steps[-1]['action'],'release')
            for step in sequence.steps:
                if step['action'] in ('grasp','release'):
                    self.assertEqual(step['settle_seconds'],motion.values['grip_settle_seconds'])
                if step['action'] in ('lift','rotate_place','approach_place','align_place','lower_place'):
                    self.assertEqual(step['angles_deg']['gripper'],profile.gripper['closed'])
            for tick in range(1,3000):
                action=sequence.steps[sequence.index]['action']
                base=driver.angles['base']
                state=sequence.tick(tick*.02)
                if action=='lift':self.assertEqual(driver.angles['base'],base)
                if action=='rotate_place':
                    self.assertEqual(driver.angles['shoulder'],153.)
                    self.assertEqual(driver.angles['elbow'],156.)
                    self.assertEqual(driver.angles['gripper'],profile.gripper['closed'])
                if state=='COMPLETE': break
            self.assertEqual(sequence.completed_transfers,2)
            self.assertGreater(driver.holds,0)

    def test_normal_mode_keeps_the_original_profile_and_retreat(self):
        motion=load_profile()
        profile=PositionArmProfile(motion.values,
            PositionStore(ROOT/'arm_positions.json',motion.limits).read())
        for start in START_POSITIONS:
            self.assertEqual(profile.plan(start.split('-')[0],start)[-1]['action'],'retreat')


if __name__=='__main__': unittest.main()
