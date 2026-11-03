"""No hardware: taught-position transfer plan and its hand-off after the QR sequence."""
from dataclasses import replace
from types import SimpleNamespace
import unittest

import test_arm
from picar_mission.arm_positions import POSITIONS, empty_profile
from picar_mission.arm_sequence import ArmSequence
from picar_mission.position_transfer import PositionArmProfile
from picar_mission.sequence_selection import START_POSITIONS, sequence_for

OPEN, CLOSED = 120., 60.


def taught(skip=()):
    data = empty_profile()
    for n, position in enumerate(POSITIONS):
        base = 150.-10*n if position.startswith('LEFT') else 30.+10*n
        for pose, (shoulder, elbow) in dict(approach=(80., 100.), pick=(110., 120.), place=(105., 118.)).items():
            if (position, pose) not in skip:
                data['positions'][position][pose] = dict(saved_at='t',
                    angles_deg=dict(base=base, shoulder=shoulder, elbow=elbow))
    common = dict(gripper_open=dict(gripper=OPEN), gripper_closed=dict(gripper=CLOSED),
                  center_carry=dict(base=90., shoulder=75., elbow=95.))
    for name, angles in common.items():
        if ('COMMON', name) not in skip:
            data['common'][name] = dict(saved_at='t', angles_deg=angles)
    return data


def profile(skip=()):
    return PositionArmProfile(test_arm.load_profile().values, taught(skip))


class PlanTests(unittest.TestCase):
    def test_left_a_follows_the_agreed_four_steps(self):
        steps = [(s['action'], s['pose'], s['gripper']) for s in profile().plan('LEFT', 'LEFT-A')]
        self.assertEqual(steps, [
            ('approach_pick', 'LEFT-A approach', None), ('open_gripper', 'LEFT-A approach', 'open'),
            ('lower_pick', 'LEFT-A pick', 'open'), ('grasp', 'LEFT-A pick', 'closed'),
            ('lift', 'LEFT-A approach', 'closed'),
            ('carry_center', 'COMMON center_carry', 'closed'), ('approach_place', 'RIGHT-A approach', 'closed'),
            ('lower_place', 'RIGHT-A place', 'closed'), ('release', 'RIGHT-A place', 'open'),
            ('retreat', 'RIGHT-A approach', 'open'),
            ('approach_pick', 'RIGHT-B approach', 'open'),
            ('lower_pick', 'RIGHT-B pick', 'open'), ('grasp', 'RIGHT-B pick', 'closed'),
            ('lift', 'RIGHT-B approach', 'closed'),
            ('carry_center', 'COMMON center_carry', 'closed'), ('approach_place', 'LEFT-B approach', 'closed'),
            ('lower_place', 'LEFT-B place', 'closed'), ('release', 'LEFT-B place', 'open'),
            ('retreat', 'LEFT-B approach', 'open')])

    def test_every_start_uses_its_sequence_and_never_swings_low(self):
        for start in START_POSITIONS:
            with self.subTest(start=start):
                steps = profile().plan(start.split('-')[0], start)
                visited = [s['pose'].split()[0] for s in steps if s['action'] in ('grasp', 'release')]
                self.assertEqual(visited, [s['position'] for s in sequence_for(start)['steps']])
                for before, after in zip(steps, steps[1:]):
                    if before['angles_deg']['base'] != after['angles_deg']['base']:
                        # BASE only turns between raised poses.
                        self.assertTrue(before['pose'].endswith(('approach', 'center_carry')), (before, after))
                        self.assertTrue(after['pose'].endswith(('approach', 'center_carry')), (before, after))
                self.assertEqual(steps[-1]['action'], 'retreat')

    def test_taught_gripper_angles_replace_the_legacy_ones(self):
        grips = {s['action']: s['angles_deg']['gripper'] for s in profile().plan('RIGHT', 'RIGHT-B')}
        self.assertEqual((grips['open_gripper'], grips['grasp']), (OPEN, CLOSED))
        self.assertTrue(profile().holding(CLOSED))

    def test_start_must_be_on_the_qr_side(self):
        with self.assertRaises(ValueError):
            profile().plan('LEFT', 'RIGHT-A')

    def test_untaught_slots_are_named(self):
        with self.assertRaisesRegex(ValueError, 'ARM_POSES_INCOMPLETE: RIGHT-B place, COMMON center_carry'):
            profile({('RIGHT-B', 'place'), ('COMMON', 'center_carry')})


class ControllerTransferTests(unittest.TestCase):
    def reach_selection(self, arm_enabled=True, arm=True, blocked=None):
        from test_direct_mission import Harness
        from picar_mission.qr_prepare import QrPreparation
        h = Harness('LEFT')
        h.controller.cfg = replace(h.controller.cfg, arm_enabled=arm_enabled)
        driver = test_arm.FakeArm()
        h.controller.qr_preparation = QrPreparation(driver, test_arm.load_profile(), {'LEFT': 150., 'RIGHT': 30.}, 100.)
        h.controller.request_side_qr = lambda direction, now: True
        h.controller.arm = ArmSequence(driver, profile()) if arm else None
        h.controller.transfer_blocked_reason = blocked
        h.echo(80.); h.near(36.); h.settle(30.)
        for _ in range(200):
            h.echo(None, dt=.1)
            if h.controller.snapshot()['state'] == 'SIDE_QR_WAIT': break
        h.qr = dict(state='DIRECTION_CONFIRMED', first_direction='LEFT',
                    sequence_selection=dict(start_position='LEFT-A'))
        h.time += .05; h.controller.step()
        return h, driver

    def test_confirmed_sequence_runs_both_transfers_with_wheels_locked(self):
        h, driver = self.reach_selection()
        wheels = len(h.drive.commands)
        self.assertEqual(h.controller.snapshot()['state'], 'ARM_TRANSFER')
        # A later, different QR reading must not change the running plan.
        h.qr['sequence_selection'] = dict(start_position='LEFT-B')
        for _ in range(3000):
            h.time += .05; h.controller.step()
            if h.controller.snapshot()['state'] == 'COMPLETE': break
        state = h.controller.snapshot()
        self.assertEqual(state['state'], 'COMPLETE')
        self.assertEqual(state['sequence_selection']['start_position'], 'LEFT-A')
        self.assertEqual(state['arm']['completed_transfers'], 2)
        self.assertFalse(state['arm_motion_enabled'])
        self.assertEqual(driver.angles['gripper'], OPEN)
        self.assertEqual({k: driver.angles[k] for k in ('base', 'shoulder', 'elbow')},
                         taught()['positions']['LEFT-B']['approach']['angles_deg'])
        self.assertTrue(all(c[0] == 'stop' for c in h.drive.commands[wheels:]))

    def test_stop_mid_transfer_holds_pose_and_gripper(self):
        h, driver = self.reach_selection()
        for _ in range(60):
            h.time += .05; h.controller.step()
        angles, writes = dict(driver.angles), len(driver.writes)
        h.controller.cancel()
        for _ in range(50):
            h.time += .05; h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'], 'CANCELLED')
        self.assertEqual(driver.angles, angles)
        self.assertEqual(len(driver.writes), writes)

    def test_disabled_or_untaught_transfer_waits_at_selection(self):
        for kwargs, reason in [(dict(arm_enabled=False), 'DISABLED'),
                               (dict(arm=False, blocked='ARM_POSES_INCOMPLETE: LEFT-A pick'),
                                'ARM_POSES_INCOMPLETE: LEFT-A pick')]:
            with self.subTest(reason=reason):
                h, driver = self.reach_selection(**kwargs)
                writes = len(driver.writes)
                for _ in range(20):
                    h.time += .05; h.controller.step()
                state = h.controller.snapshot()
                self.assertEqual(state['state'], 'SEQUENCE_READY')
                self.assertEqual(state['arm_blocked_reason'], reason)
                self.assertEqual(len(driver.writes), writes)


class BridgeTests(unittest.TestCase):
    def make(self, arm_enabled, positions):
        from test_dashboard import load_bridge
        bridge = load_bridge()
        bridge.config = replace(bridge.config, arm_enabled=arm_enabled)
        bridge._servo = SimpleNamespace(nowPos=[90.]*8, initPos=[90.]*8, stopWiggle=lambda: None,
                                        setPWM=lambda *args: None, moveAngle=lambda *args: None)
        bridge._move = SimpleNamespace(motorStop=lambda: None, move=lambda *args: None)
        bridge._ultrasound = SimpleNamespace(read_sample=lambda: None)
        bridge._position_store = lambda: SimpleNamespace(read=lambda: positions)
        return bridge._make_controller()

    def test_enabled_and_fully_taught_builds_the_transfer(self):
        controller = self.make(True, taught())
        self.assertIsInstance(controller.arm.profile, PositionArmProfile)
        self.assertIsNone(controller._transfer_blocked_reason())

    def test_untaught_positions_do_not_block_the_mission(self):
        controller = self.make(True, empty_profile())
        self.assertIsNone(controller.arm)
        self.assertRegex(controller._transfer_blocked_reason(), '^ARM_POSES_INCOMPLETE: LEFT-A approach')

    def test_disabled_never_reads_positions(self):
        controller = self.make(False, None)
        self.assertIsNone(controller.arm)
        self.assertEqual(controller._transfer_blocked_reason(), 'DISABLED')


if __name__ == '__main__': unittest.main()
