"""No hardware: taught-position transfer plan and its hand-off after the QR sequence."""
import copy
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import test_arm
from picar_mission.arm_positions import POSITIONS, empty_profile
from picar_mission.arm_sequence import ArmSequence
from picar_mission.position_transfer import PositionArmProfile
from picar_mission.sequence_selection import START_POSITIONS, sequence_for

OPEN, CLOSED = 120., 60.
FIXTURES = Path(__file__).resolve().parent/'fixtures'
RAISED = ('approach',)


def robot_profile():
    """The robot's saved files as of 2026-11-03."""
    read = lambda name: json.loads((FIXTURES/name).read_text(encoding='utf-8'))
    return PositionArmProfile(read('robot_arm_poses.json'), read('robot_arm_positions.json'))


def raised(step):
    return step['pose'].endswith(RAISED) or ' above ' in step['pose']


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
    def test_clearance_lift_then_base_only_crossing_for_every_sequence(self):
        data=taught();original=copy.deepcopy(data)
        # Make BASE differ from approach to catch a turn hidden in lowering.
        for position in POSITIONS:
            data['positions'][position]['pick']['angles_deg']['base']+=2
            data['positions'][position]['place']['angles_deg']['base']-=3
        original=copy.deepcopy(data)
        p=PositionArmProfile(test_arm.load_profile().values,data,
            carry_clearance={'shoulder':153.,'elbow':156.})
        for start in START_POSITIONS:
            steps=p.plan(start.split('-')[0],start)
            lifts=[i for i,s in enumerate(steps) if s['action']=='lift']
            self.assertEqual(len(lifts),2)
            for i in lifts:
                grasp,lift,turn,approach=steps[i-1:i+3]
                self.assertEqual(grasp['action'],'grasp')
                self.assertEqual(turn['action'],'rotate_place')
                self.assertEqual(approach['action'],'approach_place')
                self.assertEqual(lift['angles_deg']['base'],grasp['angles_deg']['base'])
                self.assertGreaterEqual(lift['settle_seconds'],.1)
                for joint,value in [('shoulder',153.),('elbow',156.),('gripper',CLOSED)]:
                    self.assertEqual(lift['angles_deg'][joint],value)
                    self.assertEqual(turn['angles_deg'][joint],value)
                lower=steps[i+3]
                self.assertEqual(lower['action'],'lower_place')
                self.assertEqual(turn['angles_deg']['base'],lower['angles_deg']['base'])
                self.assertEqual(approach['angles_deg']['base'],turn['angles_deg']['base'])
        self.assertEqual(data,original)

    def test_clearance_rejects_invalid_joint_targets(self):
        for clearance in ({'base':90,'shoulder':153,'elbow':156},
                          {'shoulder':190,'elbow':156},{'shoulder':153,'elbow':float('nan')}):
            with self.assertRaises(ValueError):
                PositionArmProfile(test_arm.load_profile().values,taught(),carry_clearance=clearance)

    def test_left_a_follows_the_agreed_four_steps(self):
        steps = [(s['action'], s['pose'], s['gripper']) for s in profile().plan('LEFT', 'LEFT-A')]
        self.assertEqual(steps, [
            ('approach_pick', 'LEFT-A approach', 'open'),
            ('lower_pick', 'LEFT-A pick', 'open'), ('grasp', 'LEFT-A pick', 'closed'),
            ('lift', 'LEFT-A approach', 'closed'), ('approach_place', 'RIGHT-A approach', 'closed'),
            ('lower_place', 'RIGHT-A place', 'closed'), ('release', 'RIGHT-A place', 'open'),
            ('retreat', 'RIGHT-A approach', 'open'),
            ('approach_pick', 'RIGHT-B approach', 'open'),
            ('lower_pick', 'RIGHT-B pick', 'open'), ('grasp', 'RIGHT-B pick', 'closed'),
            ('lift', 'RIGHT-B approach', 'closed'), ('approach_place', 'LEFT-B approach', 'closed'),
            ('lower_place', 'LEFT-B place', 'closed'), ('release', 'LEFT-B place', 'open'),
            ('retreat', 'LEFT-B approach', 'open')])

    def test_every_start_uses_its_sequence_and_never_swings_low(self):
        for name, make in [('fixture', profile), ('robot', robot_profile)]:
            for start in START_POSITIONS:
                with self.subTest(data=name, start=start):
                    steps = make().plan(start.split('-')[0], start)
                    visited = [s['pose'].split()[0] for s in steps if s['action'] in ('grasp', 'release')]
                    self.assertEqual(visited, [s['position'] for s in sequence_for(start)['steps']])
                    for before, after in zip(steps, steps[1:]):
                        if before['angles_deg']['base'] != after['angles_deg']['base']:
                            # BASE only turns between raised poses.
                            self.assertTrue(raised(before) and raised(after), (before, after))
                    self.assertEqual(steps[-1]['action'], 'retreat')
                    self.assertTrue(raised(steps[-1]))

    def test_robot_data_turns_base_above_the_box_then_lowers(self):
        steps = robot_profile().plan('LEFT', 'LEFT-A')
        self.assertEqual([(s['action'], s['pose']) for s in steps[:6]], [
            ('approach_pick', 'LEFT-A approach'), ('align_pick', 'LEFT-A above pick'),
            ('lower_pick', 'LEFT-A pick'), ('grasp', 'LEFT-A pick'),
            ('lift', 'LEFT-A above pick'), ('approach_place', 'RIGHT-A approach')])
        self.assertEqual({k: steps[1]['angles_deg'][k] for k in ('base', 'shoulder', 'elbow')},
                         dict(base=139., shoulder=111., elbow=113.))
        self.assertEqual((steps[0]['angles_deg']['gripper'], steps[3]['angles_deg']['gripper']), (150., 72.))

    def test_raised_moves_do_not_stop_and_sides_cross_in_one_move(self):
        steps = robot_profile().plan('LEFT', 'LEFT-A')
        for step in steps:
            with self.subTest(step=step['action']):
                grips = step['action'] in ('grasp', 'release')
                lowers = step['action'] in ('lower_pick', 'lower_place')
                self.assertEqual(step['settle_seconds'] > 0, grips or lowers)
        crossings = [(a['pose'], b['pose']) for a, b in zip(steps, steps[1:])
                     if a['pose'].split()[0].split('-')[0] != b['pose'].split()[0].split('-')[0]]
        self.assertEqual(crossings, [('LEFT-A above pick', 'RIGHT-A approach'),
                                     ('RIGHT-B above pick', 'LEFT-B approach')])

    def test_committed_robot_files_are_ready_for_every_start(self):
        root = FIXTURES.parent.parent
        read = lambda name: json.loads((root/name).read_text(encoding='utf-8'))
        p = PositionArmProfile(read('arm_poses.json'), read('arm_positions.json'))
        for start in START_POSITIONS:
            self.assertEqual(p.plan(start.split('-')[0], start)[-1]['action'], 'retreat')

    def test_untaught_center_carry_is_not_required(self):
        profile({('COMMON', 'center_carry')})

    def test_taught_gripper_angles_replace_the_legacy_ones(self):
        grips = {s['action']: s['angles_deg']['gripper'] for s in profile().plan('RIGHT', 'RIGHT-B')}
        self.assertEqual((grips['approach_pick'], grips['grasp']), (OPEN, CLOSED))
        self.assertTrue(profile().holding(CLOSED))

    def test_start_must_be_on_the_qr_side(self):
        with self.assertRaises(ValueError):
            profile().plan('LEFT', 'RIGHT-A')

    def test_untaught_slots_are_named(self):
        values = copy.deepcopy(test_arm.load_profile().values)
        del values['positions']
        with self.assertRaisesRegex(ValueError, 'ARM_POSES_INCOMPLETE: RIGHT-B place, COMMON gripper_closed$'):
            PositionArmProfile(values, taught({('RIGHT-B', 'place'), ('COMMON', 'gripper_closed')}))


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

    def turn(self, arm_enabled=True):
        from test_direct_mission import Harness
        from picar_mission.qr_prepare import QrPreparation
        h = Harness('LEFT')
        h.controller.cfg = replace(h.controller.cfg, arm_enabled=arm_enabled)
        driver = test_arm.FakeArm()
        h.controller.qr_preparation = QrPreparation(driver, test_arm.load_profile(), {'LEFT': 180., 'RIGHT': 0.}, 20.)
        requests = []
        h.controller.request_side_qr = lambda direction, now: requests.append((direction, h.controller.snapshot()['state'])) or True
        h.controller.arm = ArmSequence(driver, profile())
        h.echo(80.); h.near(36.); h.settle(30.)
        for _ in range(100):
            h.echo(None, dt=.1)
            if h.controller.snapshot()['state'] == 'QR_TURNING': break
        return h, driver, requests

    def test_codes_read_mid_turn_start_the_transfer_at_once(self):
        h, driver, requests = self.turn()
        # The scan was requested when the turn began, not after it ended.
        self.assertEqual(requests, [('LEFT', 'ARM_WAIT')])
        h.time += .5; h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'], 'QR_TURNING')
        mid_turn = driver.angles['base']
        self.assertLess(mid_turn, 180.)
        h.qr = dict(state='DIRECTION_CONFIRMED', first_direction='LEFT',
                    sequence_selection=dict(start_position='LEFT-B'))
        h.time += .02; h.controller.step()
        state = h.controller.snapshot()
        self.assertEqual(state['state'], 'ARM_TRANSFER')
        self.assertEqual(state['sequence_selection'], sequence_for('LEFT-B'))
        self.assertEqual(state['qr_preparation']['state'], 'INTERRUPTED')
        self.assertLess(abs(driver.angles['base']-mid_turn), 5.)
        for _ in range(3000):
            h.time += .05; h.controller.step()
            if h.controller.snapshot()['state'] == 'COMPLETE': break
        self.assertEqual(h.controller.snapshot()['arm']['completed_transfers'], 2)

    def test_finished_turn_tells_the_qr_worker(self):
        h, driver, requests = self.turn()
        settled = []
        h.controller.settle_side_qr = settled.append
        for _ in range(300):
            h.time += .05; h.controller.step()
            if h.controller.snapshot()['state'] == 'SIDE_QR_WAIT': break
        self.assertEqual(h.controller.snapshot()['state'], 'SIDE_QR_WAIT')
        self.assertEqual(len(settled), 1)

    def test_disabled_transfer_finishes_the_turn_before_showing_the_sequence(self):
        h, driver, requests = self.turn(arm_enabled=False)
        h.qr = dict(state='DIRECTION_CONFIRMED', first_direction='LEFT',
                    sequence_selection=dict(start_position='LEFT-A'))
        for _ in range(300):
            h.time += .05; h.controller.step()
            if h.controller.snapshot()['state'] == 'SEQUENCE_READY': break
        state = h.controller.snapshot()
        self.assertEqual(state['state'], 'SEQUENCE_READY')
        self.assertEqual(state['arm_blocked_reason'], 'DISABLED')
        self.assertEqual(driver.angles['base'], 180.)
        self.assertEqual(state['qr_preparation']['state'], 'READY')

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
        self.assertEqual(controller.arm.profile.carry_clearance,{'shoulder':153.,'elbow':156.})

    def test_untaught_positions_do_not_block_the_mission(self):
        controller = self.make(True, empty_profile())
        self.assertIsNone(controller.arm)
        self.assertRegex(controller._transfer_blocked_reason(), '^ARM_POSES_INCOMPLETE: LEFT-A approach')

    def test_disabled_never_reads_positions(self):
        controller = self.make(False, None)
        self.assertIsNone(controller.arm)
        self.assertEqual(controller._transfer_blocked_reason(), 'DISABLED')


if __name__ == '__main__': unittest.main()
