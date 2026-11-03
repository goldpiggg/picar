"""Automatic transfer from the four taught positions (arm_positions.json).

The confirmed QR sequence fixes the order of the two PICK/PLACE pairs. Joint
limits, speeds, settle times and timeouts still come from arm_poses.json.
arm_positions.json never confirms calibration itself: automatic transfer runs
only when every slot it can use is taught and mission arm_enabled is true.
"""
from .arm_positions import POSES, POSITIONS
from .arm_sequence import ArmMotionProfile
from .sequence_selection import START_POSITIONS, sequence_for

# center_clear is not needed: every move between taught slots starts raised.
REQUIRED_COMMON = ('gripper_open', 'gripper_closed')
ARM_JOINTS = ('base', 'shoulder', 'elbow')


class PositionArmProfile(ArmMotionProfile):
    def __init__(self, motion_values, positions):
        super().__init__(motion_values)
        missing = [position+' '+pose for position in POSITIONS for pose in POSES
                   if positions['positions'][position][pose] is None]
        missing += ['COMMON '+name for name in REQUIRED_COMMON if positions['common'][name] is None]
        # A carry pose taught on the older pose page still counts.
        carry = positions['common']['center_carry']
        legacy_carry = (motion_values.get('positions') or {}).get('center_carry')
        if carry is not None:
            carry, self.carry_source = carry['angles_deg'], 'arm_positions.json'
        elif isinstance(legacy_carry, dict) and set(legacy_carry) == set(ARM_JOINTS):
            carry, self.carry_source = legacy_carry, 'arm_poses.json'
        else:
            missing.append('COMMON center_carry')
        if missing:
            raise ValueError('ARM_POSES_INCOMPLETE: '+', '.join(missing))
        self.poses = {(position, pose): dict(positions['positions'][position][pose]['angles_deg'])
                      for position in POSITIONS for pose in POSES}
        self.poses[('COMMON', 'center_carry')] = {joint: float(carry[joint]) for joint in ARM_JOINTS}
        self.gripper = {grip: positions['common']['gripper_'+grip]['angles_deg']['gripper']
                        for grip in ('open', 'closed')}
        if abs(self.gripper['open']-self.gripper['closed']) < 1:
            raise ValueError('ARM_GRIPPER_OPEN_EQUALS_CLOSED')
        neutral = {name: self.limits[name][0] for name in ARM_JOINTS}
        for grip in self.gripper.values():
            self.check_angles({**neutral, 'gripper': grip})
        for pose in self.poses.values():
            self.check_angles({**pose, 'gripper': self.gripper['open']})
        # Taught-and-complete is the confirmation; arm_enabled is the switch.
        self.calibrated = True

    def above(self, position, low):
        """Approach height over the pick/place BASE, so BASE never turns low."""
        approach = self.poses[(position, 'approach')]
        base = self.poses[(position, low)]['base']
        if base == approach['base']:
            return position+' approach', dict(approach)
        return '%s above %s' % (position, low), {**approach, 'base': base}

    def plan(self, direction, start_position):
        if start_position not in START_POSITIONS or not start_position.startswith(direction+'-'):
            raise ValueError('Start position must be on the QR side: '+str(start_position))
        steps = []
        def add(name, angles, grip, action):
            # grip None leaves the gripper where the mission found it.
            steps.append(dict(action=action, pose=name, gripper=grip,
                angles_deg={**angles, 'gripper': None if grip is None else self.gripper[grip]},
                settle_seconds=self.values['grip_settle_seconds'] if action in ('open_gripper', 'grasp', 'release')
                    else self.values['settle_seconds']))
        def slot(position, pose):
            return position+' '+pose, self.poses[(position, pose)]
        for item in sequence_for(start_position)['steps']:
            position = item['position']
            if item['action'] == 'PICK':
                if not steps:
                    add(*slot(position, 'approach'), None, 'approach_pick')
                    add(*slot(position, 'approach'), 'open', 'open_gripper')
                else:
                    # The previous release left the gripper open on this side.
                    add(*slot(position, 'approach'), 'open', 'approach_pick')
                above = self.above(position, 'pick')
                if above[0] != position+' approach':
                    add(*above, 'open', 'align_pick')
                add(*slot(position, 'pick'), 'open', 'lower_pick')
                add(*slot(position, 'pick'), 'closed', 'grasp')
                add(*above, 'closed', 'lift')
            else:
                # Every PLACE is across from its PICK: swing only once raised.
                add(*slot('COMMON', 'center_carry'), 'closed', 'carry_center')
                add(*slot(position, 'approach'), 'closed', 'approach_place')
                above = self.above(position, 'place')
                if above[0] != position+' approach':
                    add(*above, 'closed', 'align_place')
                add(*slot(position, 'place'), 'closed', 'lower_place')
                add(*slot(position, 'place'), 'open', 'release')
                add(*above, 'open', 'retreat')
        return steps
