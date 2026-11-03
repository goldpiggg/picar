"""Automatic transfer from the four taught positions (arm_positions.json).

The confirmed QR sequence fixes the order of the two PICK/PLACE pairs. Joint
limits, speeds, settle times and timeouts still come from arm_poses.json.
arm_positions.json never confirms calibration itself: automatic transfer runs
only when every slot it can use is taught and mission arm_enabled is true.
"""
from .arm_positions import POSES, POSITIONS
from .arm_sequence import ArmMotionProfile
from .sequence_selection import START_POSITIONS, sequence_for

# center_clear/center_carry are not needed: every move between taught slots
# starts and ends raised, so the arm crosses sides in one move.
REQUIRED_COMMON = ('gripper_open', 'gripper_closed')
ARM_JOINTS = ('base', 'shoulder', 'elbow')
# Raised pass-through moves do not wait on arrival; only a move that is about
# to grip or release settles first.
PASS_THROUGH = ('approach_pick', 'align_pick', 'lift', 'approach_place', 'align_place', 'retreat')


class PositionArmProfile(ArmMotionProfile):
    def __init__(self, motion_values, positions):
        super().__init__(motion_values)
        missing = [position+' '+pose for position in POSITIONS for pose in POSES
                   if positions['positions'][position][pose] is None]
        missing += ['COMMON '+name for name in REQUIRED_COMMON if positions['common'][name] is None]
        if missing:
            raise ValueError('ARM_POSES_INCOMPLETE: '+', '.join(missing))
        self.poses = {(position, pose): dict(positions['positions'][position][pose]['angles_deg'])
                      for position in POSITIONS for pose in POSES}
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
                settle_seconds=0. if action in PASS_THROUGH else
                    self.values['grip_settle_seconds'] if action in ('grasp', 'release')
                    else self.values['settle_seconds']))
        def slot(position, pose):
            return position+' '+pose, self.poses[(position, pose)]
        for item in sequence_for(start_position)['steps']:
            position = item['position']
            if item['action'] == 'PICK':
                # The first move opens the empty gripper on the way; later the
                # previous release has already left it open on this side.
                add(*slot(position, 'approach'), 'open', 'approach_pick')
                above = self.above(position, 'pick')
                if above[0] != position+' approach':
                    add(*above, 'open', 'align_pick')
                add(*slot(position, 'pick'), 'open', 'lower_pick')
                add(*slot(position, 'pick'), 'closed', 'grasp')
                add(*above, 'closed', 'lift')
            else:
                # Every PLACE is across from its PICK: one raised move from
                # above the pick straight to above the place, without stopping.
                add(*slot(position, 'approach'), 'closed', 'approach_place')
                above = self.above(position, 'place')
                if above[0] != position+' approach':
                    add(*above, 'closed', 'align_place')
                add(*slot(position, 'place'), 'closed', 'lower_place')
                add(*slot(position, 'place'), 'open', 'release')
                add(*above, 'open', 'retreat')
        return steps
