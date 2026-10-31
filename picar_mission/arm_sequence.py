"""Pose-based, rate-limited arm sequence. All packaged angles are placeholders.

Only the existing WebServer supplies a real driver. This module's CLI prints
a plan; it never imports motor/servo drivers or writes hardware.
"""
import argparse
import json
import math
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parent.parent
JOINTS = ('base','shoulder','elbow','gripper')
DEFAULT_CARRY_SPEED = 10.
PEAK_SPEED_RATIO = 1.875  # ease() peaks at 1.875x its average speed.
MAX_TICK_SECONDS = .1  # A stalled loop must not jump ahead on the path.
POSES = ('center_clear','center_carry','left_pick_above','left_pick',
         'left_place_above','left_place','right_pick_above','right_pick',
         'right_place_above','right_place')


def number(value):
    return type(value) in (int,float) and math.isfinite(value)


class ArmProfile:
    def __init__(self, values):
        self.values = values
        if values.get('schema_version')!=1 or type(values.get('calibration_confirmed')) is not bool:
            raise ValueError('Invalid arm schema or calibration flag')
        self.calibrated = values['calibration_confirmed']
        self.limits = values['joint_limits_deg']
        if set(self.limits)!=set(JOINTS):
            raise ValueError('Arm limits must contain base/shoulder/elbow/gripper only')
        for name in JOINTS:
            limits = self.limits[name]
            if not isinstance(limits,list) or len(limits)!=2 or not all(map(number,limits)) or not 0<=limits[0]<limits[1]<=180:
                raise ValueError('Invalid limits for '+name)
        self.positions = values['positions']
        if set(self.positions)!=set(POSES):
            raise ValueError('Arm positions must contain the ten named calibration poses')
        self.gripper = values['gripper_deg']
        if set(self.gripper)!=set(('open','closed')):
            raise ValueError('Require open and closed gripper values')
        for name,pose in self.positions.items():
            if set(pose)!=set(JOINTS[:3]):
                raise ValueError('Pose must contain three arm joints: '+name)
            self.check_angles({**pose,'gripper':self.gripper['open']})
        self.check_angles({**self.positions['center_clear'],'gripper':self.gripper['closed']})
        for key in ('speed_deg_per_second','settle_seconds','grip_settle_seconds',
                    'step_timeout_seconds','sequence_timeout_seconds'):
            if not number(values.get(key)) or values[key]<=0:
                raise ValueError('Invalid arm setting: '+key)
        if values['speed_deg_per_second']>60:
            raise ValueError('Arm command rate must not exceed 60 degrees/second')
        # Optional, so profiles taught before this setting keep loading.
        carry = values.get('carry_speed_deg_per_second',min(DEFAULT_CARRY_SPEED,values['speed_deg_per_second']))
        if not number(carry) or not 0<carry<=values['speed_deg_per_second']:
            raise ValueError('carry_speed_deg_per_second must be >0 and <= speed_deg_per_second')
        self.carry_speed = carry
        if values['step_timeout_seconds']<=max(values['settle_seconds'],values['grip_settle_seconds']):
            raise ValueError('Arm step timeout is shorter than settling time')

    def holding(self, gripper_angle):
        """True when the gripper is nearer its closed than its open angle."""
        return abs(gripper_angle-self.gripper['closed'])<abs(gripper_angle-self.gripper['open'])

    def move_speed(self, holding):
        return self.carry_speed if holding else self.values['speed_deg_per_second']

    def check_angles(self, angles):
        if set(angles)!=set(JOINTS):
            raise ValueError('Exactly four named arm joints required; steering is excluded')
        for name,value in angles.items():
            if not number(value) or not self.limits[name][0]<=value<=self.limits[name][1]:
                raise ValueError('Angle outside configured limit: %s %s (allowed %s~%s)'%(name,value,*self.limits[name]))

    def plan(self, direction):
        if direction not in ('LEFT','RIGHT'):
            raise ValueError('Direction must be LEFT or RIGHT')
        first = direction.lower()
        other = 'right' if first=='left' else 'left'
        steps = []
        def add(pose, grip, action):
            steps.append(dict(action=action,pose=pose,gripper=grip,
                angles_deg={**self.positions[pose],'gripper':self.gripper[grip]},
                settle_seconds=self.values['grip_settle_seconds'] if action in ('grasp','release') else self.values['settle_seconds']))
        add('center_clear','open','prepare')
        for source,destination in ((first,other),(other,first)):
            add(source+'_pick_above','open','approach_pick')
            add(source+'_pick','open','lower_pick')
            add(source+'_pick','closed','grasp')
            add(source+'_pick_above','closed','lift')
            add('center_carry','closed','carry_center')
            add(destination+'_place_above','closed','approach_place')
            add(destination+'_place','closed','lower_place')
            add(destination+'_place','open','release')
            add(destination+'_place_above','open','retreat')
            add('center_clear','open','return_clear')
        return steps


def load_profile(path=None):
    path = ROOT/'arm_poses.json' if path is None else Path(path)
    return ArmProfile(json.loads(path.read_text(encoding='utf-8')))


GRIPPER_SLOTS = ('gripper_open','gripper_closed')


def save_pose(name, angles, path=None):
    """Store taught commanded angles under a pose (base/shoulder/elbow) or a
    gripper slot. Never touches calibration_confirmed; the whole profile is
    re-validated before the file is replaced."""
    path = ROOT/'arm_poses.json' if path is None else Path(path)
    values = json.loads(path.read_text(encoding='utf-8'))
    if name in POSES:
        saved = {joint:round(float(angles[joint]),1) for joint in JOINTS[:3]}
        values['positions'][name] = saved
    elif name in GRIPPER_SLOTS:
        saved = {'gripper':round(float(angles['gripper']),1)}
        values['gripper_deg'][name.split('_')[1]] = saved['gripper']
    else:
        raise ValueError('Unknown pose name: '+str(name))
    ArmProfile(values)
    tmp = path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(values,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    tmp.replace(path)
    return saved


def pose_targets(profile, name):
    """Joint targets for one taught slot. Joints not listed stay where they are,
    so recalling an arm pose never opens the gripper on a held object."""
    if name in POSES:
        return dict(profile.positions[name])
    if name in GRIPPER_SLOTS:
        return {'gripper':profile.gripper[name.split('_')[1]]}
    raise ValueError('Unknown pose name: '+str(name))


def ease(x):
    """Minimum-jerk profile: speed and acceleration are zero at both ends, so
    a loaded joint is never kicked into motion or stopped dead (which made the
    arm bounce and shake a held object loose)."""
    x = min(max(x,0.),1.)
    return x*x*x*(10.+x*(6.*x-15.))


class Motion:
    """One coordinated move: all joints start and arrive together on the eased
    profile, and no joint ever exceeds `speed` degrees/second."""
    def __init__(self, start, targets, speed):
        self.start = {joint:float(start[joint]) for joint in targets}
        self.targets = dict(targets)
        span = max([abs(goal-self.start[j]) for j,goal in self.targets.items()]+[0.])
        self.duration = PEAK_SPEED_RATIO*span/speed
        self.elapsed = 0.

    @property
    def done(self):
        return self.elapsed>=self.duration

    def advance(self, dt):
        self.elapsed += min(max(dt,0.),MAX_TICK_SECONDS)
        s = 1. if self.done else ease(self.elapsed/self.duration)
        return {j:self.start[j]+(goal-self.start[j])*s for j,goal in self.targets.items()}


def write_changed(driver, current, angles):
    for joint,value in angles.items():
        if abs(value-current[joint])>1e-9:
            driver.set_joint(joint,value)
            current[joint] = value


class PoseMove:
    """Eased move of the given joints to fixed targets (teach mode)."""
    def __init__(self, driver, targets, speed, clock=time.monotonic):
        self.driver,self.clock = driver,clock
        angles = driver.read_angles()
        self.current = {joint:angles[joint] for joint in targets}
        self.motion = Motion(self.current,targets,speed)
        self.last_tick = clock()

    def tick(self, now=None):
        """Advance along the path; True once every joint is on target."""
        now=self.clock() if now is None else now
        dt,self.last_tick = now-self.last_tick,now
        write_changed(self.driver,self.current,self.motion.advance(dt))
        return self.motion.done


class ArmSequence:
    def __init__(self, driver, profile, clock=time.monotonic):
        self.driver,self.profile,self.clock = driver,profile,clock
        self.state='IDLE'
        self.steps=[]
        self.index=0
        self.current={}
        self.last_tick=None
        self.started_at=None
        self.step_at=None
        self.reached_at=None
        self.motion=None
        self.planned_seconds=0.
        self.holding=False
        self.error=None
        self.completed_transfers=0

    def start(self, direction, now=None):
        if not self.profile.calibrated:
            raise RuntimeError('ARM_CALIBRATION_REQUIRED: packaged angles are provisional')
        if self.state!='IDLE':
            raise RuntimeError('Arm sequence cannot restart')
        self.steps=self.profile.plan(direction)
        self.current=dict(self.driver.read_angles())
        self.profile.check_angles(self.current)
        self.holding=self.profile.holding(self.current['gripper'])
        now=self.clock() if now is None else now
        self.last_tick=self.started_at=self.step_at=now
        self.driver.hold()
        self.state='MOVING'

    def tick(self, now=None):
        if self.state!='MOVING':
            return self.state
        now=self.clock() if now is None else now
        try:
            if now<self.last_tick:
                raise RuntimeError('ARM_CLOCK_REVERSED')
            step=self.steps[self.index]
            if self.motion is None:
                # Slow down whenever an object is (or is about to be) held.
                carrying=self.holding or step['gripper']=='closed'
                self.motion=Motion(self.current,step['angles_deg'],self.profile.move_speed(carrying))
                self.planned_seconds+=self.motion.duration
            # Timeouts count beyond the planned motion time, so a slow carry
            # speed cannot trip them; they still catch a stalled control loop.
            if now-self.started_at>self.profile.values['sequence_timeout_seconds']+self.planned_seconds:
                raise RuntimeError('ARM_SEQUENCE_TIMEOUT')
            if now-self.step_at>self.profile.values['step_timeout_seconds']+self.motion.duration:
                raise RuntimeError('ARM_STEP_TIMEOUT')
            dt,self.last_tick=now-self.last_tick,now
            write_changed(self.driver,self.current,self.motion.advance(dt))
            reached=self.motion.done
            if reached and self.reached_at is None:
                self.reached_at=now
            if reached and now-self.reached_at>=step['settle_seconds']:
                if step['action']=='release':
                    self.completed_transfers+=1
                self.index+=1
                if self.index==len(self.steps):
                    self.driver.hold()
                    self.state='COMPLETE'
                else:
                    self.holding=step['gripper']=='closed'
                    self.motion=None
                    self.step_at=now
                    self.reached_at=None
        except Exception as exc:
            self.error=str(exc)
            self.state='ERROR'
            self.driver.hold()
            raise
        return self.state

    def cancel(self):
        # Keep the last commanded pose/grip. Do not drop an object or swing home.
        self.driver.hold()
        if self.state!='COMPLETE':
            self.state='CANCELLED'

    def snapshot(self):
        step=self.steps[self.index] if self.index<len(self.steps) else None
        return dict(state=self.state,calibration_confirmed=self.profile.calibrated,
                    step_index=self.index,step_count=len(self.steps),step=step,
                    commanded_angles_deg=dict(self.current),completed_transfers=self.completed_transfers,
                    error=self.error)


def main():
    parser=argparse.ArgumentParser(description='Print a provisional arm plan; never moves hardware')
    parser.add_argument('--plan',required=True,choices=['LEFT','RIGHT'])
    parser.add_argument('--poses',type=Path)
    args=parser.parse_args()
    profile=load_profile(args.poses)
    print(json.dumps(dict(calibration_confirmed=profile.calibrated,hardware_control=False,
                         first_direction=args.plan,steps=profile.plan(args.plan)),ensure_ascii=False,indent=2))


if __name__=='__main__': main()
