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
        if values['step_timeout_seconds']<=max(values['settle_seconds'],values['grip_settle_seconds']):
            raise ValueError('Arm step timeout is shorter than settling time')

    def check_angles(self, angles):
        if set(angles)!=set(JOINTS):
            raise ValueError('Exactly four named arm joints required; steering is excluded')
        for name,value in angles.items():
            if not number(value) or not self.limits[name][0]<=value<=self.limits[name][1]:
                raise ValueError('Angle outside configured limit: '+name)

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
        now=self.clock() if now is None else now
        self.last_tick=self.started_at=self.step_at=now
        self.driver.hold()
        self.state='MOVING'

    def tick(self, now=None):
        if self.state!='MOVING':
            return self.state
        now=self.clock() if now is None else now
        try:
            if now-self.started_at>self.profile.values['sequence_timeout_seconds']:
                raise RuntimeError('ARM_SEQUENCE_TIMEOUT')
            if now-self.step_at>self.profile.values['step_timeout_seconds']:
                raise RuntimeError('ARM_STEP_TIMEOUT')
            if now<self.last_tick:
                raise RuntimeError('ARM_CLOCK_REVERSED')
            dt=min(now-self.last_tick,.1)  # A stalled loop must not jump to the next pose.
            self.last_tick=now
            step=self.steps[self.index]
            limit=self.profile.values['speed_deg_per_second']*dt
            for joint in JOINTS:
                goal=step['angles_deg'][joint]
                delta=goal-self.current[joint]
                value=self.current[joint]+max(-limit,min(limit,delta))
                if abs(value-self.current[joint])>1e-9:
                    self.driver.set_joint(joint,value)
                    self.current[joint]=value
            reached=all(abs(self.current[j]-step['angles_deg'][j])<1e-7 for j in JOINTS)
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
