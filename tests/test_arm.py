"""No hardware: provisional pose validation, sequence mechanics and hand-off."""
import copy
from dataclasses import replace
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from types import SimpleNamespace
import test_approach

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from picar_mission.arm_sequence import ArmProfile,ArmSequence,load_profile,save_pose,JOINTS
from picar_mission.config import MissionConfig
from test_approach import Harness


class FakeArm:
    def __init__(self):
        self.angles={name:90. for name in JOINTS}
        self.writes=[]
        self.holds=0
    def read_angles(self): return dict(self.angles)
    def hold(self): self.holds+=1
    def set_joint(self,joint,angle):
        self.writes.append((joint,angle))
        self.angles[joint]=angle


def calibrated_profile():
    data=copy.deepcopy(load_profile().values)
    data['calibration_confirmed']=True
    return ArmProfile(data)


class PoseTests(unittest.TestCase):
    def test_shipped_profile_is_provisional_and_automatic_arm_is_off(self):
        self.assertFalse(load_profile().calibrated)
        self.assertFalse(MissionConfig().arm_enabled)

    def test_left_and_right_plan_have_two_opposite_transfers(self):
        for direction,first,other in [('LEFT','left','right'),('RIGHT','right','left')]:
            with self.subTest(direction=direction):
                plan=load_profile().plan(direction)
                self.assertEqual([s['pose'] for s in plan if s['action']=='grasp'],[first+'_pick',other+'_pick'])
                self.assertEqual([s['pose'] for s in plan if s['action']=='release'],[other+'_place',first+'_place'])
                self.assertEqual(plan[-1]['pose'],'center_clear')
                self.assertEqual(len(plan),21)

    def test_invalid_angle_nan_and_steering_pose_are_rejected(self):
        for mutation in ['range','nan','steering']:
            data=copy.deepcopy(load_profile().values)
            if mutation=='range': data['positions']['left_pick']['base']=181
            if mutation=='nan': data['gripper_deg']['closed']=float('nan')
            if mutation=='steering': data['positions']['left_pick']['steering']=90
            with self.subTest(mutation=mutation),self.assertRaises(ValueError): ArmProfile(data)

    def test_bad_direction_cannot_generate_plan(self):
        with self.assertRaises(ValueError): load_profile().plan('PICK')

    def test_official_adapter_uses_absolute_arm_channels_and_readback(self):
        bridge=test_approach.OfficialConnectionTests().load_bridge()
        calls=[]
        servo=SimpleNamespace(nowPos=[90,55,80,100,110,90,90,90],
            setPWM=lambda *args:calls.append(args),stopWiggle=lambda:calls.append(('hold',)))
        driver=bridge.ArmAdapter(servo)
        self.assertEqual(driver.read_angles(),dict(base=55.,shoulder=80.,elbow=100.,gripper=110.))
        driver.set_joint('base',125);driver.set_joint('gripper',70);driver.hold()
        self.assertEqual(calls,[(1,125),(4,70),('hold',)])
        with self.assertRaises(KeyError): driver.set_joint('steering',90)
        servo.nowPos[4],servo.nowPos[1]=181,-1  # manufacturer wiggle overshoot
        self.assertEqual(driver.read_angles(),dict(base=0.,shoulder=80.,elbow=100.,gripper=180.))


class ServoDriverTests(unittest.TestCase):
    def test_pca9685_frequency_is_written_once_not_per_servo_command(self):
        import importlib.util
        from unittest.mock import patch
        events=[]
        class PCA:
            def __init__(self,*args,**kwargs):
                events.append('init');self.channels=list(range(16))
            def __setattr__(self,name,value):
                if name=='frequency': events.append(('frequency',value))
                object.__setattr__(self,name,value)
        class Servo:
            def __init__(self,channel,**kwargs): self.channel=channel
            def __setattr__(self,name,value):
                if name=='angle': events.append(('angle',self.channel,value))
                object.__setattr__(self,name,value)
        modules={'board':SimpleNamespace(SCL=1,SDA=2),
                 'busio':SimpleNamespace(I2C=lambda *args:object()),
                 'adafruit_pca9685':SimpleNamespace(PCA9685=PCA),
                 'adafruit_motor':SimpleNamespace(servo=SimpleNamespace(Servo=Servo))}
        spec=importlib.util.spec_from_file_location('test_rpiservo',ROOT/'Server/RPIservo.py')
        module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules): spec.loader.exec_module(module)
        sc=module.ServoCtrl()
        sc.set_angle(2,80);sc.set_angle(2,81);sc.setPWM(4,110)
        self.assertEqual(events,['init',('frequency',50),('angle',2,80),('angle',2,81),('angle',4,110)])


class SavePoseTests(unittest.TestCase):
    def setUp(self):
        self.dir=tempfile.TemporaryDirectory()
        self.path=Path(self.dir.name)/'arm_poses.json'
        shutil.copy(ROOT/'arm_poses.json',self.path)

    def tearDown(self): self.dir.cleanup()

    def test_pose_and_gripper_slots_are_written_without_confirming_calibration(self):
        angles=dict(base=61.04,shoulder=98.,elbow=112.5,gripper=74.)
        self.assertEqual(save_pose('left_pick',angles,self.path),dict(base=61.,shoulder=98.,elbow=112.5))
        self.assertEqual(save_pose('gripper_closed',angles,self.path),dict(gripper=74.))
        profile=load_profile(self.path)
        self.assertEqual(profile.positions['left_pick'],dict(base=61.,shoulder=98.,elbow=112.5))
        self.assertEqual(profile.gripper['closed'],74.)
        self.assertFalse(profile.calibrated)

    def test_unknown_name_and_out_of_limit_angles_leave_file_unchanged(self):
        before=self.path.read_text(encoding='utf-8')
        for name,angles in [('steering',dict(base=90,shoulder=90,elbow=90,gripper=90)),
                            ('left_pick',dict(base=5,shoulder=90,elbow=90,gripper=90))]:
            with self.subTest(name=name),self.assertRaises(ValueError): save_pose(name,angles,self.path)
        self.assertEqual(self.path.read_text(encoding='utf-8'),before)

    def test_bridge_refuses_to_save_in_automatic_mode(self):
        bridge=test_approach.OfficialConnectionTests().load_bridge()
        bridge.config=replace(bridge.config,auto_approach_enabled=True)
        with self.assertRaises(ValueError): bridge.save_pose('left_pick')


class SequenceTests(unittest.TestCase):
    def test_uncalibrated_start_never_reads_or_writes_driver(self):
        driver=FakeArm();s=ArmSequence(driver,load_profile())
        with self.assertRaisesRegex(RuntimeError,'CALIBRATION_REQUIRED'): s.start('LEFT',0.)
        self.assertEqual(driver.writes,[])
        self.assertEqual(driver.holds,0)

    def test_full_sequence_preserves_grip_during_carry(self):
        for direction in ['LEFT','RIGHT']:
            driver=FakeArm();s=ArmSequence(driver,calibrated_profile());s.start(direction,0.)
            for tick in range(1,4000):
                s.tick(tick*.05)
                if s.state=='COMPLETE': break
            self.assertEqual(s.state,'COMPLETE')
            self.assertEqual(s.completed_transfers,2)
            self.assertEqual(driver.angles['gripper'],110)
            self.assertTrue(all(joint in JOINTS for joint,_ in driver.writes))
            self.assertFalse(any(step['gripper']!='closed' for step in s.steps if step['action'] in ('lift','carry_center','approach_place','lower_place')))

    def test_rate_limit_and_stalled_tick_do_not_jump(self):
        driver=FakeArm();s=ArmSequence(driver,calibrated_profile());s.start('LEFT',0.)
        previous=driver.read_angles();s.tick(.05)
        self.assertLessEqual(max(abs(driver.angles[j]-previous[j]) for j in JOINTS),1.000001)
        previous=driver.read_angles();s.tick(2.)
        self.assertLessEqual(max(abs(driver.angles[j]-previous[j]) for j in JOINTS),2.000001)

    def test_cancel_holds_grip_and_does_not_home_or_release(self):
        driver=FakeArm();s=ArmSequence(driver,calibrated_profile());s.start('LEFT',0.)
        s.tick(.05);previous=dict(driver.angles);count=len(driver.writes)
        s.cancel();s.tick(1.)
        self.assertEqual(driver.angles,previous)
        self.assertEqual(len(driver.writes),count)
        self.assertEqual(s.state,'CANCELLED')

    def test_step_timeout_stops_sequence(self):
        data=copy.deepcopy(calibrated_profile().values);data['step_timeout_seconds']=1.
        driver=FakeArm();s=ArmSequence(driver,ArmProfile(data));s.start('LEFT',0.)
        with self.assertRaisesRegex(RuntimeError,'STEP_TIMEOUT'): s.tick(2.)
        self.assertEqual(s.state,'ERROR')
        self.assertGreater(driver.holds,1)


class HandoffTests(unittest.TestCase):
    def test_enabled_but_uncalibrated_stays_at_stopped_wait(self):
        h=Harness(config=replace(MissionConfig(),arm_enabled=True))
        driver=FakeArm();h.controller.arm=ArmSequence(driver,load_profile())
        h.start();h.finish()
        state=h.controller.snapshot()
        self.assertEqual(state['state'],'STOPPED_WAIT')
        self.assertEqual(state['arm_blocked_reason'],'CALIBRATION_REQUIRED')
        self.assertEqual(driver.writes,[])

    def test_calibrated_arm_runs_only_after_stop_and_never_drives_again(self):
        for direction in ['LEFT','RIGHT']:
            h=Harness(direction,config=replace(MissionConfig(),arm_enabled=True))
            driver=FakeArm();arm=ArmSequence(driver,calibrated_profile(),clock=lambda:h.time)
            h.controller.arm=arm
            h.start()
            self.assertEqual(driver.writes,[])
            h.finish()
            self.assertEqual(h.controller.snapshot()['state'],'ARM_TRANSFER')
            wheel_count=len(h.drive.commands)
            for _ in range(4000):
                h.time+=.05;h.controller.step()
                if h.controller.snapshot()['state']=='COMPLETE': break
            self.assertEqual(h.controller.snapshot()['state'],'COMPLETE')
            self.assertEqual(h.controller.snapshot()['first_direction'],direction)
            self.assertTrue(h.controller.snapshot()['drive_locked'])
            self.assertEqual(arm.completed_transfers,2)
            self.assertFalse(any(c[0]=='forward' for c in h.drive.commands[wheel_count:]))

    def test_cancel_during_arm_is_terminal(self):
        h=Harness(config=replace(MissionConfig(),arm_enabled=True))
        driver=FakeArm();h.controller.arm=ArmSequence(driver,calibrated_profile())
        h.start();h.finish();h.controller.cancel();count=len(driver.writes)
        for _ in range(20): h.time+=.05;h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'CANCELLED')
        self.assertEqual(len(driver.writes),count)
        self.assertFalse(h.controller.snapshot()['arm_motion_enabled'])


if __name__=='__main__': unittest.main()
