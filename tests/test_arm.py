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
from picar_mission.arm_sequence import ArmProfile,ArmSequence,PoseMove,load_profile,pose_targets,save_pose,JOINTS
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
        # Holding a manual arm button: each step is paced to scDelay, ~0.6 deg/step.
        sleeps=[]
        with patch.object(module.time,'sleep',sleeps.append):
            sc.singleServo(4,-1,7);del events[:]
            for _ in range(10): sc.moveWiggle()
        self.assertTrue(all(.08<s<=.09 for s in sleeps),sleeps)
        angles=[e[2] for e in events]
        self.assertEqual(len(angles),10)
        self.assertTrue(all(abs(b-a)<=1 for a,b in zip([110]+angles,angles)),angles)
        self.assertEqual(angles[-1],104)


    def load_servo_module(self):
        import importlib.util
        from unittest.mock import patch
        writes=[]
        class PCA:
            reference_clock_speed=25000000
            def __init__(self,*args,**kwargs):
                self.channels=list(range(16));self.mode1_reg=0x20;self.prescale_reg=0
            def reset(self): writes.append('reset');self.mode1_reg=0
            def __setattr__(self,name,value):
                if name=='frequency':
                    writes.append(('frequency',value));self.prescale_reg=121
                object.__setattr__(self,name,value)
        class Servo:
            def __init__(self,channel,**kwargs): self.channel=channel
            def __setattr__(self,name,value):
                if name=='angle': writes.append(('angle',self.channel,value))
                object.__setattr__(self,name,value)
        modules={'board':SimpleNamespace(SCL=1,SDA=2),
                 'busio':SimpleNamespace(I2C=lambda *args:object()),
                 'adafruit_pca9685':SimpleNamespace(PCA9685=PCA),
                 'adafruit_motor':SimpleNamespace(servo=SimpleNamespace(Servo=Servo))}
        spec=importlib.util.spec_from_file_location('test_rpiservo_hold',ROOT/'Server/RPIservo.py')
        module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules): spec.loader.exec_module(module)
        return module,writes

    def test_pwm_chip_reset_by_power_dip_restores_held_angles(self):
        module,writes=self.load_servo_module()
        self.assertFalse(module.restore_if_reset())  # Nothing set up yet.
        sc=module.ServoCtrl()
        sc.setPWM(2,80);sc.setPWM(4,62)
        del writes[:]
        self.assertFalse(module.restore_if_reset())
        self.assertEqual(writes,[])
        module.pwm_servo.mode1_reg=0x11;module.pwm_servo.prescale_reg=0x1e  # Power-on defaults.
        self.assertTrue(module.restore_if_reset())
        self.assertEqual(writes,['reset',('frequency',50),('angle',2,80),('angle',4,62)])
        self.assertFalse(module.restore_if_reset())

    def squeeze(self,module,sc,*args):
        from unittest.mock import patch
        timers=[]
        class Timer:
            def __init__(self,seconds,fn): timers.append((seconds,fn))
            def start(self): pass
        with patch.object(module.time,'sleep',lambda s:None),              patch.object(module.threading,'Timer',Timer):
            sc.squeeze(*args)
        return timers

    def test_squeeze_seats_firmly_then_eases_to_hold_past_contact(self):
        module,writes=self.load_servo_module()
        sc=module.ServoCtrl()
        sc.setPWM(4,75)  # "grab" released here, at contact.
        timers=self.squeeze(module,sc,4,-1,8,4,1.)
        self.assertEqual((sc.nowPos[4],writes[-1]),(67,('angle',4,67)))
        self.assertEqual(timers[0][0],1.)
        timers[0][1]()
        # Still 4 deg past contact: grips harder than stopping at 75.
        self.assertEqual((sc.nowPos[4],sc.goalPos[4],writes[-1]),(71,71,('angle',4,71)))
        self.assertEqual(module._last_angles[4],71)  # Restored after a power dip.

    def test_squeeze_ease_skips_when_gripper_moved_and_stays_in_limits(self):
        module,writes=self.load_servo_module()
        sc=module.ServoCtrl()
        sc.setPWM(4,75)
        timers=self.squeeze(module,sc,4,-1,8,4,1.)
        sc.setPWM(4,110)  # "loose" before the ease fired.
        timers[0][1]()
        self.assertEqual(writes[-1],('angle',4,110))
        sc.setPWM(4,3)
        timers=self.squeeze(module,sc,4,-1,8,4,1.)
        self.assertEqual(writes[-1],('angle',4,0))
        timers[0][1]()
        self.assertEqual(writes[-1],('angle',4,0))


class MotorVoltageTests(unittest.TestCase):
    def load_move(self):
        import importlib.util
        from unittest.mock import patch
        created=[]
        class PCA:
            def __init__(self,*args,**kwargs): created.append(self);self.channels=list(range(16))
        class Motor:
            def __init__(self,*args): self.throttle=None
        modules={'board':SimpleNamespace(SCL=1,SDA=2),
                 'busio':SimpleNamespace(I2C=lambda *args:object()),
                 'adafruit_pca9685':SimpleNamespace(PCA9685=PCA),
                 'adafruit_motor':SimpleNamespace(motor=SimpleNamespace(DCMotor=Motor,SLOW_DECAY=1))}
        spec=importlib.util.spec_from_file_location('test_move_volts',ROOT/'Server/Move.py')
        move=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules): spec.loader.exec_module(move)
        return move,created

    def test_speed_is_percent_of_5v_whatever_the_battery(self):
        move,_=self.load_move();move.setup()
        for battery,throttle in [(8.4,.5*5/8.4),(7.4,.5*5/7.4),(5.,.5),(None,.5*5/7.4),(2.,.5*5/7.4)]:
            with self.subTest(battery=battery):
                move.read_battery=lambda:battery
                move.Motor(1,1,50)
                self.assertAlmostEqual(move.motor1.throttle,throttle)
                # Motor voltage is the same 2.5 V at any measured charge.
                if battery and battery>=4: self.assertAlmostEqual(move.motor1.throttle*battery,2.5)
        move.read_battery=lambda:4.5
        move.Motor(1,1,100)
        self.assertEqual(move.motor1.throttle,1.)  # Cannot exceed the pack.

    def test_reconnect_reuses_the_shared_pwm_chip(self):
        move,created=self.load_move()
        move.setup();move.setup()
        self.assertEqual(len(created),1)


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
                            ('left_pick',dict(base=181,shoulder=90,elbow=90,gripper=90))]:
            with self.subTest(name=name),self.assertRaises(ValueError): save_pose(name,angles,self.path)
        self.assertEqual(self.path.read_text(encoding='utf-8'),before)

    def test_bridge_refuses_to_save_in_automatic_mode(self):
        bridge=test_approach.OfficialConnectionTests().load_bridge()
        bridge.config=replace(bridge.config,auto_approach_enabled=True)
        with self.assertRaises(ValueError): bridge.save_pose('left_pick')


class PoseRecallTests(unittest.TestCase):
    def test_arm_pose_moves_smoothly_and_leaves_gripper(self):
        arm=FakeArm();arm.angles['gripper']=70.
        targets=pose_targets(load_profile(),'left_pick')
        self.assertEqual(set(targets),{'base','shoulder','elbow'})
        t=[0.];move=PoseMove(arm,targets,10,clock=lambda:t[0])
        path=[arm.read_angles()]
        for _ in range(400):
            t[0]+=.02;done=move.tick();path.append(arm.read_angles())
            if done: break
        self.assertEqual(path[-1],dict(base=55.,shoulder=105.,elbow=115.,gripper=70.))
        self.assertFalse(any(joint=='gripper' for joint,_ in arm.writes))
        steps=[max(abs(b[j]-a[j]) for j in JOINTS) for a,b in zip(path,path[1:])]
        self.assertLessEqual(max(steps),10*.02+1e-9)  # Never faster than the speed.
        self.assertLess(steps[0],.01);self.assertLess(steps[-1],.01)  # Soft start and stop.
        # Joints move together: each covers the same fraction of its span.
        mid=path[len(path)//2]
        fractions=[(mid[j]-90.)/(targets[j]-90.) for j in targets]
        self.assertAlmostEqual(min(fractions),max(fractions))

    def test_profile_carry_speed_is_optional_and_bounded(self):
        profile=load_profile()
        self.assertEqual(profile.carry_speed,10.)  # Default for profiles without the key.
        self.assertTrue(profile.holding(72.));self.assertFalse(profile.holding(108.))
        self.assertEqual((profile.move_speed(True),profile.move_speed(False)),(10.,20))
        for carry in [0,25,float('nan')]:
            data=copy.deepcopy(profile.values);data['carry_speed_deg_per_second']=carry
            with self.subTest(carry=carry),self.assertRaises(ValueError): ArmProfile(data)

    def test_bridge_recall_uses_carry_speed_while_holding(self):
        bridge,servo=self.bridge_with_servo(False)
        servo.nowPos[4]=72.  # Gripper near "closed": an object is held.
        info=bridge.move_to_pose('center_carry');bridge.stop_pose_move()
        self.assertEqual(info['speed_deg_per_second'],10.)
        servo.nowPos[4]=110.
        info=bridge.move_to_pose('center_clear');bridge.stop_pose_move()
        self.assertEqual(info['speed_deg_per_second'],20)

    def test_gripper_slot_moves_only_the_gripper(self):
        self.assertEqual(pose_targets(load_profile(),'gripper_closed'),{'gripper':70})
        with self.assertRaises(ValueError): pose_targets(load_profile(),'steering')

    def bridge_with_servo(self, auto):
        bridge=test_approach.OfficialConnectionTests().load_bridge()
        bridge.config=replace(bridge.config,auto_approach_enabled=auto)
        servo=SimpleNamespace(nowPos=[90.]*8)
        servo.setPWM=lambda channel,angle:servo.nowPos.__setitem__(channel,angle)
        bridge._servo=servo
        return bridge,servo

    def test_bridge_refuses_recall_in_automatic_mode(self):
        bridge,_=self.bridge_with_servo(True)
        with self.assertRaises(ValueError): bridge.move_to_pose('center_carry')

    def test_bridge_recall_reaches_pose_and_manual_button_cancels(self):
        import time
        bridge,servo=self.bridge_with_servo(False)
        bridge.move_to_pose('center_carry')
        bridge._pose_move['thread'].join(3)
        self.assertEqual(bridge.pose_move_status()['state'],'DONE')
        self.assertEqual(servo.nowPos[1:4],[90,80,100])
        bridge.move_to_pose('left_pick');time.sleep(.1)
        self.assertTrue(bridge.allow_command('armup'))
        self.assertEqual(bridge.pose_move_status()['state'],'STOPPED')
        self.assertNotEqual(servo.nowPos[1],55)


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
        # First move: 20 deg at 20 deg/s on the eased path takes 1.875 s, so the
        # 1 s timeout counts from there.
        with self.assertRaisesRegex(RuntimeError,'STEP_TIMEOUT'): s.tick(3.)
        self.assertEqual(s.state,'ERROR')
        self.assertGreater(driver.holds,1)

    def test_carry_steps_use_slow_eased_speed(self):
        driver=FakeArm();s=ArmSequence(driver,calibrated_profile());s.start('LEFT',0.)
        fastest={True:0.,False:0.}
        for tick in range(1,20000):
            before=driver.read_angles();step=s.steps[s.index]
            closed=step['gripper']=='closed' or s.holding
            s.tick(tick*.02)
            moved=max(abs(driver.angles[j]-before[j]) for j in JOINTS)
            fastest[closed]=max(fastest[closed],moved/.02)
            if s.state=='COMPLETE': break
        self.assertEqual(s.state,'COMPLETE')
        self.assertLessEqual(fastest[True],10+1e-6)   # carry_speed while holding
        self.assertLessEqual(fastest[False],20+1e-6)  # speed_deg_per_second otherwise
        self.assertGreater(fastest[False],10)


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
