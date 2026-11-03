"""Production preparation and compatibility checks; only fake hardware."""
import copy
from dataclasses import replace
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from picar_mission.arm_sequence import PEAK_SPEED_RATIO, ArmProfile, load_motion_profile, load_profile
from picar_mission.config import load_config
from picar_mission.qr_prepare import QrPreparation
from test_arm import FakeArm
from test_dashboard import load_bridge


class ElbowTiltTests(unittest.TestCase):
    def test_elbow_moves_with_base_and_shoulder_and_gripper_stay(self):
        driver=FakeArm()
        driver.angles.update(shoulder=81.,elbow=103.,gripper=145.)
        preparation=QrPreparation(driver,load_profile(),{'LEFT':180.,'RIGHT':0.},20.,elbow_offset=-30.)
        preparation.start('LEFT',0.)
        for t in range(1,200):
            if preparation.tick(t*.1)=='READY': break
        self.assertEqual(preparation.state,'READY')
        self.assertEqual(driver.angles,dict(base=180.,shoulder=81.,elbow=73.,gripper=145.))
        self.assertEqual({joint for joint,angle in driver.writes},{'base','elbow'})

    def test_elbow_target_outside_limits_never_moves(self):
        driver=FakeArm()
        driver.angles['elbow']=10.
        preparation=QrPreparation(driver,load_profile(),{'LEFT':180.,'RIGHT':0.},20.,elbow_offset=-30.)
        with self.assertRaisesRegex(ValueError,'QR_ELBOW_TARGET_OUT_OF_LIMITS'):
            preparation.start('LEFT',0.)
        self.assertEqual(driver.writes,[])

    def test_shipped_tilt_lowers_the_elbow_twenty_degrees(self):
        self.assertEqual(load_config().qr_prepare_elbow_offset_deg,-20.)


class PreparationSettingsTests(unittest.TestCase):
    def test_defaults_look_between_the_two_boxes(self):
        cfg=load_config()
        self.assertEqual((cfg.qr_prepare_left_base_deg,cfg.qr_prepare_right_base_deg),(165.,7.))
        self.assertTrue(cfg.arm_enabled)

    def test_invalid_turn_targets_and_delay_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'settings.json'
            for key in ['qr_prepare_left_base_deg','qr_prepare_right_base_deg']:
                for value in [True,'90',float('nan'),float('inf'),-1,190]:
                    with self.subTest(key=key,value=value),self.assertRaises(ValueError):
                        path.write_text(json.dumps({key:value}),encoding='utf-8')
                        load_config(path)
            path.write_text(json.dumps({'arm_enabled':False,'arm_start_delay_seconds':-1}),encoding='utf-8')
            with self.assertRaises(ValueError): load_config(path)

    def test_legacy_taught_pose_names_do_not_block_base_preparation_or_get_rewritten(self):
        values=copy.deepcopy(load_profile().values)
        values['positions']={'left_pick':{'base':42,'shoulder':81,'elbow':103}}
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'arm_poses.json'
            original=json.dumps(values).encode('utf-8')
            path.write_bytes(original)
            profile=load_motion_profile(path)
            driver=FakeArm()
            preparation=QrPreparation(driver,profile,{'LEFT':0.,'RIGHT':180.},10.)
            preparation.start('LEFT',0.)
            for tick in range(1,200): preparation.tick(tick*.1)
            self.assertEqual(preparation.state,'READY')
            self.assertEqual(driver.angles['base'],0.)
            self.assertEqual({joint for joint,value in driver.writes},{'base'})
            self.assertEqual(path.read_bytes(),original)
            with self.assertRaisesRegex(ValueError,'reteach'): ArmProfile(values)


class ProductionPreparationTests(unittest.TestCase):
    def test_existing_speed_setting_applies_at_turn_start_without_retiming_a_running_turn(self):
        bridge=load_bridge()
        self.addCleanup(bridge.shutdown)
        servo=SimpleNamespace(nowPos=[90.]*8,stopWiggle=lambda:None,moveAngle=lambda *args:None,
            setPWM=lambda *args:None)
        bridge.configure(SimpleNamespace(motorStop=lambda:None),servo,SimpleNamespace(read_sample=lambda:None))
        preparation=bridge.controller.qr_preparation
        with tempfile.TemporaryDirectory() as directory:
            bridge.ARM_SPEED_FILE=Path(directory)/'speed.json'
            bridge.set_arm_speed(150)
            preparation.start('RIGHT',0.)
            expected_speed=min(bridge.arm_jog_speed(),preparation.profile.move_speed(True))
            self.assertAlmostEqual(preparation.move.motion.duration,PEAK_SPEED_RATIO*abs(90-preparation.target)/expected_speed)
            duration=preparation.move.motion.duration
            bridge.set_arm_speed(50)
            self.assertEqual(preparation.move.motion.duration,duration)
            self.assertEqual(preparation.profile.speed_scale,.5)

    def test_web_controller_turns_base_with_transfer_disabled_and_waits(self):
        bridge=load_bridge()
        self.addCleanup(bridge.shutdown)
        bridge.config=replace(bridge.config,arm_enabled=False,direct_steer_preload_seconds=0.,
            direct_steer_settle_seconds=0.,arm_start_delay_seconds=1.,fast_mission_enabled=False)
        now=[0.]
        distance=[80.]
        writes=[]
        servo=SimpleNamespace(nowPos=[90.,90.,81.,103.,145.,90.,90.,90.],stopWiggle=lambda:None,
            moveAngle=lambda *args:None)
        def write(channel,angle):
            writes.append((channel,angle))
            servo.nowPos[channel]=angle
        servo.setPWM=write
        wheels=[]
        move=SimpleNamespace(motorStop=lambda:wheels.append(('stop',0)),
            move=lambda *args:wheels.append(('drive',args)))
        sensor=SimpleNamespace(read_sample=lambda:dict(distance_cm=distance[0],captured_at=now[0]))
        with patch.object(bridge,'load_profile',side_effect=AssertionError('No transfer profile should load')):
            bridge.configure(move,servo,sensor)
        bridge._mission_started=True
        bridge.runtime.data.update(first_direction='RIGHT',state='DIRECTION_CONFIRMED')
        controller=bridge.controller
        controller.view_scan_enabled=False  # Initial target test; search is covered separately.
        controller.clock=lambda:now[0]
        def tick(value,dt=.1):
            distance[0]=value
            now[0]+=dt
            controller.step()
        tick(80.)
        for _ in range(3): tick(36.)
        tick(30.,.4)
        for _ in range(5): tick(30.)
        self.assertEqual(controller.snapshot()['state'],'ARM_WAIT')
        self.assertEqual(writes,[])
        self.assertTrue(controller.drive.locked)
        last_wheel=len(wheels)
        def forbidden(): raise AssertionError('No post-stop sensor reads')
        controller.read_sample=forbidden
        for _ in range(250): tick(None)
        status=bridge.status()
        self.assertEqual(status['state'],'SIDE_QR_WAIT')
        self.assertEqual(status['dashboard']['arm_mode'],'sequence_preview')
        self.assertFalse(status['dashboard']['finished'])
        self.assertEqual(status['dashboard']['remaining_cm'],0.)
        self.assertEqual(status['dashboard']['distance_source'],'FINAL')
        # RIGHT turns BASE down to 0 and lowers the elbow 20 degrees to see both codes.
        self.assertEqual(servo.nowPos[1:5],[7.,81.,83.,145.])
        self.assertEqual({channel for channel,angle in writes},{1,3})
        self.assertEqual(status['qr_preparation']['elbow_target_deg'],83.)
        self.assertIsNone(controller.arm)
        self.assertTrue(all(command[0]=='stop' for command in wheels[last_wheel:]))
        self.assertEqual(bridge.runtime.snapshot()['state'],'WAIT_SIDE_QR')


if __name__=='__main__': unittest.main()
