"""Same-page arm teaching with fake motors/servos and real worker lifecycle."""
from dataclasses import replace
import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from flask import Flask
from picar_mission.arm_positions import PositionStore
from picar_mission.position_routes import register_position_routes
from scripts.preview_positions import PreviewPositions

ROOT = Path(__file__).resolve().parent.parent


class JointControlTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('joint_bridge',ROOT/'Server/mission_bridge.py')
        self.bridge = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.bridge)
        self.bridge.config = replace(self.bridge.config,auto_approach_enabled=True,arm_enabled=False,
            web_start_required=True,heading_correction_enabled=False)
        self.original_config = self.bridge.config
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = PositionStore(Path(self.temp.name)/'arm_positions.json')
        self.bridge._position_store = lambda:self.store
        self.calls = []
        self.servo = SimpleNamespace(nowPos=[90.]*8,stopWiggle=lambda:self.calls.append(('hold',)))
        def write(channel,angle):
            self.calls.append(('servo',channel,angle))
            self.servo.nowPos[channel] = angle
        self.servo.setPWM = write
        self.servo.moveAngle = lambda *args:self.calls.append(('steer',*args))
        motor = SimpleNamespace(motorStop=lambda:self.calls.append(('motor_stop',)),
            move=lambda *args:self.calls.append(('drive',*args)))
        sensor = SimpleNamespace(read_sample=lambda:{'distance_cm':80.,'captured_at':0.})
        self.bridge.configure(motor,self.servo,sensor)
        self.addCleanup(self.bridge.shutdown)
        self.calls.clear()

    def mode(self,enabled):
        return self.bridge.set_position_teaching(enabled,self.bridge._control_revision,self.bridge.SESSION_ID)

    def joint(self,name,angle):
        return self.bridge.set_joint_angle(name,angle,self.bridge._control_revision,self.bridge.SESSION_ID)

    def test_teaching_stops_workers_and_reserves_wheels_without_changing_settings(self):
        old_controller, old_runtime = self.bridge.controller,self.bridge.runtime
        old_runtime.reader_factory = lambda:SimpleNamespace(detect=lambda frame:[])
        old_runtime.start()
        old_controller.start()
        self.bridge._mission_started = True
        state = self.mode(True)
        self.assertTrue(state['manual'])
        self.assertTrue(state['teaching_active'])
        self.assertFalse(old_controller.worker.is_alive())
        self.assertFalse(old_runtime.worker.is_alive())
        self.assertIn(('motor_stop',),self.calls)
        self.assertFalse(any(c[0] in ('servo','drive','steer') for c in self.calls))
        self.assertEqual(self.bridge.status()['state'],'ARM_TEACHING')
        self.assertFalse(self.bridge.status()['dashboard']['can_start'])  # No live camera yet.
        self.assertFalse(self.bridge.status()['automatic_motion_enabled'])
        self.assertTrue(self.bridge.automatic_enabled())  # Existing car controls stay reserved.
        self.assertFalse(self.bridge.allow_command('forward'))
        with self.assertRaisesRegex(ValueError,'카메라'): self.bridge.start_mission()
        self.assertIs(self.bridge.config,self.original_config)
        self.assertFalse(self.store.path.exists())

    def test_each_joint_moves_at_jog_speed_and_does_not_save_or_move_other_joints(self):
        self.mode(True)
        for joint,channel in self.bridge.ArmAdapter.channels.items():
            with self.subTest(joint=joint):
                before = list(self.servo.nowPos)
                self.calls.clear()
                data = self.joint(joint,90.2)
                self.bridge._pose_move['thread'].join(2)
                self.assertEqual(self.bridge.pose_move_status()['state'],'DONE')
                self.assertLessEqual(data['move']['speed_deg_per_second'],self.bridge.config.arm_jog_speed)
                self.assertEqual(self.servo.nowPos[channel],90.2)
                self.assertTrue(all(i==channel or a==b for i,(a,b) in enumerate(zip(before,self.servo.nowPos))))
                self.assertTrue(all(c[1]==channel for c in self.calls if c[0]=='servo'))
                self.assertFalse(any(c[0] in ('steer','drive') for c in self.calls))
                self.assertFalse(self.store.path.exists())

    def test_mode_required_and_invalid_targets_do_not_write_or_change_revision(self):
        with self.assertRaisesRegex(ValueError,'수동 저장'): self.joint('base',91)
        self.mode(True)
        revision = self.bridge._control_revision
        self.calls.clear()
        for joint,angle in [('steering',90),('base',True),('base',float('nan')),('base',float('inf')),('base',190),('elbow',-1)]:
            with self.subTest(joint=joint,angle=angle), self.assertRaises(ValueError): self.joint(joint,angle)
        profile = self.bridge.load_motion_profile()
        profile.limits = {**profile.limits,'base':[80,100]}
        with patch.object(self.bridge,'load_motion_profile',return_value=profile):
            with self.assertRaises(ValueError): self.joint('base',101)
        self.assertEqual(self.bridge._control_revision,revision)
        self.assertEqual(self.calls,[])

    def test_stop_rejects_delayed_joint_and_mode_requests(self):
        self.mode(True)
        revision = self.bridge._control_revision
        self.bridge.stop_position_move()
        self.calls.clear()
        with self.assertRaises(ValueError): self.bridge.set_joint_angle('base',100,revision,self.bridge.SESSION_ID)
        with self.assertRaises(ValueError): self.bridge.set_position_teaching(False,revision,self.bridge.SESSION_ID)
        with self.assertRaises(ValueError): self.bridge.set_joint_angle('base',100,self.bridge._control_revision,'old-session')
        self.assertEqual(self.calls,[])

    def test_unresponsive_old_arm_worker_blocks_new_target_before_servo_writes(self):
        self.mode(True)
        stop = SimpleNamespace(set=lambda:None)
        worker = SimpleNamespace(is_alive=lambda:True,join=lambda seconds:None)
        self.bridge._pose_move = dict(stop=stop,thread=worker,info={'state':'MOVING'})
        self.calls.clear()
        with self.assertRaisesRegex(RuntimeError,'이전 팔 이동'): self.joint('base',91)
        self.assertEqual(self.calls,[])
        self.bridge._pose_move = None

    def test_new_target_retires_previous_joint_worker_and_exit_waits_for_start_button(self):
        self.mode(True)
        self.joint('base',150)
        old_worker = self.bridge._pose_move['thread']
        self.joint('shoulder',90.2)
        self.assertFalse(old_worker.is_alive())
        state = self.mode(False)
        self.assertFalse(state['manual'])
        self.assertFalse(state['teaching_active'])
        self.assertEqual(self.bridge.status()['state'],'READY')
        self.assertIsNone(self.bridge.controller.worker)
        self.assertIsNone(self.bridge.runtime.worker)
        self.assertFalse(any(c[0]=='drive' for c in self.calls))
        self.assertIs(self.bridge.config,self.original_config)


class JointHttpTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.backend = PreviewPositions(Path(self.temp.name)/'virtual.json')
        app = Flask(__name__)
        register_position_routes(app,self.backend,str(ROOT/'picar_mission'))
        self.client = app.test_client()

    def payload(self,**fields):
        data = self.client.get('/positions/data').json
        return dict(expected_revision=data['control_revision'],session_id=data['session_id'],**fields)

    def test_joint_route_controls_one_joint_and_mode_toggle_does_not_save(self):
        response = self.client.post('/positions/joint',json=self.payload(joint='base',angle=91))
        self.assertEqual(response.status_code,200)
        self.assertEqual(response.json['move']['targets'],{'base':91.})
        response = self.client.post('/positions/teaching',json=self.payload(enabled=False))
        self.assertEqual(response.status_code,200)
        self.assertFalse(response.json['manual'])
        self.assertEqual(self.client.post('/positions/joint',json=self.payload(joint='base',angle=92)).status_code,400)
        self.assertEqual(self.client.post('/positions/teaching',json=self.payload(enabled=True)).status_code,200)
        self.assertFalse(self.backend.store.path.exists())

    def test_joint_and_mode_routes_reject_cross_origin_bad_values_and_missing_revision(self):
        for route in ['joint','teaching']:
            self.assertEqual(self.client.post('/positions/'+route,json={},headers={'Origin':'https://unrelated.example'}).status_code,403)
            self.assertEqual(self.client.post('/positions/'+route,json=[]).status_code,400)
        self.assertEqual(self.client.post('/positions/joint',json={'joint':'base','angle':91}).status_code,400)
        self.assertEqual(self.client.post('/positions/teaching',json=self.payload(enabled=1)).status_code,400)
        self.assertEqual(self.client.post('/positions/joint',json=self.payload(joint='steering',angle=91)).status_code,400)
        self.assertFalse(self.backend.store.path.exists())


if __name__ == '__main__': unittest.main()
