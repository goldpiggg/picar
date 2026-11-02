"""Four-position persistence and real/preview isolation; no hardware imports."""
from dataclasses import replace
import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from flask import Flask
from picar_mission.arm_positions import COMMON, POSITIONS, POSES, PositionStore
from picar_mission.position_routes import register_position_routes
from scripts.preview_positions import PreviewPositions

ROOT = Path(__file__).resolve().parent.parent
ANGLES = dict(base=70., shoulder=80., elbow=100., gripper=110.)


class PositionStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root/'arm_positions.json'
        self.store = PositionStore(self.path)

    def test_missing_file_has_exactly_four_positions_and_no_taught_values(self):
        profile = self.store.read()
        self.assertEqual(set(profile['positions']), set(POSITIONS))
        self.assertEqual(profile['reference'], 'facing_start_qr')
        self.assertFalse(profile['calibration_confirmed'])
        self.assertTrue(all(set(slots)==set(POSES) for slots in profile['positions'].values()))
        self.assertTrue(all(v is None for slots in profile['positions'].values() for v in slots.values()))
        self.assertFalse(self.path.exists())

    def test_all_twelve_poses_persist_independently_and_leave_legacy_bytes_unchanged(self):
        legacy = self.root/'arm_poses.json'
        config = self.root/'mission_config.json'
        legacy.write_bytes(b'old taught angles\n')
        config.write_bytes(b'original config\n')
        for index, position in enumerate(POSITIONS):
            for n, pose in enumerate(POSES):
                self.store.save(position, pose, {**ANGLES, 'base':40+index*20+n})
        reread = PositionStore(self.path)
        for index, position in enumerate(POSITIONS):
            for n, pose in enumerate(POSES):
                targets = reread.targets(position, pose)
                self.assertEqual(targets, dict(base=40+index*20+n, shoulder=80., elbow=100.))
                self.assertNotIn('gripper', targets)
        self.assertEqual(legacy.read_bytes(), b'old taught angles\n')
        self.assertEqual(config.read_bytes(), b'original config\n')
        self.assertFalse(reread.read()['calibration_confirmed'])

    def test_common_gripper_stores_only_gripper_and_central_pose_preserves_grip(self):
        for name in COMMON:
            self.store.save('COMMON',name,ANGLES)
            self.assertEqual(set(self.store.targets('COMMON',name)),
                {'gripper'} if name.startswith('gripper_') else {'base','shoulder','elbow'})

    def test_unstored_or_unknown_positions_cannot_move(self):
        with self.assertRaisesRegex(ValueError,'아직 저장'):
            self.store.targets('RIGHT-A','pick')
        for position, pose in [('LEFT-C','pick'),('../arm_poses','pick'),('LEFT-A','unknown'),('COMMON','pick')]:
            with self.assertRaises(ValueError): self.store.save(position,pose,ANGLES)
        self.assertFalse(self.path.exists())

    def test_rejected_angles_do_not_change_any_saved_file_or_create_backup(self):
        self.store.save('LEFT-A','approach',ANGLES)
        before = self.path.read_bytes()
        for value in [190,-1,True,float('nan'),float('inf'),'90']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.store.save('LEFT-A','pick',{**ANGLES,'base':value})
        self.assertEqual(self.path.read_bytes(),before)
        self.assertFalse(self.path.with_name(self.path.name+'.before-edit').exists())
        restricted = PositionStore(self.path,limits={joint:[60,120] for joint in ANGLES})
        with self.assertRaises(ValueError): restricted.save('LEFT-A','pick',{**ANGLES,'base':50})

    def test_first_saved_file_backup_is_retained_and_records_are_copied(self):
        self.store.save('LEFT-A','approach',ANGLES)
        before = self.path.read_bytes()
        self.store.save('RIGHT-B','place',{**ANGLES,'base':120})
        self.store.save('LEFT-A','approach',{**ANGLES,'base':60})
        self.assertEqual(self.path.with_name(self.path.name+'.before-edit').read_bytes(),before)
        target = self.store.targets('RIGHT-B','place')
        target['base'] = 0
        self.assertEqual(self.store.targets('RIGHT-B','place')['base'],120)


class PositionHttpTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.backend = PreviewPositions(Path(self.temp.name)/'preview-only.json')
        app = Flask(__name__)
        register_position_routes(app,self.backend,str(ROOT/'picar_mission'))
        self.client = app.test_client()

    def test_page_assets_and_untaught_api(self):
        for url in ['/poses','/positions/assets/positions.css','/positions/assets/positions.js']:
            response = self.client.get(url)
            self.assertEqual(response.status_code,200)
            response.close()
        self.assertEqual(self.client.get('/positions/assets/arm_positions.py').status_code,404)
        data = self.client.get('/positions/data').json
        self.assertTrue(data['preview'])
        self.assertTrue(data['manual'])
        self.assertEqual(set(data['profile']['positions']),set(POSITIONS))

    def test_preview_save_and_recall_require_taught_slot_and_current_control_state(self):
        data = self.client.get('/positions/data').json
        payload = dict(position='LEFT-A',pose='pick',expected_revision=data['control_revision'],session_id=data['session_id'])
        self.assertEqual(self.client.post('/positions/move',json=payload).status_code,400)
        self.backend.set_angles(ANGLES)
        response = self.client.post('/positions/save',json={'position':'LEFT-A','pose':'pick','angles':{'base':0}})
        self.assertEqual(response.status_code,200)
        self.assertEqual(response.json['saved']['angles_deg']['base'],70)
        data = self.client.get('/positions/data').json
        payload.update(expected_revision=data['control_revision'])
        self.assertEqual(self.client.post('/positions/move',json=payload).status_code,200)
        self.assertEqual(self.client.post('/positions/save',json={'position':'LEFT-A','pose':'place'}).status_code,400)
        self.assertEqual(self.client.post('/positions/stop',json={}).status_code,200)
        self.assertEqual(self.client.post('/positions/move',json=payload).status_code,400)
        self.assertEqual(self.client.get('/positions/data').json['move']['state'],'STOPPED')

    def test_invalid_and_cross_origin_posts_cannot_write_or_move(self):
        for route in ['save','move','stop']:
            with self.subTest(route=route):
                response = self.client.post('/positions/'+route,json={},headers={'Origin':'https://unrelated.example'})
                self.assertEqual(response.status_code,403)
                self.assertEqual(self.client.post('/positions/'+route,json=[]).status_code,400)
                self.assertEqual(self.client.post('/positions/'+route,data='{}').status_code,400)
        self.assertFalse(self.backend.store.path.exists())
        self.assertIsNone(self.backend.move)


class RealBridgePositionTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('position_bridge',ROOT/'Server/mission_bridge.py')
        self.bridge = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.bridge)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = PositionStore(Path(self.temp.name)/'actual-store.json')
        self.bridge._position_store = lambda:self.store
        self.bridge.config = replace(self.bridge.config,auto_approach_enabled=False)
        self.commands = []
        self.servo = SimpleNamespace(nowPos=[90.]*8,stopWiggle=lambda:self.commands.append('hold'))
        def write(channel,angle):
            self.commands.append((channel,angle))
            self.servo.nowPos[channel] = angle
        self.servo.setPWM = write
        self.bridge._servo = self.servo
        self.addCleanup(self.bridge.stop_position_move)

    def test_production_uses_actual_command_angles_and_does_not_activate_mission(self):
        self.servo.nowPos[1] = 123
        self.bridge.save_position('RIGHT-A','pick')
        self.assertEqual(self.store.targets('RIGHT-A','pick')['base'],123)
        data = self.bridge.position_data()
        self.assertFalse(data['preview'])
        self.assertFalse(data['profile']['calibration_confirmed'])
        self.assertFalse(self.bridge._mission_started)

    def test_auto_mode_rejects_saving_and_moving_without_any_servo_commands(self):
        self.bridge.config = replace(self.bridge.config,auto_approach_enabled=True)
        with self.assertRaisesRegex(ValueError,'수동 저장'): self.bridge.save_position('LEFT-A','pick')
        with self.assertRaisesRegex(ValueError,'수동 저장'): self.bridge.move_to_position('LEFT-A','pick')
        self.assertFalse(self.store.path.exists())
        self.assertEqual(self.commands,[])

    def test_stop_prevents_delayed_recall_and_previous_server_session_cannot_move(self):
        self.store.save('RIGHT-B','pick',ANGLES)
        revision = self.bridge._control_revision
        self.bridge.stop_position_move()
        for rev,session in [(revision,self.bridge.SESSION_ID),(self.bridge._control_revision,'previous-server')]:
            with self.assertRaisesRegex(ValueError,'제어 상태'):
                self.bridge.move_to_position('RIGHT-B','pick',rev,session)
        self.assertEqual(self.commands,['hold'])

    def test_saved_recall_moves_only_arm_joints_and_holds_gripper(self):
        self.store.save('LEFT-A','approach',dict(base=89.9,shoulder=89.9,elbow=89.9,gripper=20))
        data = self.bridge.position_data()
        self.bridge.move_to_position('LEFT-A','approach',data['control_revision'],data['session_id'])
        self.bridge._pose_move['thread'].join(2)
        self.assertEqual(self.bridge.pose_move_status()['state'],'DONE')
        self.assertEqual(self.servo.nowPos[0],90)
        self.assertEqual(self.servo.nowPos[4],90)
        self.assertIn('hold',self.commands)
        self.assertTrue(all(command[0] in (1,2,3) for command in self.commands if isinstance(command,tuple)))


if __name__ == '__main__': unittest.main()
