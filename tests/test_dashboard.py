"""Web controls with fake drivers; never initializes GPIO or runs a shell."""
from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
import threading
import time
import json
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import flask
import flask_cors

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))


def load_bridge():
    spec = importlib.util.spec_from_file_location('dashboard_bridge',ROOT/'Server/mission_bridge.py')
    bridge = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bridge)
    bridge.config = replace(bridge.config,heading_correction_enabled=False)
    return bridge


class Reader:
    def detect(self,frame):
        return [{'text':'LEFT'}]


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.bridge = load_bridge()
        self.commands = []
        self.distance = 80.
        self.servo = SimpleNamespace(nowPos=[90.]*8,stopWiggle=lambda:self.commands.append('hold'),
            moveAngle=lambda *args:self.commands.append(('steer',args)),
            setPWM=lambda *args:self.commands.append(('arm',args)))
        move = SimpleNamespace(motorStop=lambda:self.commands.append('stop'),
            move=lambda *args:self.commands.append(('drive',args)))
        sensor = SimpleNamespace(read_sample=lambda:dict(distance_cm=self.distance,captured_at=time.monotonic()))
        self.bridge.configure(move,self.servo,sensor)
        self.bridge._camera_seen_at = time.monotonic()
        self.bridge.runtime.reader_factory = Reader

    def tearDown(self):
        self.bridge.shutdown()

    def start(self):
        from picar_mission.runtime import MissionRuntime
        with patch.object(self.bridge,'MissionRuntime',lambda cfg:MissionRuntime(cfg,reader_factory=Reader)):
            return self.bridge.start_mission()

    def test_boot_waits_for_button_and_ignores_visible_qr(self):
        class Frame:
            def copy(self): raise AssertionError('QR must not be queued before start')
        self.bridge.submit_frame(Frame())
        self.bridge.controller.step()
        state = self.bridge.status()
        self.assertEqual(state['dashboard']['state'],'READY')
        self.assertTrue(state['dashboard']['can_start'])
        self.assertIsNone(self.bridge.controller.worker)
        self.assertIsNone(state['first_direction'])
        self.assertFalse(any(isinstance(c,tuple) and c[0]=='drive' for c in self.commands))

    def test_button_starts_and_double_start_keeps_same_workers(self):
        self.start()
        previous = self.bridge.controller
        qr = self.bridge.runtime
        second = self.start()
        self.assertIs(previous,self.bridge.controller)
        self.assertIs(qr,self.bridge.runtime)
        self.assertEqual(second['dashboard']['attempt'],1)
        self.assertEqual(second['dashboard']['state'],'WAIT_QR')

    def test_fresh_qr_after_button_reaches_drive_and_stop(self):
        self.bridge.config = replace(self.bridge.config,direct_steer_preload_seconds=0.,direct_steer_settle_seconds=0.)
        self.start()
        class Frame:
            def copy(self): return self
        self.bridge.submit_frame(Frame())
        deadline = time.monotonic()+1.
        while time.monotonic()<deadline and not any(isinstance(c,tuple) and c[0]=='drive' for c in self.commands):
            time.sleep(.01)
        self.assertEqual(self.bridge.status()['first_direction'],'LEFT')
        self.assertTrue(any(isinstance(c,tuple) and c[0]=='drive' for c in self.commands))
        self.bridge.cancel('WEB_STOP')
        count = len(self.commands)
        time.sleep(.05)
        self.assertFalse(any(isinstance(c,tuple) and c[0]=='drive' for c in self.commands[count:]))

    def test_cancel_then_restart_clears_qr_timer_and_drive_lock(self):
        self.start()
        old = self.bridge.controller
        old_qr = self.bridge.runtime
        old_qr.data.update(first_direction='LEFT',state='DIRECTION_CONFIRMED')
        old.drive.locked = True
        self.bridge.cancel('WEB_STOP')
        first_time = self.bridge.status()['dashboard']['elapsed_seconds']
        time.sleep(.03)
        self.assertEqual(self.bridge.status()['dashboard']['elapsed_seconds'],first_time)
        state = self.start()
        self.assertFalse(old.worker.is_alive())
        self.assertFalse(old_qr.worker.is_alive())
        self.assertIsNot(old,self.bridge.controller)
        self.assertIsNone(state['first_direction'])
        self.assertIsNone(state['qr']['first_direction'])
        self.assertFalse(self.bridge.controller.drive.locked)
        self.assertEqual(state['dashboard']['attempt'],2)
        self.assertLess(state['dashboard']['elapsed_seconds'],.5)

    def test_concurrent_start_requests_create_one_attempt(self):
        barrier = threading.Barrier(3)
        errors = []
        def launch():
            try:
                barrier.wait()
                self.bridge.start_mission()
            except Exception as exc: errors.append(exc)
        from picar_mission.runtime import MissionRuntime
        with patch.object(self.bridge,'MissionRuntime',lambda cfg:MissionRuntime(cfg,reader_factory=Reader)):
            threads = [threading.Thread(target=launch) for _ in range(2)]
            for thread in threads: thread.start()
            barrier.wait()
            for thread in threads: thread.join(3)
        self.assertEqual(errors,[])
        self.assertTrue(all(not t.is_alive() for t in threads))
        self.assertEqual(self.bridge._attempt,1)

    def test_stale_distances_never_appear_as_live(self):
        self.bridge._ultrasound = SimpleNamespace(read_sample=lambda:dict(distance_cm=90.,captured_at=time.monotonic()-2))
        info = self.bridge.status()['dashboard']
        self.assertIsNone(info['distance_cm'])
        self.assertIsNone(info['remaining_cm'])
        self.assertEqual(self.bridge.status()['dashboard']['target_distance_cm'],30.)

    def test_live_distance_and_remaining_before_start(self):
        info = self.bridge.status()['dashboard']
        self.assertEqual((info['distance_cm'],info['remaining_cm'],info['distance_source']),(80.,50.,'LIVE'))

    def test_target_stop_freezes_final_distance_without_reading_sensor(self):
        self.bridge.controller.data.update(target_stop_latched=True,final_distance_cm=30.4)
        def forbidden(): raise AssertionError('Arm may have rotated the sensor')
        self.bridge._ultrasound = SimpleNamespace(read_sample=forbidden)
        info = self.bridge.status()['dashboard']
        self.assertEqual((info['distance_cm'],info['distance_source']),(30.4,'FINAL'))

    def test_camera_failure_and_manual_mode_reject_start(self):
        self.bridge._camera_seen_at = time.monotonic()-5
        self.assertFalse(self.bridge.status()['dashboard']['can_start'])
        with self.assertRaisesRegex(ValueError,'카메라'): self.start()
        self.bridge.config = replace(self.bridge.config,auto_approach_enabled=False)
        self.assertEqual(self.bridge.status()['dashboard']['state'],'DISABLED')
        with self.assertRaisesRegex(ValueError,'수동'): self.start()

    def test_camera_recovers_health_without_automatically_starting_mission(self):
        self.bridge.camera_capture_started()
        self.bridge.camera_failed('USB camera frame read failed')
        status = self.bridge.status()['dashboard']
        self.assertFalse(status['camera_online'])
        self.assertFalse(status['can_start'])
        self.assertEqual(status['camera_error'],'USB camera frame read failed')
        with self.assertRaisesRegex(ValueError,'카메라'): self.start()
        self.bridge.camera_capture_started()
        self.bridge.submit_frame(object())  # Before start: no QR queue or motion.
        status = self.bridge.status()['dashboard']
        self.assertEqual(status['camera_generation'],2)
        self.assertTrue(status['camera_online'])
        self.assertIsNone(status['camera_error'])
        self.assertTrue(status['can_start'])
        self.assertIsNone(self.bridge.controller.worker)
        self.assertIsNone(self.bridge.runtime.worker)
        self.assertFalse(any(isinstance(c,tuple) and c[0]=='drive' for c in self.commands))

    def test_entire_stop_includes_manual_pose_worker(self):
        stopped = threading.Event()
        thread = threading.Thread(target=lambda:stopped.wait(3),daemon=True)
        thread.start()
        self.bridge._pose_move = dict(stop=stopped,thread=thread,info={})
        self.bridge.cancel('WEB_STOP')
        self.assertFalse(thread.is_alive())
        self.assertTrue(stopped.is_set())

    def test_stop_invalidates_an_older_start_request(self):
        revision = self.bridge.status()['dashboard']['control_revision']
        self.bridge.cancel('WEB_STOP')
        with self.assertRaisesRegex(ValueError,'제어 상태'):
            self.bridge.start_mission(revision)
        self.assertEqual(self.bridge._attempt,0)

    def test_arm_speed_scales_poses_carry_and_buttons_and_persists(self):
        with tempfile.TemporaryDirectory() as folder:
            self.bridge.ARM_SPEED_FILE = Path(folder)/'arm_speed.json'
            speed = self.bridge.status()['dashboard']['arm_speed']
            self.assertEqual((speed['percent'],speed['move_deg_per_second'],speed['carry_deg_per_second'],
                              speed['jog_deg_per_second']),(100,40.,20.,20))
            speed = self.bridge.set_arm_speed(150)
            self.assertEqual((speed['move_deg_per_second'],speed['carry_deg_per_second'],speed['jog_deg_per_second']),(60.,30.,30))
            self.assertEqual(self.bridge._load_profile().move_speed(False),60.)
            self.assertEqual(json.loads(self.bridge.ARM_SPEED_FILE.read_text(encoding='utf-8')),{'percent':150})
            self.assertEqual(self.bridge._read_arm_speed(),150)
            speed = self.bridge.set_arm_speed(300)
            # Capped at the profile limit (120 deg/s) and the jog limit (60 deg/s).
            self.assertEqual((speed['move_deg_per_second'],speed['carry_deg_per_second'],speed['jog_deg_per_second']),(120.,60.,60))
            for bad in [0,20,310,110,100.,True,'100',None]:
                with self.subTest(bad=bad),self.assertRaises(ValueError): self.bridge.set_arm_speed(bad)
            self.assertEqual(self.bridge.arm_speed_status()['percent'],300)
            self.bridge.ARM_SPEED_FILE.write_text('{"percent":999}',encoding='utf-8')
            self.assertEqual(self.bridge._read_arm_speed(),100)

    def test_arm_speed_change_reaches_a_running_arm_sequence(self):
        arm = SimpleNamespace(profile=self.bridge._load_profile())
        with tempfile.TemporaryDirectory() as folder,patch.object(self.bridge.controller,'arm',arm):
            self.bridge.ARM_SPEED_FILE = Path(folder)/'arm_speed.json'
            self.bridge.set_arm_speed(50)
        self.assertEqual((arm.profile.move_speed(False),arm.profile.move_speed(True)),(20.,10.))

    def test_cancel_during_pulse_does_not_busy_spin(self):
        controller = self.bridge.controller
        controller.pulse_until = time.monotonic()-1
        controller.cancel()
        self.assertGreater(controller._wait_seconds(),0.)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.bridge = load_bridge()
        camera = SimpleNamespace(Camera=lambda **kwargs:object())
        spec = importlib.util.spec_from_file_location('dashboard_app',ROOT/'Server/app.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'mission_bridge':self.bridge,'camera_opencv':camera}):
            spec.loader.exec_module(module)
        self.client = module.app.test_client()

    def test_page_and_assets_are_served_without_hardware(self):
        response = self.client.get('/dashboard')
        self.assertEqual(response.status_code,200)
        self.assertIn('미션 컨트롤',response.get_data(as_text=True))
        self.assertIn('no-store',response.headers['Cache-Control'])
        response.close()
        for name in ['dashboard.css','dashboard.js']:
            response = self.client.get('/dashboard/assets/'+name)
            self.assertEqual(response.status_code,200)
            response.close()
        self.assertEqual(self.client.get('/dashboard/assets/config.py').status_code,404)

    def test_four_position_page_keeps_legacy_teaching_page_available(self):
        for url, marker in [('/poses','positions.js'),('/poses/legacy','left_top_above')]:
            response = self.client.get(url)
            self.assertEqual(response.status_code,200)
            self.assertIn(marker,response.get_data(as_text=True))
            response.close()
        self.assertFalse(self.client.get('/positions/data').json['preview'])

    def test_start_uses_bridge_and_returns_state(self):
        with patch.object(self.bridge,'start_mission',return_value={'state':'WAIT_QR'}) as launch:
            response = self.client.post('/dashboard/start',json={'expected_revision':0,'session_id':self.bridge.SESSION_ID})
        self.assertEqual(response.json,{'state':'WAIT_QR'})
        launch.assert_called_once_with(0)

    def test_cross_origin_and_form_cannot_start_motion(self):
        with patch.object(self.bridge,'start_mission') as launch:
            self.assertEqual(self.client.post('/dashboard/start',json={},headers={'Origin':'https://unrelated.example'}).status_code,403)
            self.assertEqual(self.client.post('/dashboard/start',data='start').status_code,415)
        launch.assert_not_called()

    def test_not_ready_is_visible_conflict(self):
        response = self.client.post('/dashboard/start',json={'expected_revision':0,'session_id':self.bridge.SESSION_ID})
        self.assertEqual(response.status_code,409)
        self.assertIn('준비',response.json['error'])

    def test_missing_revision_cannot_start(self):
        with patch.object(self.bridge,'start_mission') as launch:
            self.assertEqual(self.client.post('/dashboard/start',json={}).status_code,400)
            self.assertEqual(self.client.post('/dashboard/start',json=[]).status_code,400)
            self.assertEqual(self.client.post('/dashboard/start',json={'expected_revision':True}).status_code,400)
        launch.assert_not_called()

    def test_request_from_previous_server_session_cannot_start(self):
        with patch.object(self.bridge,'start_mission') as launch:
            self.assertEqual(self.client.post('/dashboard/start',json={'expected_revision':0,'session_id':'previous-boot'}).status_code,409)
        launch.assert_not_called()

    def test_arm_speed_route_validates_and_stays_same_origin(self):
        with patch.object(self.bridge,'set_arm_speed',return_value={'percent':150}) as change:
            response = self.client.post('/dashboard/arm-speed',json={'percent':150})
            self.assertEqual((response.status_code,response.json),(200,{'arm_speed':{'percent':150}}))
            self.assertEqual(self.client.post('/dashboard/arm-speed',json={'percent':150},headers={'Origin':'https://unrelated.example'}).status_code,403)
            self.assertEqual(self.client.post('/dashboard/arm-speed',data='150').status_code,415)
        change.assert_called_once_with(150)
        with tempfile.TemporaryDirectory() as folder:
            self.bridge.ARM_SPEED_FILE = Path(folder)/'arm_speed.json'
            response = self.client.post('/dashboard/arm-speed',json={'percent':999})
        self.assertEqual(response.status_code,400)
        self.assertIn('팔 속도',response.json['error'])

    def test_stop_uses_whole_mission_cancel(self):
        with patch.object(self.bridge,'cancel') as stop:
            response = self.client.post('/mission/stop')
        self.assertEqual(response.status_code,200)
        stop.assert_called_once_with('WEB_STOP')


class SetupTests(unittest.TestCase):
    def test_setup_preserves_taught_angles_speed_and_original_backup(self):
        from scripts.enable_dashboard import enable_dashboard
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root/'mission_config.json'
            arm = root/'arm_poses.json'
            original = {'target_distance_cm':34.,'pulse_seconds':.22,'arm_enabled':False,'custom_note':'keep'}
            config.write_text(json.dumps(original),encoding='utf-8')
            arm.write_text('{"my_calibration":true}',encoding='utf-8')
            before = arm.read_bytes()
            backup = enable_dashboard(config)
            enabled = json.loads(config.read_text(encoding='utf-8'))
            self.assertTrue(enabled['web_start_required'])
            self.assertTrue(enabled['auto_approach_enabled'])
            self.assertEqual({key:enabled[key] for key in original},original)
            self.assertEqual(arm.read_bytes(),before)
            enable_dashboard(config)
            self.assertEqual(json.loads(backup.read_text(encoding='utf-8')),original)


if __name__ == '__main__': unittest.main()
