"""Start reliably with noisy startup echoes, without bypassing motion checks."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from picar_mission.config import MissionConfig, load_config
from picar_mission.runtime import MissionRuntime
import test_approach
from test_approach import Harness


class RelaxedStartTests(unittest.TestCase):
    def test_repository_and_fallback_settings_agree(self):
        self.assertEqual(load_config(),MissionConfig())
        cfg=load_config()
        self.assertEqual((cfg.required_frames,cfg.hold_seconds),(1,0.))
        self.assertEqual(cfg.direct_speed,40)
        self.assertEqual((cfg.target_distance_cm,cfg.stop_tolerance_cm),(30.,1.))
        self.assertEqual(cfg.distance_max_age_seconds,.25)
        self.assertFalse(cfg.arm_enabled)

    def test_two_matching_frames_with_slow_decode_confirm_direction(self):
        now=[0.]
        class Reader:
            def detect(self, frame):
                now[0]+=1.
                return [{'text':frame}]
        runtime=MissionRuntime(replace(load_config(),required_frames=2,hold_seconds=.15),clock=lambda:now[0])
        self.assertIsNone(runtime.process_frame('LEFT',0.,1,Reader()))
        now[0]=1.1
        self.assertEqual(runtime.process_frame('LEFT',1.1,2,Reader()),'LEFT')
        self.assertEqual(runtime.snapshot()['first_direction'],'LEFT')

    def test_startup_echo_failures_recover_before_any_motion(self):
        h=Harness()
        for value in [None,float('nan'),200.]:
            self.assertEqual(h.echo(value,.2),'CHECK_DISTANCE')
            self.assertIsNotNone(h.controller.snapshot()['distance_wait_reason'])
            self.assertNotIn('forward',[command[0] for command in h.drive.commands])
        h.echo(80.);h.echo(80.)
        self.assertEqual(h.controller.snapshot()['state'],'CHECK_DISTANCE')
        self.assertEqual(h.echo(80.),'APPROACH')
        self.assertIsNone(h.controller.snapshot()['distance_wait_reason'])
        self.assertGreater(h.controller.snapshot()['command_speed'],0)

    def test_permanent_bad_sensor_never_starts(self):
        h=Harness()
        for _ in range(55): h.echo(None,.1)
        self.assertEqual(h.controller.snapshot()['error'],'SENSOR_WARMUP_TIMEOUT')
        self.assertNotIn('forward',[command[0] for command in h.drive.commands])

    def test_camera_loss_after_confirmation_does_not_replace_distance_check(self):
        h=Harness();h.start()
        h.qr.update(camera_fresh=False,state='ERROR',error='video lost')
        h.echo(79.)
        self.assertEqual(h.controller.snapshot()['state'],'APPROACH')
        h.echo(None)
        self.assertEqual(h.controller.snapshot()['state'],'ERROR')
        self.assertEqual(h.drive.commands[-1],('stop',0))

    def test_camera_error_before_confirmation_still_prevents_start(self):
        h=Harness(None);h.qr.update(state='ERROR',error='no camera')
        h.echo(80.)
        self.assertEqual(h.controller.snapshot()['state'],'ERROR')
        self.assertNotIn('forward',[command[0] for command in h.drive.commands])

    def test_releasing_disabled_arrows_while_waiting_does_not_cancel(self):
        b=test_approach.OfficialConnectionTests().load_bridge()
        cancelled=[]
        b.controller=SimpleNamespace(snapshot=lambda:dict(state='WAIT_QR'),
                                     cancel=cancelled.append)
        for command in ['forward','DS','left','TS','armup','armstop']:
            self.assertFalse(b.allow_command(command))
        self.assertFalse(b._cancelled)
        self.assertEqual(cancelled,[])
        b.allow_command('stop')
        self.assertEqual(cancelled,['USER_STOP'])

    def test_arrow_release_during_motion_still_stops(self):
        b=test_approach.OfficialConnectionTests().load_bridge()
        cancelled=[]
        b.controller=SimpleNamespace(snapshot=lambda:dict(state='APPROACH'),
                                     cancel=cancelled.append)
        b.allow_command('DS')
        self.assertTrue(b._cancelled)
        self.assertEqual(cancelled,['USER_STOP'])


if __name__=='__main__': unittest.main()
