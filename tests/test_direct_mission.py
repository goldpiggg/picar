"""Exercise the production QR -> forward -> raw 30cm stop path, without GPIO."""
from dataclasses import replace
import importlib.util
from pathlib import Path
import time
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

from picar_mission.config import MissionConfig, load_config
from picar_mission.direct_mission_controller import DirectMissionController
from picar_mission.runtime import MissionRuntime
import test_approach
import test_arm

ROOT=Path(__file__).resolve().parent.parent


class Harness:
    def __init__(self,direction='LEFT'):
        self.time=0.
        self.sequence=0
        self.sample=None
        self.qr=dict(state='DIRECTION_CONFIRMED',first_direction=direction,camera_fresh=False)
        self.drive=test_approach.Drive()
        self.controller=DirectMissionController(load_config(),self.drive,
            lambda:self.sample,lambda:self.qr,clock=lambda:self.time)

    def echo(self,distance,dt=.06):
        self.time+=dt
        self.sequence+=1
        self.sample=dict(distance_cm=distance,captured_at=self.time,sequence=self.sequence)
        self.controller.step()
        return self.controller.snapshot()


class DirectTests(unittest.TestCase):
    def test_first_good_distance_drives_without_warmup_or_frame_gate(self):
        for direction in ['LEFT','RIGHT']:
            h=Harness(direction)
            state=h.echo(80.)
            self.assertEqual(state['state'],'DRIVING')
            self.assertEqual(state['first_direction'],direction)
            self.assertEqual(h.drive.commands,[('stop',0),('steer',0),('forward',40)])

    def test_echo_published_during_snapshot_is_not_rejected_as_future(self):
        h=Harness()
        def latest():
            h.time+=.01
            return dict(distance_cm=80.,captured_at=h.time)
        h.controller.read_sample=latest
        h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'DRIVING')
        self.assertIn(('forward',40),h.drive.commands)

    def test_constant_forward_then_immediate_raw_stop(self):
        h=Harness()
        states=[h.echo(d)['state'] for d in [80.,50.,35.,30.1,30.]]
        self.assertEqual(states,['DRIVING']*4+['STOPPED_WAIT'])
        self.assertEqual([x for x in h.drive.commands if x[0]=='forward'],[('forward',40)])
        self.assertEqual(h.drive.commands[-1],('stop',0))
        self.assertTrue(h.controller.snapshot()['approach_complete'])

    def test_near_echo_stops_despite_large_distance_jump(self):
        h=Harness();h.echo(80.)
        state=h.echo(29.8)
        self.assertEqual(state['state'],'STOPPED_WAIT')
        self.assertIsNone(state['error'])
        self.assertEqual(state['stop_reason'],'TARGET_DISTANCE_REACHED')

    def test_already_at_target_never_drives(self):
        for distance in [30.,29.,20.]:
            h=Harness();state=h.echo(distance)
            self.assertEqual(state['state'],'STOPPED_WAIT')
            self.assertNotIn('forward',[x[0] for x in h.drive.commands])

    def test_no_qr_does_not_even_request_distance(self):
        h=Harness(None)
        def forbidden(): raise AssertionError('Must wait for QR')
        h.controller.read_sample=forbidden
        h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'WAIT_QR')
        self.assertEqual(h.drive.commands,[])

    def test_missing_bad_and_stale_sensor_never_drive_blind(self):
        for sample in [None,dict(distance_cm=None,captured_at=1.),
                       dict(distance_cm=float('nan'),captured_at=1.),
                       dict(distance_cm=200.,captured_at=1.),
                       dict(distance_cm=80.,captured_at=0.)]:
            h=Harness();h.time=1.;h.sample=sample;h.controller.step()
            self.assertEqual(h.controller.snapshot()['state'],'ERROR')
            self.assertNotIn('forward',[x[0] for x in h.drive.commands])

    def test_sensor_failure_after_forward_stops(self):
        h=Harness();h.echo(80.);state=h.echo(None)
        self.assertEqual(state['state'],'ERROR')
        self.assertEqual(h.drive.commands[-1],('stop',0))

    def test_half_second_sensor_budget_is_separate_from_legacy_filter(self):
        h=Harness();h.echo(80.);h.time+=.3;h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'DRIVING')
        h.time+=.21;h.controller.step()
        self.assertEqual(h.controller.snapshot()['error'],'STALE_DISTANCE_SAMPLE')
        self.assertEqual(h.drive.commands[-1],('stop',0))

    def test_no_progress_gate_and_bounded_total_runtime(self):
        h=Harness();h.echo(80.)
        self.assertEqual(h.echo(80.,dt=6.)['state'],'DRIVING')
        self.assertEqual(h.echo(80.,dt=35.)['error'],'APPROACH_TIMEOUT')
        self.assertEqual(h.drive.commands[-1],('stop',0))

    def test_saved_direction_survives_camera_loss_and_other_qr(self):
        h=Harness();h.echo(80.)
        h.qr.update(state='ERROR',first_direction='RIGHT',error='camera offline')
        self.assertEqual(h.echo(60.)['state'],'DRIVING')
        self.assertEqual(h.echo(30.)['first_direction'],'LEFT')

    def test_stop_and_cancel_do_not_restart(self):
        for cancel in [False,True]:
            h=Harness();h.echo(80.)
            if cancel: h.controller.cancel()
            else: h.echo(30.)
            commands=list(h.drive.commands)
            h.echo(80.)
            self.assertEqual(h.drive.commands,commands)

    def test_calibrated_arm_still_uses_stationary_handoff(self):
        h=Harness()
        h.controller.cfg=replace(h.controller.cfg,arm_enabled=True)
        h.controller.arm=SimpleNamespace(profile=SimpleNamespace(calibrated=True),
                                         snapshot=lambda:dict(state='IDLE'))
        h.echo(80.)
        self.assertEqual(h.echo(30.)['state'],'VERIFY_STOP')
        self.assertEqual(h.drive.commands[-1],('stop',0))
        self.assertFalse(h.controller.snapshot()['arm_motion_enabled'])

    def test_optional_calibrated_arm_completes_both_transfers_after_direct_stop(self):
        for direction in ['LEFT','RIGHT']:
            h=Harness(direction)
            h.controller.cfg=replace(h.controller.cfg,arm_enabled=True)
            driver=test_arm.FakeArm()
            h.controller.arm=test_arm.ArmSequence(driver,test_arm.calibrated_profile(),clock=lambda:h.time)
            h.echo(80.);h.echo(30.)
            for _ in range(20): h.echo(29.9)
            self.assertEqual(h.controller.snapshot()['state'],'ARM_TRANSFER')
            wheel_count=len(h.drive.commands)
            for _ in range(4000):
                h.time+=.05;h.controller.step()
                if h.controller.snapshot()['state']=='COMPLETE': break
            self.assertEqual(h.controller.snapshot()['state'],'COMPLETE')
            self.assertEqual(h.controller.arm.completed_transfers,2)
            self.assertFalse(any(c[0]=='forward' for c in h.drive.commands[wheel_count:]))

    def test_official_motor_driver_gets_real_pwm_throttle_writes(self):
        class PCA:
            def __init__(self,*args,**kwargs): self.channels=list(range(16))
        class Motor:
            def __init__(self,*args): self.throttle=None
        modules={'board':SimpleNamespace(SCL=1,SDA=2),
                 'busio':SimpleNamespace(I2C=lambda *args:object()),
                 'adafruit_pca9685':SimpleNamespace(PCA9685=PCA),
                 'adafruit_motor':SimpleNamespace(motor=SimpleNamespace(DCMotor=Motor,SLOW_DECAY=1))}
        spec=importlib.util.spec_from_file_location('direct_test_move',ROOT/'Server/Move.py')
        move=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules): spec.loader.exec_module(move)
        move.setup()
        bridge=test_approach.OfficialConnectionTests().load_bridge()
        steering=[]
        drive=bridge.DriveAdapter(move,SimpleNamespace(moveAngle=lambda *args:steering.append(args)))
        controller=DirectMissionController(load_config(),drive,
            lambda:dict(distance_cm=80.,captured_at=1.),
            lambda:dict(first_direction='LEFT'),clock=lambda:1.)
        controller.step()
        self.assertAlmostEqual(move.motor1.throttle,-.4)
        self.assertAlmostEqual(move.motor2.throttle,-.4)
        self.assertEqual(steering,[(0,0)])

    @unittest.skipUnless(importlib.util.find_spec('cv2'),'OpenCV unavailable')
    def test_single_actual_qr_frame_reaches_production_drive(self):
        import cv2
        import numpy as np
        frame=cv2.imdecode(np.frombuffer((ROOT/'tests/fixtures/Left.png').read_bytes(),dtype=np.uint8),cv2.IMREAD_COLOR)
        runtime=MissionRuntime(load_config())
        runtime.start()
        try:
            runtime.submit_frame(frame)  # Exactly one frame, not a repeated feed.
            deadline=time.monotonic()+3.
            while runtime.snapshot()['first_direction'] is None and time.monotonic()<deadline:
                time.sleep(.01)
            self.assertEqual(runtime.snapshot()['first_direction'],'LEFT')
            drive=test_approach.Drive()
            controller=DirectMissionController(load_config(),drive,
                lambda:dict(distance_cm=80.,captured_at=time.monotonic()),runtime.snapshot)
            controller.step()
            self.assertIn(('forward',40),drive.commands)
        finally:
            runtime.stop()


if __name__=='__main__': unittest.main()
