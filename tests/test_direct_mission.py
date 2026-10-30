"""Exercise the production QR -> slow forward -> pulse to 30cm path, without GPIO."""
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


def instant_steering(cfg=None):
    # Distance tests skip the steering pre-load/settle wait; SteeringTests cover it.
    return replace(cfg or load_config(),direct_steer_preload_seconds=0.,direct_steer_settle_seconds=0.)


class Harness:
    def __init__(self,direction='LEFT',cfg=None,gyro=False,battery=7.):
        # 2.8V/7V cruise = PWM 40, 3V/7V pulse = PWM 42.9.
        self.time=0.
        self.battery=battery
        self.sequence=0
        self.sample=None
        self.yaw=0.
        self.qr=dict(state='DIRECTION_CONFIRMED',first_direction=direction,camera_fresh=False)
        self.drive=test_approach.Drive()
        heading=(lambda:None if self.yaw is None else dict(yaw_deg=self.yaw,captured_at=self.time)) if gyro else None
        self.controller=DirectMissionController(cfg or instant_steering(),self.drive,
            lambda:self.sample,lambda:self.qr,clock=lambda:self.time,read_heading=heading,
            read_battery=lambda:self.battery)

    def echo(self,distance,dt=.06):
        self.time+=dt
        self.sequence+=1
        self.sample=dict(distance_cm=distance,captured_at=self.time,sequence=self.sequence)
        self.controller.step()
        return self.controller.snapshot()

    def settle(self,distance):
        """Wait out the settle time, then feed enough fresh echoes for one median."""
        self.time+=self.controller.cfg.pulse_settle_seconds
        for _ in range(self.controller.cfg.pulse_samples): state=self.echo(distance)
        return state

    def finish_pulse(self):
        self.time=self.controller.pulse_until
        self.controller.step()
        return self.controller.snapshot()


class DirectTests(unittest.TestCase):
    def test_first_good_distance_drives_without_warmup_or_frame_gate(self):
        for direction in ['LEFT','RIGHT']:
            h=Harness(direction)
            state=h.echo(80.)
            self.assertEqual(state['state'],'DRIVING')
            self.assertEqual(state['first_direction'],direction)
            self.assertEqual(h.drive.commands,[('stop',0),('steer',10.),('steer',0),('forward',40)])

    def test_echo_published_during_snapshot_is_not_rejected_as_future(self):
        h=Harness()
        def latest():
            h.time+=.01
            return dict(distance_cm=80.,captured_at=h.time)
        h.controller.read_sample=latest
        h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'DRIVING')
        self.assertIn(('forward',40),h.drive.commands)

    def test_constant_forward_then_brake_at_pulse_zone(self):
        h=Harness()
        states=[h.echo(d)['state'] for d in [80.,50.,40.,38.1,38.]]
        self.assertEqual(states,['DRIVING']*4+['PULSE_SETTLE'])
        self.assertEqual([x for x in h.drive.commands if x[0]=='forward'],[('forward',40)])
        self.assertEqual(h.drive.commands[-1],('stop',0))
        self.assertFalse(h.controller.snapshot()['approach_complete'])

    def test_settled_median_within_tolerance_latches_without_pulse(self):
        h=Harness();h.echo(80.);h.echo(37.)
        state=h.settle(30.6)
        self.assertEqual(state['state'],'STOPPED_WAIT')
        self.assertEqual((state['final_distance_cm'],state['pulse_count']),(30.6,0))
        self.assertEqual(state['stop_reason'],'TARGET_DISTANCE_REACHED')
        self.assertTrue(state['approach_complete'])

    def test_median_ignores_one_bad_echo(self):
        h=Harness();h.echo(80.);h.echo(37.)
        h.time+=.3
        for d in [30.2,30.,45.,29.9,30.1]: state=h.echo(d)
        self.assertEqual((state['state'],state['final_distance_cm']),('STOPPED_WAIT',30.1))

    def test_settle_ignores_echoes_before_the_car_has_stopped(self):
        h=Harness();h.echo(80.);h.echo(37.)
        for _ in range(4): state=h.echo(30.)  # 0.24s < 0.3s settle.
        self.assertEqual(state['state'],'PULSE_SETTLE')
        self.assertEqual(h.controller.measure_values,[])

    def test_far_stop_pulses_forward_then_sizes_from_measured_rate(self):
        h=Harness();h.echo(80.);h.echo(37.)
        state=h.settle(34.)
        self.assertEqual(state['state'],'PULSING')
        self.assertEqual(h.drive.commands[-1],('forward',42.9))
        self.assertEqual((state['pulse_count'],state['last_pulse_seconds']),(1,.15))
        self.assertEqual(h.finish_pulse()['state'],'PULSE_SETTLE')
        self.assertEqual(h.drive.commands[-1],('stop',0))
        state=h.settle(32.5)  # Moved 1.5cm in 0.15s -> 10cm/s; 2.5cm left -> 0.25s.
        self.assertEqual((state['pulse_cm_per_second'],state['last_pulse_seconds']),(10.,.25))
        h.finish_pulse()
        state=h.settle(30.3)
        self.assertEqual((state['state'],state['pulse_count']),('STOPPED_WAIT',2))
        self.assertEqual(h.drive.commands[-1],('stop',0))

    def test_overshoot_pulses_backward(self):
        h=Harness();h.echo(80.);h.echo(37.)
        state=h.settle(27.)
        self.assertEqual(state['state'],'PULSING')
        self.assertEqual(h.drive.commands[-1],('backward',42.9))

    def test_stalled_pulse_is_lengthened_within_limits(self):
        h=Harness();h.echo(80.);h.echo(37.)
        h.settle(34.);h.finish_pulse()
        state=h.settle(33.9)
        self.assertEqual(state['last_pulse_seconds'],.195)
        self.assertIsNone(state['pulse_cm_per_second'])
        for _ in range(4):
            h.finish_pulse();state=h.settle(33.9)
        self.assertEqual(state['last_pulse_seconds'],.4)

    def test_pulse_count_is_bounded(self):
        h=Harness(cfg=replace(instant_steering(),max_pulses=1));h.echo(80.);h.echo(37.)
        h.settle(34.);h.finish_pulse()
        state=h.settle(34.)
        self.assertEqual((state['state'],state['error']),('ERROR','PULSE_LIMIT_REACHED'))
        self.assertEqual(h.drive.commands[-1],('stop',0))

    def test_settle_sensor_failure_times_out_stopped(self):
        h=Harness();h.echo(80.);h.echo(37.)
        h.time+=.3;h.echo(None)
        state=h.echo(None,dt=3.1)
        self.assertEqual(state['state'],'ERROR')
        self.assertIn('STOP_DISTANCE_TIMEOUT',state['error'])
        self.assertEqual(h.drive.commands[-1],('stop',0))

    def test_pulse_ends_on_time_not_a_control_period_late(self):
        h=Harness();h.echo(80.);h.echo(37.);h.settle(34.)
        start=h.time
        h.time=start+.14
        self.assertAlmostEqual(h.controller._wait_seconds(),.01)
        h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'PULSING')
        h.time=start+.15
        self.assertEqual(h.controller._wait_seconds(),0.)

    def test_battery_voltage_sets_pwm_once_at_start(self):
        for battery,cruise,pulse,source in [(8.4,33.3,35.7,'MEASURED'),(6.,46.7,50.,'MEASURED'),
                                            (None,37.8,40.5,'FALLBACK'),(0.,37.8,40.5,'FALLBACK')]:
            with self.subTest(battery=battery):
                h=Harness(battery=battery);state=h.echo(80.)
                self.assertEqual((state['cruise_pwm'],state['pulse_pwm'],state['battery_source']),
                                 (cruise,pulse,source))
                self.assertEqual(h.drive.commands[-1],('forward',cruise))
                h.battery=5.;h.echo(70.)  # Sag mid-drive does not change speed.
                self.assertEqual(h.controller.snapshot()['cruise_pwm'],cruise)

    def test_already_at_target_never_drives_forward(self):
        for distance in [38.,31.,30.]:
            h=Harness();state=h.echo(distance)
            self.assertEqual(state['state'],'PULSE_SETTLE')
            if distance<=31.:
                self.assertEqual(h.settle(distance)['state'],'STOPPED_WAIT')
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
            self.assertEqual(h.controller.snapshot()['state'],'WAIT_DISTANCE')
            self.assertNotIn('forward',[x[0] for x in h.drive.commands])

    def test_sensor_failure_pauses_then_recovers_before_target(self):
        h=Harness();h.echo(80.);state=h.echo(None)
        self.assertEqual(state['state'],'WAIT_DISTANCE')
        self.assertEqual(h.drive.commands[-1],('stop',0))
        self.assertFalse(state['target_stop_latched'])
        self.assertEqual(h.echo(70.)['state'],'DRIVING')
        self.assertEqual(h.drive.commands[-1],('forward',40))

    def test_permanent_sensor_failure_stays_stopped_and_times_out(self):
        h=Harness();h.echo(80.);h.echo(None)
        state=h.echo(None,dt=3.1)
        self.assertEqual(state['state'],'ERROR')
        self.assertIn('DISTANCE_RECOVERY_TIMEOUT',state['error'])
        self.assertEqual(h.drive.commands[-1],('stop',0))

    def test_sensor_recovery_inside_target_stops_permanently(self):
        h=Harness();h.echo(80.);h.echo(None)
        self.assertEqual(h.echo(34.)['state'],'PULSE_SETTLE')
        state=h.settle(30.)
        self.assertEqual(state['state'],'STOPPED_WAIT')
        self.assertTrue(state['target_stop_latched'])
        commands=list(h.drive.commands)
        h.echo(80.)
        self.assertEqual(h.drive.commands,commands)

    def test_half_second_sensor_budget_is_separate_from_legacy_filter(self):
        h=Harness();h.echo(80.);h.time+=.3;h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'DRIVING')
        h.time+=.21;h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'WAIT_DISTANCE')
        self.assertEqual(h.controller.snapshot()['distance_wait_reason'],'STALE_DISTANCE_SAMPLE')
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
        self.assertEqual(h.echo(36.)['first_direction'],'LEFT')

    def test_stop_and_cancel_do_not_restart(self):
        for cancel in [False,True]:
            h=Harness();h.echo(80.)
            if cancel: h.controller.cancel()
            else: h.echo(36.);h.settle(30.)
            commands=list(h.drive.commands)
            h.echo(80.)
            self.assertEqual(h.drive.commands,commands)

    def test_arm_wait_ignores_distance_band_and_starts_after_delay(self):
        h=Harness()
        h.controller.cfg=replace(h.controller.cfg,arm_enabled=True)
        driver=test_arm.FakeArm()
        h.controller.arm=test_arm.ArmSequence(driver,test_arm.calibrated_profile(),clock=lambda:h.time)
        h.echo(80.);h.echo(36.)
        self.assertEqual(h.settle(30.)['state'],'ARM_WAIT')
        self.assertEqual(h.drive.commands[-1],('stop',0))
        self.assertFalse(h.controller.snapshot()['arm_motion_enabled'])
        self.assertEqual(h.echo(34.,dt=.99)['state'],'ARM_WAIT')
        self.assertEqual(driver.writes,[])
        self.assertEqual(h.echo(None,dt=.02)['state'],'ARM_TRANSFER')
        self.assertEqual(h.controller.arm.state,'MOVING')

    def test_optional_calibrated_arm_completes_both_transfers_after_direct_stop(self):
        for direction in ['LEFT','RIGHT']:
            h=Harness(direction)
            h.controller.cfg=replace(h.controller.cfg,arm_enabled=True)
            driver=test_arm.FakeArm()
            h.controller.arm=test_arm.ArmSequence(driver,test_arm.calibrated_profile(),clock=lambda:h.time)
            h.echo(80.);h.echo(36.);h.settle(30.)
            wheel_count=len(h.drive.commands)
            def forbidden(): raise AssertionError('No sensor/camera reads after target stop')
            h.controller.read_sample=forbidden
            h.controller.qr_snapshot=forbidden
            for _ in range(20): h.echo(20.)
            self.assertEqual(h.controller.snapshot()['state'],'ARM_TRANSFER')
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
        now=[1.]
        controller=DirectMissionController(instant_steering(),drive,
            lambda:dict(distance_cm=80.,captured_at=now[0]),
            lambda:dict(first_direction='LEFT'),clock=lambda:now[0],read_battery=lambda:7.)
        controller.step()
        self.assertAlmostEqual(move.motor1.throttle,-.4)
        self.assertAlmostEqual(move.motor2.throttle,-.4)
        self.assertEqual(steering,[(0,10.),(0,0)])
        controller.read_sample=lambda:dict(distance_cm=27.,captured_at=now[0])
        controller.step()
        self.assertEqual((move.motor1.throttle,move.motor2.throttle),(0,0))
        now[0]+=.3
        for _ in range(5): now[0]+=.06;controller.step()
        self.assertEqual(controller.snapshot()['state'],'PULSING')
        self.assertAlmostEqual(move.motor1.throttle,.429)  # Backward pulse flips sign.
        self.assertAlmostEqual(move.motor2.throttle,.429)
        now[0]=controller.pulse_until;controller.step()
        controller.read_sample=lambda:dict(distance_cm=30.,captured_at=now[0])
        now[0]+=.3
        for _ in range(5): now[0]+=.06;controller.step()
        self.assertTrue(drive.locked)
        self.assertEqual(move.motor1.throttle,0)
        self.assertEqual(move.motor2.throttle,0)
        with self.assertRaisesRegex(RuntimeError,'DRIVE_LOCKED_AFTER_STOP'): drive.forward(40)
        with self.assertRaisesRegex(RuntimeError,'DRIVE_LOCKED_AFTER_STOP'): drive.straight()
        with self.assertRaisesRegex(RuntimeError,'DRIVE_LOCKED_AFTER_STOP'): drive.steer(5)
        with self.assertRaisesRegex(RuntimeError,'DRIVE_LOCKED_AFTER_STOP'): drive.backward(40)
        self.assertEqual(steering,[(0,10.),(0,0)])

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
            controller=DirectMissionController(instant_steering(),drive,
                lambda:dict(distance_cm=80.,captured_at=time.monotonic()),runtime.snapshot,
                read_battery=lambda:7.)
            controller.step()
            self.assertIn(('forward',40),drive.commands)
        finally:
            runtime.stop()


class SteeringTests(unittest.TestCase):
    def test_preload_then_center_then_settle_before_forward(self):
        h=Harness(cfg=load_config())
        self.assertEqual(h.echo(80.)['state'],'STEER_PRELOAD')
        self.assertEqual(h.drive.commands,[('stop',0),('steer',10.)])
        self.assertEqual(h.echo(80.,dt=.15)['state'],'STEER_PRELOAD')
        self.assertEqual(h.echo(80.,dt=.1)['state'],'STEER_SETTLE')
        self.assertEqual(h.drive.commands[-1],('steer',0))
        self.assertEqual(h.echo(80.,dt=.2)['state'],'STEER_SETTLE')
        self.assertNotIn('forward',[c[0] for c in h.drive.commands])
        self.assertEqual(h.echo(80.,dt=.15)['state'],'DRIVING')
        self.assertEqual(h.drive.commands[-1],('forward',40))

    def test_zero_preload_centers_immediately(self):
        h=Harness(cfg=replace(load_config(),direct_steer_preload_deg=0.))
        self.assertEqual(h.echo(80.)['state'],'STEER_SETTLE')
        self.assertEqual(h.drive.commands,[('stop',0),('steer',0)])

    def test_cancel_during_settle_never_drives(self):
        h=Harness(cfg=load_config());h.echo(80.)
        h.controller.cancel()
        for _ in range(10): h.echo(80.,dt=.2)
        self.assertEqual(h.controller.snapshot()['state'],'CANCELLED')
        self.assertNotIn('forward',[c[0] for c in h.drive.commands])

    def test_no_gyro_keeps_center_and_reports_off(self):
        h=Harness()
        for d in [80.,70.,60.]: h.echo(d)
        self.assertEqual(h.controller.snapshot()['heading_status'],'OFF')
        self.assertEqual([c for c in h.drive.commands if c[0]=='steer'],[('steer',10.),('steer',0)])

    def test_left_drift_steers_right_proportionally_with_limit_and_deadband(self):
        h=Harness(gyro=True)
        h.yaw=30.;h.echo(80.)  # Reference is whatever yaw is at the start.
        self.assertEqual(h.controller.snapshot()['heading_status'],'ACTIVE')
        steers=[]
        for yaw in [34.,50.,28.,30.5]:
            h.yaw=yaw;h.echo(70.)
            steers.append(h.controller.snapshot()['steer_deg'])
        self.assertEqual(steers,[6,12,-3,0])
        self.assertEqual(h.controller.snapshot()['heading_error_deg'],.5)
        self.assertEqual([c for c in h.drive.commands if c[0]=='forward'],[('forward',40)])

    def test_gyro_loss_centers_and_keeps_driving(self):
        h=Harness(gyro=True);h.echo(80.)
        h.yaw=5.;h.echo(70.)
        h.yaw=None;state=h.echo(60.)
        self.assertEqual((state['state'],state['heading_status'],state['steer_deg']),
                         ('DRIVING','GYRO_UNAVAILABLE',0))
        self.assertEqual(h.drive.commands[-1],('steer',0))

    def test_stale_gyro_sample_is_ignored(self):
        h=Harness(gyro=True);h.echo(80.)
        h.controller.read_heading=lambda:dict(yaw_deg=9.,captured_at=h.time-.5)
        self.assertEqual(h.echo(70.)['heading_status'],'GYRO_UNAVAILABLE')

    def test_late_gyro_takes_reference_when_it_appears(self):
        h=Harness(gyro=True);h.yaw=None
        self.assertEqual(h.echo(80.)['heading_status'],'GYRO_UNAVAILABLE')
        h.yaw=12.;self.assertEqual(h.echo(70.)['steer_deg'],0)
        h.yaw=14.;self.assertEqual(h.echo(60.)['steer_deg'],3)

    def test_target_stop_centers_before_locking(self):
        h=Harness(gyro=True);h.echo(80.)
        h.yaw=6.;h.echo(70.)
        state=h.echo(36.)
        self.assertEqual((state['state'],state['steer_deg']),('PULSE_SETTLE',0))
        self.assertEqual(h.drive.commands[-2:],[('stop',0),('steer',0)])
        h.yaw=20.  # Pulses stay straight; heading hold is off near the target.
        h.settle(34.)
        self.assertEqual(h.drive.commands[-1],('forward',42.9))
        h.finish_pulse()
        self.assertEqual(h.settle(30.)['state'],'STOPPED_WAIT')
        self.assertNotIn(('steer',12),h.drive.commands[-8:])


class GyroHeadingTests(unittest.TestCase):
    def make(self,rates,sign=1):
        from picar_mission.heading import GyroHeading
        it=iter(rates)
        return GyroHeading(lambda:next(it),z_sign=sign,calibration_seconds=1.)

    def test_bias_removed_then_rate_integrated(self):
        g=self.make([.5]*11+[10.5]*10)
        for i in range(11):
            g.step(i*.1)
            if i<10: self.assertIsNone(g.read_sample())
        for i in range(11,21): g.step(i*.1)
        self.assertAlmostEqual(g.read_sample()['yaw_deg'],10.,places=6)
        self.assertTrue(g.snapshot()['calibrated'])

    def test_sign_flip(self):
        g=self.make([0.]*11+[10.]*2,sign=-1)
        for i in range(13): g.step(i*.1)
        self.assertAlmostEqual(g.read_sample()['yaw_deg'],-2.)

    def test_read_failure_hides_sample_and_skips_gap(self):
        g=self.make([0.]*11+[10.])
        for i in range(12): g.step(i*.1)
        def broken(): raise OSError('i2c')
        g.read_rate_dps=broken
        g.step(1.2)
        self.assertIsNone(g.read_sample())
        self.assertIn('i2c',g.snapshot()['error'])
        g.read_rate_dps=lambda:10.
        g.step(5.)  # Gap after failure is not integrated.
        g.step(5.1)
        self.assertAlmostEqual(g.read_sample()['yaw_deg'],2.)


class SteeringConfigTests(unittest.TestCase):
    def test_underscore_keys_are_notes(self):
        import json,tempfile
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'c.json';p.write_text(json.dumps({'_note':'hi','cruise_motor_volts':2.5}))
            self.assertEqual(load_config(p).cruise_motor_volts,2.5)

    def test_bad_steering_and_gyro_settings_rejected(self):
        import json,tempfile
        for values in [dict(direct_steer_preload_deg=40),dict(direct_steer_settle_seconds=-1),
                       dict(heading_gain=0),dict(heading_max_steer_deg=31),
                       dict(gyro_z_sign=2),dict(gyro_z_sign=True),
                       dict(heading_correction_enabled='yes'),dict(gyro_max_age_seconds=float('nan')),
                       dict(cruise_motor_volts=0),dict(pulse_motor_volts=13),
                       dict(pulse_seconds=.05),dict(pulse_seconds=.5),
                       dict(pulse_zone_cm=31),dict(pulse_samples=0),dict(max_pulses=1.5)]:
            with self.subTest(values=values),tempfile.TemporaryDirectory() as d:
                p=Path(d)/'c.json';p.write_text(json.dumps(values))
                with self.assertRaises(ValueError): load_config(p)


if __name__=='__main__': unittest.main()
