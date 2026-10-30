"""Deterministic closed-loop tests with simulated echoes and motor commands."""
import ast
from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from picar_mission.config import MissionConfig
from picar_mission.distance import DistanceFilter, DistanceReading, SensorFault
from picar_mission.approach import Approach
from picar_mission.mission_controller import MissionController


class Drive:
    def __init__(self): self.commands=[]
    def stop(self): self.commands.append(('stop',0))
    def straight(self): self.commands.append(('steer',0))
    def steer(self,angle): self.commands.append(('steer',angle))
    def forward(self,speed): self.commands.append(('forward',speed))


class Harness:
    def __init__(self,direction='LEFT',config=None):
        self.time=0.
        self.sequence=0
        self.sample=None
        self.qr=dict(state='DIRECTION_CONFIRMED',first_direction=direction,camera_fresh=True)
        self.drive=Drive()
        self.controller=MissionController(config or MissionConfig(),self.drive,
            lambda:self.sample,lambda:self.qr,clock=lambda:self.time)

    def echo(self,distance,dt=.06):
        self.time+=dt
        self.sequence+=1
        self.sample=dict(distance_cm=distance,captured_at=self.time,sequence=self.sequence)
        self.controller.step()
        return self.controller.snapshot()['state']

    def start(self):
        for _ in range(7): self.echo(80.)
        assert self.controller.snapshot()['command_speed']>0

    def finish(self):
        states=[]
        target=self.controller.cfg.target_distance_cm
        for d in range(79,int(target)-1,-1): states.append(self.echo(float(d)))
        for _ in range(20): states.append(self.echo(target-.1))
        return states


class DistanceTests(unittest.TestCase):
    def setUp(self): self.filter=DistanceFilter(MissionConfig())
    def sample(self,d=80.,t=1.,seq=1): return dict(distance_cm=d,captured_at=t,sequence=seq)

    def test_raw_and_median_are_separate(self):
        for seq,d in enumerate([32.,31.,30.],1):
            r=self.filter.update(self.sample(d,seq*.06,seq),seq*.06)
        self.assertEqual((r.raw_cm,r.filtered_cm),(30.,31.))
        self.assertTrue(r.ready)

    def test_duplicate_echo_does_not_fill_filter(self):
        sample=self.sample()
        for _ in range(10): r=self.filter.update(sample,1.1)
        self.assertFalse(r.ready)
        self.assertFalse(r.is_new)

    def test_invalid_none_nan_range_and_stale(self):
        for sample,now in [(None,1.),(self.sample(None),1.),(self.sample(float('nan')),1.),
                           (self.sample(0),1.),(self.sample(200),1.),(self.sample(),2.)]:
            with self.subTest(sample=sample),self.assertRaises(SensorFault):
                DistanceFilter(MissionConfig()).update(sample,now)

    def test_impossible_jump_is_not_hidden_by_median(self):
        self.filter.update(self.sample(),1.)
        with self.assertRaisesRegex(SensorFault,'JUMP'):
            self.filter.update(self.sample(120.,1.06,2),1.06)

    def test_configured_offset(self):
        f=DistanceFilter(MissionConfig(distance_offset_cm=-1.))
        self.assertEqual(f.update(self.sample(31.),1.).raw_cm,30.)


class ApproachTests(unittest.TestCase):
    def reading(self,raw,filtered=None):
        return DistanceReading(raw,raw if filtered is None else filtered,1.,1,True,True)

    def test_raw_distance_stops_before_delayed_median(self):
        self.assertEqual(Approach(MissionConfig()).command(self.reading(36.,38.)),0)

    def test_speed_decreases_and_never_reaccelerates(self):
        p=Approach(MissionConfig())
        speeds=[p.command(self.reading(d)) for d in [80,50,46,41,43,39,37]]
        self.assertEqual(speeds[0],30)
        self.assertEqual(speeds,sorted(speeds,reverse=True))
        self.assertGreaterEqual(min(speeds),18)


class MissionTests(unittest.TestCase):
    def test_both_directions_take_identical_drive_path(self):
        commands=[]
        for direction in ['LEFT','RIGHT']:
            h=Harness(direction);h.start();states=h.finish()
            self.assertIn('SLOW_APPROACH',states)
            self.assertIn('VERIFY_STOP',states)
            s=h.controller.snapshot()
            self.assertEqual(s['state'],'STOPPED_WAIT')
            self.assertEqual(s['first_direction'],direction)
            self.assertTrue(s['approach_complete'])
            self.assertFalse(s['arm_motion_enabled'])
            self.assertEqual(s['command_speed'],0)
            commands.append(h.drive.commands)
        self.assertEqual(*commands)

    def test_no_qr_no_movement(self):
        h=Harness(None)
        for _ in range(20): h.echo(80.)
        self.assertEqual(h.drive.commands,[])

    def test_stale_saved_qr_cannot_start(self):
        h=Harness();h.qr['camera_fresh']=False;h.echo(80.)
        self.assertEqual(h.controller.snapshot()['state'],'ERROR')
        self.assertNotIn('forward',[x[0] for x in h.drive.commands])

    def test_sensor_loss_stops_and_latches(self):
        h=Harness();h.start();h.echo(None)
        self.assertEqual(h.controller.snapshot()['state'],'ERROR')
        self.assertEqual(h.drive.commands[-1],('stop',0))
        old=list(h.drive.commands)
        for _ in range(10): h.echo(80.)
        self.assertEqual(h.drive.commands,old)

    def test_frozen_echo_cannot_keep_vehicle_running(self):
        h=Harness();h.start();h.time+=.3;h.controller.step()
        self.assertEqual(h.controller.snapshot()['error'],'STALE_DISTANCE_SAMPLE')
        self.assertEqual(h.drive.commands[-1],('stop',0))

    def test_camera_failure_after_qr_keeps_ultrasonic_approach(self):
        h=Harness();h.start();h.qr['state']='ERROR';h.qr['error']='camera disconnected';h.echo(79.)
        self.assertEqual(h.controller.snapshot()['state'],'APPROACH')
        self.assertIsNone(h.controller.snapshot()['error'])
        self.assertGreater(h.controller.snapshot()['command_speed'],0)

    def test_low_pwm_stall_times_out(self):
        h=Harness(config=replace(MissionConfig(),progress_timeout_seconds=.3));h.start()
        for _ in range(15): h.echo(80.)
        self.assertEqual(h.controller.snapshot()['error'],'NO_FORWARD_PROGRESS')

    def test_user_cancel_cannot_be_overwritten_by_qr(self):
        h=Harness();h.start();h.controller.cancel()
        for _ in range(10): h.echo(79.)
        self.assertEqual(h.controller.snapshot()['state'],'CANCELLED')
        self.assertEqual(h.drive.commands[-1],('stop',0))

    def test_overshoot_is_failure_without_reverse(self):
        h=Harness();h.start()
        for d in range(79,int(h.controller.cfg.target_distance_cm)-1,-1): h.echo(float(d))
        h.echo(h.controller.cfg.target_distance_cm-1.2)
        self.assertEqual(h.controller.snapshot()['error'],'PASSED_STOP_BAND')
        self.assertFalse(h.controller.snapshot()['approach_complete'])
        self.assertNotIn('reverse',[x[0] for x in h.drive.commands])

    def test_stop_repeated_value_does_not_confirm(self):
        h=Harness();h.start()
        for d in range(79,int(h.controller.cfg.target_distance_cm)-1,-1): h.echo(float(d))
        for _ in range(5): h.time+=.02;h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'VERIFY_STOP')

    def test_complete_wait_never_restarts(self):
        h=Harness();h.start();h.finish();old=list(h.drive.commands)
        h.qr['first_direction']='RIGHT'
        for _ in range(20): h.echo(80.)
        self.assertEqual(h.drive.commands,old)
        self.assertEqual(h.controller.snapshot()['first_direction'],'LEFT')

    def test_source_exception_stops(self):
        h=Harness();h.start()
        def broken(): raise RuntimeError('sensor error')
        h.controller.read_sample=broken;h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'ERROR')
        self.assertEqual(h.drive.commands[-1],('stop',0))


class OfficialConnectionTests(unittest.TestCase):
    def load_bridge(self):
        spec=importlib.util.spec_from_file_location('test_bridge',ROOT/'Server/mission_bridge.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module

    def test_adapter_reuses_only_steering_channel_zero(self):
        bridge=self.load_bridge();calls=[]
        move=SimpleNamespace(motorStop=lambda:calls.append(('stop',)),
                             move=lambda *args:calls.append(('move',)+args))
        servo=SimpleNamespace(moveAngle=lambda *args:calls.append(('servo',)+args))
        a=bridge.DriveAdapter(move,servo);a.stop();a.straight();a.forward(18)
        self.assertEqual(calls,[('stop',),('servo',0,0),('move',18,1,'mid')])

    def test_every_mutating_legacy_command_is_blocked_until_restart(self):
        b=self.load_bridge()
        for command in ['forward','backward','left','right','automatic','trackLine','CVFL',
                        'findColor','scan','speech','armup','grab','home','PWMINIT','wsB 99']:
            with self.subTest(command=command): self.assertFalse(b.allow_command(command,{}))
        self.assertTrue(b.allow_command('get_info',{}))
        self.assertFalse(b.allow_command('DS',{}));self.assertFalse(b._cancelled)
        self.assertFalse(b.allow_command('stop',{}));self.assertTrue(b._cancelled)

    def test_websocket_does_not_reinitialize_motor_in_auto_mode(self):
        source=(ROOT/'Server/WebServer.py').read_text(encoding='utf-8')
        self.assertIn('if not mission_bridge.automatic_enabled():\n        move.setup()',source)
        tree=ast.parse(source)
        for name in ['robotCtrl','robotCtrl_speech','functionSelect','configPWM']:
            node=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name==name)
            self.assertIn('allow_command',ast.unparse(node.body[0]))

    def test_pending_cancel_prevents_startup(self):
        b=self.load_bridge();calls=[]
        move=SimpleNamespace(motorStop=lambda:calls.append('stop'),move=lambda *args:calls.append('forward'))
        steering=SimpleNamespace(moveAngle=lambda *args:calls.append('steer'),stopWiggle=lambda:calls.append('hold'))
        b.cancel()
        b.configure(move,steering,SimpleNamespace(read_sample=lambda:None))
        self.assertEqual(b.controller.snapshot()['state'],'CANCELLED')
        self.assertIsNone(b.controller.worker)
        self.assertNotIn('forward',calls)

    def test_legacy_handlers_cannot_issue_any_motion_when_locked(self):
        tree=ast.parse((ROOT/'Server/WebServer.py').read_text(encoding='utf-8'))
        nodes=[x for x in tree.body if isinstance(x,ast.FunctionDef) and
               x.name in ('robotCtrl','robotCtrl_speech','functionSelect','configPWM')]
        namespace={'mission_bridge':SimpleNamespace(allow_command=lambda *args:False)}
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'WebServer.py','exec'),namespace)
        for node in nodes:
            # No driver globals are supplied: reaching a legacy motion path
            # would raise instead of silently succeeding.
            for command in ['forward','automatic','armup','PWMINIT','home']:
                namespace[node.name](command,{})

    def test_existing_port_preflight_runs_before_hardware_imports(self):
        tree=ast.parse((ROOT/'Server/WebServer.py').read_text(encoding='utf-8'))
        self.assertIsInstance(tree.body[0],ast.If)
        self.assertIn('probe.bind',ast.unparse(tree.body[0]))

    def test_echo_observer_records_failures_and_distinct_sequences(self):
        class FakeSensor:
            def __init__(self,**kwargs): self.max_distance=kwargs['max_distance'];self.values=iter([.4,None,.15])
            def _read(self): return next(self.values)
        spec=importlib.util.spec_from_file_location('test_ultra',ROOT/'Server/Ultra.py')
        module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'gpiozero':SimpleNamespace(DistanceSensor=FakeSensor)}):
            spec.loader.exec_module(module)
        self.assertIsNone(module.read_sample())
        for expected,seq in [(80.,1),(None,2),(30.,3)]:
            module.sensor._read();packet=module.read_sample()
            self.assertEqual(packet['sequence'],seq)
            self.assertEqual(packet['distance_cm'],expected)
            self.assertIsInstance(packet['captured_at'],float)


class ClosedLoopTests(unittest.TestCase):
    def test_healthy_virtual_plants_reach_stop_band(self):
        # The motor command affects the next actual position. These are model
        # assumptions, not measured PWM/speed or braking values of the robot.
        for gain in [.2,.3,.4]:
            with self.subTest(gain=gain):
                now=[0.];distance=[87.];velocity=[0.];command=[0];packet=[None]
                history=[]
                class PlantDrive:
                    def stop(self): command[0]=0;history.append(('stop',0))
                    def straight(self): history.append(('steer',0))
                    def forward(self,speed): command[0]=speed;history.append(('forward',speed))
                qr=lambda:dict(state='DIRECTION_CONFIRMED',first_direction='LEFT',camera_fresh=True)
                # Preserve the original 30cm precision-mode model fixture.
                c=MissionController(MissionConfig(target_distance_cm=30.),PlantDrive(),lambda:packet[0],qr,clock=lambda:now[0])
                for tick in range(1250):
                    now[0]=tick*.02
                    desired=command[0]*gain
                    old=velocity[0]
                    delta=max(-35*.02,min(35*.02,desired-old))
                    velocity[0]=max(0.,old+delta)
                    distance[0]-=(old+velocity[0])*.01
                    if tick%3==0:
                        packet[0]=dict(distance_cm=distance[0],captured_at=now[0],sequence=tick//3+1)
                    c.step()
                    if c.snapshot()['state'] in ('ERROR','STOPPED_WAIT'):
                        break
                self.assertEqual(c.snapshot()['state'],'STOPPED_WAIT',c.snapshot())
                self.assertLessEqual(abs(distance[0]-c.cfg.target_distance_cm),1.)
                self.assertEqual(command[0],0)
                self.assertLess(velocity[0],.01)
                self.assertFalse(c.snapshot()['arm_motion_enabled'])


if __name__=='__main__': unittest.main()
