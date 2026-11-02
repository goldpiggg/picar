"""Warning sound control with a fake buzzer; no GPIO or actual sound."""
from collections import deque
import importlib.util
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from flask import Flask
from picar_mission.battery_alarm import BatteryAlarm
from picar_mission.battery_routes import register_battery_routes

ROOT = Path(__file__).resolve().parent.parent


class BatteryAlarmTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.alarm = BatteryAlarm()
        self.alarm.bind_buzzer(lambda note:self.calls.append(('play',note)),lambda:self.calls.append(('stop',)))

    def test_enable_at_normal_or_unknown_voltage_does_not_play_a_note(self):
        for volts in [None,6.3,7.4]:
            with self.subTest(volts=volts):
                if volts is not None: self.alarm.update_voltage(volts)
                self.alarm.set_sound_enabled(False)
                self.calls.clear()
                self.alarm.set_sound_enabled(True)
                self.assertFalse(self.alarm.play_note('C4',0))
                self.assertEqual(self.calls,[])
                self.assertFalse(self.alarm.snapshot()['sounding'])

    def test_low_voltage_only_sounds_on_monitor_note_and_mute_keeps_warning_visible(self):
        self.alarm.update_voltage(5.8)
        self.alarm.set_sound_enabled(False)
        self.calls.clear()
        self.alarm.set_sound_enabled(True)
        self.assertEqual(self.calls,[])  # Enabling is never a test-tone command.
        self.assertTrue(self.alarm.play_note('C4',0))
        self.assertEqual(self.calls,[('play','C4'),('stop',)])
        state = self.alarm.set_sound_enabled(False)
        self.assertTrue(state['low_voltage'])
        self.assertEqual(state['voltage_volts'],5.8)
        self.assertTrue(state['muted'])
        self.calls.clear()
        self.assertFalse(self.alarm.play_note('C4',0))
        self.assertEqual(self.calls,[])

    def test_mute_interrupts_a_current_long_note(self):
        started = threading.Event()
        self.alarm.bind_buzzer(lambda note:started.set(),lambda:self.calls.append(('stop',)))
        self.alarm.update_voltage(5.8)
        worker = threading.Thread(target=lambda:self.alarm.play_note('C4',5),daemon=True)
        worker.start()
        self.assertTrue(started.wait(1))
        self.assertTrue(self.alarm.snapshot()['sounding'])
        state = self.alarm.set_sound_enabled(False)
        worker.join(.5)
        self.assertFalse(worker.is_alive())
        self.assertFalse(state['sounding'])
        self.assertIn(('stop',),self.calls)

    def test_normal_voltage_interrupts_note_without_changing_sound_preference(self):
        started = threading.Event()
        self.alarm.bind_buzzer(lambda note:started.set(),lambda:None)
        self.alarm.update_voltage(5.8)
        worker = threading.Thread(target=lambda:self.alarm.play_note('C4',5),daemon=True)
        worker.start()
        self.assertTrue(started.wait(1))
        self.alarm.update_voltage(7.4)
        worker.join(.5)
        self.assertFalse(worker.is_alive())
        self.assertFalse(self.alarm.snapshot()['low_voltage'])
        self.assertTrue(self.alarm.snapshot()['sound_enabled'])

    def test_sound_preference_persists_separately_from_arm_values(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            angles = root/'arm_poses.json'
            angles.write_bytes(b'untouched taught poses')
            path = root/'data'/'battery_alarm.json'
            controller = BatteryAlarm(path)
            controller.set_sound_enabled(False)
            self.assertFalse(BatteryAlarm(path).snapshot()['sound_enabled'])
            controller.set_sound_enabled(True)
            self.assertTrue(BatteryAlarm(path).snapshot()['sound_enabled'])
            self.assertEqual(angles.read_bytes(),b'untouched taught poses')

    def test_invalid_preferences_and_samples_are_rejected(self):
        for enabled in [None,1,0,'false']:
            with self.assertRaises(ValueError): self.alarm.set_sound_enabled(enabled)
        for volts in [None,True,0,-1,float('nan'),float('inf'),'5.8']:
            with self.assertRaises(ValueError): self.alarm.update_voltage(volts)


class BatteryHttpTests(unittest.TestCase):
    def setUp(self):
        self.alarm = BatteryAlarm()
        self.calls = []
        self.alarm.bind_buzzer(lambda note:self.calls.append('play'),lambda:self.calls.append('stop'))
        app = Flask(__name__)
        register_battery_routes(app,self.alarm,str(ROOT/'picar_mission'))
        self.client = app.test_client()

    def test_sound_buttons_do_not_spoof_voltage_or_force_a_beep(self):
        self.alarm.update_voltage(7.4)
        self.assertEqual(self.client.post('/battery/sound',json={'enabled':False}).status_code,200)
        self.calls.clear()
        response = self.client.post('/battery/sound',json={'enabled':True,'voltage_volts':5.8})
        self.assertTrue(response.json['sound_enabled'])
        self.assertEqual(response.json['voltage_volts'],7.4)
        self.assertFalse(response.json['sounding'])
        self.assertEqual(self.calls,[])
        self.assertEqual(self.client.post('/battery/preview/voltage',json={'volts':5.8}).status_code,404)

    def test_mute_status_preserves_low_voltage_and_assets_are_allowlisted(self):
        self.alarm.update_voltage(5.8)
        response = self.client.post('/battery/sound',json={'enabled':False})
        self.assertTrue(response.json['muted'])
        self.assertTrue(response.json['low_voltage'])
        self.assertFalse(response.json['preview'])
        response = self.client.get('/battery/status')
        self.assertIn('no-store',response.headers['Cache-Control'])
        for filename in ['battery_alarm.css','battery_alarm.js']:
            response = self.client.get('/battery/assets/'+filename)
            self.assertEqual(response.status_code,200)
            response.close()
        self.assertEqual(self.client.get('/battery/assets/battery_alarm.py').status_code,404)

    def test_bad_posts_and_cross_origin_cannot_change_preference(self):
        for payload in [[],{}, {'enabled':'false'}, {'enabled':0}]:
            self.assertEqual(self.client.post('/battery/sound',json=payload).status_code,400)
        self.assertEqual(self.client.post('/battery/sound',data='enabled=false').status_code,400)
        self.assertEqual(self.client.post('/battery/sound',json={'enabled':False},headers={'Origin':'https://unrelated.example'}).status_code,403)
        self.assertTrue(self.alarm.snapshot()['sound_enabled'])
        self.assertEqual(self.calls,[])


class VoltageConnectionTests(unittest.TestCase):
    def load_voltage(self, alarm, calls):
        import picar_mission.battery_alarm as settings
        buzzer = SimpleNamespace(tb=SimpleNamespace(play=lambda note:calls.append(('play',note)),stop=lambda:calls.append(('stop',))))
        screen = SimpleNamespace(start=lambda:None,screen_show=lambda *args:None)
        modules={'Buzzer':buzzer,'Switch':SimpleNamespace(switch=lambda *args:None),
            'smbus':SimpleNamespace(SMBus=lambda bus:SimpleNamespace(read_byte_data=lambda *args:75)),
            'OLED':SimpleNamespace(OLED_ctrl=lambda:screen)}
        spec = importlib.util.spec_from_file_location('voltage_alarm_test',ROOT/'Server/Voltage.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules),patch.object(settings,'alarm',alarm):
            spec.loader.exec_module(module)
        return module

    def test_existing_monitor_uses_mute_and_continues_measuring_voltage(self):
        alarm = BatteryAlarm()
        calls = []
        module = self.load_voltage(alarm,calls)
        module.SINGLE_NOTE = [('C4',0)]
        alarm.update_voltage(5.8)
        monitor = module.BatteryLevelMonitor()
        monitor.play_note()
        self.assertIn(('play','C4'),calls)
        alarm.set_sound_enabled(False)
        calls.clear()
        monitor.play_note()
        self.assertEqual(calls,[])
        monitor.voltage_data = deque(maxlen=1)
        class EndSample(Exception): pass
        monitor.trigger_alarm = lambda:(_ for _ in ()).throw(EndSample())
        with self.assertRaises(EndSample): monitor.run()
        state = alarm.snapshot()
        self.assertTrue(state['low_voltage'])
        self.assertTrue(state['muted'])
        self.assertAlmostEqual(state['voltage_volts'],module.average_voltage,places=2)


if __name__ == '__main__': unittest.main()
