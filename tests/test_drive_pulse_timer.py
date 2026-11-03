"""Pulse cutoff works even if the mission loop stops stepping; fake motors only."""
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from test_dashboard import load_bridge


class PulseTimerTests(unittest.TestCase):
    def make(self):
        bridge=load_bridge()
        calls=[]
        move=SimpleNamespace(motorStop=lambda:calls.append('stop'),
                             move=lambda *args:calls.append(args))
        return bridge,bridge.DriveAdapter(move,None),calls

    def test_real_timer_stops_without_a_controller_step(self):
        bridge,drive,calls=self.make()
        stopped=threading.Event()
        drive.move.motorStop=lambda:(calls.append('stop'),stopped.set())
        try:
            drive.pulse(1,40,.02)
            self.assertTrue(stopped.wait(1.))
            self.assertEqual(calls,[(40,1,'mid'),'stop'])
        finally:drive.stop()

    def test_old_timer_cannot_stop_new_command_or_write_after_stop(self):
        for action in ('forward','pulse','stop','lock'):
            bridge,drive,calls=self.make()
            callbacks=[]
            def timer(seconds,callback):
                callbacks.append(callback)
                return SimpleNamespace(start=lambda:None,cancel=lambda:None)
            with patch.object(bridge,'Timer',timer):
                drive.pulse(-1,40,.1)
                if action=='pulse':drive.pulse(1,40,.1)
                elif action=='forward':drive.forward(40)
                else:getattr(drive,action)()
                before=list(calls)
                callbacks[0]()  # A cancelled callback that was already queued.
                self.assertEqual(calls,before)
                drive.stop()

    def test_command_error_stops_and_cancels_cutoff(self):
        bridge,drive,calls=self.make()
        def fail(*args):raise OSError('write failed')
        drive.move.move=fail
        with self.assertRaisesRegex(OSError,'write failed'):drive.pulse(1,40,.1)
        self.assertEqual(calls,['stop'])
        self.assertIsNone(drive._pulse_timer)

    def test_locked_drive_rejects_pulses(self):
        _,drive,calls=self.make();drive.lock()
        with self.assertRaisesRegex(RuntimeError,'DRIVE_LOCKED'):drive.pulse(1,40,.1)
        self.assertEqual(calls,['stop'])
