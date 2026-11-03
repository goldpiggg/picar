"""The four agreed plans, side-view QR geometry and fresh-frame boundaries."""
from dataclasses import replace
import unittest
from types import SimpleNamespace

from picar_mission.config import MissionConfig
from picar_mission.runtime import MissionRuntime
from picar_mission.sequence_selection import classify_side_start, sequence_for, SideQrGate


def pair(pick_x, place_x):
    return [dict(text='PICK', label='PICK', center_normalized=[pick_x,.4]),
            dict(text='PLACE', label='PLACE', center_normalized=[place_x,.6])]


class SequenceGeometryTests(unittest.TestCase):
    def test_exact_four_sequences_and_camera_mapping(self):
        cases=[('LEFT',.8,.2,'LEFT-A',['LEFT-A','RIGHT-A','RIGHT-B','LEFT-B']),
               ('LEFT',.2,.8,'LEFT-B',['LEFT-B','RIGHT-B','RIGHT-A','LEFT-A']),
               ('RIGHT',.2,.8,'RIGHT-A',['RIGHT-A','LEFT-A','LEFT-B','RIGHT-B']),
               ('RIGHT',.8,.2,'RIGHT-B',['RIGHT-B','LEFT-B','LEFT-A','RIGHT-A'])]
        for number,(side,pick,place,start,positions) in enumerate(cases,1):
            with self.subTest(start=start):
                self.assertEqual(classify_side_start(pair(pick,place),side),start)
                plan=sequence_for(start)
                self.assertEqual(plan['number'],number)
                self.assertEqual([s['position'] for s in plan['steps']],positions)
                self.assertEqual([s['action'] for s in plan['steps']],['PICK','PLACE','PICK','PLACE'])
                # Both QR centers may be within one image half; only their order matters.
                shifted=pair(.45 if pick>.5 else .1,.45 if place>.5 else .1)
                self.assertEqual(classify_side_start(shifted,side),start)

    def test_missing_duplicate_close_or_invalid_codes_do_not_guess(self):
        for items in [[],pair(.2,.8)[:1],pair(.2,.8)*2,pair(.49,.51),pair(float('nan'),.8),
                      pair(True,.8),[{'text':'PICK'},{'text':'PLACE'}]]:
            with self.subTest(items=items):
                self.assertIsNone(classify_side_start(items,'RIGHT'))
        self.assertIsNone(classify_side_start(pair(.2,.8),'UNKNOWN'))

    def test_gate_requires_three_consecutive_frames_and_resets_on_gaps(self):
        gate=SideQrGate()
        self.assertIsNone(gate.update(pair(.2,.8),'RIGHT',0.))
        self.assertIsNone(gate.update(pair(.2,.8),'RIGHT',.1))
        self.assertIsNone(gate.update([],'RIGHT',.2))
        self.assertIsNone(gate.update(pair(.2,.8),'RIGHT',.3))
        self.assertIsNone(gate.update(pair(.2,.8),'RIGHT',3.))
        self.assertEqual(gate.count,1)
        self.assertIsNone(gate.update(pair(.2,.8),'RIGHT',3.1))
        self.assertEqual(gate.update(pair(.2,.8),'RIGHT',3.2),'RIGHT-A')
        self.assertEqual(gate.update(pair(.8,.2),'RIGHT',3.3),'RIGHT-A')


class SideRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.now=1.
        self.runtime=MissionRuntime(MissionConfig(),clock=lambda:self.now)
        self.reader=SimpleNamespace(detect=lambda frame:[{'text':frame}] if isinstance(frame,str) else frame)

    def process(self,frame,sequence,stamp=None):
        return self.runtime.process_frame(frame,self.now if stamp is None else stamp,sequence,self.reader)

    def test_reset_and_turn_frames_cannot_latch_a_start_or_sequence(self):
        self.runtime.wait_for_arm_reset()
        self.assertIsNone(self.process('RIGHT',1))
        self.assertTrue(self.runtime.begin_start_scan())
        self.assertEqual(self.process('RIGHT',2),'RIGHT')
        self.assertIsNone(self.process(pair(.2,.8),3))  # Still turning.
        self.assertTrue(self.runtime.request_side_qr('RIGHT',self.now))
        self.assertIsNone(self.process(pair(.2,.8),4))  # Pre-settle frame.
        self.now=1.4
        self.assertIsNone(self.process(pair(.2,.8),5,stamp=1.1))
        self.assertIsNone(self.process(pair(.2,.8),6))
        self.now+=.1;self.assertIsNone(self.process(pair(.2,.8),7))
        self.now+=.1;self.assertEqual(self.process(pair(.2,.8),8),'RIGHT-A')
        selected=self.runtime.snapshot()['sequence_selection']
        self.assertEqual(selected,sequence_for('RIGHT-A'))
        self.assertEqual(self.runtime.snapshot()['first_direction'],'RIGHT')

    def test_inflight_decode_from_an_old_scan_epoch_cannot_confirm(self):
        def old_decode(frame):
            self.runtime.wait_for_arm_reset()
            self.runtime.begin_start_scan()
            return [{'text':'RIGHT'}]
        self.reader.detect=old_decode
        self.assertIsNone(self.process('RIGHT',1))
        self.assertIsNone(self.runtime.snapshot()['first_direction'])

    def test_unconfirmed_pair_is_not_a_selected_sequence(self):
        self.process('LEFT',1)
        self.runtime.request_side_qr('LEFT',self.now)
        self.now+=.4
        self.process(pair(.8,.2),2)
        self.assertIsNone(self.runtime.snapshot()['sequence_selection'])
        self.process([],3)
        self.assertEqual(self.runtime.snapshot()['side_frames'],0)

    def test_new_runtime_has_no_previous_sequence_or_qr(self):
        self.process('RIGHT',1)
        self.runtime.request_side_qr('RIGHT',self.now)
        self.now+=.4
        for sequence in range(2,5): self.process(pair(.8,.2),sequence)
        self.assertEqual(self.runtime.snapshot()['sequence_selection']['start_position'],'RIGHT-B')
        fresh=MissionRuntime(MissionConfig())
        self.assertIsNone(fresh.snapshot()['sequence_selection'])
        self.assertIsNone(fresh.snapshot()['first_direction'])


class ControllerSelectionTests(unittest.TestCase):
    def test_all_four_side_views_select_without_driving_again_or_executing_a_transfer(self):
        from test_direct_mission import Harness
        from test_arm import FakeArm, load_profile
        from picar_mission.qr_prepare import QrPreparation
        for side,pick_x,place_x,start in [('LEFT',.8,.2,'LEFT-A'),('LEFT',.2,.8,'LEFT-B'),
                                         ('RIGHT',.2,.8,'RIGHT-A'),('RIGHT',.8,.2,'RIGHT-B')]:
            with self.subTest(start=start):
                h=Harness(side)
                runtime=MissionRuntime(h.controller.cfg,clock=lambda:h.time)
                runtime.data.update(state='DIRECTION_CONFIRMED',first_direction=side)
                driver=FakeArm()
                h.controller.qr_preparation=QrPreparation(driver,load_profile(),{'LEFT':180.,'RIGHT':0.},20.)
                h.controller.qr_snapshot=runtime.snapshot
                h.controller.request_side_qr=runtime.request_side_qr
                h.echo(80.);h.near(36.);h.settle(30.)
                wheel_count=len(h.drive.commands)
                for _ in range(200):
                    h.echo(None,dt=.1)
                    if h.controller.snapshot()['state']=='SIDE_QR_WAIT': break
                self.assertEqual(h.controller.snapshot()['state'],'SIDE_QR_WAIT')
                self.assertEqual(driver.angles['base'],180. if side=='LEFT' else 0.)
                h.time+=.4
                reader=SimpleNamespace(detect=lambda frame:pair(pick_x,place_x))
                for sequence in range(3):
                    h.time+=.05
                    runtime.process_frame(None,h.time,sequence,reader)
                    h.controller.step()
                self.assertEqual(h.controller.snapshot()['state'],'SEQUENCE_READY')
                self.assertEqual(h.controller.snapshot()['sequence_selection'],sequence_for(start))
                count=len(driver.writes)
                for _ in range(50): h.echo(80.)
                self.assertEqual(len(driver.writes),count)
                self.assertEqual({joint for joint,value in driver.writes},{'base'})
                self.assertTrue(all(c[0]=='stop' for c in h.drive.commands[wheel_count:]))


if __name__=='__main__': unittest.main()
