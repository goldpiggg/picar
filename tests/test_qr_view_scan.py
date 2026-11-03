"""View centering, bounded yaw search, early success and cancellation."""
from dataclasses import replace
import unittest
from picar_mission.qr_prepare import QrPreparation,side_view_targets
from picar_mission.arm_sequence import ArmSequence
from picar_mission.runtime import MissionRuntime
from picar_mission.config import MissionConfig
from test_direct_mission import Harness
from test_arm import FakeArm,load_profile
from test_position_transfer import profile,taught


class ViewSearchTests(unittest.TestCase):
    def make(self):
        h=Harness('LEFT')
        h.controller.cfg=replace(h.controller.cfg,fast_mission_enabled=True,arm_enabled=True)
        driver=FakeArm()
        h.controller.qr_preparation=QrPreparation(driver,load_profile(),{'LEFT':165.,'RIGHT':7.},150.,elbow_offset=-20.)
        h.controller.arm=ArmSequence(driver,profile(),clock=lambda:h.time)
        h.controller.view_scan_enabled=True
        h.controller.request_side_qr=lambda side,now:True
        h.echo(80.);h.near(36.);h.settle(30.)
        for _ in range(100):
            h.echo(None,dt=.02)
            if h.controller.snapshot()['state']=='SIDE_QR_WAIT':break
        self.assertEqual(h.controller.snapshot()['state'],'SIDE_QR_WAIT')
        return h,driver

    def test_center_uses_both_approach_angles_and_missing_slots_use_fallback(self):
        data=taught();limits=load_profile().limits
        expected={side:sum(data['positions'][side+'-'+row]['approach']['angles_deg']['base'] for row in ('A','B'))/2 for side in ('LEFT','RIGHT')}
        self.assertEqual(side_view_targets(data,limits,{'LEFT':165.,'RIGHT':7.}),expected)
        data['positions']['LEFT-A']['approach']=None
        self.assertEqual(side_view_targets(data,limits,{'LEFT':165.,'RIGHT':7.})['LEFT'],165.)

    def test_first_view_success_starts_transfer_without_extra_scanning(self):
        h,driver=self.make();before=driver.angles['base']
        h.qr['sequence_selection']={'start_position':'LEFT-B'}
        h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'ARM_TRANSFER')
        self.assertEqual(driver.angles['base'],before)
        self.assertEqual(h.controller.qr_preparation.scan_targets,[155.,175.])

    def test_search_stays_bounded_does_not_retilt_and_times_out(self):
        h,driver=self.make();started=h.time;wheels=len(h.drive.commands);seen=[]
        for _ in range(120):
            h.echo(None,dt=.02);seen.append(h.controller.qr_preparation.target)
            if h.controller.snapshot()['state']=='ERROR':break
        self.assertIn(155.,seen);self.assertIn(175.,seen)
        self.assertLessEqual(h.time-started,2.13)
        self.assertIn('QR_VIEW_NOT_FOUND',h.controller.snapshot()['error'])
        self.assertEqual(driver.angles['elbow'],70.)
        self.assertTrue(all(c[0]=='stop' for c in h.drive.commands[wheels:]))

    def test_final_view_waits_for_decode_without_rescanning(self):
        h,driver=self.make()
        h.time=h.controller.view_scan_started+h.controller.cfg.qr_scan_timeout_seconds+.01
        before=list(driver.writes)
        h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'SIDE_QR_WAIT')
        self.assertEqual(driver.writes,before)
        h.qr['sequence_selection']={'start_position':'LEFT-B'}
        h.time+=.2
        h.controller.step()
        self.assertEqual(h.controller.snapshot()['state'],'ARM_TRANSFER')

    def test_single_code_triggers_pitch_before_yaw_then_success_stops_search(self):
        for y,expected in ((.2,78.),(.8,62.)):
            h,driver=self.make();base=driver.angles['base'];wheels=len(h.drive.commands)
            h.qr['detections']=[dict(text='PICK',label='PICK',center_normalized=[.7,y])]
            h.time=h.controller.view_scan_next
            h.controller.step()
            self.assertEqual(h.controller.snapshot()['state'],'QR_SCAN_MOVING')
            self.assertEqual(h.controller.qr_preparation.elbow_target,expected)
            for _ in range(40):
                h.echo(None,dt=.02)
                if h.controller.snapshot()['state']=='SIDE_QR_WAIT':break
            self.assertEqual(driver.angles['base'],base)
            self.assertEqual(driver.angles['elbow'],expected)
            self.assertEqual(driver.angles['shoulder'],90.)
            self.assertEqual(driver.angles['gripper'],90.)
            h.qr['sequence_selection']={'start_position':'LEFT-B'}
            h.controller.step()
            self.assertEqual(h.controller.snapshot()['state'],'ARM_TRANSFER')
            self.assertTrue(all(c[0]=='stop' for c in h.drive.commands[wheels:]))

    def test_pitch_attempts_are_relative_bounded_and_clamped(self):
        for origin in (70.,0.,189.):
            h,driver=self.make();prep=h.controller.qr_preparation
            prep.pitch_origin=origin
            driver.angles['elbow']=origin
            code=[dict(text='PICK',label='PICK',center_normalized=[.7,.2])]
            targets=[]
            for attempt in range(3):
                target=prep.next_pitch_target(code)
                if target is None:break
                targets.append(target)
                start=10.+attempt*2
                prep.scan_pitch(start)
                for tick in range(1,101):prep.tick(start+tick*.02)
            self.assertLessEqual(len(targets),2)
            self.assertTrue(all(0<=t<=189 and abs(t-origin)<=8 for t in targets))
            self.assertIsNone(prep.next_pitch_target(code))

    def test_decidable_pair_does_not_trigger_pitch(self):
        h,_=self.make();prep=h.controller.qr_preparation
        code=dict(text='PICK',label='PICK',center_normalized=[.7,.2])
        for other in (dict(text='PLACE',label='PLACE',center_normalized=[.2,.2]),
                      dict(text='',label=None,undecoded=True,center_normalized=[.2,.2])):
            self.assertIsNone(prep.next_pitch_target([code,other]))
        self.assertIsNone(prep.pitch_targets)

    def test_pitch_timeout_and_cancel_prevent_further_arm_motion(self):
        for cancel in (False,True):
            h,driver=self.make()
            h.qr['detections']=[dict(text='PICK',label='PICK',center_normalized=[.7,.2])]
            h.time=h.controller.view_scan_next;h.controller.step()
            if cancel:h.controller.cancel('WEB_STOP')
            else:
                h.time=h.controller.view_scan_started+h.controller.cfg.qr_scan_timeout_seconds+.61
                h.controller.step()
                self.assertIn('QR_VIEW_NOT_FOUND',h.controller.snapshot()['error'])
            before=list(driver.writes)
            for _ in range(100):h.echo(None,dt=.02)
            self.assertEqual(driver.writes,before)

    def test_confirmed_result_wins_over_deadline_in_both_scan_states(self):
        for state in ('SIDE_QR_WAIT','QR_SCAN_MOVING'):
            h,_=self.make()
            h.controller.data['state']=state
            h.time=h.controller.view_scan_started+h.controller.cfg.qr_scan_timeout_seconds+.61
            h.qr['sequence_selection']={'start_position':'LEFT-B'}
            h.controller.step()
            self.assertEqual(h.controller.snapshot()['state'],'ARM_TRANSFER')

    def test_no_result_in_moving_state_still_has_hard_deadline(self):
        h,_=self.make()
        h.controller.data['state']='QR_SCAN_MOVING'
        h.time=h.controller.view_scan_started+h.controller.cfg.qr_scan_timeout_seconds+.61
        h.controller.step()
        self.assertIn('QR_VIEW_NOT_FOUND',h.controller.snapshot()['error'])

    def test_mid_scan_success_and_stop_prevent_further_search(self):
        for cancel in (False,True):
            h,driver=self.make()
            for _ in range(50):
                h.echo(None,dt=.02)
                if h.controller.snapshot()['state']=='QR_SCAN_MOVING':break
            self.assertEqual(h.controller.snapshot()['state'],'QR_SCAN_MOVING')
            if cancel:
                h.controller.cancel('WEB_STOP');before=list(driver.writes)
                for _ in range(100):h.echo(None,dt=.02)
                self.assertEqual(driver.writes,before)
            else:
                h.qr['sequence_selection']={'start_position':'LEFT-B'};h.controller.step()
                self.assertEqual(h.controller.snapshot()['state'],'ARM_TRANSFER')
                self.assertEqual(h.controller.qr_preparation.state,'INTERRUPTED')

    def test_same_side_rescan_keeps_inflight_pair_result(self):
        now=[1.];runtime=MissionRuntime(MissionConfig(layout_frames=1),clock=lambda:now[0])
        runtime.data.update(state='DIRECTION_CONFIRMED',first_direction='LEFT')
        runtime.request_side_qr('LEFT',1.);now[0]=2.
        class Reader:
            def detect(self,frame):
                runtime.request_side_qr('LEFT',now[0])
                return [dict(text='PICK',label='PICK',center_normalized=[.8,.5]),dict(text='PLACE',label='PLACE',center_normalized=[.2,.5])]
        self.assertEqual(runtime.process_frame(None,2.,1,Reader()),'LEFT-A')
        self.assertEqual(runtime.snapshot()['sequence_selection']['start_position'],'LEFT-A')

    def test_same_side_rescan_keeps_settled_frame_inference_inflight(self):
        now=[1.];runtime=MissionRuntime(MissionConfig(layout_frames=1),clock=lambda:now[0])
        runtime.data.update(state='DIRECTION_CONFIRMED',first_direction='LEFT')
        runtime.request_side_qr('LEFT',1.);now[0]=2.;runtime.side_view_settled(2.);now[0]=3.
        class Reader:
            def detect(self,frame):
                runtime.request_side_qr('LEFT',now[0])
                return [dict(text='PICK',label='PICK',center_normalized=[.2,.5]),
                        dict(text='',label=None,undecoded=True,center_normalized=[.6,.5])]
        self.assertEqual(runtime.process_frame(None,3.,1,Reader()),'LEFT-B')


if __name__=='__main__':unittest.main()
