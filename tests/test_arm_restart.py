"""Restart, stop-to-teach and old-file compatibility with fake hardware only."""
import copy
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from picar_mission.arm_sequence import load_motion_profile, load_profile
from picar_mission.arm_positions import PositionStore
from picar_mission.mission_controller import MissionController
from picar_mission.runtime import MissionRuntime
from picar_mission.sequence_selection import sequence_for
from test_dashboard import load_bridge


class RestartAndTeachingTests(unittest.TestCase):
    def setUp(self):
        self.bridge=load_bridge()
        self.addCleanup(self.bridge.shutdown)
        self.bridge.config=replace(self.bridge.config,direct_steer_preload_seconds=0.,direct_steer_settle_seconds=0.)
        self.calls=[]
        self.servo=SimpleNamespace(nowPos=[90.,180.,105.,75.,130.,90.,90.,90.],initPos=[90.]*8,
            stopWiggle=lambda:None,moveAngle=lambda *args:self.calls.append(('steer',args)))
        def write(channel,angle):
            self.calls.append(('arm',channel,angle))
            self.servo.nowPos[channel]=angle
        self.servo.setPWM=write
        self.now=1.
        sensor=SimpleNamespace(read_sample=lambda:dict(distance_cm=80.,captured_at=self.now))
        motor=SimpleNamespace(motorStop=lambda:self.calls.append(('stop',)),
            move=lambda *args:self.calls.append(('drive',args)))
        self.bridge.configure(motor,self.servo,sensor)
        self.bridge._camera_seen_at=time.monotonic()
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.actual_position_store=self.bridge._position_store
        self.bridge._position_store=lambda:PositionStore(Path(self.temp.name)/'positions.json')

    def start(self):
        # Exercise the production reset-start logic, advancing its clock ourselves.
        with patch.object(MissionController,'start',return_value=None),patch.object(MissionRuntime,'start',return_value=None):
            result=self.bridge.start_mission(self.bridge._control_revision)
        self.bridge.controller.clock=self.bridge.runtime.clock=lambda:self.now
        return result

    def test_new_mission_resets_all_arm_joints_before_accepting_a_departure_qr(self):
        state=self.start()
        self.assertEqual(state['state'],'ARM_RESET')
        self.assertIsNone(state['sequence_selection'])
        self.assertEqual(state['qr']['state'],'WAIT_ARM_RESET')
        reader=SimpleNamespace(detect=lambda frame:[{'text':'RIGHT'}])
        self.assertIsNone(self.bridge.runtime.process_frame(None,self.now,1,reader))
        for _ in range(400):
            self.now+=.05;self.bridge.controller.step()
            if self.bridge.controller.snapshot()['state']=='WAIT_QR': break
        self.assertEqual(self.servo.nowPos[1:5],[90.]*4)
        self.assertEqual(self.bridge.controller.snapshot()['state'],'WAIT_QR')
        self.assertFalse(any(c[0] in ('drive','steer') for c in self.calls))
        self.assertEqual({c[1] for c in self.calls if c[0]=='arm'},{1,2,3,4})
        self.bridge.runtime.process_frame(None,self.now,2,reader)
        self.bridge.controller.step()
        self.assertTrue(any(c[0]=='drive' for c in self.calls))

    def test_global_stop_immediately_enables_teaching_without_moving_any_joint(self):
        self.start()
        self.now+=.1;self.bridge.controller.step()
        held=list(self.servo.nowPos)
        self.bridge.cancel('WEB_STOP')
        count=len(self.calls)
        for _ in range(20): self.now+=.1;self.bridge.controller.step()
        data=self.bridge.position_data()
        self.assertTrue(data['manual'])
        self.assertTrue(data['teaching_active'])
        self.assertEqual(self.bridge.status()['state'],'ARM_TEACHING')
        self.assertEqual(self.servo.nowPos,held)
        self.assertFalse(any(c[0]=='arm' for c in self.calls[count:]))
        # No extra mode toggle is needed to adjust the arm after a full stop.
        self.bridge.set_joint_angle('elbow',self.servo.nowPos[3]+.1,
            self.bridge._control_revision,self.bridge.SESSION_ID)
        self.bridge._pose_move['thread'].join(2)
        self.assertEqual(self.bridge.pose_move_status()['state'],'DONE')

    def test_restart_from_teaching_stops_manual_worker_and_clears_the_old_selection(self):
        self.bridge.cancel('WEB_STOP')
        self.bridge.controller.data.update(state='SEQUENCE_READY',sequence_selection=sequence_for('RIGHT-B'))
        self.bridge.runtime.data['sequence_selection']=sequence_for('RIGHT-B')
        self.bridge.set_joint_angle('base',120.,self.bridge._control_revision,self.bridge.SESSION_ID)
        old_worker=self.bridge._pose_move['thread']
        result=self.start()
        self.assertFalse(old_worker.is_alive())
        self.assertFalse(self.bridge._position_teaching)
        self.assertEqual(result['state'],'ARM_RESET')
        self.assertIsNone(result['sequence_selection'])
        self.assertIsNone(result['qr']['sequence_selection'])
        self.assertIsNone(result['first_direction'])

    def test_old_pose_names_do_not_break_page_jog_or_position_saving(self):
        values=copy.deepcopy(load_profile().values)
        values['positions']={'left_pick':{'base':42.,'shoulder':81.,'elbow':103.}}
        legacy=Path(self.temp.name)/'old_arm_poses.json'
        original=json.dumps(values).encode('utf-8');legacy.write_bytes(original)
        self.bridge.ROOT=Path(self.temp.name)
        with patch.object(self.bridge,'load_motion_profile',lambda:load_motion_profile(legacy)),\
             patch.object(self.bridge,'load_profile',side_effect=AssertionError('No transfer pose schema needed')),\
             patch.object(self.bridge,'_position_store',self.actual_position_store):
            self.bridge.cancel('WEB_STOP')
            self.assertTrue(self.bridge.position_data()['manual'])
            self.bridge.set_joint_angle('elbow',75.1,self.bridge._control_revision,self.bridge.SESSION_ID)
            self.bridge._pose_move['thread'].join(2)
            saved=self.bridge.save_position('LEFT-A','approach')
            self.assertEqual(saved['angles_deg']['elbow'],75.1)
            self.assertEqual(legacy.read_bytes(),original)


if __name__=='__main__': unittest.main()
