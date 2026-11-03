#!/usr/bin/env python3
"""Four-position teaching preview. Uses virtual angles and a separate data file."""
import argparse
from pathlib import Path
import sys
from threading import RLock
import time
import uuid
import math

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from flask import jsonify, request
from picar_mission.arm_positions import JOINTS, PositionStore
from picar_mission.arm_sequence import PEAK_SPEED_RATIO, ease
from picar_mission.position_routes import register_position_routes
from scripts.preview_dashboard import app


class PreviewPositions:
    def __init__(self, path):
        self.store = PositionStore(path)
        self.angles = {joint: 90. for joint in JOINTS}
        self.move = None
        self.lock = RLock()
        self.revision = 0
        self.session = uuid.uuid4().hex
        self.teaching = True

    def tick(self):
        if self.move is not None and self.move['state'] == 'MOVING':
            progress = min(1., (time.monotonic()-self.move['started_at'])/self.move.get('duration',1.8))
            for joint, target in self.move['targets'].items():
                start = self.move['start_angles'][joint]
                self.angles[joint] = round(start+(target-start)*ease(progress), 2)
            if progress >= 1.:
                self.move['state'] = 'DONE'

    def position_data(self):
        with self.lock:
            self.tick()
            return dict(profile=self.store.read(), manual=self.teaching, angles=dict(self.angles),
                move=None if self.move is None else {key: value for key, value in self.move.items()
                    if key not in ('started_at', 'start_angles')},
                preview=True, legacy_available=False, control_revision=self.revision, session_id=self.session,
                teaching_active=self.teaching, can_switch_teaching=True,
                joint_limits=self.store.limits, supports_joint_control=True)

    def set_angles(self, angles):
        with self.lock:
            if not isinstance(angles, dict) or set(angles) != set(JOINTS):
                raise ValueError('가상 관절 네 개의 각도가 필요합니다')
            self.store._angles(angles, JOINTS)
            self.stop_position_move()
            self.angles = {joint: float(value) for joint, value in angles.items()}

    def save_position(self, position, pose):
        with self.lock:
            if not self.teaching: raise ValueError('팔 수동 조절 시작을 누르세요')
            self.tick()
            if self.move is not None and self.move['state'] == 'MOVING':
                raise ValueError('가상 이동이 끝난 뒤 저장하세요')
            return self.store.save(position, pose, self.angles)

    def move_to_position(self, position, pose, expected_revision=None, session_id=None):
        with self.lock:
            if not self.teaching: raise ValueError('팔 수동 조절 시작을 누르세요')
            if type(expected_revision) is not int or expected_revision != self.revision or session_id != self.session:
                raise ValueError('제어 상태가 바뀌었습니다. 다시 확인하세요')
            self.tick()
            targets = self.store.targets(position, pose)
            self.revision += 1
            self.move = dict(name=position+' / '+pose, state='MOVING', targets=targets,
                started_at=time.monotonic(), start_angles=dict(self.angles), error=None)
            return self.position_data()['move']

    def set_position_teaching(self, enabled, expected_revision=None, session_id=None):
        with self.lock:
            if type(enabled) is not bool or type(expected_revision) is not int or expected_revision != self.revision or session_id != self.session:
                raise ValueError('현재 제어 상태와 모드를 확인하세요')
            self.stop_position_move()
            self.teaching = enabled
            return self.position_data()

    def set_joint_angle(self, joint, angle, expected_revision=None, session_id=None):
        with self.lock:
            if not self.teaching: raise ValueError('팔 수동 조절 시작을 누르세요')
            if type(expected_revision) is not int or expected_revision != self.revision or session_id != self.session:
                raise ValueError('제어 상태가 바뀌었습니다')
            if joint not in JOINTS or type(angle) not in (int,float) or not math.isfinite(angle):
                raise ValueError('관절과 각도를 확인하세요')
            if not self.store.limits[joint][0] <= angle <= self.store.limits[joint][1]:
                raise ValueError('허용 각도를 확인하세요')
            self.tick()
            duration = max(.06,PEAK_SPEED_RATIO*abs(self.angles[joint]-angle)/10.)
            self.revision += 1
            self.move = dict(name='JOINT / '+joint,state='MOVING',targets={joint:float(angle)},
                started_at=time.monotonic(),start_angles=dict(self.angles),duration=duration,error=None)
            return self.position_data()

    def stop_position_move(self):
        with self.lock:
            self.revision += 1
            self.tick()
            if self.move is not None and self.move['state'] == 'MOVING':
                self.move['state'] = 'STOPPED'
            return self.position_data()['move']


def configure_preview(path):
    backend = PreviewPositions(path)
    register_position_routes(app, backend, str(ROOT/'picar_mission'))

    @app.post('/positions/preview/angles')
    def set_preview_angles():
        from urllib.parse import urlsplit
        origin = request.headers.get('Origin')
        if origin and urlsplit(origin).netloc != request.host:
            return jsonify(error='미리보기 화면에서 조작하세요'), 403
        body = request.get_json(silent=True)
        try:
            backend.set_angles(body.get('angles') if isinstance(body, dict) else None)
            return jsonify(backend.position_data())
        except ValueError as exc:
            return jsonify(error=str(exc)), 400
    return backend


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=5082)
    parser.add_argument('--data-dir', type=Path, default=ROOT.parent/'tmp'/'position-preview')
    args = parser.parse_args()
    configure_preview(args.data_dir/'arm_positions.json')
    print(f'PC preview only: http://127.0.0.1:{args.port}/poses', flush=True)
    app.run(host='127.0.0.1', port=args.port, threaded=True)
