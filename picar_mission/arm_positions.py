"""Physical four-position teaching data, independent of legacy mission poses.

A always faces the start QR; B always faces the starting line. Saving these
positions does not enable calibration or change the legacy automatic arm plan.
"""
import copy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
from threading import RLock

POSITIONS = ('LEFT-A', 'RIGHT-A', 'LEFT-B', 'RIGHT-B')
POSES = ('approach', 'pick', 'place')
COMMON = ('gripper_open', 'gripper_closed', 'center_clear', 'center_carry')
JOINTS = ('base', 'shoulder', 'elbow', 'gripper')
_lock = RLock()


def empty_profile():
    return dict(schema_version=1, reference='facing_start_qr',
        calibration_confirmed=False,
        positions={name: {pose: None for pose in POSES} for name in POSITIONS},
        common={name: None for name in COMMON})


def slot_keys(position, pose):
    if position == 'COMMON' and pose in COMMON:
        return ('gripper',) if pose.startswith('gripper_') else JOINTS[:3]
    if position in POSITIONS and pose in POSES:
        return JOINTS[:3]
    raise ValueError('저장할 자리와 자세 이름을 확인하세요')


class PositionStore:
    def __init__(self, path, limits=None):
        self.path = Path(path)
        self.limits = limits or {joint: [0, 189] for joint in JOINTS}

    def _angles(self, angles, keys):
        if not isinstance(angles, dict) or set(angles) != set(keys):
            raise ValueError('저장된 관절 값을 확인하세요')
        for joint, value in angles.items():
            low, high = self.limits[joint]
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError('허용 각도를 벗어났습니다: '+joint)
        return angles

    def _validate(self, profile):
        if (not isinstance(profile, dict) or profile.get('schema_version') != 1 or
            profile.get('reference') != 'facing_start_qr' or
            profile.get('calibration_confirmed') is not False or
            set(profile.get('positions', {})) != set(POSITIONS) or
            set(profile.get('common', {})) != set(COMMON)):
            raise ValueError('자리별 자세 파일 형식을 확인하세요')
        for position in (*POSITIONS, 'COMMON'):
            slots = profile['common'] if position == 'COMMON' else profile['positions'][position]
            names = COMMON if position == 'COMMON' else POSES
            if not isinstance(slots, dict) or set(slots) != set(names):
                raise ValueError('자리별 자세 구성을 확인하세요')
            for name, record in slots.items():
                if record is None:
                    continue
                if not isinstance(record, dict) or not isinstance(record.get('saved_at'), str):
                    raise ValueError('저장 기록 형식을 확인하세요')
                self._angles(record.get('angles_deg'), slot_keys(position, name))
        return profile

    def read(self):
        with _lock:
            if not self.path.exists():
                return empty_profile()
            return self._validate(json.loads(self.path.read_text(encoding='utf-8')))

    def save(self, position, pose, angles):
        keys = slot_keys(position, pose)
        if not isinstance(angles, dict) or any(key not in angles for key in keys):
            raise ValueError('현재 서보 명령각을 읽을 수 없습니다')
        values = self._angles({key: angles[key] for key in keys}, keys)
        record = dict(angles_deg={key: round(float(value), 1) for key, value in values.items()},
            saved_at=datetime.now(timezone.utc).isoformat(timespec='seconds'))
        with _lock:
            profile = self.read()
            slots = profile['common'] if position == 'COMMON' else profile['positions'][position]
            slots[pose] = record
            self._validate(profile)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            original = self.path.stat() if self.path.exists() else None
            backup = self.path.with_name(self.path.name+'.before-edit')
            if original is not None and not backup.exists():
                backup.write_bytes(self.path.read_bytes())
            pending = self.path.with_name(self.path.name+'.tmp')
            pending.write_text(json.dumps(profile, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            if original is not None:
                os.chmod(pending, original.st_mode & 0o7777)
                if hasattr(os, 'chown'):
                    try:
                        os.chown(pending, original.st_uid, original.st_gid)
                    except PermissionError:
                        pass
            pending.replace(self.path)
            return copy.deepcopy(record)

    def targets(self, position, pose):
        slot_keys(position, pose)
        profile = self.read()
        slots = profile['common'] if position == 'COMMON' else profile['positions'][position]
        record = slots[pose]
        if record is None:
            raise ValueError('아직 저장하지 않은 자세입니다')
        return dict(record['angles_deg'])
