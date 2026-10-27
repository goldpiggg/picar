"""Load repository-relative settings without importing hardware drivers."""
from dataclasses import dataclass
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class MissionConfig:
    qr_enabled: bool = True
    target_distance_cm: float = 30.0
    distance_offset_cm: float = 0.0
    camera_backend: str = 'usb'
    camera_source: object = 0
    required_frames: int = 3
    hold_seconds: float = 0.35
    max_gap_seconds: float = 0.8
    max_frame_age_seconds: float = 0.8


def load_config(path=None):
    path = ROOT / 'mission_config.json' if path is None else Path(path)
    values = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
    cfg = MissionConfig(**values)
    if type(cfg.qr_enabled) is not bool:
        raise ValueError('qr_enabled must be a boolean')
    if cfg.camera_backend not in ('usb', 'csi'):
        raise ValueError('camera_backend must be usb or csi')
    if not (type(cfg.camera_source) is int or isinstance(cfg.camera_source, str)):
        raise ValueError('camera_source must be a device index or path')
    if type(cfg.required_frames) is not int or cfg.required_frames < 2:
        raise ValueError('required_frames must be an integer >= 2')
    for name in ('target_distance_cm', 'hold_seconds', 'max_gap_seconds', 'max_frame_age_seconds'):
        value = getattr(cfg, name)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(name + ' must be positive and finite')
    if not isinstance(cfg.distance_offset_cm, (int, float)) or isinstance(cfg.distance_offset_cm, bool) or not math.isfinite(cfg.distance_offset_cm):
        raise ValueError('distance_offset_cm must be finite')
    return cfg
