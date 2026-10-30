"""Load repository-relative settings without importing hardware drivers."""
from dataclasses import dataclass
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class MissionConfig:
    qr_enabled: bool = True
    target_distance_cm: float = 36.0
    distance_offset_cm: float = 0.0
    camera_backend: str = 'usb'
    camera_source: object = 0
    required_frames: int = 1
    hold_seconds: float = 0.0
    max_gap_seconds: float = 1.5
    max_frame_age_seconds: float = 1.5
    auto_approach_enabled: bool = True
    direct_speed: int = 40
    direct_distance_max_age_seconds: float = 0.5
    direct_distance_retry_seconds: float = 3.0
    arm_start_delay_seconds: float = 1.0
    cruise_speed: int = 30
    slow_speed: int = 18
    slow_distance_cm: float = 50.0
    distance_max_age_seconds: float = 0.25
    distance_filter_size: int = 3
    distance_jump_slack_cm: float = 3.0
    max_closing_speed_cm_s: float = 60.0
    control_period_seconds: float = 0.02
    sensor_warmup_seconds: float = 5.0
    approach_timeout_seconds: float = 40.0
    progress_timeout_seconds: float = 5.0
    progress_min_cm: float = 0.3
    steering_settle_seconds: float = 0.25
    stop_tolerance_cm: float = 1.0
    stop_hold_seconds: float = 0.6
    stop_timeout_seconds: float = 3.0
    stop_stability_cm: float = 0.6
    stop_samples: int = 4
    arm_enabled: bool = False


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
    if type(cfg.required_frames) is not int or cfg.required_frames < 1:
        raise ValueError('required_frames must be an integer >= 1')
    if isinstance(cfg.hold_seconds,bool) or not isinstance(cfg.hold_seconds,(int,float)) or not math.isfinite(cfg.hold_seconds) or cfg.hold_seconds < 0:
        raise ValueError('hold_seconds must be nonnegative and finite')
    for name in ('target_distance_cm', 'max_gap_seconds', 'max_frame_age_seconds'):
        value = getattr(cfg, name)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(name + ' must be positive and finite')
    if not isinstance(cfg.distance_offset_cm, (int, float)) or isinstance(cfg.distance_offset_cm, bool) or not math.isfinite(cfg.distance_offset_cm):
        raise ValueError('distance_offset_cm must be finite')
    if type(cfg.auto_approach_enabled) is not bool:
        raise ValueError('auto_approach_enabled must be boolean')
    if type(cfg.arm_enabled) is not bool:
        raise ValueError('arm_enabled must be boolean')
    if cfg.arm_enabled and not cfg.auto_approach_enabled:
        raise ValueError('Automatic arm sequence requires automatic approach')
    if cfg.auto_approach_enabled and not cfg.qr_enabled:
        raise ValueError('Auto approach requires QR recognition')
    for name in ('direct_distance_max_age_seconds','direct_distance_retry_seconds',
                 'control_period_seconds','approach_timeout_seconds'):
        value = getattr(cfg, name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(name + ' must be positive and finite')
    if not 2 < cfg.target_distance_cm < 199:
        raise ValueError('Require 2 < target_distance_cm < 199')
    if type(cfg.direct_speed) is not int or not 0 < cfg.direct_speed <= 100:
        raise ValueError('Require 0 < direct_speed <= 100')
    if cfg.arm_enabled:
        if isinstance(cfg.arm_start_delay_seconds,bool) or not isinstance(cfg.arm_start_delay_seconds,(int,float)) or not math.isfinite(cfg.arm_start_delay_seconds) or cfg.arm_start_delay_seconds < 0:
            raise ValueError('arm_start_delay_seconds must be nonnegative and finite')
    # Legacy precision settings remain readable for old files; the direct
    # controller does not construct or validate those unused gates.
    return cfg
