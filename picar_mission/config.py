"""Load repository-relative settings without importing hardware drivers."""
from dataclasses import dataclass
import json
import math
from pathlib import Path

MAX_ARM_JOG_SPEED = 150  # degrees/second; also caps the dashboard speed scale.

ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class MissionConfig:
    qr_enabled: bool = True
    target_distance_cm: float = 30.0
    # Battery-compensated drive: motor volts -> PWM % = volts / battery volts.
    cruise_motor_volts: float = 2.8
    pulse_motor_volts: float = 3.0
    pulse_seconds: float = 0.15
    pulse_zone_cm: float = 38.0
    pulse_settle_seconds: float = 0.3
    pulse_samples: int = 5
    pulse_min_seconds: float = 0.08
    pulse_max_seconds: float = 0.4
    max_pulses: int = 15
    reverse_correction_enabled: bool = True
    battery_fallback_volts: float = 7.4
    distance_offset_cm: float = 0.0
    camera_backend: str = 'usb'
    camera_source: object = 0
    required_frames: int = 1
    hold_seconds: float = 0.0
    max_gap_seconds: float = 1.5
    max_frame_age_seconds: float = 1.5
    auto_approach_enabled: bool = True
    web_start_required: bool = True
    direct_speed: int = 40
    direct_distance_max_age_seconds: float = 0.5
    direct_distance_retry_seconds: float = 3.0
    arm_start_delay_seconds: float = 1.0
    # Center is BASE 90 degrees: look left/right by 90 degrees, BASE only.
    # A larger BASE angle turns left, like the manufacturer's lookleft button.
    qr_prepare_left_base_deg: float = 165.0
    qr_prepare_right_base_deg: float = 7.0
    # Applied after the taught A/B average; QR viewing only, not transfer poses.
    qr_prepare_right_base_offset_deg: float = -10.0
    # Height to carry a held object across sides. Smaller is higher on this
    # arm; the transfer never carries lower than the highest taught approach.
    carry_clearance_shoulder_deg: float = 105.0
    carry_clearance_elbow_deg: float = 113.0
    qr_scan_timeout_seconds: float = 1.5
    # Elbow change while looking at the side QR codes, from wherever the elbow
    # is then (the reset pose). Negative lowers it, like the web "hand down".
    qr_prepare_elbow_offset_deg: float = -30.0
    side_qr_settle_seconds: float = 0.3
    # After the turn, decide from one PICK or PLACE code alone when the other
    # cannot be decoded. Off: both codes are always required.
    side_single_code: bool = True
    # Box layout from PICK/PLACE codes, read after the stop and before the arm.
    layout_frames: int = 3
    layout_min_gap: float = 0.05
    layout_timeout_seconds: float = 5.0
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
    fast_mission_enabled: bool = True
    direct_steer_preload_deg: float = 10.0
    direct_steer_preload_seconds: float = 0.2
    direct_steer_settle_seconds: float = 0.3
    heading_correction_enabled: bool = True
    heading_gain: float = 1.5
    heading_max_steer_deg: float = 12.0
    heading_deadband_deg: float = 1.0
    gyro_max_age_seconds: float = 0.2
    gyro_z_sign: int = 1
    gyro_calibration_seconds: float = 1.5
    # Web ARM/gripper button speed, degrees/second (RPIservo uses whole numbers).
    arm_jog_speed: int = 40


def load_config(path=None):
    path = ROOT / 'mission_config.json' if path is None else Path(path)
    values = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
    # Keys starting with "_" are notes for people editing the file.
    cfg = MissionConfig(**{k:v for k,v in values.items() if not k.startswith('_')})
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
    if type(cfg.web_start_required) is not bool:
        raise ValueError('web_start_required must be boolean')
    if type(cfg.arm_enabled) is not bool:
        raise ValueError('arm_enabled must be boolean')
    if type(cfg.reverse_correction_enabled) is not bool:
        raise ValueError('reverse_correction_enabled must be boolean')
    if type(cfg.fast_mission_enabled) is not bool:
        raise ValueError('fast_mission_enabled must be boolean')
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
    if isinstance(cfg.arm_start_delay_seconds,bool) or not isinstance(cfg.arm_start_delay_seconds,(int,float)) or not math.isfinite(cfg.arm_start_delay_seconds) or cfg.arm_start_delay_seconds < 0:
        raise ValueError('arm_start_delay_seconds must be nonnegative and finite')
    def number(name):
        value = getattr(cfg, name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(name + ' must be a finite number')
        return value
    for name in ('qr_prepare_left_base_deg', 'qr_prepare_right_base_deg'):
        if not 0 <= number(name) <= 189:
            raise ValueError(name+' must be within 0~189 degrees')
    if abs(number('qr_prepare_right_base_offset_deg')) > 30:
        raise ValueError('Require |qr_prepare_right_base_offset_deg| <= 30')
    for name in ('carry_clearance_shoulder_deg','carry_clearance_elbow_deg'):
        if not 0 <= number(name) <= 189:
            raise ValueError(name+' must be within 0~189 degrees')
    if abs(number('qr_prepare_elbow_offset_deg')) > 90:
        raise ValueError('Require |qr_prepare_elbow_offset_deg| <= 90')
    if type(cfg.side_single_code) is not bool:
        raise ValueError('side_single_code must be boolean')
    if number('side_qr_settle_seconds') < 0:
        raise ValueError('side_qr_settle_seconds must be nonnegative')
    if number('qr_scan_timeout_seconds') <= 0:
        raise ValueError('qr_scan_timeout_seconds must be positive')
    if abs(number('direct_steer_preload_deg')) > 30:
        raise ValueError('Require |direct_steer_preload_deg| <= 30')
    for name in ('direct_steer_preload_seconds', 'direct_steer_settle_seconds', 'heading_deadband_deg'):
        if number(name) < 0:
            raise ValueError(name + ' must be nonnegative')
    for name in ('heading_gain', 'gyro_max_age_seconds', 'gyro_calibration_seconds'):
        if number(name) <= 0:
            raise ValueError(name + ' must be positive')
    if not 0 < number('heading_max_steer_deg') <= 30:
        raise ValueError('Require 0 < heading_max_steer_deg <= 30')
    if type(cfg.heading_correction_enabled) is not bool:
        raise ValueError('heading_correction_enabled must be boolean')
    if cfg.gyro_z_sign not in (1, -1) or type(cfg.gyro_z_sign) is not int:
        raise ValueError('gyro_z_sign must be 1 or -1')
    for name in ('cruise_motor_volts', 'pulse_motor_volts', 'battery_fallback_volts'):
        if not 0 < number(name) <= 12:
            raise ValueError('Require 0 < '+name+' <= 12')
    for name in ('pulse_min_seconds', 'pulse_seconds', 'pulse_max_seconds'):
        if not 0 < number(name) <= 2:
            raise ValueError('Require 0 < '+name+' <= 2')
    if not cfg.pulse_min_seconds <= cfg.pulse_seconds <= cfg.pulse_max_seconds:
        raise ValueError('Require pulse_min_seconds <= pulse_seconds <= pulse_max_seconds')
    if number('stop_tolerance_cm') <= 0 or number('pulse_settle_seconds') < 0:
        raise ValueError('stop_tolerance_cm must be positive, pulse_settle_seconds nonnegative')
    if not number('pulse_zone_cm') > cfg.target_distance_cm+cfg.stop_tolerance_cm:
        raise ValueError('Require pulse_zone_cm > target_distance_cm + stop_tolerance_cm')
    for name in ('pulse_samples', 'max_pulses'):
        value = getattr(cfg, name)
        if type(value) is not int or value < (1 if name=='pulse_samples' else 0):
            raise ValueError(name+' must be a nonnegative integer (pulse_samples >= 1)')
    if type(cfg.layout_frames) is not int or cfg.layout_frames < 1:
        raise ValueError('layout_frames must be an integer >= 1')
    if not 0 < number('layout_min_gap') < 1:
        raise ValueError('Require 0 < layout_min_gap < 1')
    if number('layout_timeout_seconds') <= 0:
        raise ValueError('layout_timeout_seconds must be positive')
    if type(cfg.arm_jog_speed) is not int or not 1 <= cfg.arm_jog_speed <= MAX_ARM_JOG_SPEED:
        raise ValueError('arm_jog_speed must be an integer from 1 to %d'%MAX_ARM_JOG_SPEED)
    # Legacy precision settings remain readable for old files; the direct
    # controller does not construct or validate those unused gates.
    return cfg
