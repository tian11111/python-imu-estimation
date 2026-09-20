"""Shared data contracts used across the Python IMU estimation package."""
from __future__ import absolute_import

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Sequence


class ImuStatus(IntEnum):
    OK = 0
    NO_DATA = 1
    INVALID_ARGUMENT = -1
    BUS_ERROR = -2
    DEVICE_NOT_FOUND = -3
    CONFIG_ERROR = -4
    TIME_ERROR = -5
    NUMERIC_ERROR = -6


SAMPLE_STATUS_VALID = 1 << 0
SAMPLE_STATUS_TEMP_CLAMPED = 1 << 1
SAMPLE_STATUS_ACCEL_SATURATED = 1 << 2
SAMPLE_STATUS_GYRO_SATURATED = 1 << 3

OUTPUT_STATE_CALIBRATING = 1 << 0
OUTPUT_STATE_READY = 1 << 1
OUTPUT_STATE_STATIC = 1 << 2
OUTPUT_STATE_ATTITUDE_VALID = 1 << 3


class MotionState(IntEnum):
    UNKNOWN = 0
    MOVING = 1
    ANGULAR_STATIC = 2


class YawRateSemantic(IntEnum):
    BODY_Z = 0
    WORLD_HEADING = 1


@dataclass
class RawSample:
    sequence: int = 0
    timestamp_us: int = 0
    accel_raw: Sequence[int] = (0, 0, 0)
    gyro_raw: Sequence[int] = (0, 0, 0)
    temperature_raw: int = 0
    status: int = 0


@dataclass
class ImuSample:
    sequence: int = 0
    timestamp_us: int = 0
    dt_s: float = 0.0
    specific_force_body_mps2: Sequence[float] = (0.0, 0.0, 0.0)
    gyro_pre_body_rad_s: Sequence[float] = (0.0, 0.0, 0.0)
    temperature_c: float = 0.0
    status: int = 0


@dataclass
class YawRateObservation:
    timestamp_us: int = 0
    rate_rad_s: float = 0.0
    variance_rad2_s2: float = 0.0
    source_sequence: int = 0
    semantic: YawRateSemantic = YawRateSemantic.BODY_Z
    valid: bool = False
    stationary_valid: bool = False
    vehicle_stationary: bool = False


@dataclass
class ImuOutput:
    sequence: int = 0
    timestamp_us: int = 0
    quaternion_wb: Sequence[float] = (1.0, 0.0, 0.0, 0.0)
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    mekf_fused_yaw_rad: float = 0.0
    gyro_only_fused_yaw_rad: float = 0.0
    gyro_only_pitch_rad: float = 0.0
    gravity_tilt_roll_rad: float = 0.0
    gravity_tilt_pitch_rad: float = 0.0
    predicted_specific_force_direction_body: Sequence[float] = (0.0, 0.0, 1.0)
    attitude_error_variance_rad2: Sequence[float] = (0.0, 0.0, 0.0)
    initial_roll_rad: float = 0.0
    initial_pitch_rad: float = 0.0
    accel_roll_rad: float = 0.0
    accel_pitch_rad: float = 0.0
    gyro_bias_body_rad_s: Sequence[float] = (0.0, 0.0, 0.0)
    gyro_corrected_body_rad_s: Sequence[float] = (0.0, 0.0, 0.0)
    temperature_c: float = 0.0
    specific_force_body_mps2: Sequence[float] = (0.0, 0.0, 0.0)
    linear_acceleration_world_mps2: Sequence[float] = (0.0, 0.0, 0.0)
    specific_force_update_count: int = 0
    zaru_update_count: int = 0
    state_flags: int = 0
    upright_hemisphere: bool = True
