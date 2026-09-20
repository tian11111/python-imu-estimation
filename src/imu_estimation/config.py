"""Configuration structures for the Python port (Python 3.7 compatible)."""
from __future__ import absolute_import

import dataclasses
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence


def _identity_matrix():
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _zero_matrix():
    return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


@dataclass
class DeviceScale:
    """MPU conversion factors selected by the driver after WHO_AM_I."""
    accel_lsb_per_g: float
    gyro_lsb_per_dps: float
    temperature_lsb_per_c: float
    temperature_offset_c: float


@dataclass
class CalibrationConfig:
    temperature_reference_c: float = 25.0
    temperature_min_c: float = -40.0
    temperature_max_c: float = 85.0
    gyro_temperature_coeff_sensor: Sequence[Sequence[float]] = field(default_factory=_zero_matrix)
    gyro_correction_matrix: Sequence[Sequence[float]] = field(default_factory=_identity_matrix)
    accel_bias_sensor_mps2: Sequence[float] = (0.0, 0.0, 0.0)
    accel_correction_matrix: Sequence[Sequence[float]] = field(default_factory=_identity_matrix)
    sensor_to_body: Sequence[Sequence[float]] = field(default_factory=_identity_matrix)


@dataclass
class StationaryConfig:
    window_samples: int = 200
    enter_static_windows: int = 3
    exit_static_windows: int = 1
    gyro_pre_mean_max_rad_s: float = 0.35
    gyro_pre_std_max_rad_s: float = 0.003
    gyro_pre_peak_to_peak_max_rad_s: float = 0.012
    gyro_corrected_mean_max_rad_s: float = 0.0015
    accel_norm_tolerance_mps2: float = 1.5
    accel_norm_std_max_mps2: float = 0.12


def calibration_from_dict(values):
    """Build a calibration config from a JSON-compatible mapping."""
    if not isinstance(values, dict):
        raise ValueError("calibration profile must be an object")
    allowed = {item.name for item in dataclasses.fields(CalibrationConfig)}
    unknown = set(values) - allowed
    if unknown:
        raise ValueError("unknown calibration fields: " + ", ".join(sorted(unknown)))
    return CalibrationConfig(**values)


def load_calibration_json(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return calibration_from_dict(json.load(stream))


def save_calibration_json(config, path):
    with Path(path).open("w", encoding="utf-8") as stream:
        json.dump(dataclasses.asdict(config), stream, indent=2, ensure_ascii=False)
        stream.write("\n")
