"""Non-overlapping Welford stationary detection with C-equivalent hysteresis."""
from __future__ import absolute_import

import math
import numpy as np

from .config import StationaryConfig
from .math3d import vec3_is_finite
from .types import ImuStatus, MotionState, SAMPLE_STATUS_VALID

STANDARD_GRAVITY_MPS2 = 9.80665


class StationaryResult(object):
    def __init__(self, state=MotionState.UNKNOWN):
        self.sequence = 0
        self.sample_count = 0
        self.gyro_pre_mean_rad_s = np.zeros(3)
        self.gyro_pre_std_rad_s = np.zeros(3)
        self.gyro_pre_peak_to_peak_rad_s = np.zeros(3)
        self.gyro_corrected_mean_rad_s = np.zeros(3)
        self.specific_force_mean_mps2 = np.zeros(3)
        self.accel_norm_mean_mps2 = 0.0
        self.accel_norm_std_mps2 = 0.0
        self.candidate_static = False
        self.window_ready = False
        self.state = state


def config_is_valid(config):
    if config is None:
        return False
    try:
        values = (config.gyro_pre_mean_max_rad_s, config.gyro_pre_std_max_rad_s,
                  config.gyro_pre_peak_to_peak_max_rad_s, config.gyro_corrected_mean_max_rad_s,
                  config.accel_norm_tolerance_mps2, config.accel_norm_std_max_mps2)
        return (config.window_samples >= 2 and config.enter_static_windows > 0 and
                config.exit_static_windows > 0 and all(math.isfinite(v) and v > 0.0 for v in values))
    except (AttributeError, TypeError):
        return False


class StationaryDetector(object):
    def __init__(self, config=None):
        self.config = config if config is not None else StationaryConfig()
        if not config_is_valid(self.config):
            raise ValueError("invalid stationary detector configuration")
        self.state = MotionState.UNKNOWN
        self.static_window_count = 0
        self.moving_window_count = 0
        self._reset_window()

    def _reset_window(self):
        self.count = 0
        self.accel_norm_mean = 0.0
        self.accel_norm_m2 = 0.0
        self.gyro_pre_mean = np.zeros(3)
        self.gyro_pre_m2 = np.zeros(3)
        self.gyro_pre_min = np.zeros(3)
        self.gyro_pre_max = np.zeros(3)
        self.gyro_corrected_mean = np.zeros(3)
        self.specific_force_mean = np.zeros(3)

    def update(self, sample, gyro_bias_body_rad_s, gyro_bias_valid, startup_mode):
        result = StationaryResult(self.state)
        if (sample is None or not (sample.status & SAMPLE_STATUS_VALID) or
                not vec3_is_finite(sample.specific_force_body_mps2) or
                not vec3_is_finite(sample.gyro_pre_body_rad_s) or
                (gyro_bias_valid and not vec3_is_finite(gyro_bias_body_rad_s))):
            return ImuStatus.INVALID_ARGUMENT, result
        accel = np.asarray(sample.specific_force_body_mps2, dtype=float)
        gyro_pre = np.asarray(sample.gyro_pre_body_rad_s, dtype=float)
        accel_norm = float(np.linalg.norm(accel))
        if not math.isfinite(accel_norm):
            return ImuStatus.NUMERIC_ERROR, result
        self.count += 1
        inverse_count = 1.0 / self.count
        delta = accel_norm - self.accel_norm_mean
        self.accel_norm_mean += delta * inverse_count
        self.accel_norm_m2 += delta * (accel_norm - self.accel_norm_mean)
        bias = np.asarray(gyro_bias_body_rad_s, dtype=float) if gyro_bias_valid else np.zeros(3)
        corrected = gyro_pre - bias
        delta = gyro_pre - self.gyro_pre_mean
        self.gyro_pre_mean += delta * inverse_count
        self.gyro_pre_m2 += delta * (gyro_pre - self.gyro_pre_mean)
        self.gyro_corrected_mean += (corrected - self.gyro_corrected_mean) * inverse_count
        self.specific_force_mean += (accel - self.specific_force_mean) * inverse_count
        if self.count == 1:
            self.gyro_pre_min = gyro_pre.copy()
            self.gyro_pre_max = gyro_pre.copy()
        else:
            self.gyro_pre_min = np.minimum(self.gyro_pre_min, gyro_pre)
            self.gyro_pre_max = np.maximum(self.gyro_pre_max, gyro_pre)
        if self.count < self.config.window_samples:
            return ImuStatus.OK, result
        result.sequence = sample.sequence
        result.sample_count = self.count
        result.accel_norm_mean_mps2 = self.accel_norm_mean
        result.accel_norm_std_mps2 = math.sqrt(self.accel_norm_m2 / (self.count - 1))
        result.gyro_pre_mean_rad_s = self.gyro_pre_mean.copy()
        result.gyro_pre_std_rad_s = np.sqrt(self.gyro_pre_m2 / (self.count - 1))
        result.gyro_pre_peak_to_peak_rad_s = self.gyro_pre_max - self.gyro_pre_min
        result.gyro_corrected_mean_rad_s = self.gyro_corrected_mean.copy()
        result.specific_force_mean_mps2 = self.specific_force_mean.copy()
        candidate = (np.linalg.norm(result.gyro_pre_mean_rad_s) <= self.config.gyro_pre_mean_max_rad_s and
                     abs(result.accel_norm_mean_mps2 - STANDARD_GRAVITY_MPS2) <= self.config.accel_norm_tolerance_mps2 and
                     result.accel_norm_std_mps2 <= self.config.accel_norm_std_max_mps2 and
                     bool(np.all(result.gyro_pre_std_rad_s <= self.config.gyro_pre_std_max_rad_s)) and
                     bool(np.all(result.gyro_pre_peak_to_peak_rad_s <= self.config.gyro_pre_peak_to_peak_max_rad_s)))
        if not startup_mode and gyro_bias_valid:
            candidate = candidate and np.linalg.norm(result.gyro_corrected_mean_rad_s) <= self.config.gyro_corrected_mean_max_rad_s
        result.candidate_static = bool(candidate)
        result.window_ready = True
        if candidate:
            self.moving_window_count = 0
            self.static_window_count = min(65535, self.static_window_count + 1)
            if self.static_window_count >= self.config.enter_static_windows:
                self.state = MotionState.ANGULAR_STATIC
        else:
            self.static_window_count = 0
            self.moving_window_count = min(65535, self.moving_window_count + 1)
            if self.moving_window_count >= self.config.exit_static_windows:
                self.state = MotionState.MOVING
        result.state = self.state
        self._reset_window()
        return ImuStatus.OK, result
