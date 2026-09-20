"""Front-end orchestration of calibration, stationary detection and MEKF."""
from __future__ import absolute_import

import math
from dataclasses import dataclass, field

import numpy as np

from . import calibration
from .config import CalibrationConfig, StationaryConfig
from .mekf import Mekf, MekfConfig, _fused_yaw, _normalize, _qexp, _qmul
from .stationary import StationaryDetector
from .types import (ImuOutput, ImuStatus, MotionState, OUTPUT_STATE_ATTITUDE_VALID,
                    OUTPUT_STATE_READY, OUTPUT_STATE_STATIC, YawRateSemantic)


@dataclass
class SystemConfig(object):
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    stationary: StationaryConfig = field(default_factory=StationaryConfig)
    mekf: MekfConfig = field(default_factory=MekfConfig)


class ImuSystem(object):
    """Process already-read raw samples; sensor I/O deliberately stays outside."""

    def __init__(self, config=None):
        self.config = config if config is not None else SystemConfig()
        self.mekf = Mekf(self.config.mekf)
        self.stationary_detector = StationaryDetector(self.config.stationary)
        self.latest_raw = None
        self.latest_sample = None
        self.stationary_result = None
        self.output = None
        self.gyro_only_quaternion_wb = np.array((1.0, 0.0, 0.0, 0.0))
        self.previous_timestamp_us = None
        self.raw_valid = self.sample_valid = self.output_valid = False
        self.yaw_rate_provider = None
        self.last_yaw_rate_source_sequence = None

    def set_yaw_rate_provider(self, provider):
        self.yaw_rate_provider = provider
        self.last_yaw_rate_source_sequence = None

    def _predict_gyro_only(self, gyro_pre, dt_s):
        rotation = (np.asarray(gyro_pre, dtype=float) - self.mekf.gyro_bias_body_rad_s) * dt_s
        updated = _normalize(_qmul(self.gyro_only_quaternion_wb, _qexp(rotation)))
        if updated is None:
            return False
        self.gyro_only_quaternion_wb = updated
        return True

    def process_raw(self, raw, device):
        """Publish one raw frame and, if valid, advance the estimator.

        ``device`` is the driver's selected ``DeviceScale``.  The method does
        no I2C itself, so callers can use it from a polling loop or a worker.
        """
        if raw is None or device is None:
            return ImuStatus.INVALID_ARGUMENT
        if self.previous_timestamp_us is not None and raw.timestamp_us <= self.previous_timestamp_us:
            return ImuStatus.TIME_ERROR
        dt_s = 0.0 if self.previous_timestamp_us is None else (raw.timestamp_us - self.previous_timestamp_us) * 1.0e-6
        self.latest_raw = raw
        self.raw_valid = True
        self.previous_timestamp_us = raw.timestamp_us
        status, sample = calibration.apply(self.config.calibration, device, raw, dt_s)
        if status != ImuStatus.OK:
            return status
        self.latest_sample = sample
        self.sample_valid = True
        bias = self.mekf.gyro_bias_body_rad_s if self.output_valid else np.zeros(3)
        status, stationary = self.stationary_detector.update(sample, bias, self.output_valid,
                                                             not self.output_valid)
        if status != ImuStatus.OK:
            return status
        self.stationary_result = stationary
        initial_roll = 0.0
        initial_pitch = 0.0
        if not self.output_valid and stationary.window_ready and stationary.state == MotionState.ANGULAR_STATIC:
            status = self.mekf.initialize(stationary.specific_force_mean_mps2,
                                          stationary.gyro_pre_mean_rad_s)
            if status != ImuStatus.OK:
                return status
            self.gyro_only_quaternion_wb = self.mekf.quaternion_wb.copy()
            self.output_valid = True
            initial_force = np.asarray(stationary.specific_force_mean_mps2, dtype=float)
            lateral = math.hypot(initial_force[1], initial_force[2])
            initial_roll = math.atan2(initial_force[1], initial_force[2])
            initial_pitch = math.atan2(-initial_force[0], lateral)
        elif self.output_valid:
            initial_roll = self.output.initial_roll_rad
            initial_pitch = self.output.initial_pitch_rad
            status = self.mekf.predict(sample.gyro_pre_body_rad_s, sample.dt_s)
            if status != ImuStatus.OK:
                return status
            if not self._predict_gyro_only(sample.gyro_pre_body_rad_s, sample.dt_s):
                return ImuStatus.NUMERIC_ERROR
            if (stationary.window_ready and stationary.candidate_static and
                    stationary.accel_norm_std_mps2 <= self.config.stationary.accel_norm_std_max_mps2):
                status = self.mekf.update_specific_force_direction(stationary.specific_force_mean_mps2)
                if status.value < 0:
                    return status
            if (stationary.window_ready and stationary.candidate_static and
                    stationary.state == MotionState.ANGULAR_STATIC):
                status = self.mekf.update_zero_angular_rate(stationary.gyro_pre_mean_rad_s)
                if status.value < 0:
                    return status
            status = self._apply_yaw_observation(sample)
            if status.value < 0:
                return status
        if not self.output_valid:
            return ImuStatus.OK
        return self._publish_output(sample, stationary, initial_roll, initial_pitch)

    def _apply_yaw_observation(self, sample):
        if self.yaw_rate_provider is None:
            return ImuStatus.OK
        observation = self.yaw_rate_provider(sample.timestamp_us)
        if observation is None or not observation.valid or observation.semantic != YawRateSemantic.BODY_Z:
            return ImuStatus.OK
        if observation.source_sequence == self.last_yaw_rate_source_sequence:
            return ImuStatus.OK
        if (not math.isfinite(observation.rate_rad_s) or
                not math.isfinite(observation.variance_rad2_s2) or observation.variance_rad2_s2 <= 0.0):
            return ImuStatus.OK
        status = self.mekf.update_body_yaw_rate(sample.gyro_pre_body_rad_s,
                                                observation.rate_rad_s,
                                                observation.variance_rad2_s2)
        if status == ImuStatus.OK:
            self.last_yaw_rate_source_sequence = observation.source_sequence
        return status

    def _publish_output(self, sample, stationary, initial_roll, initial_pitch):
        if not self.output_valid:
            return ImuStatus.OK
        euler = self.mekf.get_euler_rad()
        predicted = self.mekf.get_predicted_specific_force_direction_body()
        tilt = self.mekf.get_gravity_tilt_rad()
        if euler is None or predicted is None or tilt is None:
            return ImuStatus.NUMERIC_ERROR
        force = np.asarray(sample.specific_force_body_mps2, dtype=float)
        lateral = math.hypot(force[1], force[2])
        flags = OUTPUT_STATE_READY | OUTPUT_STATE_ATTITUDE_VALID
        if stationary.state == MotionState.ANGULAR_STATIC:
            flags |= OUTPUT_STATE_STATIC
        self.output = ImuOutput(sequence=sample.sequence, timestamp_us=sample.timestamp_us,
            quaternion_wb=self.mekf.quaternion_wb.copy(), roll_rad=euler[0], pitch_rad=euler[1], yaw_rad=euler[2],
            mekf_fused_yaw_rad=_fused_yaw(self.mekf.quaternion_wb),
            gyro_only_fused_yaw_rad=_fused_yaw(self.gyro_only_quaternion_wb),
            gyro_only_pitch_rad=math.asin(max(-1.0, min(1.0, 2.0 * (self.gyro_only_quaternion_wb[0] * self.gyro_only_quaternion_wb[2] - self.gyro_only_quaternion_wb[3] * self.gyro_only_quaternion_wb[1])))),
            gravity_tilt_roll_rad=tilt[0], gravity_tilt_pitch_rad=tilt[1], upright_hemisphere=tilt[2],
            predicted_specific_force_direction_body=predicted.copy(), attitude_error_variance_rad2=np.diag(self.mekf.covariance)[:3].copy(),
            initial_roll_rad=initial_roll, initial_pitch_rad=initial_pitch,
            accel_roll_rad=math.atan2(force[1], force[2]), accel_pitch_rad=math.atan2(-force[0], lateral),
            gyro_bias_body_rad_s=self.mekf.gyro_bias_body_rad_s.copy(),
            gyro_corrected_body_rad_s=np.asarray(sample.gyro_pre_body_rad_s) - self.mekf.gyro_bias_body_rad_s,
            temperature_c=sample.temperature_c, specific_force_update_count=self.mekf.specific_force_update_count,
            zaru_update_count=self.mekf.zaru_update_count, state_flags=flags)
        return ImuStatus.OK

    def get_latest_raw(self):
        return self.latest_raw

    def get_latest_sample(self):
        return self.latest_sample

    def get_stationary_result(self):
        return self.stationary_result if self.stationary_result is not None and self.stationary_result.window_ready else None

    def get_output(self):
        return self.output if self.output_valid else None
