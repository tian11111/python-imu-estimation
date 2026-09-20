"""Six state multiplicative extended Kalman filter for a 6-axis IMU.

The nominal state is a Body-to-World Hamilton quaternion and one gyro bias.
All public methods return :class:`ImuStatus`; rejected observations return
``NO_DATA`` without changing the filter state.
"""
from __future__ import division

import math
from dataclasses import dataclass

import numpy as np

from .types import ImuStatus


GRAVITY_MPS2 = 9.80665


@dataclass
class MekfConfig(object):
    """Continuous-time noise and safety limits matching ``ImuMekfConfig``."""
    gyro_noise_density_rad_s_sqrt_hz: float = 0.00035
    gyro_bias_random_walk_rad_s2_sqrt_hz: float = 0.00005
    initial_tilt_variance_rad2: float = (5.0 * math.pi / 180.0) ** 2
    initial_yaw_variance_rad2: float = math.pi ** 2
    initial_bias_variance_rad2_s2: float = (0.5 * math.pi / 180.0) ** 2
    specific_force_direction_variance_rad2: float = (3.0 * math.pi / 180.0) ** 2
    zaru_angular_rate_variance_rad2_s2: float = 2.25e-6
    specific_force_norm_tolerance_mps2: float = 0.8
    minimum_dt_s: float = 0.0002
    maximum_dt_s: float = 0.02
    maximum_attitude_correction_rad: float = 0.35
    maximum_bias_correction_rad_s: float = 0.01


def _config_is_valid(config):
    if config is None:
        return False
    values = (config.gyro_noise_density_rad_s_sqrt_hz,
              config.gyro_bias_random_walk_rad_s2_sqrt_hz,
              config.initial_tilt_variance_rad2,
              config.initial_yaw_variance_rad2,
              config.initial_bias_variance_rad2_s2,
              config.specific_force_direction_variance_rad2,
              config.zaru_angular_rate_variance_rad2_s2,
              config.specific_force_norm_tolerance_mps2,
              config.minimum_dt_s, config.maximum_dt_s,
              config.maximum_attitude_correction_rad,
              config.maximum_bias_correction_rad_s)
    return (all(math.isfinite(value) and value > 0.0 for value in values) and
            config.maximum_dt_s >= config.minimum_dt_s)


def _ok(value):
    return np.all(np.isfinite(value))


def _norm(value):
    return float(np.linalg.norm(value))


def _skew(v):
    return np.array(((0.0, -v[2], v[1]), (v[2], 0.0, -v[0]),
                     (-v[1], v[0], 0.0)), dtype=float)


def _normalize(v):
    n = _norm(v)
    if not math.isfinite(n) or n <= 0.0:
        return None
    return np.asarray(v, dtype=float) / n


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array((aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw), dtype=float)


def _qconj(q):
    return np.array((q[0], -q[1], -q[2], -q[3]), dtype=float)


def _qexp(rotation):
    rotation = np.asarray(rotation, dtype=float)
    theta_squared = float(np.dot(rotation, rotation))
    if theta_squared < 1.0e-8:
        scalar_scale = 0.5 - theta_squared / 48.0
        scalar = 1.0 - theta_squared / 8.0
    else:
        theta = math.sqrt(theta_squared)
        half = 0.5 * theta
        scalar_scale = math.sin(half) / theta
        scalar = math.cos(half)
    return np.r_[scalar, scalar_scale * rotation]


def _rotate_world_to_body(q, vector):
    return _qmul(_qmul(_qconj(q), np.r_[0.0, vector]), q)[1:]


def _so3_exp(rotation):
    theta_squared = float(np.dot(rotation, rotation))
    k = _skew(rotation)
    if theta_squared < 1.0e-8:
        coefficient_a = 1.0 - theta_squared / 6.0
        coefficient_b = 0.5 - theta_squared / 24.0
    else:
        theta = math.sqrt(theta_squared)
        coefficient_a = math.sin(theta) / theta
        coefficient_b = (1.0 - math.cos(theta)) / theta_squared
    return np.eye(3) + coefficient_a * k + coefficient_b * np.dot(k, k)


def _right_jacobian(rotation):
    theta_squared = float(np.dot(rotation, rotation))
    k = _skew(rotation)
    if theta_squared < 1.0e-8:
        coefficient_a = 0.5 - theta_squared / 24.0
        coefficient_b = 1.0 / 6.0 - theta_squared / 120.0
    else:
        theta = math.sqrt(theta_squared)
        coefficient_a = (1.0 - math.cos(theta)) / theta_squared
        coefficient_b = (theta - math.sin(theta)) / (theta_squared * theta)
    return np.eye(3) - coefficient_a * k + coefficient_b * np.dot(k, k)


def _fused_yaw(q):
    denom = q[0] * q[0] + q[3] * q[3]
    if denom <= 0.0 or not math.isfinite(float(denom)):
        return None
    return math.atan2(2.0 * q[0] * q[3], q[0] * q[0] - q[3] * q[3])


class Mekf(object):
    """C-reference-compatible six-state MEKF."""

    def __init__(self, config):
        self.config = config
        self.quaternion_wb = np.array((1.0, 0.0, 0.0, 0.0), dtype=float)
        self.gyro_bias_body_rad_s = np.zeros(3, dtype=float)
        self.covariance = np.zeros((6, 6), dtype=float)
        self.prediction_count = 0
        self.specific_force_update_count = 0
        self.rejected_specific_force_count = 0
        self.zaru_update_count = 0
        self.rejected_zaru_count = 0
        self.body_yaw_rate_update_count = 0
        self.rejected_body_yaw_rate_count = 0
        self.initialized = False

    def initialize(self, initial_specific_force_body_mps2,
                   initial_gyro_bias_body_rad_s):
        raw_force = np.asarray(initial_specific_force_body_mps2, dtype=float)
        if raw_force.shape != (3,) or not _config_is_valid(self.config):
            return ImuStatus.INVALID_ARGUMENT
        force = _normalize(raw_force)
        bias = np.asarray(initial_gyro_bias_body_rad_s, dtype=float)
        if force is None or bias.shape != (3,) or not _ok(bias):
            return ImuStatus.INVALID_ARGUMENT
        lateral = math.hypot(force[1], force[2])
        roll = math.atan2(force[1], force[2])
        pitch = math.atan2(-force[0], lateral)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        self.quaternion_wb = np.array((cp * cr, cp * sr, sp * cr, -sp * sr))
        self.gyro_bias_body_rad_s = bias.copy()
        self.covariance.fill(0.0)
        self.covariance[0, 0] = self.config.initial_tilt_variance_rad2
        self.covariance[1, 1] = self.config.initial_tilt_variance_rad2
        self.covariance[2, 2] = self.config.initial_yaw_variance_rad2
        self.covariance[3:, 3:] = np.eye(3) * self.config.initial_bias_variance_rad2_s2
        self.prediction_count = self.specific_force_update_count = 0
        self.rejected_specific_force_count = self.zaru_update_count = 0
        self.rejected_zaru_count = self.body_yaw_rate_update_count = 0
        self.rejected_body_yaw_rate_count = 0
        self.initialized = True
        return ImuStatus.OK

    def predict(self, gyro_pre_body_rad_s, dt_s):
        gyro = np.asarray(gyro_pre_body_rad_s, dtype=float)
        if not self.initialized or gyro.shape != (3,) or not _ok(gyro) or not math.isfinite(dt_s):
            return ImuStatus.INVALID_ARGUMENT
        if dt_s < self.config.minimum_dt_s or dt_s > self.config.maximum_dt_s:
            return ImuStatus.TIME_ERROR
        rotation = (gyro - self.gyro_bias_body_rad_s) * dt_s
        q = _normalize(_qmul(self.quaternion_wb, _qexp(rotation)))
        if q is None:
            return ImuStatus.NUMERIC_ERROR
        phi = np.eye(6)
        phi[:3, :3] = _so3_exp(-rotation)
        phi[:3, 3:] = -dt_s * _right_jacobian(rotation)
        p = np.dot(np.dot(phi, self.covariance), phi.T)
        qg = self.config.gyro_noise_density_rad_s_sqrt_hz ** 2
        qb = self.config.gyro_bias_random_walk_rad_s2_sqrt_hz ** 2
        p[:3, :3] += np.eye(3) * (qg * dt_s + qb * dt_s ** 3 / 3.0)
        p[:3, 3:] -= np.eye(3) * (0.5 * qb * dt_s ** 2)
        p[3:, :3] -= np.eye(3) * (0.5 * qb * dt_s ** 2)
        p[3:, 3:] += np.eye(3) * qb * dt_s
        return self._commit(q, self.gyro_bias_body_rad_s, p, prediction=True)

    def _commit(self, quaternion, bias, covariance, prediction=False):
        covariance = 0.5 * (covariance + covariance.T)
        diagonal = np.diag(covariance)
        if not _ok(quaternion) or not _ok(bias) or not _ok(covariance) or np.any(diagonal < -1.0e-7):
            return ImuStatus.NUMERIC_ERROR
        covariance[np.diag_indices(6)] = np.maximum(diagonal, 0.0)
        self.quaternion_wb = np.asarray(quaternion, dtype=float).copy()
        self.gyro_bias_body_rad_s = np.asarray(bias, dtype=float).copy()
        self.covariance = covariance
        if prediction:
            self.prediction_count += 1
        return ImuStatus.OK

    def _measurement3(self, residual, h, variance, attitude_projection,
                      bias_projection, preserve_yaw):
        if not self.initialized or variance <= 0.0 or not math.isfinite(variance):
            return ImuStatus.INVALID_ARGUMENT
        s = np.dot(np.dot(h, self.covariance), h.T) + np.eye(3) * variance
        try:
            gain = np.linalg.solve(s, np.dot(self.covariance, h.T).T).T
        except np.linalg.LinAlgError:
            return ImuStatus.NUMERIC_ERROR
        gain[:3] = np.dot(attitude_projection, gain[:3])
        gain[3:] = np.dot(bias_projection, gain[3:])
        error = np.dot(gain, residual)
        if (_norm(error[:3]) > self.config.maximum_attitude_correction_rad or
                _norm(error[3:]) > self.config.maximum_bias_correction_rad_s):
            return ImuStatus.NO_DATA
        a = np.eye(6) - np.dot(gain, h)
        p = np.dot(np.dot(a, self.covariance), a.T) + variance * np.dot(gain, gain.T)
        q = _normalize(_qmul(self.quaternion_wb, _qexp(error[:3])))
        if q is None:
            return ImuStatus.NUMERIC_ERROR
        if preserve_yaw is not None:
            candidate = _fused_yaw(q)
            if candidate is not None:
                correction = (preserve_yaw - candidate + math.pi) % (2.0 * math.pi) - math.pi
                q = _normalize(_qmul(np.array((math.cos(correction / 2.0), 0.0, 0.0,
                                                math.sin(correction / 2.0))), q))
        reset = np.eye(6)
        reset[:3, :3] = _right_jacobian(error[:3])
        p = np.dot(np.dot(reset, p), reset.T)
        return self._commit(q, self.gyro_bias_body_rad_s + error[3:], p)

    def update_specific_force_direction(self, specific_force_body_mps2):
        force = np.asarray(specific_force_body_mps2, dtype=float)
        if not self.initialized or force.shape != (3,) or not _ok(force):
            return ImuStatus.INVALID_ARGUMENT
        magnitude = _norm(force)
        if not math.isfinite(magnitude) or magnitude <= 0.0:
            return ImuStatus.NUMERIC_ERROR
        if abs(magnitude - GRAVITY_MPS2) > self.config.specific_force_norm_tolerance_mps2:
            self.rejected_specific_force_count += 1
            return ImuStatus.NO_DATA
        measured = force / magnitude
        predicted = _normalize(_rotate_world_to_body(self.quaternion_wb, np.array((0., 0., 1.))))
        if predicted is None:
            return ImuStatus.NUMERIC_ERROR
        tangent = np.eye(3) - np.outer(predicted, predicted)
        h = np.zeros((3, 6)); h[:, :3] = tangent
        status = self._measurement3(np.cross(measured, predicted), h,
                                    self.config.specific_force_direction_variance_rad2,
                                    tangent, tangent, _fused_yaw(self.quaternion_wb))
        if status == ImuStatus.OK:
            self.specific_force_update_count += 1
        elif status == ImuStatus.NO_DATA:
            self.rejected_specific_force_count += 1
        return status

    def update_zero_angular_rate(self, gyro_pre_mean_body_rad_s):
        gyro = np.asarray(gyro_pre_mean_body_rad_s, dtype=float)
        if not self.initialized or gyro.shape != (3,) or not _ok(gyro):
            return ImuStatus.INVALID_ARGUMENT
        h = np.zeros((3, 6)); h[:, 3:] = -np.eye(3)
        status = self._measurement3(-(gyro - self.gyro_bias_body_rad_s), h,
                                    self.config.zaru_angular_rate_variance_rad2_s2,
                                    np.zeros((3, 3)), np.eye(3), None)
        if status == ImuStatus.OK:
            self.zaru_update_count += 1
        elif status == ImuStatus.NO_DATA:
            self.rejected_zaru_count += 1
        return status

    def update_body_yaw_rate(self, gyro_pre_body_rad_s, body_yaw_rate_rad_s,
                              variance_rad2_s2):
        gyro = np.asarray(gyro_pre_body_rad_s, dtype=float)
        if (not self.initialized or gyro.shape != (3,) or not _ok(gyro) or
                not math.isfinite(body_yaw_rate_rad_s) or not math.isfinite(variance_rad2_s2) or
                variance_rad2_s2 <= 0.0):
            return ImuStatus.INVALID_ARGUMENT
        innovation = self.covariance[5, 5] + variance_rad2_s2
        if not math.isfinite(float(innovation)) or innovation <= np.finfo(np.float32).eps:
            return ImuStatus.NUMERIC_ERROR
        correction = -self.covariance[5, 5] / innovation * (body_yaw_rate_rad_s -
                      (gyro[2] - self.gyro_bias_body_rad_s[2]))
        if abs(correction) > self.config.maximum_bias_correction_rad_s:
            self.rejected_body_yaw_rate_count += 1
            return ImuStatus.NO_DATA
        gain = -self.covariance[5, 5] / innovation
        transition = np.eye(6); transition[5, 5] += gain
        p = np.dot(np.dot(transition, self.covariance), transition.T)
        p[5, 5] += gain * gain * variance_rad2_s2
        bias = self.gyro_bias_body_rad_s.copy(); bias[2] += correction
        status = self._commit(self.quaternion_wb, bias, p)
        if status == ImuStatus.OK:
            self.body_yaw_rate_update_count += 1
        return status

    def get_euler_rad(self):
        if not self.initialized:
            return None
        w, x, y, z = self.quaternion_wb
        return (math.atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y)),
                math.asin(max(-1.0, min(1.0, 2.0 * (w*y - z*x)))),
                math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z)))

    def get_predicted_specific_force_direction_body(self):
        return _normalize(_rotate_world_to_body(self.quaternion_wb, np.array((0., 0., 1.)))) if self.initialized else None

    def get_gravity_tilt_rad(self):
        direction = self.get_predicted_specific_force_direction_body()
        if direction is None:
            return None
        return (math.asin(max(-1.0, min(1.0, direction[1]))),
                math.asin(max(-1.0, min(1.0, -direction[0]))), bool(direction[2] >= 0.0))
