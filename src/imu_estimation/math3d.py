"""Numerical primitives with the C implementation's Hamilton conventions."""
from __future__ import absolute_import

import math
import numpy as np

SMALL_ANGLE_SQ = 1.0e-8


def is_finite(value):
    return bool(np.isfinite(value))


def vec3_is_finite(vector):
    try:
        return np.asarray(vector, dtype=float).shape == (3,) and bool(np.all(np.isfinite(vector)))
    except (TypeError, ValueError):
        return False


def vec3_norm(vector):
    if not vec3_is_finite(vector):
        return 0.0
    return float(np.linalg.norm(np.asarray(vector, dtype=float)))


def vec3_normalize(vector):
    """Return a normalized copy, or None if the C operation would fail."""
    value = np.asarray(vector, dtype=float)
    norm = vec3_norm(value)
    if not norm > 0.0 or not is_finite(norm):
        return None
    result = value / norm
    return result if vec3_is_finite(result) else None


def skew(vector):
    x, y, z = np.asarray(vector, dtype=float)
    return np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)), dtype=float)


def quaternion_identity():
    return np.array((1.0, 0.0, 0.0, 0.0), dtype=float)


def quaternion_multiply(left, right):
    lw, lx, ly, lz = np.asarray(left, dtype=float)
    rw, rx, ry, rz = np.asarray(right, dtype=float)
    return np.array((lw * rw - lx * rx - ly * ry - lz * rz,
                     lw * rx + lx * rw + ly * rz - lz * ry,
                     lw * ry - lx * rz + ly * rw + lz * rx,
                     lw * rz + lx * ry - ly * rx + lz * rw), dtype=float)


def quaternion_normalize(quaternion):
    value = np.asarray(quaternion, dtype=float)
    if value.shape != (4,):
        return None
    norm_squared = float(np.dot(value, value))
    if not norm_squared > 0.0 or not is_finite(norm_squared):
        return None
    result = value / math.sqrt(norm_squared)
    return result if bool(np.all(np.isfinite(result))) else None


def quaternion_exp(rotation_vector_rad):
    phi = np.asarray(rotation_vector_rad, dtype=float)
    angle_squared = float(np.dot(phi, phi))
    if angle_squared < SMALL_ANGLE_SQ:
        scalar_scale = 0.5 - angle_squared / 48.0
        w = 1.0 - angle_squared / 8.0
    else:
        angle = math.sqrt(angle_squared)
        scalar_scale = math.sin(0.5 * angle) / angle
        w = math.cos(0.5 * angle)
    return np.array((w, scalar_scale * phi[0], scalar_scale * phi[1], scalar_scale * phi[2]), dtype=float)


def quaternion_get_fused_yaw_rad(quaternion):
    w, _, _, z = np.asarray(quaternion, dtype=float)
    return math.atan2(2.0 * w * z, w * w - z * z)


def quaternion_get_zyx_pitch_rad(quaternion):
    w, x, y, z = np.asarray(quaternion, dtype=float)
    return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))


def quaternion_rotate_body_to_world(q_wb, vector_body):
    q = np.asarray(q_wb, dtype=float)
    v = np.asarray(vector_body, dtype=float)
    qv = q[1:]
    cross_once = np.cross(qv, v)
    return v + 2.0 * (q[0] * cross_once + np.cross(qv, cross_once))


def quaternion_rotate_world_to_body(q_wb, vector_world):
    q = np.asarray(q_wb, dtype=float).copy()
    q[1:] *= -1.0
    return quaternion_rotate_body_to_world(q, vector_world)


def so3_exp(rotation_vector_rad):
    phi = np.asarray(rotation_vector_rad, dtype=float)
    angle_squared = float(np.dot(phi, phi))
    if angle_squared < SMALL_ANGLE_SQ:
        coefficient_a = 1.0 - angle_squared / 6.0
        coefficient_b = 0.5 - angle_squared / 24.0
    else:
        angle = math.sqrt(angle_squared)
        coefficient_a = math.sin(angle) / angle
        coefficient_b = (1.0 - math.cos(angle)) / angle_squared
    phi_skew = skew(phi)
    return np.eye(3) + coefficient_a * phi_skew + coefficient_b * np.dot(phi_skew, phi_skew)


def so3_right_jacobian(rotation_vector_rad):
    phi = np.asarray(rotation_vector_rad, dtype=float)
    angle_squared = float(np.dot(phi, phi))
    if angle_squared < SMALL_ANGLE_SQ:
        coefficient_a = 0.5 - angle_squared / 24.0
        coefficient_b = 1.0 / 6.0 - angle_squared / 120.0
    else:
        angle = math.sqrt(angle_squared)
        coefficient_a = (1.0 - math.cos(angle)) / angle_squared
        coefficient_b = (angle - math.sin(angle)) / (angle_squared * angle)
    phi_skew = skew(phi)
    return np.eye(3) - coefficient_a * phi_skew + coefficient_b * np.dot(phi_skew, phi_skew)
