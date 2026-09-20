import math
import numpy as np

from imu_estimation import math3d


def test_hamilton_right_increment_rotates_body_vector_to_world():
    quarter_turn_z = math3d.quaternion_exp(np.array((0.0, 0.0, math.pi / 2.0)))
    world = math3d.quaternion_rotate_body_to_world(quarter_turn_z, (1.0, 0.0, 0.0))
    assert np.allclose(world, (0.0, 1.0, 0.0), atol=1e-12)
    assert np.allclose(math3d.quaternion_rotate_world_to_body(quarter_turn_z, world), (1.0, 0.0, 0.0))


def test_quaternion_multiply_and_fused_yaw():
    left = math3d.quaternion_exp((0.0, 0.0, math.pi / 4.0))
    right = math3d.quaternion_exp((0.0, 0.0, math.pi / 4.0))
    joined = math3d.quaternion_normalize(math3d.quaternion_multiply(left, right))
    assert np.allclose(joined, math3d.quaternion_exp((0.0, 0.0, math.pi / 2.0)), atol=1e-12)
    assert math.isclose(math3d.quaternion_get_fused_yaw_rad(joined), math.pi / 2.0, abs_tol=1e-12)


def test_small_angle_series_and_so3_right_jacobian():
    phi = np.array((1.0e-6, -2.0e-6, 3.0e-6))
    q = math3d.quaternion_exp(phi)
    assert np.allclose(q[1:], 0.5 * phi, rtol=1e-10, atol=1e-15)
    j_right = math3d.so3_right_jacobian(phi)
    assert np.allclose(j_right, np.eye(3) - 0.5 * math3d.skew(phi), atol=1e-11)


def test_normalization_rejects_zero_and_nonfinite():
    assert math3d.quaternion_normalize((0.0, 0.0, 0.0, 0.0)) is None
    assert math3d.vec3_normalize((float("nan"), 0.0, 0.0)) is None
