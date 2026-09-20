import math

import numpy as np

from imu_estimation.mekf import Mekf, MekfConfig, _fused_yaw
from imu_estimation.types import ImuStatus


def initialized_filter():
    filter_ = Mekf(MekfConfig())
    assert filter_.initialize((0.0, 0.0, 9.80665), (0.0, 0.0, 0.0)) == ImuStatus.OK
    return filter_


def test_predict_integrates_body_z_rotation():
    filter_ = initialized_filter()
    assert filter_.predict((0.0, 0.0, 1.0), 0.01) == ImuStatus.OK
    assert abs(filter_.get_euler_rad()[2] - 0.01) < 1.0e-10
    assert filter_.prediction_count == 1


def test_specific_force_preserves_fused_yaw_and_rejects_non_gravity():
    filter_ = initialized_filter()
    assert filter_.predict((0.0, 0.0, 1.0), 0.01) == ImuStatus.OK
    before = filter_.get_euler_rad()[2]
    assert filter_.update_specific_force_direction((0.0, 0.0, 9.80665)) == ImuStatus.OK
    assert abs(filter_.get_euler_rad()[2] - before) < 1.0e-10
    assert filter_.update_specific_force_direction((0.0, 0.0, 1.0)) == ImuStatus.NO_DATA


def test_zaru_and_body_yaw_rate_only_change_bias():
    filter_ = initialized_filter()
    assert filter_.update_zero_angular_rate((0.002, 0.0, 0.0)) == ImuStatus.OK
    assert filter_.zaru_update_count == 1
    before = filter_.quaternion_wb.copy()
    assert filter_.update_body_yaw_rate((0.0, 0.0, 0.02), 0.0, 1.0e-5) == ImuStatus.OK
    assert np.allclose(filter_.quaternion_wb, before)
    assert filter_.body_yaw_rate_update_count == 1


def test_invalid_dt_does_not_advance_state():
    filter_ = initialized_filter()
    before = filter_.quaternion_wb.copy()
    assert filter_.predict((0.0, 0.0, 0.0), 0.1) == ImuStatus.TIME_ERROR
    assert np.array_equal(filter_.quaternion_wb, before)


def test_fused_yaw_matches_c_definition_when_tilted():
    quaternion = np.array((0.8, 0.2, -0.3, 0.45), dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    expected = math.atan2(2.0 * quaternion[0] * quaternion[3],
                          quaternion[0] ** 2 - quaternion[3] ** 2)
    assert abs(_fused_yaw(quaternion) - expected) < 1.0e-12


def test_initialize_rejects_invalid_force_shape_and_config():
    filter_ = Mekf(MekfConfig())
    assert filter_.initialize((0.0, 9.80665), (0.0, 0.0, 0.0)) == ImuStatus.INVALID_ARGUMENT
    bad = MekfConfig(maximum_dt_s=0.0)
    filter_ = Mekf(bad)
    assert filter_.initialize((0.0, 0.0, 9.80665), (0.0, 0.0, 0.0)) == ImuStatus.INVALID_ARGUMENT
