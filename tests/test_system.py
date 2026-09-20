import numpy as np

from imu_estimation.calibration import STANDARD_GRAVITY_MPS2
from imu_estimation.config import DeviceScale, StationaryConfig
from imu_estimation.mekf import MekfConfig
from imu_estimation.system import ImuSystem, SystemConfig
from imu_estimation.types import (ImuStatus, RawSample, SAMPLE_STATUS_VALID,
                                  YawRateObservation)


def test_system_waits_for_startup_static_and_publishes_output():
    config = SystemConfig(stationary=StationaryConfig(window_samples=2, enter_static_windows=1))
    system = ImuSystem(config)
    device = DeviceScale(2048.0, 16.4, 340.0, 36.53)
    raw0 = RawSample(sequence=1, timestamp_us=10000, accel_raw=(0, 0, 2048), gyro_raw=(0, 0, 0), status=SAMPLE_STATUS_VALID)
    raw1 = RawSample(sequence=2, timestamp_us=20000, accel_raw=(0, 0, 2048), gyro_raw=(0, 0, 0), status=SAMPLE_STATUS_VALID)
    assert system.process_raw(raw0, device) == ImuStatus.OK
    assert system.get_output() is None
    assert system.process_raw(raw1, device) == ImuStatus.OK
    output = system.get_output()
    assert output is not None
    assert output.sequence == 2
    assert np.allclose(output.specific_force_body_mps2,
                       (0.0, 0.0, STANDARD_GRAVITY_MPS2))
    assert np.allclose(output.linear_acceleration_world_mps2,
                       (0.0, 0.0, 0.0), atol=1.0e-12)


def test_system_rejects_duplicate_yaw_observation_sequence():
    config = SystemConfig(stationary=StationaryConfig(window_samples=2, enter_static_windows=1))
    system = ImuSystem(config)
    device = DeviceScale(2048.0, 16.4, 340.0, 36.53)
    for index in range(2):
        assert system.process_raw(RawSample(sequence=index + 1, timestamp_us=(index + 1) * 10000,
                                            accel_raw=(0, 0, 2048), gyro_raw=(0, 0, 0), status=SAMPLE_STATUS_VALID), device) == ImuStatus.OK
    system.set_yaw_rate_provider(lambda timestamp: YawRateObservation(valid=True, source_sequence=7,
                                 rate_rad_s=0.0, variance_rad2_s2=1.0e-4))
    for index in range(2, 4):
        assert system.process_raw(RawSample(sequence=index + 1, timestamp_us=(index + 1) * 10000,
                                            accel_raw=(0, 0, 2048), gyro_raw=(0, 0, 0), status=SAMPLE_STATUS_VALID), device) == ImuStatus.OK
    assert system.mekf.body_yaw_rate_update_count == 1


def test_linear_acceleration_uses_body_to_world_attitude_before_gravity_removal():
    config = SystemConfig(stationary=StationaryConfig(window_samples=2, enter_static_windows=1))
    system = ImuSystem(config)
    device = DeviceScale(2048.0, 16.4, 340.0, 36.53)
    for index in range(2):
        raw = RawSample(sequence=index + 1, timestamp_us=(index + 1) * 10000,
                        accel_raw=(2048, 0, 0), gyro_raw=(0, 0, 0),
                        status=SAMPLE_STATUS_VALID)
        assert system.process_raw(raw, device) == ImuStatus.OK
    output = system.get_output()
    assert np.allclose(output.specific_force_body_mps2,
                       (STANDARD_GRAVITY_MPS2, 0.0, 0.0))
    assert np.allclose(output.linear_acceleration_world_mps2,
                       (0.0, 0.0, 0.0), atol=1.0e-12)
