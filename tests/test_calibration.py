import math
from pathlib import Path

import numpy as np

from imu_estimation.calibration import STANDARD_GRAVITY_MPS2, apply, identity_config, validate
from imu_estimation.config import CalibrationConfig, DeviceScale, load_calibration_json
from imu_estimation.types import (ImuStatus, RawSample, SAMPLE_STATUS_TEMP_CLAMPED,
                                  SAMPLE_STATUS_VALID)


DEVICE = DeviceScale(accel_lsb_per_g=16384.0, gyro_lsb_per_dps=131.0,
                     temperature_lsb_per_c=340.0, temperature_offset_c=36.53)


def test_identity_calibration_converts_units_and_preserves_metadata():
    raw = RawSample(sequence=7, timestamp_us=123, accel_raw=(0, 0, 16384),
                    gyro_raw=(131, 0, 0), temperature_raw=0, status=SAMPLE_STATUS_VALID)
    status, sample = apply(identity_config(), DEVICE, raw, 0.005)
    assert status == ImuStatus.OK
    assert sample.sequence == 7 and sample.timestamp_us == 123 and sample.status == SAMPLE_STATUS_VALID
    assert np.allclose(sample.specific_force_body_mps2, (0.0, 0.0, STANDARD_GRAVITY_MPS2))
    assert np.allclose(sample.gyro_pre_body_rad_s, (math.pi / 180.0, 0.0, 0.0))


def test_calibration_order_is_sensor_bias_then_matrix_then_mount_rotation():
    config = CalibrationConfig(accel_bias_sensor_mps2=(1.0, 0.0, 0.0),
                               accel_correction_matrix=((2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                               sensor_to_body=((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))
    raw = RawSample(accel_raw=(16384, 0, 0), gyro_raw=(0, 0, 0), status=SAMPLE_STATUS_VALID)
    status, sample = apply(config, DEVICE, raw, 0.0)
    assert status == ImuStatus.OK
    assert np.allclose(sample.specific_force_body_mps2, (0.0, 2.0 * (STANDARD_GRAVITY_MPS2 - 1.0), 0.0))


def test_temperature_is_clamped_only_for_model_input():
    config = CalibrationConfig(temperature_min_c=0.0, temperature_max_c=10.0,
                               gyro_temperature_coeff_sensor=((1.0, 1.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    # raw zero maps to 36.53 C, but the bias is evaluated at 10 C, delta=-15 C.
    status, sample = apply(config, DEVICE, RawSample(status=SAMPLE_STATUS_VALID), 0.0)
    assert status == ImuStatus.OK
    assert sample.status & SAMPLE_STATUS_TEMP_CLAMPED
    assert math.isclose(sample.temperature_c, 36.53)
    assert math.isclose(sample.gyro_pre_body_rad_s[0], 14.0)


def test_invalid_temperature_range_is_rejected():
    assert validate(CalibrationConfig(temperature_min_c=2.0, temperature_max_c=1.0)) == ImuStatus.CONFIG_ERROR


def test_rpi4b_car_profile_maps_side_mounted_mpu6500_to_body_frame():
    profile_path = (Path(__file__).resolve().parents[1] / "examples" /
                    "calibration.rpi4b-car-mpu6500.json")
    config = load_calibration_json(profile_path)
    rotation = np.asarray(config.sensor_to_body, dtype=float)
    assert np.allclose(np.dot(rotation, rotation.T), np.eye(3))
    assert math.isclose(float(np.linalg.det(rotation)), 1.0)

    device = DeviceScale(2048.0, 16.4, 333.87, 21.0)
    raw = RawSample(accel_raw=(16, -2041, 85), gyro_raw=(0, 0, 0),
                    status=SAMPLE_STATUS_VALID)
    status, sample = apply(config, device, raw, 0.005)
    assert status == ImuStatus.OK
    expected_sensor = (np.asarray(raw.accel_raw, dtype=float) / 2048.0 *
                       STANDARD_GRAVITY_MPS2)
    assert np.allclose(sample.specific_force_body_mps2,
                       (expected_sensor[0], expected_sensor[2], -expected_sensor[1]))
