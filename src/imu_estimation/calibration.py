"""Deterministic raw-count to body-frame calibration pipeline."""
from __future__ import absolute_import

import math
import numpy as np

from .config import CalibrationConfig, DeviceScale
from .math3d import vec3_is_finite
from .types import ImuSample, ImuStatus, SAMPLE_STATUS_TEMP_CLAMPED

STANDARD_GRAVITY_MPS2 = 9.80665
DEGREES_TO_RADIANS = math.pi / 180.0


def identity_config():
    return CalibrationConfig()


def validate(config):
    if config is None:
        return ImuStatus.CONFIG_ERROR
    try:
        matrices = (config.gyro_temperature_coeff_sensor, config.gyro_correction_matrix,
                    config.accel_correction_matrix, config.sensor_to_body)
        scalars = (config.temperature_reference_c, config.temperature_min_c,
                   config.temperature_max_c)
        if (not np.all(np.isfinite(scalars)) or config.temperature_min_c > config.temperature_max_c or
                not all(np.asarray(matrix, dtype=float).shape == (3, 3) and np.all(np.isfinite(matrix))
                        for matrix in matrices) or not vec3_is_finite(config.accel_bias_sensor_mps2)):
            return ImuStatus.CONFIG_ERROR
    except (AttributeError, TypeError, ValueError):
        return ImuStatus.CONFIG_ERROR
    return ImuStatus.OK


def apply(config, device, raw, dt_s):
    """Apply C-equivalent calibration; runtime MEKF bias is deliberately excluded."""
    if (config is None or device is None or raw is None or not math.isfinite(dt_s) or dt_s < 0.0):
        return ImuStatus.INVALID_ARGUMENT, None
    try:
        if (device.accel_lsb_per_g <= 0.0 or device.gyro_lsb_per_dps <= 0.0 or
                device.temperature_lsb_per_c <= 0.0 or not math.isfinite(device.temperature_offset_c)):
            return ImuStatus.INVALID_ARGUMENT, None
    except AttributeError:
        return ImuStatus.INVALID_ARGUMENT, None
    status = validate(config)
    if status != ImuStatus.OK:
        return status, None
    temperature_c = raw.temperature_raw / device.temperature_lsb_per_c + device.temperature_offset_c
    model_temperature_c = min(max(temperature_c, config.temperature_min_c), config.temperature_max_c)
    sample_status = raw.status
    if model_temperature_c != temperature_c:
        sample_status |= SAMPLE_STATUS_TEMP_CLAMPED
    temperature_delta = model_temperature_c - config.temperature_reference_c
    accel_sensor = (np.asarray(raw.accel_raw, dtype=float) / device.accel_lsb_per_g * STANDARD_GRAVITY_MPS2 -
                    np.asarray(config.accel_bias_sensor_mps2, dtype=float))
    gyro_sensor = np.asarray(raw.gyro_raw, dtype=float) / device.gyro_lsb_per_dps * DEGREES_TO_RADIANS
    coeff = np.asarray(config.gyro_temperature_coeff_sensor, dtype=float)
    gyro_sensor -= coeff[:, 0] + coeff[:, 1] * temperature_delta + coeff[:, 2] * temperature_delta * temperature_delta
    specific_force = np.dot(np.asarray(config.sensor_to_body, dtype=float),
                            np.dot(np.asarray(config.accel_correction_matrix, dtype=float), accel_sensor))
    gyro_pre = np.dot(np.asarray(config.sensor_to_body, dtype=float),
                      np.dot(np.asarray(config.gyro_correction_matrix, dtype=float), gyro_sensor))
    if not vec3_is_finite(specific_force) or not vec3_is_finite(gyro_pre) or not math.isfinite(temperature_c):
        return ImuStatus.NUMERIC_ERROR, None
    return ImuStatus.OK, ImuSample(sequence=raw.sequence, timestamp_us=raw.timestamp_us, dt_s=dt_s,
                                   specific_force_body_mps2=specific_force, gyro_pre_body_rad_s=gyro_pre,
                                   temperature_c=temperature_c, status=sample_status)
