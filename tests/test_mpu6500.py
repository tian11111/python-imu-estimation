import struct

import pytest

from imu_estimation.calibration import apply
from imu_estimation.config import CalibrationConfig
from imu_estimation.mpu6500 import ImuDeviceNotFound, Mpu6500, SensorConfig
from imu_estimation.types import ImuStatus, SAMPLE_STATUS_VALID


class FakeBus(object):
    def __init__(self):
        self.writes = []

    def write_byte_data(self, address, register, value):
        self.writes.append((address, register, value))

    def read_byte_data(self, address, register):
        assert address == 0x68
        assert register == 0x75
        return 0x70

    def read_i2c_block_data(self, address, register, length):
        assert (address, register, length) == (0x68, 0x3B, 14)
        return list(struct.pack(">hhhhhhh", 100, -200, 2048, 1234, -1, 2, -3))


def test_mpu6500_initialization_and_raw_decode(monkeypatch):
    monkeypatch.setattr("imu_estimation.mpu6500.time.sleep", lambda _: None)
    bus = FakeBus()
    device = Mpu6500(SensorConfig(sample_rate_hz=200), bus=bus)
    assert device.initialize() == 0x70
    assert device.model == "MPU6500"
    assert device.device_scale.temperature_lsb_per_c == 333.87
    assert (0x68, 0x19, 4) in bus.writes
    assert (0x68, 0x1D, 2) in bus.writes
    sample = device.read_raw(123456)
    assert sample.sequence == 1
    assert sample.accel_raw == (100, -200, 2048)
    assert sample.gyro_raw == (-1, 2, -3)
    assert sample.temperature_raw == 1234
    assert sample.status & SAMPLE_STATUS_VALID
    status, converted = apply(CalibrationConfig(), device.device_scale,
                              sample, 0.005)
    assert status == ImuStatus.OK
    assert abs(converted.temperature_c - (1234.0 / 333.87 + 21.0)) < 1.0e-12


def test_unexpected_identity_clears_driver_state(monkeypatch):
    monkeypatch.setattr("imu_estimation.mpu6500.time.sleep", lambda _: None)
    bus = FakeBus()
    bus.read_byte_data = lambda address, register: 0x71
    device = Mpu6500(SensorConfig(), bus=bus)
    with pytest.raises(ImuDeviceNotFound):
        device.initialize()
    assert not device.initialized
    assert device.model is None
    assert device.device_scale is None


def test_invalid_range_is_rejected_before_bus_access():
    config = SensorConfig()
    config.gyro_range = 99
    with pytest.raises(ValueError):
        Mpu6500(config, bus=FakeBus()).initialize()
