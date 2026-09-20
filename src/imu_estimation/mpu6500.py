"""MPU6050/MPU6500 I2C driver for Raspberry Pi polling."""
from __future__ import absolute_import

import struct
import time
from dataclasses import dataclass
from enum import IntEnum

from .config import DeviceScale
from .types import (RawSample, SAMPLE_STATUS_ACCEL_SATURATED,
                    SAMPLE_STATUS_GYRO_SATURATED, SAMPLE_STATUS_VALID)


class GyroRange(IntEnum):
    DPS_250 = 0
    DPS_500 = 1
    DPS_1000 = 2
    DPS_2000 = 3


class AccelRange(IntEnum):
    G_2 = 0
    G_4 = 1
    G_8 = 2
    G_16 = 3


class Dlpf(IntEnum):
    HZ_260 = 0
    HZ_184 = 1
    HZ_94 = 2
    HZ_44 = 3
    HZ_21 = 4
    HZ_10 = 5
    HZ_5 = 6


@dataclass
class SensorConfig(object):
    bus_number: int = 1
    address_7bit: int = 0x68
    sample_rate_hz: int = 200
    gyro_range: GyroRange = GyroRange.DPS_2000
    accel_range: AccelRange = AccelRange.G_16
    dlpf: Dlpf = Dlpf.HZ_94


class ImuBusError(IOError):
    pass


class ImuDeviceNotFound(ImuBusError):
    pass


class Mpu6500(object):
    REG_SMPLRT_DIV = 0x19
    REG_CONFIG = 0x1A
    REG_GYRO_CONFIG = 0x1B
    REG_ACCEL_CONFIG = 0x1C
    REG_ACCEL_CONFIG_2 = 0x1D
    REG_FIFO_EN = 0x23
    REG_INT_PIN_CFG = 0x37
    REG_INT_ENABLE = 0x38
    REG_ACCEL_XOUT_H = 0x3B
    REG_SIGNAL_PATH_RESET = 0x68
    REG_USER_CTRL = 0x6A
    REG_PWR_MGMT_1 = 0x6B
    REG_PWR_MGMT_2 = 0x6C
    REG_WHO_AM_I = 0x75

    def __init__(self, config=None, bus=None):
        self.config = config if config is not None else SensorConfig()
        self._bus = bus
        self._owns_bus = bus is None
        self.model = None
        self.device_scale = None
        self.sample_sequence = 0
        self.initialized = False

    @staticmethod
    def _open_bus(bus_number):
        try:
            from smbus2 import SMBus
        except ImportError:
            try:
                from smbus import SMBus
            except ImportError as error:
                raise ImuBusError("install python3-smbus or smbus2") from error
        return SMBus(bus_number)

    def _write(self, register, value):
        try:
            self._bus.write_byte_data(self.config.address_7bit, register, value)
        except (IOError, OSError) as error:
            raise ImuBusError(str(error)) from error

    def _read_byte(self, register):
        try:
            return int(self._bus.read_byte_data(self.config.address_7bit, register))
        except (IOError, OSError) as error:
            raise ImuBusError(str(error)) from error

    def initialize(self):
        config = self.config
        self.initialized = False
        self.model = None
        self.device_scale = None
        self.sample_sequence = 0
        if config.address_7bit not in (0x68, 0x69):
            raise ValueError("MPU address must be 0x68 or 0x69")
        try:
            gyro_range = GyroRange(config.gyro_range)
            accel_range = AccelRange(config.accel_range)
            dlpf = Dlpf(config.dlpf)
        except ValueError as error:
            raise ValueError("invalid MPU range or DLPF setting") from error
        base_rate = 8000 if dlpf == Dlpf.HZ_260 else 1000
        if config.sample_rate_hz <= 0 or base_rate % config.sample_rate_hz != 0:
            raise ValueError("sample rate must divide the MPU base rate exactly")
        divider_plus_one = base_rate // config.sample_rate_hz
        if divider_plus_one < 1 or divider_plus_one > 256:
            raise ValueError("sample divider is out of range")
        try:
            if self._bus is None:
                self._bus = self._open_bus(config.bus_number)
            self._write(self.REG_PWR_MGMT_1, 0x80)
            time.sleep(0.1)
            self._write(self.REG_PWR_MGMT_1, 0x01)
            self._write(self.REG_PWR_MGMT_2, 0x00)
            self._write(self.REG_SIGNAL_PATH_RESET, 0x07)
            time.sleep(0.1)

            who_am_i = self._read_byte(self.REG_WHO_AM_I)
            if who_am_i == 0x68:
                self.model = "MPU6050"
                temperature_scale = (340.0, 36.53)
            elif who_am_i == 0x70:
                self.model = "MPU6500"
                temperature_scale = (333.87, 21.0)
            else:
                raise ImuDeviceNotFound("unexpected WHO_AM_I 0x%02x" % who_am_i)

            self._write(self.REG_SMPLRT_DIV, divider_plus_one - 1)
            self._write(self.REG_CONFIG, int(dlpf))
            self._write(self.REG_GYRO_CONFIG, int(gyro_range) << 3)
            self._write(self.REG_ACCEL_CONFIG, int(accel_range) << 3)
            if self.model == "MPU6500":
                self._write(self.REG_ACCEL_CONFIG_2, int(dlpf))
            self._write(self.REG_FIFO_EN, 0x00)
            self._write(self.REG_USER_CTRL, 0x00)
            self._write(self.REG_INT_PIN_CFG, 0x10)
            self._write(self.REG_INT_ENABLE, 0x00)

            accel_lsb = (16384.0, 8192.0, 4096.0, 2048.0)[int(accel_range)]
            gyro_lsb = (131.0, 65.5, 32.8, 16.4)[int(gyro_range)]
            self.device_scale = DeviceScale(accel_lsb, gyro_lsb,
                                            temperature_scale[0], temperature_scale[1])
            self.initialized = True
            return who_am_i
        except Exception:
            if self._owns_bus:
                self.close()
            raise

    def read_raw(self, timestamp_us):
        if not self.initialized:
            raise ImuBusError("device is not initialized")
        try:
            data = self._bus.read_i2c_block_data(
                self.config.address_7bit, self.REG_ACCEL_XOUT_H, 14)
        except (IOError, OSError) as error:
            raise ImuBusError(str(error)) from error
        if len(data) != 14:
            raise ImuBusError("short I2C frame: %d bytes" % len(data))
        values = struct.unpack(">hhhhhhh", bytes(bytearray(data)))
        accel = values[0:3]
        gyro = values[4:7]
        status = SAMPLE_STATUS_VALID
        if any(value in (-32768, 32767) for value in accel):
            status |= SAMPLE_STATUS_ACCEL_SATURATED
        if any(value in (-32768, 32767) for value in gyro):
            status |= SAMPLE_STATUS_GYRO_SATURATED
        self.sample_sequence += 1
        return RawSample(sequence=self.sample_sequence,
                         timestamp_us=int(timestamp_us),
                         accel_raw=accel,
                         gyro_raw=gyro,
                         temperature_raw=values[3],
                         status=status)

    def close(self):
        if self._owns_bus and self._bus is not None:
            close = getattr(self._bus, "close", None)
            if close is not None:
                close()
            self._bus = None
        self.initialized = False
