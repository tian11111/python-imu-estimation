import time

from imu_estimation.config import DeviceScale
from imu_estimation.mpu6500 import SensorConfig
from imu_estimation.runner import ImuEstimator, RuntimeConfig
from imu_estimation.types import RawSample, SAMPLE_STATUS_VALID


class StaticSensor(object):
    def __init__(self, config):
        self.config = config
        self.device_scale = DeviceScale(2048.0, 16.4, 333.87, 21.0)
        self.sequence = 0
        self.closed = False

    def initialize(self):
        self.closed = False
        return 0x70

    def read_raw(self, timestamp_us):
        self.sequence += 1
        return RawSample(self.sequence, timestamp_us, (0, 0, 2048),
                         (0, 0, 0), 0, SAMPLE_STATUS_VALID)

    def close(self):
        self.closed = True


def test_runner_reaches_ready_and_stops():
    runtime = RuntimeConfig(sample_rate_hz=100, stale_after_s=0.1)
    sensor_config = SensorConfig(sample_rate_hz=100)
    sensor = StaticSensor(sensor_config)
    estimator = ImuEstimator(sensor_config=sensor_config,
                             runtime_config=runtime, sensor=sensor)
    estimator.start()
    deadline = time.monotonic() + 1.2
    while estimator.get_angles() is None and time.monotonic() < deadline:
        time.sleep(0.01)
    angles = estimator.get_angles()
    assert angles is not None
    assert abs(angles.roll_deg) < 1.0e-8
    assert abs(angles.pitch_deg) < 1.0e-8
    assert estimator.get_health().ready
    estimator.stop()
    assert sensor.closed
    assert estimator.get_angles() is None


def test_restart_requires_fresh_stationary_calibration():
    runtime = RuntimeConfig(sample_rate_hz=100, stale_after_s=0.1)
    sensor_config = SensorConfig(sample_rate_hz=100)
    sensor = StaticSensor(sensor_config)
    estimator = ImuEstimator(sensor_config=sensor_config,
                             runtime_config=runtime, sensor=sensor)
    estimator.start()
    deadline = time.monotonic() + 1.2
    while estimator.get_angles() is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert estimator.get_angles() is not None
    estimator.stop()
    estimator.start()
    assert estimator.get_angles() is None
    estimator.stop()


def test_stop_is_idempotent():
    runtime = RuntimeConfig(sample_rate_hz=100)
    sensor_config = SensorConfig(sample_rate_hz=100)
    estimator = ImuEstimator(sensor_config=sensor_config,
                             runtime_config=runtime,
                             sensor=StaticSensor(sensor_config))
    estimator.start()
    estimator.stop()
    estimator.stop()
