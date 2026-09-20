"""Threaded in-process Raspberry Pi sampling facade."""
from __future__ import absolute_import

import copy
import math
import threading
import time
from dataclasses import dataclass

from .config import CalibrationConfig, StationaryConfig
from .mekf import MekfConfig
from .mpu6500 import ImuBusError, Mpu6500, SensorConfig
from .system import ImuSystem, SystemConfig
from .types import ImuStatus


@dataclass(frozen=True)
class Angles(object):
    roll_deg: float
    pitch_deg: float
    relative_yaw_deg: float
    sequence: int
    timestamp_us: int


@dataclass(frozen=True)
class Health(object):
    running: bool
    faulted: bool
    ready: bool
    stale: bool
    last_status: ImuStatus
    successful_frames: int
    bus_error_count: int
    consecutive_bus_errors: int
    missed_deadlines: int
    data_age_ms: float
    effective_rate_hz: float
    fault_message: str


@dataclass
class RuntimeConfig(object):
    sample_rate_hz: int = 200
    stale_after_s: float = 0.05
    maximum_consecutive_bus_errors: int = 20


class ImuEstimator(object):
    def __init__(self, sensor_config=None, calibration_config=None,
                 mekf_config=None, runtime_config=None, sensor=None):
        self.runtime_config = runtime_config if runtime_config is not None else RuntimeConfig()
        if self.runtime_config.sample_rate_hz <= 0:
            raise ValueError("sample_rate_hz must be positive")
        self.sensor_config = sensor_config if sensor_config is not None else SensorConfig(
            sample_rate_hz=self.runtime_config.sample_rate_hz)
        if self.sensor_config.sample_rate_hz != self.runtime_config.sample_rate_hz:
            raise ValueError("sensor and runtime sample rates must match")
        stationary = StationaryConfig(window_samples=max(
            2, int(round(self.runtime_config.sample_rate_hz * 0.2))))
        self._system_config = SystemConfig(
            calibration=calibration_config if calibration_config is not None else CalibrationConfig(),
            stationary=stationary,
            mekf=mekf_config if mekf_config is not None else MekfConfig())
        self.system = ImuSystem(self._system_config)
        self.sensor = sensor if sensor is not None else Mpu6500(self.sensor_config)
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread = None
        self._running = False
        self._faulted = False
        self._fault_message = ""
        self._last_status = ImuStatus.NO_DATA
        self._last_success_monotonic = None
        self._first_success_monotonic = None
        self._successful_frames = 0
        self._bus_error_count = 0
        self._consecutive_bus_errors = 0
        self._missed_deadlines = 0
        self._yaw_reference_rad = None
        self._pending_yaw_reference_rad = 0.0
        self._yaw_rate_provider = None

    def _reset_runtime_state(self):
        self.system = ImuSystem(self._system_config)
        self.system.set_yaw_rate_provider(self._yaw_rate_provider)
        self._last_status = ImuStatus.NO_DATA
        self._last_success_monotonic = None
        self._first_success_monotonic = None
        self._successful_frames = 0
        self._bus_error_count = 0
        self._consecutive_bus_errors = 0
        self._missed_deadlines = 0
        self._yaw_reference_rad = None
        self._pending_yaw_reference_rad = 0.0

    def start(self):
        with self._lock:
            if self._running:
                return
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("sampling thread is still stopping")
            self._reset_runtime_state()
            try:
                self.sensor.initialize()
            except Exception as error:
                self.sensor.close()
                self._faulted = True
                self._fault_message = "%s: %s" % (type(error).__name__, error)
                raise
            self._stop_event.clear()
            self._faulted = False
            self._fault_message = ""
            self._running = True
            self._thread = threading.Thread(target=self._run,
                                            name="imu-estimator", daemon=False)
            self._thread.start()

    def stop(self, timeout=2.0):
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout)
            if thread.is_alive():
                raise RuntimeError("IMU sampling thread did not stop")
        with self._lock:
            self._running = False
            self.sensor.close()
            self.system.output = None
            self.system.output_valid = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

    def _run(self):
        period = 1.0 / float(self.runtime_config.sample_rate_hz)
        deadline = time.monotonic()
        try:
            while not self._stop_event.is_set():
                now = time.monotonic()
                if now < deadline:
                    self._stop_event.wait(deadline - now)
                    if self._stop_event.is_set():
                        break
                    now = time.monotonic()
                elif now - deadline >= period:
                    skipped = int((now - deadline) // period)
                    self._missed_deadlines += skipped
                    deadline += skipped * period
                timestamp_us = time.monotonic_ns() // 1000
                try:
                    raw = self.sensor.read_raw(timestamp_us)
                    with self._lock:
                        status = self.system.process_raw(raw, self.sensor.device_scale)
                        self._last_status = status
                        if status.value >= 0:
                            self._successful_frames += 1
                            self._consecutive_bus_errors = 0
                            current = time.monotonic()
                            if self._first_success_monotonic is None:
                                self._first_success_monotonic = current
                            self._last_success_monotonic = current
                        elif status != ImuStatus.TIME_ERROR:
                            self._faulted = True
                            self._fault_message = "estimator status %s" % status.name
                            self._stop_event.set()
                except ImuBusError as error:
                    with self._lock:
                        self._last_status = ImuStatus.BUS_ERROR
                        self._bus_error_count += 1
                        self._consecutive_bus_errors += 1
                        if self._consecutive_bus_errors >= self.runtime_config.maximum_consecutive_bus_errors:
                            self._faulted = True
                            self._fault_message = str(error)
                            self._stop_event.set()
                deadline += period
        except Exception as error:
            with self._lock:
                self._faulted = True
                self._fault_message = "%s: %s" % (type(error).__name__, error)
                self._last_status = ImuStatus.NUMERIC_ERROR
                self._stop_event.set()
        finally:
            self.sensor.close()
            with self._lock:
                self._running = False

    def set_yaw_rate_provider(self, provider):
        with self._lock:
            self._yaw_rate_provider = provider
            self.system.set_yaw_rate_provider(provider)

    def reset_relative_yaw(self, new_yaw_deg=0.0):
        target = math.radians(float(new_yaw_deg))
        with self._lock:
            output = self.system.get_output()
            if output is None:
                self._yaw_reference_rad = None
                self._pending_yaw_reference_rad = target
            else:
                self._yaw_reference_rad = output.mekf_fused_yaw_rad - target
                self._pending_yaw_reference_rad = target

    def get_output(self):
        with self._lock:
            return copy.deepcopy(self.system.get_output())

    def get_raw(self):
        with self._lock:
            return copy.deepcopy(self.system.get_latest_raw())

    def get_angles(self):
        with self._lock:
            if self._faulted or not self._running:
                return None
            if (self._last_success_monotonic is None or
                    time.monotonic() - self._last_success_monotonic >
                    self.runtime_config.stale_after_s):
                return None
            output = self.system.get_output()
            if output is None:
                return None
            if self._yaw_reference_rad is None:
                self._yaw_reference_rad = (output.mekf_fused_yaw_rad -
                                           self._pending_yaw_reference_rad)
            relative = output.mekf_fused_yaw_rad - self._yaw_reference_rad
            relative = (relative + math.pi) % (2.0 * math.pi) - math.pi
            return Angles(math.degrees(output.gravity_tilt_roll_rad),
                          math.degrees(output.gravity_tilt_pitch_rad),
                          math.degrees(relative), output.sequence,
                          output.timestamp_us)

    def get_health(self):
        with self._lock:
            now = time.monotonic()
            if self._last_success_monotonic is None:
                age = float("inf")
            else:
                age = now - self._last_success_monotonic
            elapsed = 0.0 if self._first_success_monotonic is None else max(
                0.0, now - self._first_success_monotonic)
            rate = 0.0 if elapsed <= 0.0 else max(
                0, self._successful_frames - 1) / elapsed
            return Health(self._running, self._faulted,
                          self.system.get_output() is not None,
                          age > self.runtime_config.stale_after_s,
                          self._last_status, self._successful_frames,
                          self._bus_error_count, self._consecutive_bus_errors,
                          self._missed_deadlines, age * 1000.0, rate,
                          self._fault_message)
