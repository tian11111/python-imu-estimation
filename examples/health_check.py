#!/usr/bin/env python3
"""Run a bounded Raspberry Pi IMU health and timing acceptance check."""
from __future__ import print_function

import argparse
import json
import math
import time
from pathlib import Path

from imu_estimation import (ImuEstimator, RuntimeConfig, SensorConfig,
                            load_calibration_json)


DEFAULT_PROFILE = Path(__file__).with_name("calibration.rpi4b-car-mpu6500.json")


def _finite_or_none(value):
    return float(value) if math.isfinite(float(value)) else None


def _health_dict(health):
    return {
        "running": health.running,
        "faulted": health.faulted,
        "ready": health.ready,
        "stale": health.stale,
        "last_status": health.last_status.name,
        "successful_frames": health.successful_frames,
        "bus_error_count": health.bus_error_count,
        "consecutive_bus_errors": health.consecutive_bus_errors,
        "missed_deadlines": health.missed_deadlines,
        "data_age_ms": _finite_or_none(health.data_age_ms),
        "effective_rate_hz": _finite_or_none(health.effective_rate_hz),
        "fault_message": health.fault_message,
    }


def _angles_dict(angles):
    if angles is None:
        return None
    return {
        "roll_deg": angles.roll_deg,
        "pitch_deg": angles.pitch_deg,
        "relative_yaw_deg": angles.relative_yaw_deg,
        "sequence": angles.sequence,
        "timestamp_us": angles.timestamp_us,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x68)
    parser.add_argument("--rate", type=int, default=200)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--report-interval", type=float, default=5.0)
    parser.add_argument("--calibration", default=str(DEFAULT_PROFILE))
    args = parser.parse_args()
    if args.duration <= 0.0 or args.report_interval <= 0.0:
        parser.error("duration and report interval must be positive")

    calibration = load_calibration_json(args.calibration)
    sensor = SensorConfig(bus_number=args.bus, address_7bit=args.address,
                          sample_rate_hz=args.rate)
    runtime = RuntimeConfig(sample_rate_hz=args.rate)
    started = time.monotonic()
    deadline = started + args.duration
    next_report = started
    ready_after_s = None
    last_angles = None
    final_health = None

    with ImuEstimator(sensor_config=sensor,
                      calibration_config=calibration,
                      runtime_config=runtime) as estimator:
        while time.monotonic() < deadline:
            now = time.monotonic()
            health = estimator.get_health()
            angles = estimator.get_angles()
            if angles is not None:
                last_angles = angles
                if ready_after_s is None:
                    ready_after_s = now - started
            if now >= next_report:
                print(json.dumps({"elapsed_s": round(now - started, 3),
                                  "health": _health_dict(health),
                                  "angles": _angles_dict(angles)}, sort_keys=True))
                next_report += args.report_interval
            if health.faulted:
                break
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        final_health = estimator.get_health()
        last_angles = estimator.get_angles() or last_angles

    minimum_frames = int(args.duration * args.rate * 0.90)
    maximum_missed = max(1, int(args.duration * args.rate * 0.02))
    failures = []
    if final_health.faulted:
        failures.append("estimator faulted")
    if not final_health.ready or last_angles is None:
        failures.append("estimator did not become ready")
    if final_health.stale:
        failures.append("data was stale at completion")
    if final_health.last_status.name != "OK":
        failures.append("last status was %s" % final_health.last_status.name)
    if final_health.bus_error_count != 0:
        failures.append("I2C bus errors were observed")
    if final_health.successful_frames < minimum_frames:
        failures.append("successful frame count below 90 percent of target")
    if final_health.effective_rate_hz < args.rate * 0.90:
        failures.append("effective rate below 90 percent of target")
    if final_health.data_age_ms > runtime.stale_after_s * 1000.0:
        failures.append("final data age exceeded stale threshold")
    if final_health.missed_deadlines > maximum_missed:
        failures.append("more than 2 percent of deadlines were missed")

    summary = {
        "pass": not failures,
        "duration_s": args.duration,
        "target_rate_hz": args.rate,
        "ready_after_s": _finite_or_none(ready_after_s) if ready_after_s is not None else None,
        "maximum_allowed_missed_deadlines": maximum_missed,
        "minimum_required_frames": minimum_frames,
        "health": _health_dict(final_health),
        "angles": _angles_dict(last_angles),
        "failures": failures,
    }
    print("FINAL_METRICS=" + json.dumps(summary, sort_keys=True))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
