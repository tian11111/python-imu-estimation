#!/usr/bin/env python3
"""Read the onboard MPU6500 and print car-friendly attitude snapshots."""
from __future__ import print_function

import argparse
import time

from imu_estimation import (ImuEstimator, RuntimeConfig, SensorConfig,
                            load_calibration_json)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x68)
    parser.add_argument("--rate", type=int, default=200)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--calibration")
    args = parser.parse_args()
    calibration = (load_calibration_json(args.calibration)
                   if args.calibration else None)
    sensor = SensorConfig(bus_number=args.bus, address_7bit=args.address,
                          sample_rate_hz=args.rate)
    runtime = RuntimeConfig(sample_rate_hz=args.rate)
    deadline = time.monotonic() + args.duration if args.duration > 0.0 else None
    with ImuEstimator(sensor_config=sensor,
                      calibration_config=calibration,
                      runtime_config=runtime) as estimator:
        while deadline is None or time.monotonic() < deadline:
            health = estimator.get_health()
            angles = estimator.get_angles()
            if angles is None:
                print("calibrating frames=%d rate=%.1fHz" %
                      (health.successful_frames, health.effective_rate_hz))
            else:
                print("roll=%+8.3f pitch=%+8.3f yaw=%+8.3f seq=%d rate=%.1fHz" %
                      (angles.roll_deg, angles.pitch_deg,
                       angles.relative_yaw_deg, angles.sequence,
                       health.effective_rate_hz))
            if health.faulted:
                raise RuntimeError(health.fault_message)
            time.sleep(0.1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
