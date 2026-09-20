#!/usr/bin/env python3
"""PyQt5 digital dashboard for the Raspberry Pi MPU6500 estimator."""
from __future__ import print_function

import argparse
import math
import sys
from pathlib import Path

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
    PYQT_IMPORT_ERROR = None
except ImportError as error:
    QtCore = QtGui = QtWidgets = None
    PYQT_IMPORT_ERROR = error

from imu_estimation import (ImuEstimator, RuntimeConfig, SensorConfig,
                            load_calibration_json)


DEFAULT_PROFILE = Path(__file__).with_name("calibration.rpi4b-car-mpu6500.json")
DEG_PER_RAD = 180.0 / math.pi
MISSING = "--"


def _degrees(value):
    return float(value) * DEG_PER_RAD


def _format(value, digits=3):
    if value is None:
        return MISSING
    try:
        value = float(value)
    except (TypeError, ValueError):
        return MISSING
    return ("%%.%df" % digits) % value if math.isfinite(value) else MISSING


if QtWidgets is not None:
    class ImuDashboard(QtWidgets.QMainWindow):
        """A 20 Hz display facade; the estimator keeps owning I2C sampling."""

        def __init__(self, estimator, bus, address, rate, calibration_path):
            super(ImuDashboard, self).__init__()
            self.estimator = estimator
            self._values = {}
            self._telemetry_keys = []
            self.setWindowTitle("MPU6500 IMU Dashboard")
            self.setMinimumSize(900, 650)
            self.resize(980, 700)
            self._build_ui(bus, address, rate, calibration_path)
            self._refresh_timer = QtCore.QTimer(self)
            self._refresh_timer.setInterval(50)
            self._refresh_timer.timeout.connect(self.refresh)
            self._refresh_timer.start()
            QtCore.QTimer.singleShot(0, self.start_estimator)

        def _build_ui(self, bus, address, rate, calibration_path):
            central = QtWidgets.QWidget(self)
            root = QtWidgets.QVBoxLayout(central)
            root.setContentsMargins(10, 10, 10, 10)

            controls = QtWidgets.QHBoxLayout()
            title = QtWidgets.QLabel("MPU6500 六轴姿态监控")
            title.setStyleSheet("font-size: 20px; font-weight: bold;")
            controls.addWidget(title)
            controls.addStretch(1)
            self.start_button = QtWidgets.QPushButton("Start")
            self.stop_button = QtWidgets.QPushButton("Stop")
            self.yaw_button = QtWidgets.QPushButton("Yaw 清零")
            self.start_button.clicked.connect(self.start_estimator)
            self.stop_button.clicked.connect(self.stop_estimator)
            self.yaw_button.clicked.connect(lambda: self.estimator.reset_relative_yaw())
            controls.addWidget(self.start_button)
            controls.addWidget(self.stop_button)
            controls.addWidget(self.yaw_button)
            root.addLayout(controls)

            info = QtWidgets.QLabel(
                "I2C%d  地址 0x%02X  采样 %d Hz  标定 %s" %
                (bus, address, rate, Path(calibration_path).name))
            info.setStyleSheet("color: #555;")
            root.addWidget(info)

            self.status_label = QtWidgets.QLabel("STOPPED")
            self.status_label.setAlignment(QtCore.Qt.AlignCenter)
            self.status_label.setMinimumHeight(28)
            root.addWidget(self.status_label)

            tabs = QtWidgets.QTabWidget()
            tabs.addTab(self._overview_tab(), "概览")
            tabs.addTab(self._raw_tab(), "原始数据")
            tabs.addTab(self._diagnostics_tab(), "解算诊断")
            root.addWidget(tabs, 1)
            self.setCentralWidget(central)

        def _value_label(self):
            label = QtWidgets.QLabel(MISSING)
            label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            label.setStyleSheet("font-family: monospace; font-size: 15px; font-weight: bold;")
            return label

        def _group(self, title, rows):
            group = QtWidgets.QGroupBox(title)
            form = QtWidgets.QFormLayout(group)
            for key, caption in rows:
                label = self._value_label()
                self._values[key] = label
                self._telemetry_keys.append(key)
                form.addRow(caption, label)
            return group

        def _grid_tab(self, groups):
            page = QtWidgets.QWidget()
            layout = QtWidgets.QGridLayout(page)
            for index, group in enumerate(groups):
                layout.addWidget(group, index // 2, index % 2)
            layout.setRowStretch((len(groups) - 1) // 2 + 1, 1)
            return page

        def _overview_tab(self):
            return self._grid_tab((
                self._group("姿态（度）", (
                    ("roll", "Roll"), ("pitch", "Pitch"),
                    ("yaw", "相对 Yaw"), ("quat", "四元数 [w,x,y,z]"))),
                self._group("校正角速度（度/秒）", (
                    ("gyro_x", "X"), ("gyro_y", "Y"), ("gyro_z", "Z"),
                    ("temperature", "温度（°C）"))),
                self._group("机体系比力（m/s²）", (
                    ("force_x", "X"), ("force_y", "Y"), ("force_z", "Z"),
                    ("force_norm", "模长"))),
                self._group("世界系线加速度（m/s²）", (
                    ("linear_x", "X"), ("linear_y", "Y"), ("linear_z", "Z"),
                    ("linear_norm", "模长"), ("speed", "线速度"))),
                self._group("运行健康", (
                    ("rate", "有效频率（Hz）"), ("age", "数据年龄（ms）"),
                    ("frames", "成功帧"), ("bus_errors", "I2C 错误"),
                    ("deadlines", "错过截止"))),
            ))

        def _raw_tab(self):
            return self._grid_tab((
                self._group("加速度原始计数", (
                    ("raw_accel_x", "X"), ("raw_accel_y", "Y"),
                    ("raw_accel_z", "Z"), ("raw_temperature", "温度原始值"))),
                self._group("陀螺原始计数", (
                    ("raw_gyro_x", "X"), ("raw_gyro_y", "Y"),
                    ("raw_gyro_z", "Z"), ("raw_status", "状态位"))),
                self._group("帧信息", (
                    ("raw_sequence", "序号"), ("raw_timestamp", "时间戳（us）"),
                    ("model_status", "估计器状态"))),
                self._group("陀螺零偏（度/秒）", (
                    ("bias_x", "X"), ("bias_y", "Y"), ("bias_z", "Z"))),
            ))

        def _diagnostics_tab(self):
            return self._grid_tab((
                self._group("角度对照（度）", (
                    ("euler_roll", "MEKF Roll"), ("euler_pitch", "MEKF Pitch"),
                    ("euler_yaw", "MEKF Yaw"), ("accel_roll", "加速度 Roll"),
                    ("accel_pitch", "加速度 Pitch"))),
                self._group("陀螺积分对照（度）", (
                    ("gyro_only_pitch", "Pitch"), ("gyro_only_yaw", "Yaw"),
                    ("initial_roll", "初始 Roll"), ("initial_pitch", "初始 Pitch"))),
                self._group("姿态不确定度（度²）", (
                    ("variance_x", "Roll"), ("variance_y", "Pitch"),
                    ("variance_z", "Yaw"), ("flags", "状态标志"))),
                self._group("滤波器更新", (
                    ("force_updates", "比力更新"), ("zaru_updates", "ZARU 更新"),
                    ("consecutive_errors", "连续 I2C 错误"))),
            ))

        def _set(self, key, value):
            self._values[key].setText(str(value))

        def _clear_telemetry(self):
            for key in self._telemetry_keys:
                self._set(key, MISSING)

        def _set_status(self, text, color):
            self.status_label.setText(text)
            self.status_label.setStyleSheet(
                "background: %s; color: white; font-weight: bold; border-radius: 4px;" % color)

        def _set_buttons(self, health):
            running = health.running and not health.faulted
            self.start_button.setEnabled(not running)
            self.stop_button.setEnabled(health.running)
            self.yaw_button.setEnabled(running and health.ready and not health.stale)

        def _set_vector(self, keys, vector, converter=lambda value: value):
            for key, value in zip(keys, vector):
                self._set(key, _format(converter(value)))

        def start_estimator(self):
            try:
                self.estimator.start()
            except Exception as error:
                self._set_status("START FAILED: %s" % error, "#b71c1c")
                self._clear_telemetry()
            self.refresh()

        def stop_estimator(self):
            try:
                self.estimator.stop()
            except Exception as error:
                self._set_status("STOP FAILED: %s" % error, "#b71c1c")
            self.refresh()

        def refresh(self):
            try:
                health = self.estimator.get_health()
                raw = self.estimator.get_raw()
                output = self.estimator.get_output()
                angles = self.estimator.get_angles()
            except Exception as error:
                self._set_status("DISPLAY ERROR: %s" % error, "#b71c1c")
                self._clear_telemetry()
                return

            self._set_buttons(health)
            self._set("rate", _format(health.effective_rate_hz, 1))
            self._set("age", _format(health.data_age_ms, 2))
            self._set("frames", health.successful_frames)
            self._set("bus_errors", health.bus_error_count)
            self._set("deadlines", health.missed_deadlines)
            self._set("consecutive_errors", health.consecutive_bus_errors)
            self._set("model_status", health.last_status.name)

            if health.faulted:
                self._set_status("FAULT: %s" % (health.fault_message or health.last_status.name), "#b71c1c")
                self._clear_telemetry()
                return
            if not health.running:
                self._set_status("STOPPED", "#616161")
                self._clear_telemetry()
                return
            if health.stale:
                self._set_status("STALE DATA", "#b71c1c")
                self._clear_telemetry()
                return
            if health.ready:
                self._set_status("READY", "#2e7d32")
            else:
                self._set_status("CALIBRATING - keep the car still", "#f9a825")

            if raw is not None:
                self._set_vector(("raw_accel_x", "raw_accel_y", "raw_accel_z"), raw.accel_raw,
                                 lambda value: int(value))
                self._set_vector(("raw_gyro_x", "raw_gyro_y", "raw_gyro_z"), raw.gyro_raw,
                                 lambda value: int(value))
                self._set("raw_temperature", raw.temperature_raw)
                self._set("raw_status", "0x%02X" % raw.status)
                self._set("raw_sequence", raw.sequence)
                self._set("raw_timestamp", raw.timestamp_us)

            if output is None or angles is None:
                return
            self._set("roll", _format(angles.roll_deg))
            self._set("pitch", _format(angles.pitch_deg))
            self._set("yaw", _format(angles.relative_yaw_deg))
            self._set("quat", "[" + ", ".join(_format(value, 5) for value in output.quaternion_wb) + "]")
            self._set_vector(("gyro_x", "gyro_y", "gyro_z"), output.gyro_corrected_body_rad_s, _degrees)
            self._set("temperature", _format(output.temperature_c, 2))
            self._set_vector(("force_x", "force_y", "force_z"), output.specific_force_body_mps2)
            self._set("force_norm", _format(math.sqrt(
                sum(value * value for value in output.specific_force_body_mps2))))
            self._set_vector(("linear_x", "linear_y", "linear_z"), output.linear_acceleration_world_mps2)
            self._set("linear_norm", _format(math.sqrt(
                sum(value * value for value in output.linear_acceleration_world_mps2))))
            self._set("speed", "未接入（六轴 IMU 不积分）")
            self._set_vector(("bias_x", "bias_y", "bias_z"), output.gyro_bias_body_rad_s, _degrees)
            self._set("euler_roll", _format(_degrees(output.roll_rad)))
            self._set("euler_pitch", _format(_degrees(output.pitch_rad)))
            self._set("euler_yaw", _format(_degrees(output.yaw_rad)))
            self._set("accel_roll", _format(_degrees(output.accel_roll_rad)))
            self._set("accel_pitch", _format(_degrees(output.accel_pitch_rad)))
            self._set("gyro_only_pitch", _format(_degrees(output.gyro_only_pitch_rad)))
            self._set("gyro_only_yaw", _format(_degrees(output.gyro_only_fused_yaw_rad)))
            self._set("initial_roll", _format(_degrees(output.initial_roll_rad)))
            self._set("initial_pitch", _format(_degrees(output.initial_pitch_rad)))
            variance_scale = DEG_PER_RAD * DEG_PER_RAD
            self._set_vector(("variance_x", "variance_y", "variance_z"), output.attitude_error_variance_rad2,
                             lambda value: value * variance_scale)
            self._set("flags", "0x%02X" % output.state_flags)
            self._set("force_updates", output.specific_force_update_count)
            self._set("zaru_updates", output.zaru_update_count)

        def closeEvent(self, event):
            self._refresh_timer.stop()
            try:
                self.estimator.stop()
            except Exception:
                pass
            event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x68)
    parser.add_argument("--rate", type=int, default=200)
    parser.add_argument("--calibration", default=str(DEFAULT_PROFILE))
    parser.add_argument("--duration", type=float, default=0.0,
                        help="auto-close after this many seconds; 0 keeps the window open")
    args = parser.parse_args()
    if PYQT_IMPORT_ERROR is not None:
        print("PyQt5 is required. Install it with: sudo apt-get install python3-pyqt5", file=sys.stderr)
        return 2
    if args.duration < 0.0:
        parser.error("duration must be zero or positive")
    if not sys.platform.startswith("win") and not ("DISPLAY" in __import__("os").environ or
                                                    "WAYLAND_DISPLAY" in __import__("os").environ):
        print("No graphical display is available. Run this from the Raspberry Pi desktop session.",
              file=sys.stderr)
        return 2

    calibration = load_calibration_json(args.calibration)
    sensor = SensorConfig(bus_number=args.bus, address_7bit=args.address,
                          sample_rate_hz=args.rate)
    estimator = ImuEstimator(sensor_config=sensor, calibration_config=calibration,
                             runtime_config=RuntimeConfig(sample_rate_hz=args.rate))
    application = QtWidgets.QApplication(sys.argv[:1])
    window = ImuDashboard(estimator, args.bus, args.address, args.rate, args.calibration)
    window.show()
    if args.duration > 0.0:
        QtCore.QTimer.singleShot(int(args.duration * 1000), window.close)
    return application.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
