# Raspberry Pi 4B Python IMU

这是原 C 版六轴 MEKF 的 Python 移植，面向树莓派 4B 小车。当前适配的实机是
I2C1 地址 `0x68`、`WHO_AM_I=0x70` 的 MPU6500 兼容设备。

## 环境

- Python 3.7 或更高版本
- NumPy 1.17 或更高版本
- Raspberry Pi OS 自带的 `python3-smbus`，也兼容可选的 `smbus2`

在本目录安装：

```bash
sudo apt-get install python3-numpy python3-smbus
python3 -m pip install --user -e . --no-deps
```

目标树莓派是 ARMv7/Python 3.7，优先使用系统已经提供的 NumPy 1.17 和
`python3-smbus`。`--no-deps` 用于避免 pip 在板上尝试源码编译新版 NumPy。

树莓派必须存在 `/dev/i2c-1`，用户需要属于 `i2c` 组。

## 实机测试

上电后让小车保持静止至少一秒，然后运行：

```bash
python3 examples/car_demo.py \
  --calibration examples/calibration.rpi4b-car-mpu6500.json \
  --duration 10
```

默认配置为 200 Hz、94 Hz DLPF、陀螺 `+-2000 dps`、加速度 `+-16 g`。
程序约用 600 ms 的连续静止窗口完成倾角和陀螺零偏初始化；完成前
`get_angles()` 返回 `None`。

`calibration.rpi4b-car-mpu6500.json` 使用当前侧装 MPU6500 的候选右手系映射：
`Body X = Sensor X`、`Body Y = Sensor Z`、`Body Z = -Sensor Y`。实测已经确认
`Sensor -Y` 朝上；该档案进一步假设 `Sensor X` 朝车头。若板上 X 箭头不朝车头，
必须先修正 XY 映射，不能只凭静止 Roll/Pitch 判断车头方向。

部署到树莓派后执行 60 秒稳定性验收：

```bash
cd /home/pi/imu-estimation
python3 -m pip install --user -e . --no-deps
python3 examples/health_check.py --duration 60
```

脚本每 5 秒输出完整健康快照，结束时输出 `FINAL_METRICS` JSON。验收要求包括：
无故障、无 I2C 错误、数据未过期、最终状态为 `OK`、有效频率和成功帧数不低于
目标的 90%，并且错过的调度截止时间不超过目标帧数的 2%。

## 小车程序集成

```python
from imu_estimation import ImuEstimator

from imu_estimation import load_calibration_json

calibration = load_calibration_json(
    "examples/calibration.rpi4b-car-mpu6500.json")
imu = ImuEstimator(calibration_config=calibration)
imu.start()
try:
    angles = imu.get_angles()
    health = imu.get_health()
finally:
    imu.stop()
```

`roll_deg` 和 `pitch_deg` 是与 Yaw 解耦的重力倾斜角；`relative_yaw_deg` 是可清零的
短期相对航向。六轴 IMU 没有绝对航向观测，Yaw 长期一定会漂移。

差速轮角速度可通过 `set_yaw_rate_provider()` 注入。回调接收当前 IMU 微秒时间戳，
返回新的 `YawRateObservation`；每条轮速必须使用新的 `source_sequence`，不得重复融合。

标定档案格式见 `examples/calibration.identity.json`。单位档案只能用于先验证数据链路，
不能替代针对当前传感器的六面、比例和温漂标定。
