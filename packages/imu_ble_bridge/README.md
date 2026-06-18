# imu_ble_bridge

WT BLE IMU 数据采集模块。完整采集流程见 `../../docs/system_workflow.md`。

## 职责

```text
WT-series BLE IMU packets -> timestamped IMU stream
```

`Dual_IMU` 只是外部参考项目名。当前 3DMotion 硬件只有两个 IMU 角色：

```text
head_imu
wrist_imu
```

## 当前输出

- acceleration
- gyroscope
- Euler angle
- optional quaternion
- optional magnetometer
- `timestamp_monotonic_ns`
- `timestamp_unix_ns`
- `timestamp_source = host_receive`

## 扫描排查

`--scan` 只返回看起来像 WT IMU 的设备。若扫不到，先用 `--scan-all` 检查系统是否能看到任何 BLE 广播：

```bash
python scripts/capture_imu_jsonl.py --scan-all --scan-timeout-s 12
```

## 后续骨架

- 如果 IMU 支持 device timestamp，加入 `timestamp_device_ns`。
- 估计 BLE latency / jitter。
- 输出 ROS2 `sensor_msgs/msg/Imu` live bridge。
