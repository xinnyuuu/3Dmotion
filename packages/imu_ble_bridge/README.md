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
- `timestamp_source`
- `host_receive_monotonic_ns`
- `host_receive_unix_ns`
- optional `timestamp_reconstruction`

串口适配器默认使用 `reconstructed-rate` 时间戳模式：同一批从 `/dev/ttyACM*` 读出的 WT901 包会按 nominal 200 Hz 展开到 `timestamp_*` 字段，原始主机接收时间保存在 `host_receive_*` 字段。BLE 直连默认仍使用 `host-receive`；如需强制旧逻辑，可加：

```bash
python scripts/capture_imu_jsonl.py --transport serial-adapter --serial-port /dev/ttyACM1 --address C4:65:91:2C:E2:20 --sensor-id head_imu --output data/raw/head_imu.jsonl --timestamp-mode host-receive
```

## 扫描排查

`--scan` 只返回看起来像 WT IMU 的设备。若扫不到，先用 `--scan-all` 检查系统是否能看到任何 BLE 广播：

```bash
python scripts/capture_imu_jsonl.py --scan-all --scan-timeout-s 20
```

如果使用 WIT/BWT901 USB 串口蓝牙适配器，每个适配器先绑定一个 IMU：

```bash
python scripts/capture_imu_jsonl.py --adapter-scan --serial-port /dev/ttyACM0
python scripts/capture_imu_jsonl.py --transport serial-adapter --serial-port /dev/ttyACM0 --address C4:65:91:2C:E2:20 --sensor-id head_imu --output data/raw/head_imu.jsonl

python scripts/capture_imu_jsonl.py --adapter-scan --serial-port /dev/ttyACM1
python scripts/capture_imu_jsonl.py --transport serial-adapter --serial-port /dev/ttyACM1 --address C5:64:B9:44:66:D6 --sensor-id wrist_imu --output data/raw/wrist_imu.jsonl
```

如果打开 `/dev/ttyACM*` 报 `Permission denied`，先把当前用户加入串口设备组，然后重新登录：

```bash
sudo usermod -a -G dialout "$USER"
```

临时测试也可以用 `sudo chmod a+rw /dev/ttyACM0 /dev/ttyACM1`，重插设备后可能需要重新执行。

Dashboard 当前实测默认使用 `/dev/ttyACM1` 连接 `head_imu`，`/dev/ttyACM0` 连接 `wrist_imu`：

```bash
python scripts/capture_dashboard.py
```

## 后续骨架

- 如果 IMU 支持 device timestamp，加入 `timestamp_device_ns`。
- 估计 BLE latency / jitter。
- 输出 ROS2 `sensor_msgs/msg/Imu` live bridge。
