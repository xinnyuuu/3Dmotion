# imu_ble_bridge

Purpose:

```text
WT-series BLE IMU packets -> timestamped IMU stream
```

This package should reuse the existing `Dual_IMU/device_model.py` parser during
the first prototype.

Current skeleton implementation:

- no GUI
- scans WT BLE devices
- captures acceleration, gyroscope, Euler angles, optional quaternion, optional magnetometer
- writes JSONL with both Unix and monotonic host receive timestamps

Scan:

```bash
python scripts/capture_imu_jsonl.py --scan
```

Capture:

```bash
python scripts/capture_imu_jsonl.py \
  --address XX:XX:XX:XX:XX:XX \
  --sensor-id wrist_imu \
  --output data/raw/wrist_imu.jsonl \
  --duration-s 30
```

The current timestamp source is `host_receive`. If the IMU can expose device
timestamps later, add them and keep host receive time as a latency/debug field.

