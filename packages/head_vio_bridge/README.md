# head_vio_bridge

OpenVINS head VIO 适配模块。完整 P3a 操作流程见 `../../docs/system_workflow.md`。

## 职责

```text
C0 + head_imu -> OpenVINS inputs / rosbag2 -> T_W_H
```

当前实现覆盖：

- P3a readiness check。
- dashboard session 到 OpenVINS JSONL 中间层的转换。
- 第一版 OpenVINS config 生成。
- ROS2 环境下的 rosbag2 导出。

## Frame 约定

P3a 只用一个 camera 和一个刚性固定的 head IMU：

```text
C0 + head_imu -> T_W_H
```

临时约定：

```text
I_H = head_imu frame
H := I_H
T_W_H := T_W_IH
```

后续测出 IMU-to-head 外参后：

```text
T_W_H = T_W_IH * T_IH_H
```

`head_imu` 必须固定在头环/相机刚体上。OpenVINS 默认 camera 和 IMU 属于同一个 rigid body。

## 后续骨架

- P3b: 用 Kalibr 或固定标定板测正式 `T_imu_cam` 和 time offset。
- P3c: 从 `C0` 扩展到 `C0+C1`，再扩展到四目。
- P4: 把 `T_W_H` 与 AprilTag wrist pose 组合成 `T_W_B`。
