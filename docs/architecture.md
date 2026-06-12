# 系统架构

这个项目先按“记录数据”和“处理映射”两层来设计。这样做的好处是：采集层可以先稳定下来，后面的 AprilTag 检测、VIO、ESKF、MOLA 回放都可以离线反复调，不会被实时系统拖着跑。

## 总体数据流

```text
头环摄像头 + 头环 IMU
  -> head_vio_bridge
  -> /motion/head_pose

头环摄像头看到手环 AprilTag
  -> apriltag_ring_node
  -> /motion/wrist_visual_pose

手环 IMU
  -> imu_ble_bridge
  -> /motion/wrist_imu

/motion/wrist_imu + /motion/wrist_visual_pose + /motion/head_pose
  -> wrist_eskf
  -> /motion/wrist_pose

/motion/head_pose + /motion/wrist_pose
  -> wam_token_writer
  -> JSONL / binary WAM motion stream
```

第一阶段不强求全实时。推荐先录：

```text
IMU JSONL
四目图片
四目 frames.jsonl
时间戳
相机 ID
```

然后离线处理：

```text
图片 -> AprilTag -> T_H_B
IMU -> propagation
OpenVINS -> T_W_H
T_W_H * T_H_B -> T_W_B
```

## 模块职责

### `head_vio_bridge`

作用：把 OpenVINS 的头部 VIO 输出转成项目内部统一格式。

输入：

- 头环摄像头图像
- 头环 IMU
- camera/IMU calibration

输出：

- `T_W_H`

含义：头环坐标系 `H` 在世界坐标系 `W` 里的 6DoF 位姿。

### `apriltag_ring_node`

作用：检测手环上的 AprilTag，并估计手环刚体中心的位姿。

输入：

- 四个头环摄像头图像
- camera intrinsics
- camera-to-head extrinsics，即 `T_H_Ci`
- 手环几何参数

输出：

- `T_H_B`

含义：手环坐标系 `B` 在头环坐标系 `H` 里的视觉观测。

近期实现可以先复用已有 `AprilTag` 项目的逻辑：

- tag detection
- tag pose estimation
- 正六边形手环中心估计
- 从相邻可见 tag 推断目标 tag pose

之后再把单相机 pose selection 升级为真正的 multi-camera joint PnP / optimizer。

### `imu_ble_bridge`

作用：把 WT 系列 BLE IMU 数据包转成带时间戳的项目数据或 ROS 2 message。

输入：

- BLE notification packets

输出：

- acceleration
- angular velocity
- Euler angle
- optional quaternion
- optional magnetometer
- timestamp

当前实现不需要 GUI，只复用 `Dual_IMU/device_model.py` 里的协议解析思路。

### `quad_camera_capture`

作用：四目摄像头采集与落盘。

当前逻辑：

```text
grab C0/C1/C2/C3
retrieve C0/C1/C2/C3
给每张图打 host timestamp
保存图片
写 frames.jsonl
```

这是软件近似同步，不是最终高精度同步。15-30 FPS 原型期够用，但需要记录 `skew_us` 来判断四个相机之间的时间差。

### `wrist_eskf`

作用：用手环 IMU 做高频 propagation，用 AprilTag 视觉位姿做 correction。

状态：

```text
p_W, v_W, q_W, b_a, b_g
```

输出：

- 平滑后的 wrist 6DoF pose
- linear velocity
- bias-corrected acceleration
- tracking state

直觉上：

```text
AprilTag 给绝对位置，但帧率低、会遮挡
IMU 给高频运动趋势，但会漂
ESKF 把两者融合
```

### `wam_token_writer`

作用：把 head 和 wrist 的最终位姿序列写成下游 WAM 需要的数据格式。

输出：

- JSONL debug stream
- 后续可扩展 binary stream

## OpenVINS 的角色

OpenVINS 是第一阶段推荐的 head VIO 核心，因为它直接解决：

```text
head cameras + head IMU -> T_W_H
```

优点：

- 成熟的 Visual-Inertial Odometry
- 支持 multi-camera
- ROS 集成相对直接
- 有 camera/IMU calibration 工作流

## MOLA 的角色

MOLA 不建议作为第一版 VIO 核心。它更适合第二阶段工程化：

- rosbag2 replay
- sensor pipeline 配置
- trajectory tools
- 地图或 AprilTag anchor-grid 约束
- visualization / evaluation

简单说：

```text
OpenVINS: 先把头部 T_W_H 跑起来
MOLA: 后面做回放、评估、地图/anchor 约束和系统集成
```

