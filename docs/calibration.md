# 标定清单

这个项目的精度主要卡在标定。算法可以后面慢慢换，但坐标系、尺寸、时间戳如果一开始混乱，后面会很难 debug。

## Camera Intrinsics

每个摄像头都需要单独标定：

- resolution
- camera matrix
- distortion coefficients
- camera model

保存位置：

```text
configs/cameras.yaml
```

注意：四个摄像头即使型号一样，也不要假设 intrinsics 完全相同。

## Camera-to-Head Extrinsics

每个摄像头都需要知道它相对头环中心的位置和朝向。

需要其中一种约定：

```text
T_H_Ci
```

或：

```text
T_Ci_H
```

项目里建议统一使用：

```text
T_A_B 表示把 B 坐标系里的点变换到 A 坐标系
```

所以推荐保存：

```text
T_H_C0
T_H_C1
T_H_C2
T_H_C3
```

也就是四个 camera frame 到 headset frame 的外参。

约定写在：

```text
configs/frames.yaml
```

## Head IMU-to-Head Extrinsics

头环 IMU 也要标定到 `H`：

```text
T_H_IMU_FRONT
T_H_IMU_BACK
```

第一版 OpenVINS 原型建议先用一个 IMU 跑通：

```text
one camera + one IMU -> T_W_H
```

等单 IMU VIO 稳定后，再加 dual-IMU 逻辑。不要一开始就把 dual-IMU、四目、手环融合全部混在一起，那会很难定位问题。

## Wrist IMU-to-Wristband Extrinsics

手环 IMU 需要标定到手环刚体中心 `B`：

```text
T_B_IMU_WRIST
```

这个外参决定了：

- IMU 测到的角速度属于哪个刚体方向
- acceleration 怎么转到 wristband frame
- ESKF 里 propagation 和视觉 correction 是否一致

如果这个外参不准，静态时可能看起来还好，一快速转动就会出问题。

## Bracelet Geometry

手环几何参数需要固定下来：

- AprilTag family
- tag IDs
- tag side length
- flat-to-flat distance
- tag order around bracelet
- 每个 tag frame 到 wristband frame 的刚体变换

保存位置：

```text
configs/bracelet.yaml
```

核心变换：

```text
T_Ti_B
```

含义：把 wristband frame `B` 里的点变换到第 `i` 个 tag frame `T_i`。

如果从摄像头看到了 tag，就可以组合：

```text
T_H_B = T_H_Ci * T_Ci_Tj * T_Tj_B
```

## Time Offset Calibration

时间标定也很关键。需要关心：

- camera 和 head IMU 的时间偏移
- wrist IMU 和 camera/head clock 的时间偏移
- BLE host receive latency
- 四个摄像头之间的 frame skew

原型期可以先记录：

```text
timestamp_monotonic_ns
timestamp_unix_ns
timestamp_source
```

后面再用动作峰值对齐估计 offset，例如比较：

```text
visual angular velocity peak
gyro magnitude peak
```

如果条件允许，最终版本最好上 hardware trigger 或 device timestamp。

