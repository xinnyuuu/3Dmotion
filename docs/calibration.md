# 标定清单

这个项目的精度主要卡在标定。算法可以后面慢慢换，但坐标系、尺寸、时间戳如果一开始混乱，后面会很难 debug。

快速检查当前配置是否齐全：

```bash
python scripts/check_calibration_readiness.py
```

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

当前四路头环镜头按超广角/全向相机处理。最终主线优先使用 Kalibr + AprilGrid，并比较：

```text
ds-none
eucm-none
omni-radtan
```

OpenCV fisheye 只作为当前 3D Motion 原型链路的 fallback。VimasCalibration 的 OpenCV fallback
结果手动填入本项目时，应写成 OpenVINS/Kalibr 兼容的：

```text
camera_model: fisheye
distortion_model: equidistant
distortion: [k1, k2, k3, k4]
```

当前离线主线 session 对应的标定档位是：

```text
MJPG 1600x1200 @ 25fps
```

注意：

- 四个摄像头即使型号一样，也不要假设 intrinsics 完全相同。
- 内参必须和真实录制 resolution 一致；从 1600x1200 切到 1920x1080、800x600、1:1 方形传感器后都要重新标定。
- VimasCalibration 和 3DMotion 是独立仓库；VimasCalibration 产出标定结果，3DMotion 只消费本仓库 `configs/` 里的 YAML。
- 从 VimasCalibration 复制标定值到 3DMotion 时，需要人工审阅 frame convention、单位和 transform 方向，不做跨仓库一键写入。
- 当前 AprilTag/OpenVINS 代码不会把 DS/EUCM/omni 参数静默当 pinhole 使用；OpenVINS config generator 只导出当前支持的 compatibility view。
- 在 `T_H_C` 外参没有标定前，AprilTag 可以先做单相机诊断，但多相机融合位姿不能当最终结果。

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

当前 Head frame `H` 使用手量原点：

```text
H origin: 头环左右宽度 17.6 cm 的中点，也就是距左右外侧边缘各 8.8 cm
H origin: 距头环最前端外侧边缘 10.7 cm
H origin: 头环上下厚度 3.4 cm 的中心，也就是距上下外侧边缘各 1.7 cm
+X: 朝头环前方
+Y: 使用者左侧
+Z: 向上
```

按这个符号，头环最前端外侧边缘相对 `H` 是 `x = +0.107 m`；上边缘是
`z = +0.017 m`；左右边缘是 `y = +/-0.088 m`。

相机外参最终落在：

```text
configs/cameras.yaml
```

每个 camera entry 的 `T_H_C` 必须是 4x4 matrix 或 `[x, y, z, yaw_deg, pitch_deg, roll_deg]`。

VimasCalibration 输出 `camera_extrinsics/four_head_camera_extrinsics.yaml` 后，可以在
`~/lxy/VimasCalibration` 里生成一个只用于人工复制的片段：

```bash
cd ~/lxy/VimasCalibration
python scripts/export_3dmotion_camera_extrinsics.py \
  --extrinsics camera_extrinsics/four_head_camera_extrinsics.yaml \
  --output camera_extrinsics/3dmotion_T_H_C_snippet.yaml
```

复制前确认映射：

```text
left_side   -> C0
left_front  -> C1
right_front -> C2
right_side  -> C3
```

## Head IMU-to-Head Extrinsics

当前硬件里只有一个头环 IMU，项目里统一叫：

```text
head_imu
```

建议把它的坐标系记为 `I_H`，也就是 head IMU frame。最终要标定它到头环刚体中心 `H` 的外参：

```text
T_H_IH
```

当前测量值：

```text
t_H_IH = [0.112, 0.0, 0.0] m

IMU +X = H -Z
IMU +Y = H +Y
IMU +Z = H +X
```

对应：

```text
R_H_IH =
[[ 0, 0, 1],
 [ 0, 1, 0],
 [-1, 0, 0]]
```

也就是说，head IMU 原点在 `H +X` 方向 11.2 cm，`Y/Z` 平移保持 0。后续可以用
Kalibr camera-IMU 标定再 refine 这个手量外参。

第一版 OpenVINS 只接：

```text
one camera + head_imu -> T_W_IH
```

手环 IMU 不接入 P3a；它属于后续 wrist ESKF / wrist motion pipeline。`Dual_IMU` 项目只作为 BLE 数据抓取和解析协议参考，不代表本项目有两个头环 IMU。

## Wrist IMU-to-Wristband Extrinsics

手环上也只有一个 IMU，项目里统一叫：

```text
wrist_imu
```

建议把它的坐标系记为 `I_B`，也就是 wrist IMU frame。它需要标定到手环刚体中心 `B`：

```text
T_B_IB
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
