# 原型计划

这个项目不要一上来追求完整实时系统。更稳的路线是：

```text
先记录
再离线处理
最后实时化
```

下面每个阶段都应该能独立验证。

## P0: 只做手环视觉位姿

目标：

```text
固定头环/摄像头坐标系 -> 稳定的 T_H_B
```

步骤：

1. 先固定一个 camera 或固定整个头环。
2. 使用现有 AprilTag ring tracker。
3. 正常移动手环，覆盖不同角度和遮挡情况。
4. 记录 `T_H_B`、source tag IDs、reprojection error、tracking state。

通过标准：

- 静止时 wrist pose 抖动小于 1-2 cm。
- 可见 tag 切换时没有明显 pose flip。
- 短暂遮挡后能恢复。

这一阶段的意义是确认手环几何和 AprilTag 检测方案本身成立。

## P1: 记录手环 IMU

目标：

```text
wrist IMU + visual T_H_B 记录在同一条时间线上
```

步骤：

1. 用 `imu_ble_bridge` 记录 raw acceleration 和 gyroscope。
2. 记录 `timestamp_monotonic_ns` 和 `timestamp_unix_ns`。
3. 摄像头按 15-30 FPS 记录 frame group。
4. 检查 timestamp 是否单调、是否丢帧、是否有明显延迟抖动。

通过标准：

- IMU timestamp 单调。
- camera frame group 有稳定 `group_id`。
- 四目 `skew_us` 不离谱。
- 可以重复采集短动作日志。

## P2: 离线 Wrist ESKF

目标：

```text
IMU propagation + AprilTag visual correction -> 平滑手环位姿
```

步骤：

1. 先写最小离线 ESKF。
2. 用 AprilTag 视觉 pose 做 correction。
3. 人为模拟视觉丢失，例如遮挡 0.5s / 1.0s / 2.0s。
4. 测量漂移和重捕获后的跳变。

通过标准：

- `VISUAL_OK` 时稳定。
- 0.5-1.0s 视觉丢失不会明显炸飞。
- 视觉重捕获时不会产生不可接受的大跳。

## P3: OpenVINS 头部 VIO

目标：

```text
head cameras + head IMU -> T_W_H
```

步骤：

1. 先用 one camera + one head IMU。
2. 确认 scale、重力方向、初始 pose 稳定。
3. 再加入多摄像头。
4. 输出 `T_W_H`，并统一 timestamp convention。

通过标准：

- 开机世界原点稳定。
- 没有明显 scale failure。
- 短距离动作捕捉场景下漂移可接受。

## P4: 输出完整刚体轨迹

目标：

```text
T_W_B = T_W_H * T_H_B
```

步骤：

1. 读取 head pose 和 wrist visual/fused pose。
2. 按 timestamp 对齐。
3. 必要时对 pose 做 interpolation。
4. 输出 `Context.md` 里的 WAM schema。

通过标准：

- head 和 wrist 共享同一个 world frame。
- 输出 schema 稳定。
- 同一段数据可以离线重复处理。

## P5: MOLA 集成

目标：

```text
可回放、可检查、可扩展的 motion-capture pipeline
```

步骤：

1. 录制 rosbag2 数据集。
2. 添加 MOLA replay configs。
3. 使用 trajectory tools 做评估。
4. 如果 world drift 需要约束，再加入环境 AprilTag anchor grids。

通过标准：

- 数据集可以稳定 replay。
- 轨迹可以导出、对齐、评估。
- 后续能接 map/anchor constraints，而不是重写采集层。

