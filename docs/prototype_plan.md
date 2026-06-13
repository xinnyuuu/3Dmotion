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

1. 先把 `head_imu` 刚性固定在四目头环/相机结构上。
2. 先用 one camera + one head IMU，也就是 `C0 + head_imu`。
3. 确认 scale、重力方向、初始 pose 稳定。
4. 再加入多摄像头。
5. 输出 `T_W_H`，并统一 timestamp convention。

P3a 的临时 frame 约定：

```text
I = head_imu frame
H := I
T_W_H := T_W_I
```

也就是说，第一版先把 OpenVINS 的 body frame 当成头环 frame。这样可以先验证 VIO 本身是否能跑通。等单目 VIO 稳定后，再标定真实的 IMU-to-head 外参：

```text
T_W_H = T_W_I * T_I_H
```

注意：`head_imu` 必须固定在头环刚体上，不能拿在手里，不能贴在会晃的绑带上，也不能相对相机有滑动。OpenVINS 默认 camera 和 IMU 属于同一个刚体，任何相对运动都会破坏初始化、scale 和重力方向。

通过标准：

- 开机世界原点稳定。
- 没有明显 scale failure。
- 短距离动作捕捉场景下漂移可接受。

接入方式：

1. 先不要直接把 OpenVINS 塞进 dashboard。dashboard 只负责稳定记录 session。
2. 每次 session 里需要至少有：
   - `cameras/frames.jsonl`
   - `cameras/C*/00000000.jpg`
   - `imus/head_imu.jsonl`
   - `session_manifest.json`
3. 写一个离线转换脚本，把 session 转成 OpenVINS 能吃的 ROS2 topic 或 rosbag2：
   - `/cam0/image_raw`
   - `/cam1/image_raw`，多目时继续增加
   - `/imu0`
4. 第一版建议只用 `C0 + head_imu`，先验证时间戳、重力方向和 scale。
5. 单目+IMU 稳定后，再接四目/多相机外参。

当前 P3a 离线命令：

```bash
python scripts/check_head_vio_readiness.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

通过后准备 OpenVINS 输入和单目配置：

```bash
python scripts/prepare_p3_head_vio.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

这会生成：

```text
data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/
  images.jsonl
  imu.jsonl
  openvins_session_manifest.json
  p3_head_vio_summary.json
  config/
    estimator_config.yaml
    kalibr_imu_chain.yaml
    kalibr_imucam_chain.yaml
```

在 ROS2 环境下再转成 rosbag2：

```bash
source /opt/ros/humble/setup.bash
python scripts/write_openvins_rosbag2.py \
  --prepared-dir data/processed/session_YYYYMMDD_HHMMSS/openvins_c0 \
  --bag-dir data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/rosbag2 \
  --frame-id head_imu
```

这里的关键是：OpenVINS 解决的是 `T_W_H`，也就是头环/头部在 world frame 下的位姿。它不直接解决手腕 AprilTag ground truth。

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

## 当前建议：先把 AprilTag 手环监测接进来

现在应该优先引入 AprilTag 手环监测，但方式建议是“离线处理”，不是先做实时闭环。

原因：

- AprilTag 手环监测可以直接验证 `T_H_B`，也就是手环在头环坐标系里的位姿。
- 它不依赖 OpenVINS 是否已经跑通。
- 它能作为 wrist IMU 的 visual correction / ground truth。
- 它可以用当前 camera session 直接反复处理，方便调 tag size、bracelet geometry、reprojection error 阈值。

推荐下一步：

1. 先确保 record session 能稳定产生：
   ```text
   data/raw/session_xxx/
     cameras/frames.jsonl
     cameras/C0/*.jpg
     imus/head_imu.jsonl
     imus/wrist_imu.jsonl
     session_manifest.json
   ```
2. 跑 AprilTag 离线处理：
   ```bash
   python scripts/process_apriltag_session.py \
     --session-dir data/raw/session_xxx/cameras \
     --cameras configs/cameras.yaml \
     --bracelet configs/bracelet.yaml
   ```
3. 检查输出的 wrist visual pose 是否稳定。
4. 再把 wrist IMU 和 wrist visual pose 对齐，进入 P2 ESKF。
5. OpenVINS 放在后面接 head pose：`T_W_H` 稳定后，再组合：
   ```text
   T_W_B = T_W_H * T_H_B
   ```

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
