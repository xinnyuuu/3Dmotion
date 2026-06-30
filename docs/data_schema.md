# 数据格式

最终输出格式参考 `Context.md`。建议同时保留两层数据：

1. 原始记录层：IMU、图片、tag 检测、时间戳。
2. 处理结果层：head pose、wrist pose、WAM motion stream。

原型阶段优先保证原始记录层稳定，因为它可以反复离线处理。

## 最终 JSONL Debug Format

每一行是一帧融合后的状态：

```json
{
  "timestamp_us": 1718128030045123,
  "tracking_state": 1,
  "head_6dof": {
    "pos_w": [0.0, 0.0, 0.0],
    "rot_w": [1.0, 0.0, 0.0, 0.0]
  },
  "wrist_6dof": {
    "pos_w": [0.0, 0.0, 0.0],
    "rot_w": [1.0, 0.0, 0.0, 0.0],
    "linear_vel_w": [0.0, 0.0, 0.0],
    "angular_vel_b": [0.0, 0.0, 0.0],
    "linear_acc_b": [0.0, 0.0, 0.0]
  }
}
```

约定：

- `pos_w`: 在 world frame `W` 下的位置，单位 meter。
- `rot_w`: 在 world frame `W` 下的姿态 quaternion，顺序为 `[qw, qx, qy, qz]`。
- `linear_vel_w`: world frame 下线速度。
- `angular_vel_b`: wristband/body frame 下角速度。
- `linear_acc_b`: wristband/body frame 下加速度，后续应使用 bias-corrected acceleration。

## Tracking State

```text
0 = LOST
1 = VISUAL_OK
2 = PURE_IMU
```

建议含义：

- `LOST`: 没有可信视觉，也不信任纯 IMU 外推。
- `VISUAL_OK`: 当前有 AprilTag / VIO 等视觉约束。
- `PURE_IMU`: 短时间视觉丢失，正在靠 IMU propagation 补帧。

## 原始 IMU 记录

当前 `imu_ble_bridge` 建议写 JSONL，每行一个 IMU sample：

```json
{
  "sensor_id": "wrist_imu",
  "timestamp_unix_ns": 1718128030045123000,
  "timestamp_monotonic_ns": 123456789000,
  "timestamp_source": "host_receive",
  "accel_mps2": [0.0, 0.0, 9.8],
  "gyro_radps": [0.0, 0.0, 0.0],
  "euler_deg": [0.0, 0.0, 0.0],
  "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
  "mag": null
}
```

## 原始四目图片记录

当前 `quad_camera_capture` 输出：

```text
data/raw/session/
  session_manifest.json
  cameras/
    frames.jsonl
    capture_errors.jsonl
    C0/00000000.jpg
    C1/00000000.jpg
    C2/00000000.jpg
    C3/00000000.jpg
  imus/
    head_imu.jsonl
    wrist_imu.jsonl
```

`frames.jsonl` 每行一张图：

```json
{
  "group_id": 0,
  "camera_id": "C0",
  "timestamp_unix_ns": 1718128030045123000,
  "timestamp_monotonic_ns": 123456789000,
  "timestamp_source": "host_retrieve",
  "skew_us": 250.0,
  "image_path": "C0/00000000.jpg",
  "width": 1920,
  "height": 1080
}
```

`group_id` 表示一次四目近似同步采集组。`skew_us` 越小，四目越接近同步。

录制后先用诊断脚本确认实际结果：

```bash
python scripts/validate_session.py --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

Dashboard 在按 Stop 后也会自动生成：

```text
data/raw/session_YYYYMMDD_HHMMSS/session_summary.json
```

重点看：

- `ok_for_camera_replay`: 是否至少有可用图片和 `frames.jsonl`。
- `camera_frame_count`: 实际写入多少张图片记录。
- `has_capture_warnings`: 是否有部分相机打不开或失败。
- `imu_counts`: 哪些 IMU 文件里有样本。

也可以用一条命令串起验证和后处理：

```bash
python scripts/postprocess_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --apriltag \
  --openvins \
  --openvins-config
```

它会生成：

```text
data/processed/session_YYYYMMDD_HHMMSS/postprocess_summary.json
```

## 中间处理流

后续建议产生这些中间文件：

```text
openvins_session_manifest.json
images.jsonl
imu.jsonl
wrist_visual_pose.jsonl
head_pose.jsonl
wrist_fused_pose.jsonl
wam_motion.jsonl
```

字段可以先保持简单：

```text
timestamp_us, frame_id, child_frame_id, x, y, z, qw, qx, qy, qz, covariance_hint
timestamp_us, camera_id, tag_id, corners, reprojection_error
```

`prepare_openvins_session.py` 会先生成一个 ROS-free 的 OpenVINS 中间层：

```text
data/processed/openvins_session/
  openvins_session_manifest.json
  images.jsonl
  imu.jsonl
```

`images.jsonl` 把录制图片映射到 OpenVINS 习惯的 topic：

```json
{
  "topic": "/cam0/image_raw",
  "camera_id": "C0",
  "timestamp_unix_ns": 1718128030045123000,
  "timestamp_monotonic_ns": 123456789000,
  "image_path": "/abs/path/to/data/raw/session_xxx/cameras/C0/00000000.jpg",
  "width": 1920,
  "height": 1080
}
```

`imu.jsonl` 把 `head_imu` 映射成 `/imu0`：

```json
{
  "topic": "/imu0",
  "sensor_id": "head_imu",
  "timestamp_unix_ns": 1718128030045123000,
  "timestamp_monotonic_ns": 123456789000,
  "accel_mps2": [0.0, 0.0, 9.8],
  "gyro_radps": [0.0, 0.0, 0.0]
}
```

下一步再把这两个 JSONL 转成 rosbag2。这样 dashboard 不需要依赖 ROS2 环境，OpenVINS 也可以独立调试。

`write_openvins_rosbag2.py` 会在 ROS2 环境里把这两个文件写成 rosbag2：

```text
data/processed/openvins_session/rosbag2/
data/processed/openvins_session/rosbag2_summary.json
```

对应 ROS topic：

```text
/cam0/image_raw  sensor_msgs/msg/Image
/imu0            sensor_msgs/msg/Imu
```

当前默认导出四目 head cameras + `head_imu`。调试时仍可用 `--camera-id C0` 退回单目。

## AprilGrid world-anchor 输出

当前可复现主线写入：

```text
data/processed/session_YYYYMMDD_HHMMSS/world_anchor/
  world_anchor_candidates.jsonl
  wrist_world_candidates.jsonl
  hand_skeletons.jsonl
  head_pose.jsonl
  motion_frame.jsonl
  motion_frame_filtered.jsonl
  world_anchor_summary.json
```

`motion_frame.jsonl` 每行是一帧 head/wrist 状态：

```json
{
  "timestamp_us": 1782800258715900,
  "timestamp_monotonic_ns": 15123456789000,
  "head": {
    "position": [0.0, 0.0, 0.0],
    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    "tracking_state": "visual"
  },
  "wrist": {
    "position": [0.1, 0.0, 0.2],
    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
    "tracking_state": "visual"
  },
  "relative": {
    "T_H_B": {
      "position": [0.1, 0.0, 0.2],
      "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]
    }
  },
  "fusion": {
    "method": "aprilgrid_world_anchor"
  }
}
```

`head` 或 `wrist` 可以为 `null`，表示该帧没有可信估计。RViz replay 会跳过缺失对象，而不是复用旧 pose。

`hand_skeletons.jsonl` 每行保存一个手部 skeleton candidate；`motion_frame.jsonl` 只保留每帧选中的 skeleton：

```json
{
  "timestamp_us": 1782800258715900,
  "group_id": 42,
  "hand_index": 0,
  "camera_id": "C1+C2",
  "source": {
    "method": "multiview_triangulated",
    "cameras_used": ["C1", "C2"]
  },
  "landmarks": [
    {
      "index": 0,
      "world": [0.1, 0.0, 0.2],
      "observation_count": 2
    }
  ],
  "connections": [[0, 1], [1, 2]]
}
```

`source.method` 常见值：

```text
multiview_triangulated          two or more cameras triangulated the hand
singleview_guided_by_multiview  one camera plus recent multiview state
mediapipe_2d_backprojected_to_wrist_depth
                                raw wrist-depth fallback, disabled by default
```

`motion_frame_filtered.jsonl` 是对 `motion_frame.jsonl` 的数据层滤波结果，当前由 `scripts/filter_motion_frame.py` 或 `scripts/process_dashboard_session.py` 生成。它不是 RViz display smoothing；后续算法如果需要平滑 pose，也应该显式选择这个文件。

## 时间戳规则

优先级：

1. sensor/device timestamp
2. host monotonic timestamp
3. host unix timestamp

估计算法里不要直接调用 wall-clock time。算法应该只消费数据文件或 ROS message 里的 timestamp，这样 live capture 和 offline replay 才会表现一致。
