# camera 项目与 3DMotion pipeline 对比

这个文档对比 `/home/lxy/lxy/camera` 旧项目和当前 `3DMotion` 的 camera pipeline。

## 旧 camera 项目

旧项目重点是：

```text
四路 V4L2 camera
-> Web UI 预览
-> 录制每路视频
-> CSV 记录 capture/pad
-> 离线 COLMAP / MediaPipe / viewer
```

优点：

- Web UI 适合手动选择 `/dev/video*`。
- V4L2 device / format / FPS 枚举做得很好。
- 录制视频比保存大量图片更省空间。
- CSV 记录 `capture` / `pad`，方便审计掉帧和补帧。
- 后处理 pipeline 分层清晰。

不足：

- 目标偏 scene reconstruction / hand landmark visualization，不是刚体 wrist 6DoF。
- 录视频会隐藏每帧真实 capture timestamp，后面做 IMU fusion 不够直接。
- COLMAP/MediaPipe 路线不适合直接输出 metric wrist rigid-body pose。
- 当时还没有 OpenVINS，因此 head pose `T_W_H` 不是主线。

## 当前 3DMotion pipeline

当前项目重点是：

```text
四目图片帧组 + IMU JSONL
-> AprilTag wrist visual pose
-> 后续 wrist ESKF
-> OpenVINS head pose
-> WAM motion stream
```

优点：

- 每张图片都有 `timestamp_monotonic_ns` 和 `timestamp_unix_ns`。
- 四目按 `group_id` 组织，直接适合 multi-camera AprilTag 处理。
- 输出目标明确：`T_H_B`、`T_W_H`、`T_W_B`。
- 更适合后续 IMU fusion 和 offline replay。

不足：

- 当前没有 Web UI，设备选择没有旧项目方便。
- 当前直接保存 JPEG 图片，长时间录制会比较占空间。
- 当前四目同步还是软件近似同步，尚未硬件 trigger。
- 当前 `T_H_C` 外参还需要补。

## 已经吸收的能力

从旧 camera 项目吸收到了：

- V4L2 camera device 枚举。
- format / resolution / FPS 探测。
- 采集和后处理分离的设计思想。

新增入口：

```bash
python scripts/list_cameras.py --configs
```

## 暂不直接沿用的能力

暂时不把旧项目的 Web UI 直接搬过来。原因：

- 当前最急的是稳定数据格式和 3D pose 处理。
- Web UI 会引入较多 HTTP / threading 状态，容易让原型变重。
- 等 CLI pipeline 跑稳后，再考虑把旧 Web UI 改造成采集控制台。

暂时不把 COLMAP / MediaPipe 作为主线。原因：

- 本项目目标是 head/wrist rigid-body 6DoF，不是 scene dense reconstruction。
- 手腕 AprilTag 已经给了更直接的 wrist rigid-body 观测。
- MediaPipe hand world landmark 不应当直接当 metric pose。

## 推荐下一步

1. 用 `scripts/list_cameras.py --configs` 确认四个 camera 的 device path 和 FPS。
2. 把 `configs/capture.yaml` 里的 camera source 固定为稳定 `/dev/video*` 或 `/dev/v4l/by-id/*`。
3. 填写四个 camera 的 `T_H_C`。
4. 用 `capture_quad_camera.py` 录一小段 AprilTag 手环数据。
5. 用 `process_apriltag_session.py` 生成 `wrist_visual_pose.jsonl`。
6. 检查 `source_camera_ids`、`source_tag_ids`、`mean_reprojection_error_px`、`T_H_B` 是否稳定。

