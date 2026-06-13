# apriltag_ring_node

作用：

```text
四目图片 + 手环 AprilTag -> T_H_B 视觉观测
```

参考项目`AprilTag/`：https://github.com/Leowang980/AprilTag


## 当前处理流程

```text
quad_camera_capture session
  -> 读取 frames.jsonl
  -> 逐 camera 读取图片
  -> OpenCV AprilTag detection
  -> solvePnP 得到 T_C_Ti
  -> 用 T_H_Ci 转到头环坐标系
  -> 用 T_Ti_B 转到手环中心
  -> 同一个 group_id 内多观测加权融合
  -> wrist_visual_pose.jsonl
```

当前融合是第一版工程实现：每个 tag/camera 先独立估计 `T_H_B`，再按
reprojection error 加权融合。后续可以替换成真正的 multi-camera joint PnP。

## 前置配置

必须先填写：

```text
configs/cameras.yaml
configs/bracelet.yaml
```

`configs/cameras.yaml` 需要每个 camera 的：

- `intrinsics`
- `distortion`
- `T_H_C`

`configs/bracelet.yaml` 需要：

- `tag_family`
- `tag_size_m`
- `center_offset_m` 或 `flat_to_flat_m`
- `tag_order`
- 可选 `tag_to_wrist_transforms`

如果 `tag_to_wrist_transforms` 为空，代码会使用正六边形 fallback 几何。

## 离线处理命令

先采集四目：

```bash
python scripts/capture_quad_camera.py \
  --source C0:0 \
  --source C1:1 \
  --source C2:2 \
  --source C3:3 \
  --fps 30 \
  --duration-s 30 \
  --output-dir data/raw/quad_camera_test
```

再处理 AprilTag：

```bash
python scripts/process_apriltag_session.py \
  --session-dir data/raw/quad_camera_test \
  --cameras configs/cameras.yaml \
  --bracelet configs/bracelet.yaml \
  --output-dir data/processed/wrist_visual
```

输出：

```text
data/processed/wrist_visual/
  wrist_visual_candidates.jsonl
  wrist_visual_pose.jsonl
```

`wrist_visual_candidates.jsonl` 是每个 camera/tag 的候选观测。
`wrist_visual_pose.jsonl` 是每个 `group_id` 融合后的 `T_H_B`。
