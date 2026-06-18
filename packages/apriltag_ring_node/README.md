# apriltag_ring_node

手环 AprilTag 离线视觉位姿模块。完整操作流程见 `../../docs/system_workflow.md`。

## 职责

```text
四目图片 + 手环 AprilTag -> T_H_B 视觉观测
```

参考外部 `AprilTag/` 项目的 tag detection 和 pose estimation 经验。

## 输入

```text
data/raw/session_*/cameras/frames.jsonl
data/raw/session_*/cameras/C*/00000000.jpg
configs/cameras.yaml
configs/bracelet.yaml
```

## 输出

```text
wrist_visual_candidates.jsonl
wrist_visual_pose.jsonl
```

当前融合方式：每个 tag/camera 先独立估计 `T_H_B`，再按 reprojection error 加权融合。同一个 `group_id` 输出一条 wrist visual pose。

## 后续骨架

- 用真正的 multi-camera joint PnP / optimizer 替换加权平均。
- 输出 covariance / quality score，供 wrist ESKF 使用。
- 处理 tag 切换、遮挡和 pose flip。
