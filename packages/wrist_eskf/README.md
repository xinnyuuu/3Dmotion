# wrist_eskf

手环 IMU 与视觉位姿融合模块骨架。

## 目标

```text
wrist IMU propagation + AprilTag pose correction -> smoothed wrist state
```

## 状态

```text
p_W, v_W, q_W, b_a, b_g
```

## 输入

```text
wrist_imu.jsonl
wrist_visual_pose.jsonl
head_pose.jsonl 或 T_W_H
```

## 输出

```text
wrist_fused_pose.jsonl
T_W_B
tracking_state
```

## 当前状态

完整 ESKF 还未实现。当前已在 `packages/session_tools/motion_fusion.py` 里实现一个实用的离线互补融合层：

```text
wrist gyro propagation between AprilTag visual poses
+ AprilTag visual orientation correction
+ AprilTag visual translation
-> wrist_fused_pose.jsonl / motion_frame.jsonl
```

也就是说，`wrist_imu` 的 gyro 已经会影响 wrist orientation 的连续性；AprilTag 仍是绝对校正和位置来源。后续如果要处理长时间遮挡、bias、速度和协方差，再升级到这里规划的 ESKF 状态。
