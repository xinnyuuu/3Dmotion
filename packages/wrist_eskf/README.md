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

还未实现。保留为 P2/P4 的工程骨架。
