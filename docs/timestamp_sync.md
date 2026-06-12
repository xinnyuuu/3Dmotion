# 时间戳同步策略

时间同步是这个项目里最容易“悄悄变坏”的部分。原型期不用一步到位做硬件同步，但必须从第一天开始把时间戳记录清楚。

## 三类时间戳

建议每条数据尽量区分三种时间：

1. `timestamp_monotonic_ns`: host monotonic time。不会因为系统时间调整而跳变，适合原型期融合。
2. `timestamp_unix_ns`: host wall-clock time。适合查日志、跨文件关联、人工 debug。
3. `timestamp_device_ns`: sensor/device 自己的时间。如果硬件支持，这是后续最理想的主时间。

当前原型先用：

```text
timestamp_monotonic_ns
```

作为融合和离线处理的主要时间轴。

## 当前实用方案

15-30 FPS 原型期建议：

- Camera: 每次 `retrieve()` 后立即打时间戳。
- 四目：先对所有 camera 调 `grab()`，再逐个 `retrieve()`。
- BLE IMU: notification callback 收到 bytes 时立即打时间戳。
- 同时保存 `time.monotonic_ns()` 和 `time.time_ns()`。
- 记录 `timestamp_source`，例如 `host_retrieve` 或 `host_receive`。

这不是最终高精度方案，但足够先验证：

- 数据链路是否通。
- 四目采集是否稳定。
- AprilTag pose 是否稳定。
- IMU 与视觉大致是否能对齐。

## 四目 Skew

当前 `quad_camera_capture` 会把四个 camera 的帧归到同一个 `group_id`。

每张图都记录：

```text
skew_us = camera_timestamp - group_center_timestamp
```

用它判断四目是否足够接近同步。

经验判断：

- 慢速手部运动：几毫秒 skew 通常还能先测。
- 快速手部运动：skew 太大时，多目 PnP 会出现几何不一致。
- 如果经常超过 5-10 ms，需要认真考虑硬件同步或降低动作速度做验证。

## IMU-to-Camera Offset

BLE IMU 当前只能先用 host receive timestamp，所以会有：

- BLE latency
- OS scheduling jitter
- Python callback jitter

建议先估计一个 constant offset：

1. 把 wrist IMU 固定到 AprilTag board 或手环上。
2. 做几次明显快速转动。
3. 从视觉 pose 计算 angular velocity。
4. 和 gyro magnitude 的峰值做对齐。
5. 先拟合一个固定时间偏移。

如果固定 offset 后还不够，再考虑更复杂的 clock drift 模型。

## 最终系统建议

如果要做到更稳定的 motion capture，优先级是：

1. Global shutter cameras。
2. Hardware trigger 同步四个 camera。
3. 同一个 trigger / sync pulse 进入 IMU logger。
4. 使用 device timestamp。
5. host receive timestamp 只作为 diagnostics。

如果硬件同步暂时做不到，就用软件同步但显式记录不确定性：

- 估计 camera-IMU constant offset。
- IMU 插值到 camera timestamp。
- 根据 jitter 放大 measurement covariance。
- 丢弃 `skew_us` 过大的 frame group。

## Estimator 里的规则

估计算法里不要直接调用 `time.time()` 或 `time.monotonic()`。

Estimator 应该只消费输入数据里已有的 timestamp。这样同一套算法可以同时用于：

- live capture
- offline replay
- rosbag2 replay

这点也和 MOLA 的思想一致：SLAM / fusion 逻辑不要依赖系统当前时钟。

