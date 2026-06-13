# Session Quality

`check_session_quality.py` 用于检查真实采集 session 的最低可用性。

输入目录应类似：

```text
data/raw/session_YYYYMMDD_HHMMSS/
  session_manifest.json
  session_summary.json
  cameras/
    frames.jsonl
    C0/*.jpg
    C1/*.jpg
    C2/*.jpg
    C3/*.jpg
  imus/
    head_imu.jsonl
    wrist_imu.jsonl
```

## 检查内容

1. `imu_timestamp_monotonic`
   - 检查每个 `imus/*.jsonl` 的 `timestamp_monotonic_ns` 是否倒退。
   - 如果没有 `timestamp_monotonic_ns`，退回检查 `timestamp_unix_ns`。

2. `camera_group_stability`
   - 检查 `frames.jsonl` 里是否有 `group_id`。
   - 检查 `group_id` 是否连续。
   - 检查每组是否包含预期相机，默认是 `C0,C1,C2,C3`。

3. `camera_skew`
   - 检查每组四目 timestamp span。
   - 检查每条 frame record 里的 `skew_us` 绝对值。

4. `repeatable_short_log`
   - 检查采集时长是否足够。
   - 检查每个相机是否有足够帧数。
   - 检查 frame 指向的图片文件是否存在。

## 默认阈值

默认阈值偏向原型调试，不是最终同步标准：

```text
max_group_span_us = 30000
max_abs_skew_us = 20000
min_complete_groups = 5
min_duration_s = 2.0
min_frames_per_camera = 15
min_group_completion_ratio = 0.90
```

如果未来要做严肃四目几何融合，应把 `max_group_span_us` 收紧到更小，例如 5000 us 或更低。

