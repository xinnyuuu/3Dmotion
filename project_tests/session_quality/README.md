# Session Quality

`check_session_quality.py` 用于检查真实采集 session 的最低可用性。完整运行命令见 `../../docs/system_workflow.md`。

## 输入结构

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

## 检查项

- `imu_timestamp_monotonic`: IMU timestamp 是否倒退。
- `camera_group_stability`: `group_id` 是否存在、连续、包含预期相机。
- `camera_skew`: 每组四目 timestamp span 和每条 `skew_us` 是否超过阈值。
- `repeatable_short_log`: 采集时长、每相机帧数和图片文件是否足够。

默认阈值偏原型调试，不是最终同步标准。严肃四目几何融合时，应把 `max_group_span_us` 收紧到更小，例如 5000 us 或更低。
