# quad_camera_capture

四目 camera 采集模块。完整采集流程见 `../../docs/system_workflow.md`。

## 职责

```text
four headset cameras -> timestamped frame groups
```

当前是软件近似同步：

```text
grab C0/C1/C2/C3
retrieve C0/C1/C2/C3
record host timestamp
write JPEG + frames.jsonl
```

每条 frame record 包含：

- `group_id`
- `camera_id`
- `timestamp_unix_ns`
- `timestamp_monotonic_ns`
- `timestamp_source`
- `skew_us`
- relative image path

## 边界

15-30 FPS 原型期可以先用 host timestamp 和 `skew_us` 判断四目同步质量。最终 motion capture 应优先考虑 hardware trigger 或 device timestamp。
