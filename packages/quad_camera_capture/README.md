# quad_camera_capture

Purpose:

```text
four headset cameras -> timestamped frame groups
```

List available V4L2 cameras first:

```bash
python scripts/list_cameras.py --configs
```

This is a first-pass OpenCV capture pipeline for cameras that are not yet
hardware synchronized. It uses `grab()` on all cameras first, then `retrieve()`
on each camera and records host timestamps.

Capture four local camera indexes:

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

Output layout:

```text
data/raw/quad_camera_test/
  frames.jsonl
  C0/00000000.jpg
  C1/00000000.jpg
  C2/00000000.jpg
  C3/00000000.jpg
```

Each JSONL record includes:

- `group_id`
- `camera_id`
- `timestamp_unix_ns`
- `timestamp_monotonic_ns`
- `timestamp_source`
- `skew_us`
- relative image path

This is good enough for early 15-30 FPS feasibility testing. For final motion
capture, prefer hardware trigger or cameras with device timestamps.
