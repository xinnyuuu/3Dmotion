# AprilGrid World Anchor Workflow

This is the current reproducible MVP workflow. It uses a fixed desktop AprilGrid as the world frame `W` and estimates:

```text
desktop AprilGrid tag36h11 ids 100-111 -> T_W_H
wrist tag16h5 ring                         -> T_W_B
optional hand landmarks                    -> hand_skeletons in W
```

It is a simpler MVP path than sparse-feature head VIO because it does not require OpenVINS initialization. If `imus/head_imu.jsonl` is present, the processor still uses it in an OpenVINS-style propagate-then-update loop: gyro carries short-term orientation, AprilGrid observations correct absolute position and slow orientation drift. `wrist_imu` is used the same way for wrist orientation between bracelet visual updates.

The OpenVINS route remains in the repository as a future extension path for AprilGrid-free operation.

## 0. Reproduce From The Tracked Example

The repository includes a small excerpt from `session_20260630_141240`:

```text
data/examples/session_20260630_141240_excerpt/
```

Run:

```bash
cd ~/lxy/3DMotion
source .venv/bin/activate

python scripts/process_dashboard_session.py \
  data/examples/session_20260630_141240_excerpt \
  --hands
```

Expected outputs:

```text
data/processed/session_20260630_141240_excerpt/world_anchor/
  world_anchor_candidates.jsonl
  wrist_world_candidates.jsonl
  hand_skeletons.jsonl
  head_pose.jsonl
  motion_frame.jsonl
  motion_frame_filtered.jsonl
  world_anchor_summary.json
```

The wrapper prints a copy-paste RViz command. Manual replay:

```bash
cd ~/lxy/3DMotion/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vimas_motion_bringup --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1

ros2 launch vimas_motion_bringup motion_replay_rviz.launch.py \
  motion_jsonl:=$(realpath ../data/processed/session_20260630_141240_excerpt/world_anchor/motion_frame_filtered.jsonl)
```

Use `motion_frame.jsonl` for raw estimates. Use `motion_frame_filtered.jsonl` for the data-level filtered result.

## 1. Generate The World Tag Grid

Current default grid:

```text
family: tag36h11
ids: 100-111
layout: 3 rows x 4 cols on A4 landscape
tag width: 37 mm
gap: 11 mm
pitch: 48 mm
```

```bash
python scripts/generate_apriltag_grid.py \
  --family tag36h11 \
  --start-id 100 \
  --rows 3 \
  --cols 4 \
  --tag-size-mm 37 \
  --gap-mm 11 \
  --paper A4 \
  --output data/processed/world_tag_grid_a4.png \
  --config configs/world_tags.yaml
```

Print it, measure the printed black-square side length, then update:

```text
configs/world_tags.yaml
```

Set `tag_size_m` to the measured size in meters. If the print scale is off, regenerate with the measured tag/gap dimensions so every `T_W_T` translation matches the physical page.

## 2. Record A Session

Place the AprilGrid where at least one head camera can see it. For the first test, keep both the grid and wrist tag visible in the same head camera view.

Use the dashboard or camera capture as usual.

## 3. Offline Processing

Recommended dashboard-session entry:

```bash
python scripts/process_dashboard_session.py \
  data/raw/session_YYYYMMDD_HHMMSS \
  --hands
```

Lower-level entry:

```bash
python scripts/process_world_anchor_session.py offline \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --world-tags configs/world_tags.yaml \
  --bracelet configs/bracelet.yaml \
  --output-dir data/processed/session_YYYYMMDD_HHMMSS/world_anchor \
  --hands
```

The default head fusion keeps visual position dominant and lets `head_imu` reduce short-term rotational jitter:

```text
max_prediction_gap_s: 1.0
max_position_prediction_gap_s: 0.25
head_position_visual_alpha: 0.75
head_orientation_visual_alpha: 0.12
```

Equivalent one-command postprocess entry:

```bash
python scripts/postprocess_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --world-anchor
```

Outputs:

```text
world_anchor_candidates.jsonl
wrist_world_candidates.jsonl
hand_skeletons.jsonl
head_pose.jsonl
motion_frame.jsonl
world_anchor_summary.json
```

`process_dashboard_session.py` also writes `motion_frame_filtered.jsonl` by default. That file is produced by `packages/session_tools/motion_frame_filter.py`, a data-level constant-velocity filter with innovation gating. It is intentionally separate from RViz display smoothing.

Replay in RViz:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vimas_motion_bringup motion_replay_rviz.launch.py \
  motion_jsonl:=$(realpath ../data/processed/session_YYYYMMDD_HHMMSS/world_anchor/motion_frame_filtered.jsonl)
```

## 4. Hand Landmark Modes

The hand skeleton is a visualization/debug signal, not yet a fully constrained hand model.

Default behavior:

```text
multi-view triangulation when two or more cameras see the same hand
guided single-view fallback when a recent multi-view hand exists
no direct raw single-view fallback unless explicitly requested
max one selected hand skeleton per frame
```

Strict mode:

```bash
python scripts/process_dashboard_session.py \
  data/raw/session_YYYYMMDD_HHMMSS \
  --hands \
  --hand-multiview-only
```

Debug raw single-view fallback:

```bash
python scripts/process_dashboard_session.py \
  data/raw/session_YYYYMMDD_HHMMSS \
  --hands \
  --hand-allow-direct-singleview-fallback
```

Raw single-view fallback can produce depth and scale jumps, especially with fisheye side cameras. Use it only to inspect visibility coverage.

## 5. Live Preview

Run direct camera processing with a top-down OpenCV map:

```bash
python scripts/process_world_anchor_session.py live \
  --source C1:/dev/video0 \
  --source C2:/dev/video2 \
  --world-tags configs/world_tags.yaml \
  --bracelet configs/bracelet.yaml \
  --fps 10 \
  --show \
  --output-jsonl /tmp/3dmotion_world_anchor_live.jsonl
```

Use the actual `/dev/video*` paths from the dashboard/device list.

To add MediaPipe hand skeletons to JSON and RViz, add `--hands`:

```bash
python scripts/process_world_anchor_session.py live \
  --source C0:/dev/video4 \
  --source C1:/dev/video0 \
  --source C2:/dev/video6 \
  --source C3:/dev/video2 \
  --world-tags configs/world_tags.yaml \
  --bracelet configs/bracelet.yaml \
  --fps 10 \
  --ros-publish \
  --hands
```

When launched from the dashboard with IMUs connected, the backend writes live `head_imu` and `wrist_imu` JSONL side streams and passes them to this live processor. The live fusion then uses the same IMU-aided stabilization as offline processing: `head_imu` stabilizes head orientation, while `wrist_imu` propagates wrist orientation between AprilTag visual updates. Camera and IMU timestamps are both on the host monotonic clock.

Hand skeletons are only published when a fused wrist pose exists and the lifted hand geometry passes simple length/extent gates. This avoids RViz frames with stretched fingers or multiple selected hands from different cameras.

Offline:

```bash
python scripts/postprocess_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --world-anchor \
  --hands
```

## 6. Notes

- The grid is solved as one board PnP when two or more world tags are visible.
- Single-tag bootstrap is disabled by default because planar ambiguity can jump. A tracked session may still use a gated single tag as a fallback after a stable board pose exists.
- Tag size and gap must match the printed page. Regenerate `configs/world_tags.yaml` if the print scale changes.
- Results are only available while at least one world-grid tag is visible.
- For robust hand tracking, keep a wrist tag visible at the same time as the grid during the first MVP tests.
- Hand skeleton landmarks are currently MediaPipe 2D landmarks lifted by multi-view triangulation when possible, with guided single-view fallback. They are useful for RViz visualization and motion debugging, but they are not yet a bone-length constrained kinematic hand filter.
