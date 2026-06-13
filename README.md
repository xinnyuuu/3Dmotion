# 3D Motion

Rigid-body 3D motion capture prototype for head and wrist trajectories.

The project goal is to reconstruct high-frequency 6DoF motion streams for:

- `H`: headset rigid body
- `B`: wristband rigid body

The output is a unified world-frame motion stream for downstream World Action
Model ingestion. This project intentionally avoids anatomy-based skeleton
reconstruction and inverse kinematics in the first stage.

## Core Idea

Two transforms are central:

- `T_W_H`: pose of the headset frame `H` in the world frame `W`
- `T_H_B`: pose of the wristband frame `B` in the headset frame `H`

The final wrist pose in world coordinates is:

```text
T_W_B = T_W_H * T_H_B
```

In the notation used in `Context.md`, `T_W<-H` means the same thing as
`T_W_H`: a transform that maps coordinates expressed in `H` into coordinates
expressed in `W`.

## Recommended Prototype Stack

Use OpenVINS for the fast head-pose prototype, and use MOLA later for replay,
system integration, trajectory tooling, and map/anchor constraints.

```text
Head cameras + head IMU
  -> OpenVINS
  -> T_W_H

Headset cameras observing wrist AprilTags
  -> AprilTag ring solver
  -> T_H_B

Wrist IMU + visual wrist pose
  -> wrist ESKF
  -> smoothed T_W_B

ROS 2 + rosbag2
  -> logging, replay, debugging

MOLA
  -> later-stage replay, trajectory tools, map/anchor integration
```

## Repository Layout

```text
3DMotion/
  Context.md                       # Original project context and target schema
  README.md                        # This file
  docs/                            # Architecture and design notes
  configs/                         # Calibration and pipeline configuration
  packages/                        # Implementation packages, initially stubs
  ros2_ws/                         # ROS 2 workspace skeleton
  scripts/                         # Developer and data utility scripts
  data/                            # Local datasets, ignored by git
  tests/                           # Test plans and future automated tests
  notebooks/                       # Local analysis notebooks, ignored by git
  open_vins/                       # Upstream OpenVINS checkout
  mola/                            # Upstream MOLA checkout
```

## Quick Start

Clone or enter the workspace:

```bash
cd 3DMotion
```

## Python Environment

Use a local Python virtual environment for the Python-only prototype tools:

- BLE IMU capture
- quad-camera capture
- AprilTag experiments
- offline logs and plotting
- early wrist ESKF prototypes

ROS 2, OpenVINS, and MOLA should still use their normal system/ROS setup
instead of being forced into this Python venv.

Install the Ubuntu/Debian prerequisites if needed:

```bash
sudo apt-get update
sudo apt-get install -y python3.10-venv python3-pip
```

Create and activate the environment:

```bash
python3 -m venv --clear .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Equivalent explicit dependency install:

```bash
python -m pip install bleak opencv-contrib-python numpy pyyaml
```

Verify the capture CLIs:

```bash
python scripts/capture_imu_jsonl.py --help
python scripts/capture_quad_camera.py --help
python scripts/capture_dashboard.py --help
```

Read the core docs:

```bash
cat docs/coordinate_frames.md
cat docs/architecture.md
cat docs/prototype_plan.md
cat docs/timestamp_sync.md
cat docs/camera_pipeline_comparison.md
```

For the first feasibility test, run the existing AprilTag project outside this
repo skeleton:

```bash
cd ../AprilTag
python main.py --camera 0 --config config.yaml
```

The first target is to prove that a fixed camera/headset frame can produce a
stable `T_H_B` wrist pose before integrating OpenVINS or MOLA.

## ROS2 / OpenVINS Setup

The Python venv is for capture and offline preparation. ROS2, rosbag2, and
OpenVINS need the ROS environment.

Install the ROS2 tools and OpenVINS build dependencies:

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  ros-humble-rosbag2-py \
  ros-humble-rosbag2-storage-default-plugins \
  ros-humble-rosbag2-transport \
  libboost-all-dev \
  libceres-dev \
  libeigen3-dev
```

Check that ROS2 Python bindings are visible:

```bash
source /opt/ros/humble/setup.bash

python3 - <<'PY'
import rclpy
import rosbag2_py
import sensor_msgs
print("ROS2 Python environment OK")
PY
```

Build the local OpenVINS checkout:

```bash
cd ~/lxy/3DMotion/open_vins
source /opt/ros/humble/setup.bash

colcon build \
  --event-handlers console_cohesion+ \
  --packages-select ov_core ov_init ov_msckf ov_eval \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

After a successful build, this file should exist:

```text
open_vins/install/setup.bash
```

Source OpenVINS before launching `ov_msckf`:

```bash
cd ~/lxy/3DMotion
source scripts/source_openvins_ros2.bash
```

The helper sources ROS2 Humble and the local OpenVINS package setup files:

```text
open_vins/install/ov_core/share/ov_core/local_setup.bash
open_vins/install/ov_init/share/ov_init/local_setup.bash
open_vins/install/ov_msckf/share/ov_msckf/local_setup.bash
open_vins/install/ov_eval/share/ov_eval/local_setup.bash
```

This project uses the helper instead of only `source open_vins/install/setup.bash`
because this OpenVINS checkout may not add the isolated package prefixes to
`AMENT_PREFIX_PATH` from the top-level setup script.

If `source scripts/source_openvins_ros2.bash` reports a missing package setup
file, OpenVINS has not been built yet.

Recommended terminal roles for P3:

- Terminal A: source ROS2, then play `rosbag2`.
- Terminal B: source `scripts/source_openvins_ros2.bash`, then launch OpenVINS.
- Python venv is only needed when running this project's Python scripts.

## Data Capture Skeleton

For interactive capture, start the local dashboard:

```bash
source .venv/bin/activate
python scripts/capture_dashboard.py
```

Open:

```text
http://127.0.0.1:8766/
```

During recording, the dashboard preview switches from direct camera streaming
to the latest frame already written by the recorder. This avoids opening the
same `/dev/videoX` twice while still letting you watch the captured data.

If Start/Stop does not appear to save camera data, first run a direct camera
preflight outside the dashboard:

```bash
python scripts/list_cameras.py --configs
python scripts/check_camera_capture.py \
  --source C0:/dev/video0 \
  --format MJPG \
  --width 1280 \
  --height 720 \
  --fps 15 \
  --output-dir data/raw/camera_preflight
```

This writes:

```text
data/raw/camera_preflight/C0_probe.jpg
data/raw/camera_preflight/camera_preflight_summary.json
```

If this reports `open_failed`, fix the camera node, permission, selected format,
or another process holding the device before debugging dashboard recording.

After pressing Start/Stop, validate the created session before running any
offline processing:

```bash
python scripts/validate_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

The dashboard also runs this validation automatically after Stop and writes:

```text
data/raw/session_YYYYMMDD_HHMMSS/session_summary.json
```

If camera recording worked, the dashboard/session summary should report
`ok_for_camera_replay: true` and nonzero `camera_frame_count`. If it reports
capture errors but still has frame records, at least one camera recorded
successfully and the missing camera can be debugged separately.

To run the standard offline checks and optional postprocessing in one command:

```bash
python scripts/postprocess_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --apriltag \
  --openvins \
  --openvins-config
```

This writes a combined report:

```text
data/processed/session_YYYYMMDD_HHMMSS/postprocess_summary.json
```

Use this after a short capture to quickly answer: camera data saved, wrist
AprilTag processed, and OpenVINS inputs prepared.

Scan and capture WT-series BLE IMU data without the old GUI:

```bash
source .venv/bin/activate
python scripts/capture_imu_jsonl.py --scan
python scripts/capture_imu_jsonl.py \
  --address XX:XX:XX:XX:XX:XX \
  --sensor-id wrist_imu \
  --output data/raw/wrist_imu.jsonl \
  --duration-s 30
```

Capture four camera streams at 15-30 FPS:

```bash
source .venv/bin/activate
python scripts/list_cameras.py --configs
python scripts/capture_quad_camera.py \
  --source C0:0 \
  --source C1:1 \
  --source C2:2 \
  --source C3:3 \
  --fps 30 \
  --duration-s 30 \
  --output-dir data/raw/quad_camera_test
```

Process the recorded camera session into wrist AprilTag visual poses:

```bash
source .venv/bin/activate
python scripts/process_apriltag_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS/cameras \
  --cameras configs/cameras.yaml \
  --bracelet configs/bracelet.yaml \
  --output-dir data/processed/wrist_visual
```

This creates:

```text
data/processed/wrist_visual/wrist_visual_candidates.jsonl
data/processed/wrist_visual/wrist_visual_pose.jsonl
```

Prepare P3a head VIO with one camera and one rigid-mounted head IMU:

For the P3a recording itself, use this motion pattern:

```text
0-3s: keep the rigid headset still
3-10s: translate the headset 0.3-1.0m with visible acceleration
       add mild yaw/pitch/roll motion
       keep C0 looking at textured, non-blank scenery
```

Avoid testing OpenVINS with a mostly static session. If the IMU has almost no
acceleration excitation and the image disparity is tiny, OpenVINS will keep
printing messages like `failed static init: no accel jerk detected`.

```bash
source .venv/bin/activate
python scripts/check_head_vio_readiness.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

If readiness passes, generate the OpenVINS intermediate streams and config:

```bash
python scripts/prepare_p3_head_vio.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

This uses the P3a frame convention:

```text
I = head_imu frame
H := I
T_W_H := T_W_I
```

This writes:

```text
data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/openvins_session_manifest.json
data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/images.jsonl
data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/imu.jsonl
data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/config/
```

Important: `head_imu` must be rigidly fixed to the headset/camera rig. The
generated config only uses the current `T_H_C` values as a first-pass
`T_imu_cam` assumption. If `T_H_C` is still `null`, the generated extrinsic is
identity. Replace it after IMU-camera extrinsic calibration.

When you are in a ROS2 terminal, write the rosbag2 dataset:

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python scripts/write_openvins_rosbag2.py \
  --prepared-dir data/processed/session_YYYYMMDD_HHMMSS/openvins_c0 \
  --bag-dir data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/rosbag2 \
  --frame-id head_imu
```

Check the generated bag:

```bash
source /opt/ros/humble/setup.bash
ros2 bag info data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/rosbag2
```

Expected first-pass topic counts should match the prepared streams, for example:

```text
/cam0/image_raw: 360
/imu0: 2384
```

Then use two terminals.

Terminal B: start OpenVINS and wait for topics:

```bash
cd ~/lxy/3DMotion
source scripts/source_openvins_ros2.bash

ros2 launch ov_msckf subscribe.launch.py \
  config_path:=$(pwd)/data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/config/estimator_config.yaml \
  max_cameras:=1 \
  use_stereo:=false
```

Terminal A: replay the bag:

```bash
cd ~/lxy/3DMotion
source /opt/ros/humble/setup.bash

ros2 bag play data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/rosbag2
```

The OpenVINS config still needs matching Kalibr-style camera/IMU calibration
files. For fast iteration, keep this path simple: `C0 + head_imu` first, then
add more cameras after the mono pipeline produces a reasonable `T_W_H`.

Early timestamps are host-side timestamps:

- IMU: `host_receive`
- Camera: `host_retrieve`

See `docs/timestamp_sync.md` for the sync plan and the upgrade path toward
hardware sync.

## Development Phases

1. `P0`: Fixed headset frame, AprilTag wristband pose only.
2. `P1`: Add wrist IMU logging on the same timeline.
3. `P2`: Offline wrist ESKF using AprilTag correction.
4. `P3`: Add OpenVINS head VIO for `T_W_H`.
5. `P4`: Combine `T_W_H * T_H_B` into `T_W_B`.
6. `P5`: Add MOLA-based replay, trajectory tools, and anchor-map constraints.

## Git Notes

The `open_vins/` and `mola/` directories are upstream repositories. Keep project
code in this repository's own `packages/`, `configs/`, `docs/`, and `ros2_ws/`
directories unless intentionally patching upstream code.
