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

Read the core docs:

```bash
cat docs/coordinate_frames.md
cat docs/architecture.md
cat docs/prototype_plan.md
```

For the first feasibility test, run the existing AprilTag project outside this
repo skeleton:

```bash
cd ../AprilTag
python main.py --camera 0 --config config.yaml
```

The first target is to prove that a fixed camera/headset frame can produce a
stable `T_H_B` wrist pose before integrating OpenVINS or MOLA.

## Development Phases

1. `P0`: Fixed headset frame, AprilTag wristband pose only.
2. `P1`: Add wrist IMU logging and offline wrist ESKF.
3. `P2`: Add OpenVINS head VIO for `T_W_H`.
4. `P3`: Combine `T_W_H * T_H_B` into `T_W_B`.
5. `P4`: Record ROS 2 bags and replay reproducibly.
6. `P5`: Add MOLA-based replay, trajectory tools, and anchor-map constraints.

## Git Notes

The `open_vins/` and `mola/` directories are upstream repositories. Keep project
code in this repository's own `packages/`, `configs/`, `docs/`, and `ros2_ws/`
directories unless intentionally patching upstream code.

