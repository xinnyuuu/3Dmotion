# 3D Motion

Rigid-body 3D motion capture prototype for head and wrist trajectories.

当前仓库保留两条路线：

1. **AprilGrid world-anchor MVP**：当前可复现主线。桌面 AprilGrid 定义世界坐标 `W`，四目头环相机估计头环 `T_W_H`，手环 AprilTag 估计手腕 `T_W_B`，可选 MediaPipe hand landmarks 用于 RViz 调试。
2. **OpenVINS sparse-feature head VIO**：未来扩展路线。不依赖 AprilGrid，用 sparse visual feature tracks + `head_imu` 输出 `T_W_H`。这条路线的代码和文档保留，但由于相机模型、外参和时间同步还没收敛，暂时不是默认复现路径。

目标是重建两类刚体的 6DoF motion stream：

- `H`: headset / head camera rig
- `B`: wristband

第一阶段不做 anatomy skeleton，也不做 inverse kinematics。系统先稳定输出刚体位姿，再给后续 World Action Model 使用。

## Core Transform

项目里统一使用：

```text
T_A_B 表示把 B 坐标系里的点变换到 A 坐标系
```

核心关系：

```text
T_W_H: headset frame H in world frame W
T_H_B: wristband frame B in headset frame H
T_W_B = T_W_H * T_H_B
```

`Context.md` 里的 `T_W<-H` 与这里的 `T_W_H` 是同一含义。

## Current Stack

```text
Dashboard recording
  -> raw session: cameras + head_imu + wrist_imu

AprilGrid world-anchor MVP
  -> desktop tag36h11 ids 100-111 + wrist tag ring -> T_W_H and T_W_B

OpenVINS sparse-feature route, experimental
  -> selected head cameras (default C1,C2,C0,C3) + head_imu -> T_W_H

Future wrist ESKF
  -> wrist_imu + wrist visual pose -> smoother wrist pose

Future WAM export
  -> T_W_H + T_W_B -> motion stream
```

OpenVINS 用于快速跑通 head VIO。MOLA 暂时放在后面，用于 replay、trajectory tools、map/anchor constraints 和系统集成，不是当前 dashboard 采集的前置条件。

## Reproduce The Current MVP

This repository includes a small excerpt from `session_20260630_141240` under:

```text
data/examples/session_20260630_141240_excerpt/
```

It contains four-camera JPG frames plus `head_imu` / `wrist_imu` JSONL snippets. To run the current AprilGrid world-anchor pipeline from a fresh clone:

```bash
cd ~/lxy/3DMotion
python3 -m venv --clear .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

python scripts/process_dashboard_session.py \
  data/examples/session_20260630_141240_excerpt \
  --hands
```

The command validates the session, runs the AprilGrid world-anchor processor, writes:

```text
data/processed/session_20260630_141240_excerpt/world_anchor/motion_frame.jsonl
data/processed/session_20260630_141240_excerpt/world_anchor/motion_frame_filtered.jsonl
```

and prints an RViz replay command. To replay manually:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vimas_motion_bringup --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
ros2 launch vimas_motion_bringup motion_replay_rviz.launch.py \
  motion_jsonl:=$(realpath ../data/processed/session_20260630_141240_excerpt/world_anchor/motion_frame_filtered.jsonl)
```

Use `motion_frame.jsonl` instead if you want the raw unfiltered estimates.

## Start Here

日常配置、采集、检查和处理流程集中在：

[docs/system_workflow.md](docs/system_workflow.md)

AprilGrid 主线细节：

[docs/world_anchor_workflow.md](docs/world_anchor_workflow.md)

文档索引：

[docs/README.md](docs/README.md)

核心技术文档：

- [docs/architecture.md](docs/architecture.md)
- [docs/coordinate_frames.md](docs/coordinate_frames.md)
- [docs/calibration.md](docs/calibration.md)
- [docs/timestamp_sync.md](docs/timestamp_sync.md)
- [docs/data_schema.md](docs/data_schema.md)
- [docs/prototype_plan.md](docs/prototype_plan.md)

## Quick Commands

Python tools:

```bash
cd ~/lxy/3DMotion
python3 -m venv --clear .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Start capture dashboard:

```bash
source .venv/bin/activate
python scripts/capture_dashboard.py
```

Open:

```text
http://127.0.0.1:8766/
```

Run automated tests:

```bash
python -m pytest tests
```

Check whether calibration files are ready for four cameras + dual IMUs:

```bash
python scripts/check_calibration_readiness.py
```

## Repository Layout

```text
3DMotion/
  Context.md          original project context and target schema
  README.md           project entry
  docs/               long-term technical documentation
  configs/            calibration and pipeline configuration
  packages/           Python prototype modules
  scripts/            CLI entrypoints
  project_tests/      real-session quality checks
  tests/              no-hardware automated tests
  ros2_ws/            ROS 2 workspace skeleton
  data/               local datasets; raw/processed ignored, examples tracked
  notebooks/          local analysis notes, ignored by git
  open_vins/          upstream OpenVINS checkout
  mola/               upstream MOLA checkout
```

## Git Notes

`open_vins/` and `mola/` are upstream repositories. Keep project code in this repository's own `packages/`, `configs/`, `docs/`, `scripts/`, and `ros2_ws/` directories unless intentionally patching upstream code.
