# 3D Motion

Rigid-body 3D motion capture prototype for head and wrist trajectories.

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

AprilTag wrist visual
  -> T_H_B

OpenVINS P3a
  -> C0 + head_imu -> T_W_H

Future wrist ESKF
  -> wrist_imu + wrist visual pose -> smoother wrist pose

Future WAM export
  -> T_W_H + T_W_B -> motion stream
```

OpenVINS 用于快速跑通 head VIO。MOLA 暂时放在后面，用于 replay、trajectory tools、map/anchor constraints 和系统集成，不是当前 dashboard 采集的前置条件。

## Start Here

日常配置、采集、检查和处理流程集中在：

[docs/system_workflow.md](docs/system_workflow.md)

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
  data/               local datasets, ignored by git
  notebooks/          local analysis notes, ignored by git
  open_vins/          upstream OpenVINS checkout
  mola/               upstream MOLA checkout
```

## Git Notes

`open_vins/` and `mola/` are upstream repositories. Keep project code in this repository's own `packages/`, `configs/`, `docs/`, `scripts/`, and `ros2_ws/` directories unless intentionally patching upstream code.
