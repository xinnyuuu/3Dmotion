#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

export ROS_LOG_DIR="${ROS_LOG_DIR:-$repo_root/data/processed/ros_logs}"
mkdir -p "$ROS_LOG_DIR"

set +u
source scripts/source_openvins_ros2.bash
source ros2_ws/install/setup.bash
set -u

exec ros2 launch vimas_motion_bringup head_vio_rviz.launch.py "$@"
