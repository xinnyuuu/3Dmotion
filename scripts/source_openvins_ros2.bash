#!/usr/bin/env bash
# Source ROS2 Humble and the local OpenVINS packages built under open_vins/.
#
# Usage:
#   source scripts/source_openvins_ros2.bash

_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_repo_root="$(cd "${_script_dir}/.." && pwd)"
_openvins_root="${_repo_root}/open_vins"

source /opt/ros/humble/setup.bash

for _pkg in ov_core ov_init ov_msckf ov_eval; do
  _setup="${_openvins_root}/install/${_pkg}/share/${_pkg}/local_setup.bash"
  if [ -f "${_setup}" ]; then
    source "${_setup}"
  else
    echo "missing OpenVINS package setup: ${_setup}" >&2
    echo "Build OpenVINS first, e.g. cd open_vins && colcon build --packages-select ${_pkg}" >&2
    return 1 2>/dev/null || exit 1
  fi
done

unset _pkg
unset _setup
unset _openvins_root
unset _repo_root
unset _script_dir
