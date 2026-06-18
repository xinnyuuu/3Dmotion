#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/process_euroc_openvins_stereo_ros2.bash --mav0-dir ../VIO_data/VIO/V2_01_easy/mav0

Options:
  --mav0-dir PATH              Required EuRoC mav0 directory.
  --output-dir PATH            Default: data/processed/euroc_v2_01_easy/openvins_stereo.
  --max-duration-s SECONDS     Optional debug rosbag2 duration.
  --start-offset-s SECONDS     Skip this many seconds from the first camera frame.
  --image-stride N             Write every Nth image while keeping all IMU samples.
  --keep-existing-rosbag2      Do not overwrite an existing rosbag2 directory.
  -h, --help                   Show this help.
EOF
}

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mav0_dir=""
output_dir="data/processed/euroc_v2_01_easy/openvins_stereo"
max_duration_s=""
start_offset_s="0.0"
image_stride="1"
keep_existing_rosbag2=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mav0-dir)
      mav0_dir="$2"
      shift 2
      ;;
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    --max-duration-s)
      max_duration_s="$2"
      shift 2
      ;;
    --start-offset-s)
      start_offset_s="$2"
      shift 2
      ;;
    --image-stride)
      image_stride="$2"
      shift 2
      ;;
    --keep-existing-rosbag2)
      keep_existing_rosbag2=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$mav0_dir" ]]; then
  echo "--mav0-dir is required." >&2
  usage >&2
  exit 2
fi

cd "$repo_root"

if [[ "$output_dir" = /* ]]; then
  output_dir_abs="$output_dir"
else
  output_dir_abs="$(pwd)/$output_dir"
fi

project_python="$repo_root/.venv/bin/python"
if [[ ! -x "$project_python" ]]; then
  project_python="python3"
fi

"$project_python" scripts/prepare_euroc_openvins_session.py \
  --mav0-dir "$mav0_dir" \
  --output-dir "$output_dir_abs" \
  --config-source-dir open_vins/config/euroc_mav

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "$keep_existing_rosbag2" -eq 0 && -d "$output_dir_abs/rosbag2" ]]; then
  rm -rf "$output_dir_abs/rosbag2"
  rm -f "$output_dir_abs/rosbag2_summary.json"
fi

rosbag_args=(
  scripts/write_openvins_rosbag2.py
  --prepared-dir "$output_dir_abs"
  --bag-dir "$output_dir_abs/rosbag2"
  --frame-id euroc_imu
  --start-offset-s "$start_offset_s"
  --image-stride "$image_stride"
)
if [[ -n "$max_duration_s" ]]; then
  rosbag_args+=(--max-duration-s "$max_duration_s")
fi

/usr/bin/python3 "${rosbag_args[@]}"

cat <<EOF

EuRoC stereo OpenVINS session is ready:
  prepared dir: $output_dir_abs
  rosbag2:      $output_dir_abs/rosbag2

Run OpenVINS:
  source scripts/source_openvins_ros2.bash
  ros2 launch ov_msckf subscribe.launch.py config_path:=$output_dir_abs/config/estimator_config.yaml max_cameras:=2 use_stereo:=true

Play bag:
  source /opt/ros/humble/setup.bash
  ros2 bag play $output_dir_abs/rosbag2

RViz:
  cd ros2_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch vimas_motion_bringup head_vio_rviz.launch.py child_frame:=euroc_imu
EOF
