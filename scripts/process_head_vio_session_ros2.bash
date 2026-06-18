#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/process_head_vio_session_ros2.bash --session-dir data/raw/session_YYYYMMDD_HHMMSS

Options:
  --session-dir PATH             Required raw dashboard session.
  --output-dir PATH              Default: data/processed/<session_name>/openvins_c0.
  --camera-id ID                 Default: C0.
  --imu-slot SLOT                Default: head_imu.
  --cameras PATH                 Default: configs/cameras.yaml.
  --template-config-dir PATH     Default: open_vins/config/euroc_mav.
  --max-duration-s SECONDS       Optional debug rosbag2 duration.
  --start-offset-s SECONDS       Skip this many seconds before rosbag2 export.
  --image-stride N               Write every Nth image while keeping all IMU samples.
  --keep-existing-rosbag2        Do not overwrite an existing rosbag2 directory.
  -h, --help                     Show this help.

This wrapper uses .venv Python for project processing, then /usr/bin/python3
after sourcing ROS2 for rosbag2 export. It avoids mixing ROS2 packages into
the project venv.
EOF
}

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
session_dir=""
output_dir=""
camera_id="C0"
imu_slot="head_imu"
cameras="configs/cameras.yaml"
template_config_dir="open_vins/config/euroc_mav"
keep_existing_rosbag2=0
max_duration_s=""
start_offset_s="0.0"
image_stride="1"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session-dir)
      session_dir="$2"
      shift 2
      ;;
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    --camera-id)
      camera_id="$2"
      shift 2
      ;;
    --imu-slot)
      imu_slot="$2"
      shift 2
      ;;
    --cameras)
      cameras="$2"
      shift 2
      ;;
    --template-config-dir)
      template_config_dir="$2"
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

if [[ -z "$session_dir" ]]; then
  echo "--session-dir is required." >&2
  usage >&2
  exit 2
fi

cd "$repo_root"

if [[ -z "$output_dir" ]]; then
  session_name="$(basename "$session_dir")"
  output_dir="data/processed/${session_name}/openvins_c0"
fi

if [[ "$output_dir" = /* ]]; then
  output_dir_abs="$output_dir"
else
  output_dir_abs="$(pwd)/$output_dir"
fi

project_python="$repo_root/.venv/bin/python"
if [[ ! -x "$project_python" ]]; then
  project_python="python3"
fi

"$project_python" scripts/process_head_vio_session.py \
  --session-dir "$session_dir" \
  --output-dir "$output_dir_abs" \
  --cameras "$cameras" \
  --template-config-dir "$template_config_dir" \
  --camera-id "$camera_id" \
  --imu-slot "$imu_slot" \
  --no-rosbag2

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "$keep_existing_rosbag2" -eq 0 && -d "$output_dir_abs/rosbag2" ]]; then
  rm -rf "$output_dir_abs/rosbag2"
  rm -f "$output_dir_abs/rosbag2_summary.json"
fi

rosbag_args=(
  scripts/write_openvins_rosbag2.py
  --prepared-dir "$output_dir_abs" \
  --bag-dir "$output_dir_abs/rosbag2" \
  --frame-id "$imu_slot" \
  --start-offset-s "$start_offset_s" \
  --image-stride "$image_stride"
)
if [[ -n "$max_duration_s" ]]; then
  rosbag_args+=(--max-duration-s "$max_duration_s")
fi

/usr/bin/python3 "${rosbag_args[@]}"

PROCESS_OUTPUT_DIR="$output_dir_abs" "$project_python" -c 'import json, os
from pathlib import Path
output = Path(os.environ["PROCESS_OUTPUT_DIR"])
summary_path = output / "head_vio_process_summary.json"
rosbag_summary_path = output / "rosbag2_summary.json"
if summary_path.exists() and rosbag_summary_path.exists():
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    rosbag_summary = json.loads(rosbag_summary_path.read_text(encoding="utf-8"))
    summary["steps"]["rosbag2"] = {
        "ok": True,
        "skipped": False,
        "bag_dir": str(output / "rosbag2"),
        "summary": rosbag_summary,
    }
    summary["ok"] = bool(summary.get("ready_for_p3a"))
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
'

cat <<EOF

Head VIO session is ready:
  prepared dir: $output_dir_abs
  rosbag2:      $output_dir_abs/rosbag2

Next:
  source scripts/source_openvins_ros2.bash
  ros2 launch ov_msckf subscribe.launch.py config_path:=$output_dir_abs/config/estimator_config.yaml max_cameras:=1 use_stereo:=false

  source /opt/ros/humble/setup.bash
  ros2 bag play $output_dir_abs/rosbag2
EOF
