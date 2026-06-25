from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import yaml

from packages.apriltag_ring_node.config import load_camera_calibrations, load_camera_priority


DEFAULT_HEAD_CAMERA_IDS = ["C1", "C2", "C0", "C3"]


class _OpenCvYamlDumper(yaml.SafeDumper):
    def increase_indent(self, flow: bool = False, indentless: bool = False):
        return super().increase_indent(flow, False)


def generate_openvins_config(
    cameras_path: Path,
    output_dir: Path,
    camera_ids: list[str] | None = None,
    template_config_dir: Path | None = None,
) -> dict:
    calibrations = load_camera_calibrations(cameras_path)
    configured_priority = load_camera_priority(cameras_path)
    default_priority = configured_priority or DEFAULT_HEAD_CAMERA_IDS
    selected_ids = camera_ids or [camera_id for camera_id in default_priority if camera_id in calibrations]
    if not selected_ids:
        selected_ids = list(calibrations)
    missing = [camera_id for camera_id in selected_ids if camera_id not in calibrations]
    if missing:
        raise RuntimeError(f"Missing camera calibration entries: {missing}")

    output_dir.mkdir(parents=True, exist_ok=True)
    estimator_path = output_dir / "estimator_config.yaml"
    imu_path = output_dir / "kalibr_imu_chain.yaml"
    imucam_path = output_dir / "kalibr_imucam_chain.yaml"

    _write_estimator_config(estimator_path, template_config_dir, camera_count=len(selected_ids))
    _write_imu_chain(imu_path)
    _write_imucam_chain(imucam_path, [calibrations[camera_id] for camera_id in selected_ids])

    summary = {
        "output_dir": str(output_dir),
        "files": {
            "estimator_config": str(estimator_path),
            "kalibr_imu_chain": str(imu_path),
            "kalibr_imucam_chain": str(imucam_path),
        },
        "camera_ids": selected_ids,
        "topics": {
            "imu": "/imu0",
            "cameras": {camera_id: f"/cam{index}/image_raw" for index, camera_id in enumerate(selected_ids)},
        },
        "camera_priority": selected_ids,
        "projection_note": (
            "3DMotion stores the physical camera model as Mei/omni when xi is present. "
            "This OpenVINS checkout supports pinhole-radtan/equidistant, so kalibr_imucam_chain.yaml "
            "exports a compatibility pinhole-radtan view from fu/fv/cu/cv/k1/k2/p1/p2. "
            "For strict omni VIO, rectify frames to a virtual pinhole camera or extend OpenVINS camera models."
        ),
        "warning": "T_H_C is used as T_imu_cam assuming headset frame H equals the OpenVINS IMU frame. Replace this after IMU-camera extrinsic calibration.",
    }
    (output_dir / "openvins_config_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def _write_estimator_config(path: Path, template_config_dir: Path | None, *, camera_count: int) -> None:
    max_cameras = max(1, int(camera_count))
    if template_config_dir is not None:
        template = template_config_dir / "estimator_config.yaml"
        if template.exists():
            shutil.copyfile(template, path)
            text = path.read_text(encoding="utf-8")
            text = text.replace("use_stereo: true", "use_stereo: false")
            text = _replace_scalar_yaml_line(text, "max_cameras", str(max_cameras))
            text = _replace_scalar_yaml_line(text, "init_imu_thresh", "0.5")
            text = _replace_scalar_yaml_line(text, "fast_threshold", "10")
            text = _replace_scalar_yaml_line(text, "track_frequency", "15.0")
            path.write_text(text, encoding="utf-8")
            return

    path.write_text(
        """%YAML:1.0

verbosity: "INFO"
use_fej: true
integration: "rk4"
use_stereo: false
max_cameras: {max_cameras}

calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true
calib_imu_intrinsics: false
calib_imu_g_sensitivity: false

max_clones: 11
max_slam: 50
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1
gravity_mag: 9.81

feat_rep_msckf: "GLOBAL_3D"
feat_rep_slam: "ANCHORED_MSCKF_INVERSE_DEPTH"
feat_rep_aruco: "ANCHORED_MSCKF_INVERSE_DEPTH"

try_zupt: false
init_window_time: 2.0
init_imu_thresh: 0.5
init_max_disparity: 10.0
init_max_features: 50
init_dyn_use: false

record_timing_information: false
save_total_state: false
filepath_est: "/tmp/ov_estimate.txt"
filepath_std: "/tmp/ov_estimate_std.txt"
filepath_gt: "/tmp/ov_groundtruth.txt"

use_klt: true
num_pts: 200
fast_threshold: 10
grid_x: 5
grid_y: 5
min_px_dist: 10
knn_ratio: 0.70
track_frequency: 15.0
downsample_cameras: false
num_opencv_threads: 4
histogram_method: "HISTOGRAM"

use_aruco: false
use_mask: false

relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
""".format(max_cameras=max_cameras),
        encoding="utf-8",
    )


def _write_imu_chain(path: Path) -> None:
    data = {
        "imu0": {
            "T_i_b": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            "accelerometer_noise_density": 0.02,
            "accelerometer_random_walk": 0.002,
            "gyroscope_noise_density": 0.0017,
            "gyroscope_random_walk": 0.0002,
            "rostopic": "/imu0",
            "time_offset": 0.0,
            "update_rate": 100.0,
            "model": "kalibr",
            "Tw": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "R_IMUtoGYRO": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "Ta": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "R_IMUtoACC": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "Tg": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        }
    }
    _write_opencv_yaml(path, data)


def _replace_scalar_yaml_line(text: str, key: str, value: str) -> str:
    lines = []
    prefix = f"{key}:"
    for line in text.splitlines():
        stripped = line.lstrip()
        indent = line[: len(line) - len(stripped)]
        if stripped.startswith(prefix):
            comment = ""
            if "#" in stripped:
                comment = " " + stripped[stripped.index("#") :]
            lines.append(f"{indent}{key}: {value}{comment}")
        else:
            lines.append(line)
    return "\n".join(lines) + ("\n" if text.endswith("\n") else "")


def _write_imucam_chain(path: Path, calibrations) -> None:
    data = {}
    for index, calibration in enumerate(calibrations):
        if not calibration.supported_opencv_projection:
            raise RuntimeError(
                f"Camera {calibration.camera_id} uses projection_model={calibration.projection_model!r}. "
                "This OpenVINS config generator supports pinhole/radtan and fisheye/equidistant only. "
                "Use a Kalibr/OpenVINS-compatible equidistant fallback, or extend OpenVINS camera models first."
            )
        intrinsics = calibration.intrinsics
        distortion = calibration.distortion.reshape(-1).tolist()
        width, height = calibration.image_size or (0, 0)
        distortion_model = _openvins_distortion_model(calibration.distortion_model)
        data[f"cam{index}"] = {
            "T_imu_cam": calibration.T_H_C.as_matrix().tolist(),
            "cam_overlaps": [i for i in range(len(calibrations)) if i != index],
            "source_camera_id": calibration.camera_id,
            "source_camera_model": calibration.camera_model,
            "source_projection_model": calibration.projection_model,
            "source_xi": calibration.xi,
            "camera_model": "pinhole",
            "distortion_coeffs": [float(v) for v in distortion[:4]],
            "distortion_model": distortion_model,
            "intrinsics": [
                float(intrinsics[0, 0]),
                float(intrinsics[1, 1]),
                float(intrinsics[0, 2]),
                float(intrinsics[1, 2]),
            ],
            "resolution": [int(width), int(height)],
            "rostopic": f"/cam{index}/image_raw",
        }
    _write_opencv_yaml(path, data)


def _openvins_distortion_model(value: str) -> str:
    if value.lower() in {"equidistant", "opencv_fisheye", "fisheye"}:
        return "equidistant"
    return "radtan"


def _write_opencv_yaml(path: Path, data: dict) -> None:
    text = "%YAML:1.0\n\n" + yaml.dump(
        data,
        Dumper=_OpenCvYamlDumper,
        sort_keys=False,
        default_flow_style=None,
        width=1000,
    )
    path.write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate first-pass OpenVINS config files from 3DMotion camera calibration.")
    parser.add_argument("--cameras", default="configs/cameras.yaml", help="3DMotion camera calibration YAML.")
    parser.add_argument("--output-dir", default="configs/openvins/generated_head_vio", help="Output config directory.")
    parser.add_argument(
        "--camera-id",
        action="append",
        dest="camera_ids",
        help="Camera ID to include. Repeat for multiple cameras. Default priority: C1,C2,C0,C3.",
    )
    parser.add_argument("--template-config-dir", default="open_vins/config/euroc_mav", help="Optional OpenVINS template config directory.")
    args = parser.parse_args()

    summary = generate_openvins_config(
        cameras_path=Path(args.cameras),
        output_dir=Path(args.output_dir),
        camera_ids=args.camera_ids,
        template_config_dir=Path(args.template_config_dir) if args.template_config_dir else None,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
