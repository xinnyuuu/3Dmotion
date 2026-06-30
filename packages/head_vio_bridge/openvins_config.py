from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
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
    imu_calibration_path: Path | None = Path("configs/imu_calibration.yaml"),
    frames_path: Path | None = Path("configs/frames.yaml"),
    kalibr_imucam_path: Path | None = None,
    init_imu_thresh: float = 0.5,
    init_max_disparity: float = 10.0,
    init_dyn_use: bool = False,
    timeshift_cam_imu: float | None = None,
    calib_cam_timeoffset: bool = False,
    calib_cam_extrinsics: bool = False,
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

    _write_estimator_config(
        estimator_path,
        template_config_dir,
        camera_count=len(selected_ids),
        init_imu_thresh=init_imu_thresh,
        init_max_disparity=init_max_disparity,
        init_dyn_use=init_dyn_use,
        calib_cam_timeoffset=calib_cam_timeoffset,
        calib_cam_extrinsics=calib_cam_extrinsics,
    )
    imu_calibration = _load_imu_calibration(imu_calibration_path)
    _write_imu_chain(imu_path, imu_calibration.get("head_imu", {}))
    t_h_ih = _load_t_h_ih(frames_path)
    kalibr_imucam = _load_kalibr_imucam_overrides(kalibr_imucam_path, camera_count=len(selected_ids))
    _write_imucam_chain(
        imucam_path,
        [calibrations[camera_id] for camera_id in selected_ids],
        t_h_ih=t_h_ih,
        kalibr_imucam=kalibr_imucam,
        timeshift_cam_imu=timeshift_cam_imu,
    )

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
        "imu_calibration": {
            "path": str(imu_calibration_path) if imu_calibration_path is not None else None,
            "head_imu_source": imu_calibration.get("head_imu", {}).get("source"),
            "head_imu_sample_count": imu_calibration.get("head_imu", {}).get("sample_count"),
        },
        "frames": {
            "path": str(frames_path) if frames_path is not None else None,
            "T_H_IH_used": t_h_ih.tolist(),
            "T_imu_cam_formula": "T_IH_C = inverse(T_H_IH) * T_H_C",
        },
        "kalibr_imucam": {
            "path": str(kalibr_imucam_path) if kalibr_imucam_path is not None else None,
            "used_camera_indices": sorted(kalibr_imucam),
        },
        "camera_priority": selected_ids,
        "initializer": {
            "init_imu_thresh": init_imu_thresh,
            "init_max_disparity": init_max_disparity,
            "init_dyn_use": init_dyn_use,
        },
        "time_alignment": {
            "timeshift_cam_imu": timeshift_cam_imu,
            "calib_cam_timeoffset": calib_cam_timeoffset,
            "calib_cam_extrinsics": calib_cam_extrinsics,
            "note": "OpenVINS convention: imu_time = camera_time + timeshift_cam_imu.",
        },
        "projection_note": (
            "3DMotion stores the physical camera model as Mei/omni when xi is present. "
            "The local OpenVINS checkout includes a fixed Mei/omni-radtan camera model, so "
            "kalibr_imucam_chain.yaml exports camera_model=omni with intrinsics=[xi, fu, fv, cu, cv]. "
            "Online camera intrinsic calibration is disabled for omni because the filter state is still 8D."
        ),
        "warning": (
            "T_imu_cam was loaded from Kalibr where available; missing cameras fall back to measured T_H_IH/T_H_C."
            if kalibr_imucam
            else (
                "T_imu_cam is derived from headset-frame T_H_C and T_H_IH. This is exact only if both transforms "
                "are calibrated in the same H frame; if T_H_IH is only measured from axes/translation, validate it "
                "with a camera-IMU Kalibr run."
            )
        ),
    }
    (output_dir / "openvins_config_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def _write_estimator_config(
    path: Path,
    template_config_dir: Path | None,
    *,
    camera_count: int,
    init_imu_thresh: float,
    init_max_disparity: float,
    init_dyn_use: bool,
    calib_cam_timeoffset: bool,
    calib_cam_extrinsics: bool,
) -> None:
    max_cameras = max(1, int(camera_count))
    init_imu_thresh_text = f"{float(init_imu_thresh):g}"
    init_max_disparity_text = f"{float(init_max_disparity):g}"
    init_dyn_use_text = "true" if init_dyn_use else "false"
    calib_cam_timeoffset_text = "true" if calib_cam_timeoffset else "false"
    calib_cam_extrinsics_text = "true" if calib_cam_extrinsics else "false"
    if template_config_dir is not None:
        template = template_config_dir / "estimator_config.yaml"
        if template.exists():
            shutil.copyfile(template, path)
            text = path.read_text(encoding="utf-8")
            text = text.replace("use_stereo: true", "use_stereo: false")
            text = _replace_scalar_yaml_line(text, "max_cameras", str(max_cameras))
            text = _replace_scalar_yaml_line(text, "calib_cam_extrinsics", calib_cam_extrinsics_text)
            text = _replace_scalar_yaml_line(text, "calib_cam_intrinsics", "false")
            text = _replace_scalar_yaml_line(text, "calib_cam_timeoffset", calib_cam_timeoffset_text)
            text = _replace_scalar_yaml_line(text, "init_imu_thresh", init_imu_thresh_text)
            text = _replace_scalar_yaml_line(text, "init_max_disparity", init_max_disparity_text)
            text = _replace_scalar_yaml_line(text, "init_dyn_use", init_dyn_use_text)
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

calib_cam_extrinsics: {calib_cam_extrinsics}
calib_cam_intrinsics: false
calib_cam_timeoffset: {calib_cam_timeoffset}
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
init_imu_thresh: {init_imu_thresh}
init_max_disparity: {init_max_disparity}
init_max_features: 50
init_dyn_use: {init_dyn_use}
init_dyn_mle_opt_calib: false
init_dyn_mle_max_iter: 50
init_dyn_mle_max_time: 0.05
init_dyn_mle_max_threads: 6
init_dyn_num_pose: 6
init_dyn_min_deg: 10.0
init_dyn_inflation_ori: 10
init_dyn_inflation_vel: 100
init_dyn_inflation_bg: 10
init_dyn_inflation_ba: 100
init_dyn_min_rec_cond: 1e-12
init_dyn_bias_g: [0.0, 0.0, 0.0]
init_dyn_bias_a: [0.0, 0.0, 0.0]

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
""".format(
            max_cameras=max_cameras,
            init_imu_thresh=init_imu_thresh_text,
            init_max_disparity=init_max_disparity_text,
            init_dyn_use=init_dyn_use_text,
            calib_cam_timeoffset=calib_cam_timeoffset_text,
            calib_cam_extrinsics=calib_cam_extrinsics_text,
        ),
        encoding="utf-8",
    )


def _write_imu_chain(path: Path, head_imu_calibration: dict | None = None) -> None:
    head_imu_calibration = head_imu_calibration or {}
    data = {
        "imu0": {
            "T_i_b": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            "accelerometer_noise_density": float(head_imu_calibration.get("accelerometer_noise_density", 0.02)),
            "accelerometer_random_walk": float(head_imu_calibration.get("accelerometer_random_walk", 0.002)),
            "gyroscope_noise_density": float(head_imu_calibration.get("gyroscope_noise_density", 0.0017)),
            "gyroscope_random_walk": float(head_imu_calibration.get("gyroscope_random_walk", 0.0002)),
            "accel_bias_mps2": head_imu_calibration.get("accel_bias_mps2", [0.0, 0.0, 0.0]),
            "gyro_bias_radps": head_imu_calibration.get("gyro_bias_radps", [0.0, 0.0, 0.0]),
            "source_imu_id": "head_imu",
            "source_calibration": head_imu_calibration.get("source"),
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


def _load_imu_calibration(path: Path | None) -> dict:
    if path is None or not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    imus = data.get("imus") or {}
    if not isinstance(imus, dict):
        return {}
    return imus


def _load_t_h_ih(path: Path | None) -> np.ndarray:
    if path is None or not path.exists():
        return np.eye(4, dtype=np.float64)
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    node = ((data.get("transforms") or {}).get("T_H_IH") or {})
    rotation = node.get("rotation_matrix")
    translation = node.get("translation_m")
    if rotation is None or translation is None:
        return np.eye(4, dtype=np.float64)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


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


def _write_imucam_chain(
    path: Path,
    calibrations,
    *,
    t_h_ih: np.ndarray | None = None,
    kalibr_imucam: dict[int, dict] | None = None,
    timeshift_cam_imu: float | None = None,
) -> None:
    t_h_ih = np.asarray(t_h_ih if t_h_ih is not None else np.eye(4), dtype=np.float64)
    t_ih_h = np.linalg.inv(t_h_ih)
    kalibr_imucam = kalibr_imucam or {}
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
        uses_omni = calibration.uses_omni_projection and calibration.xi is not None
        camera_model = "omni" if uses_omni else "pinhole"
        openvins_intrinsics = (
            [
                float(calibration.xi),
                float(intrinsics[0, 0]),
                float(intrinsics[1, 1]),
                float(intrinsics[0, 2]),
                float(intrinsics[1, 2]),
            ]
            if uses_omni
            else [
                float(intrinsics[0, 0]),
                float(intrinsics[1, 1]),
                float(intrinsics[0, 2]),
                float(intrinsics[1, 2]),
            ]
        )
        t_h_c = calibration.T_H_C.as_matrix()
        override = kalibr_imucam.get(index)
        t_ih_c = np.asarray(override["T_imu_cam"], dtype=np.float64) if override else t_ih_h @ t_h_c
        cam_timeshift = (
            float(timeshift_cam_imu)
            if timeshift_cam_imu is not None
            else float(override["timeshift_cam_imu"])
            if override
            else 0.0
        )
        data[f"cam{index}"] = {
            "T_imu_cam": t_ih_c.tolist(),
            "cam_overlaps": [i for i in range(len(calibrations)) if i != index],
            "timeshift_cam_imu": cam_timeshift,
            "source_T_imu_cam": override["source_T_imu_cam"] if override else "head_frame_T_H_C_and_T_H_IH",
            "source_camera_id": calibration.camera_id,
            "source_camera_model": calibration.camera_model,
            "source_projection_model": calibration.projection_model,
            "source_xi": calibration.xi,
            "camera_model": camera_model,
            "distortion_coeffs": [float(v) for v in distortion[:4]],
            "distortion_model": distortion_model,
            "intrinsics": openvins_intrinsics,
            "resolution": [int(width), int(height)],
            "rostopic": f"/cam{index}/image_raw",
        }
    _write_opencv_yaml(path, data)


def _openvins_distortion_model(value: str) -> str:
    if value.lower() in {"equidistant", "opencv_fisheye", "fisheye"}:
        return "equidistant"
    return "radtan"


def _load_kalibr_imucam_overrides(path: Path | None, *, camera_count: int) -> dict[int, dict]:
    if path is None:
        return {}
    data = _load_yaml_compat(path)
    overrides: dict[int, dict] = {}
    for index in range(camera_count):
        node = data.get(f"cam{index}") or {}
        if not isinstance(node, dict):
            continue
        matrix = None
        source = None
        if node.get("T_imu_cam") is not None:
            matrix = _as_matrix4(node["T_imu_cam"])
            source = "kalibr_T_imu_cam"
        elif node.get("T_cam_imu") is not None:
            matrix = np.linalg.inv(_as_matrix4(node["T_cam_imu"]))
            source = "kalibr_T_cam_imu_inverted"
        if matrix is None:
            continue
        overrides[index] = {
            "T_imu_cam": matrix,
            "timeshift_cam_imu": float(node.get("timeshift_cam_imu", 0.0) or 0.0),
            "source_T_imu_cam": source,
        }
    return overrides


def _load_yaml_compat(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    lines = [line for line in text.splitlines() if not line.lstrip().startswith("%YAML")]
    text = "\n".join(lines).replace("!!opencv-matrix", "")
    return yaml.safe_load(text) or {}


def _as_matrix4(value) -> np.ndarray:
    if isinstance(value, dict) and "data" in value:
        rows = int(value.get("rows", 4))
        cols = int(value.get("cols", 4))
        matrix = np.asarray(value["data"], dtype=np.float64).reshape(rows, cols)
    else:
        matrix = np.asarray(value, dtype=np.float64).reshape(4, 4)
    if matrix.shape != (4, 4):
        raise ValueError(f"Expected 4x4 camera-IMU transform, got {matrix.shape}")
    return matrix


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
    parser.add_argument("--imu-calibration", default="configs/imu_calibration.yaml", help="IMU noise/bias calibration YAML.")
    parser.add_argument("--frames", default="configs/frames.yaml", help="Frame extrinsics YAML containing T_H_IH.")
    parser.add_argument("--kalibr-imucam", help="Optional Kalibr camchain-imucam YAML with T_cam_imu/T_imu_cam and timeshift_cam_imu.")
    parser.add_argument("--init-max-disparity", type=float, default=10.0, help="OpenVINS static initializer max image disparity. Default: 10.")
    parser.add_argument("--init-dyn-use", action="store_true", help="Use OpenVINS dynamic initializer when the static init window is moving.")
    parser.add_argument("--timeshift-cam-imu", type=float, help="Static camera-to-IMU offset in seconds. OpenVINS uses imu_time = camera_time + offset.")
    parser.add_argument("--calib-cam-timeoffset", action="store_true", help="Let OpenVINS estimate camera-IMU time offset online.")
    parser.add_argument("--calib-cam-extrinsics", action="store_true", help="Let OpenVINS estimate camera-IMU extrinsics online.")
    args = parser.parse_args()

    summary = generate_openvins_config(
        cameras_path=Path(args.cameras),
        output_dir=Path(args.output_dir),
        camera_ids=args.camera_ids,
        template_config_dir=Path(args.template_config_dir) if args.template_config_dir else None,
        imu_calibration_path=Path(args.imu_calibration) if args.imu_calibration else None,
        frames_path=Path(args.frames) if args.frames else None,
        kalibr_imucam_path=Path(args.kalibr_imucam) if args.kalibr_imucam else None,
        init_max_disparity=args.init_max_disparity,
        init_dyn_use=args.init_dyn_use,
        timeshift_cam_imu=args.timeshift_cam_imu,
        calib_cam_timeoffset=args.calib_cam_timeoffset,
        calib_cam_extrinsics=args.calib_cam_extrinsics,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
