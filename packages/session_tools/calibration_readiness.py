from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass
class CalibrationCheck:
    name: str
    ok: bool
    message: str
    details: dict[str, Any]


def check_calibration_readiness(
    *,
    cameras_path: Path = Path("configs/cameras.yaml"),
    frames_path: Path = Path("configs/frames.yaml"),
    imu_calibration_path: Path = Path("configs/imu_calibration.yaml"),
    bracelet_path: Path = Path("configs/bracelet.yaml"),
) -> dict[str, Any]:
    checks = [
        _check_cameras(cameras_path),
        _check_frames(frames_path),
        _check_imu_calibration(imu_calibration_path),
        _check_bracelet(bracelet_path),
    ]
    ready_for_fourcam_wrist_visual = _check_ok(checks, "camera_calibration") and _check_ok(checks, "bracelet_geometry")
    ready_for_head_vio = _check_ok(checks, "camera_calibration") and _check_ok(checks, "imu_calibration")
    ready_for_wrist_fusion = ready_for_fourcam_wrist_visual and _check_ok(checks, "frames")
    return {
        "ready_for_fourcam_wrist_visual": ready_for_fourcam_wrist_visual,
        "ready_for_head_vio": ready_for_head_vio,
        "ready_for_wrist_fusion": ready_for_wrist_fusion,
        "checks": [asdict(check) for check in checks],
        "next_steps": _next_steps(checks),
    }


def _check_cameras(path: Path) -> CalibrationCheck:
    data = _load_yaml(path)
    cameras = data.get("cameras") or {}
    missing_intrinsics = []
    missing_extrinsics = []
    for camera_id, camera in cameras.items():
        if not camera.get("intrinsics") or not camera.get("distortion"):
            missing_intrinsics.append(camera_id)
        if camera.get("T_H_C") is None:
            missing_extrinsics.append(camera_id)
    ok = bool(cameras) and not missing_intrinsics and not missing_extrinsics
    return CalibrationCheck(
        name="camera_calibration",
        ok=ok,
        message=(
            "All cameras have intrinsics and T_H_C."
            if ok
            else "Camera calibration is incomplete; fill intrinsics/distortion and T_H_C for every camera."
        ),
        details={
            "path": str(path),
            "camera_count": len(cameras),
            "missing_intrinsics": missing_intrinsics,
            "missing_T_H_C": missing_extrinsics,
        },
    )


def _check_frames(path: Path) -> CalibrationCheck:
    data = _load_yaml(path)
    transforms = data.get("transforms") or {}
    t_b_ib = transforms.get("T_B_IB") or {}
    t_h_ih = transforms.get("T_H_IH") or {}
    wrist_ok = t_b_ib.get("rotation_matrix") is not None and t_b_ib.get("translation_m") is not None
    head_ok = (
        t_h_ih.get("status") not in {None, "pending"}
        and t_h_ih.get("rotation_matrix") is not None
        and t_h_ih.get("translation_m") is not None
    )
    ok = wrist_ok and head_ok
    return CalibrationCheck(
        name="frames",
        ok=ok,
        message=(
            "Head and wrist IMU frame extrinsics are present."
            if ok
            else "Frame calibration is incomplete; fill T_B_IB and T_H_IH."
        ),
        details={
            "path": str(path),
            "T_B_IB_present": wrist_ok,
            "T_H_IH_present": head_ok,
            "note": "OpenVINS config generation uses T_IH_C = inverse(T_H_IH) * T_H_C.",
        },
    )


def _check_imu_calibration(path: Path) -> CalibrationCheck:
    data = _load_yaml(path)
    imus = data.get("imus") or {}
    missing = []
    required = ["accelerometer_noise_density", "accelerometer_random_walk", "gyroscope_noise_density", "gyroscope_random_walk"]
    for imu_id in ["head_imu", "wrist_imu"]:
        imu = imus.get(imu_id) or {}
        if any(key not in imu for key in required):
            missing.append(imu_id)
    ok = not missing
    return CalibrationCheck(
        name="imu_calibration",
        ok=ok,
        message="Head and wrist IMU noise estimates are present." if ok else "IMU noise calibration is incomplete.",
        details={"path": str(path), "missing_or_incomplete": missing},
    )


def _check_bracelet(path: Path) -> CalibrationCheck:
    data = _load_yaml(path)
    tag_order = data.get("tag_order") or []
    explicit = data.get("tag_to_wrist_transforms") or {}
    ok = bool(data.get("tag_size_m")) and (bool(explicit) or bool(tag_order))
    return CalibrationCheck(
        name="bracelet_geometry",
        ok=ok,
        message="Bracelet tag geometry is available." if ok else "Bracelet geometry needs tag size and tag transforms/order.",
        details={
            "path": str(path),
            "tag_size_m": data.get("tag_size_m"),
            "tag_order_count": len(tag_order),
            "explicit_transform_count": len(explicit),
            "uses_fallback_geometry": not bool(explicit),
        },
    )


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _check_ok(checks: list[CalibrationCheck], name: str) -> bool:
    return next((check.ok for check in checks if check.name == name), False)


def _next_steps(checks: list[CalibrationCheck]) -> list[str]:
    steps = []
    for check in checks:
        if check.ok:
            continue
        if check.name == "camera_calibration":
            steps.append("Run VimasCalibration camera intrinsics and multi-camera extrinsics, then manually fill configs/cameras.yaml T_H_C.")
        elif check.name == "imu_calibration":
            steps.append("Run VimasCalibration static IMU calibration and manually copy noise/bias into configs/imu_calibration.yaml.")
        elif check.name == "frames":
            steps.append("Fill configs/frames.yaml with measured T_B_IB and final T_H_IH when head IMU extrinsics are available.")
        elif check.name == "bracelet_geometry":
            steps.append("Fill configs/bracelet.yaml tag size, tag order, and calibrated tag_to_wrist transforms if available.")
    return steps


def main() -> None:
    parser = argparse.ArgumentParser(description="Check whether local 3DMotion calibration configs are ready.")
    parser.add_argument("--cameras", default="configs/cameras.yaml")
    parser.add_argument("--frames", default="configs/frames.yaml")
    parser.add_argument("--imu-calibration", default="configs/imu_calibration.yaml")
    parser.add_argument("--bracelet", default="configs/bracelet.yaml")
    args = parser.parse_args()
    result = check_calibration_readiness(
        cameras_path=Path(args.cameras),
        frames_path=Path(args.frames),
        imu_calibration_path=Path(args.imu_calibration),
        bracelet_path=Path(args.bracelet),
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
