from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass
class CheckResult:
    name: str
    ok: bool
    message: str


def validate_session(session_dir: Path) -> dict:
    session_dir = session_dir.resolve()
    cameras_dir = session_dir / "cameras"
    imus_dir = session_dir / "imus"
    manifest_path = session_dir / "session_manifest.json"
    frames_path = cameras_dir / "frames.jsonl"
    errors_path = cameras_dir / "capture_errors.jsonl"

    checks: list[CheckResult] = []
    checks.append(_exists("session_manifest", manifest_path))
    checks.append(_exists("cameras_dir", cameras_dir))
    checks.append(_exists("imus_dir", imus_dir))

    frame_records = []
    if frames_path.exists() and frames_path.stat().st_size > 0:
        frame_records = list(_read_jsonl(frames_path))
        missing_images = _missing_frame_images(cameras_dir, frame_records)
        checks.append(
            CheckResult(
                "camera_frames",
                not missing_images,
                f"{len(frame_records)} frame records; {len(missing_images)} missing image files",
            )
        )
    else:
        checks.append(CheckResult("camera_frames", False, f"missing or empty: {frames_path}"))

    capture_errors = list(_read_jsonl(errors_path)) if errors_path.exists() else []
    if capture_errors:
        checks.append(CheckResult("camera_capture_errors", False, _summarize_capture_errors(capture_errors)))
    else:
        checks.append(CheckResult("camera_capture_errors", True, "no capture_errors.jsonl entries"))

    imu_counts = {}
    if imus_dir.exists():
        for imu_path in sorted(imus_dir.glob("*.jsonl")):
            count = sum(1 for _ in _read_jsonl(imu_path))
            imu_counts[imu_path.stem] = count
    checks.append(
        CheckResult(
            "imu_logs",
            bool(imu_counts),
            ", ".join(f"{name}={count}" for name, count in imu_counts.items()) if imu_counts else "no IMU JSONL files",
        )
    )

    camera_counts: dict[str, int] = {}
    for record in frame_records:
        camera_id = str(record.get("camera_id", "unknown"))
        camera_counts[camera_id] = camera_counts.get(camera_id, 0) + 1

    camera_frame_check = next(check for check in checks if check.name == "camera_frames")
    ok = camera_frame_check.ok
    summary = {
        "session_dir": str(session_dir),
        "ok_for_camera_replay": ok,
        "has_capture_warnings": bool(capture_errors),
        "camera_frame_count": len(frame_records),
        "camera_counts": camera_counts,
        "imu_counts": imu_counts,
        "checks": [asdict(check) for check in checks],
        "next_steps": _next_steps(frame_records, capture_errors, imu_counts),
    }
    return summary


def _exists(name: str, path: Path) -> CheckResult:
    return CheckResult(name=name, ok=path.exists(), message=str(path))


def _read_jsonl(path: Path):
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def _missing_frame_images(cameras_dir: Path, records: list[dict]) -> list[Path]:
    missing = []
    for record in records:
        image_path = cameras_dir / str(record.get("image_path", ""))
        if not image_path.exists():
            missing.append(image_path)
    return missing


def _summarize_capture_errors(errors: list[dict]) -> str:
    grouped = {}
    for error in errors:
        key = f"{error.get('camera_id', 'unknown')}:{error.get('source', '')}:{error.get('error', '')}"
        grouped[key] = grouped.get(key, 0) + 1
    return "; ".join(f"{key} x{count}" for key, count in sorted(grouped.items()))


def _next_steps(frame_records: list[dict], capture_errors: list[dict], imu_counts: dict[str, int]) -> list[str]:
    steps = []
    if not frame_records and capture_errors:
        steps.append("Camera opened failed or produced no frames. Check preview is stopped, selected format/fps, /dev/video permissions, and whether another process owns the camera.")
    elif not frame_records:
        steps.append("No camera frames found. Confirm the dashboard session path and record duration.")
    else:
        steps.append("Camera frames exist. Run scripts/process_world_anchor_session.py offline for the AprilGrid world-anchor MVP.")

    if frame_records and imu_counts.get("head_imu", 0) > 0:
        steps.append("Camera + head_imu data exists. OpenVINS head VIO remains available via scripts/process_head_vio_session_rosfree.py when needed.")
    elif frame_records:
        steps.append("Camera frames exist but head_imu is missing. OpenVINS prep needs a connected head IMU recording.")

    if imu_counts.get("wrist_imu", 0) > 0:
        steps.append("wrist_imu exists. It can be used later for wrist smoothing; AprilGrid world-anchor visual does not require it.")
    return steps


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate a recorded 3DMotion dashboard session.")
    parser.add_argument("--session-dir", required=True, help="Dashboard session directory, e.g. data/raw/session_YYYYMMDD_HHMMSS.")
    args = parser.parse_args()
    print(json.dumps(validate_session(Path(args.session_dir)), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
