from __future__ import annotations

import argparse
import json
import math
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

from packages.head_vio_bridge.openvins_config import DEFAULT_HEAD_CAMERA_IDS, generate_openvins_config
from packages.head_vio_bridge.openvins_session import prepare_openvins_session


@dataclass
class P3Check:
    name: str
    ok: bool
    message: str
    details: dict


def check_head_vio_readiness(
    session_dir: Path,
    *,
    camera_id: str = "C0",
    imu_slot: str = "head_imu",
    min_duration_s: float = 5.0,
    gravity_min_mps2: float = 7.5,
    gravity_max_mps2: float = 12.0,
    min_image_imu_overlap_s: float = 1.0,
    min_accel_std_mps2: float = 0.5,
    min_imu_rate_hz: float = 180.0,
    max_imu_dt_ms: float = 50.0,
    max_imu_p99_dt_ms: float = 20.0,
    imu_time_mode: str = "raw",
    imu_rate_hz: float = 200.0,
    start_offset_s: float = 0.0,
    max_duration_s: float | None = None,
) -> dict:
    """Check whether one camera stream in a session is ready for head_imu VIO."""

    session_dir = session_dir.resolve()
    cameras_dir = _resolve_cameras_dir(session_dir)
    frames_path = cameras_dir / "frames.jsonl"
    imu_path = session_dir / "imus" / f"{imu_slot}.jsonl"

    frame_records = _load_camera_records(frames_path, cameras_dir, camera_id)
    imu_records = _load_imu_records(imu_path)
    window_summary = {
        "enabled": False,
        "start_offset_s": start_offset_s,
        "max_duration_s": max_duration_s,
    }
    if start_offset_s > 0 or max_duration_s is not None:
        frame_records, imu_records, window_summary = _filter_readiness_window(
            frame_records=frame_records,
            imu_records=imu_records,
            start_offset_s=start_offset_s,
            max_duration_s=max_duration_s,
        )
    timing_imu_records = _records_for_imu_timing_check(imu_records, imu_time_mode=imu_time_mode, imu_rate_hz=imu_rate_hz)

    checks = [
        _check_camera_stream(frame_records, camera_id=camera_id, min_duration_s=min_duration_s),
        _check_imu_stream(
            timing_imu_records,
            imu_slot=imu_slot,
            min_duration_s=min_duration_s,
            min_rate_hz=min_imu_rate_hz,
            max_dt_ms=max_imu_dt_ms,
            max_p99_dt_ms=max_imu_p99_dt_ms,
        ),
        _check_image_imu_overlap(frame_records, timing_imu_records, min_overlap_s=min_image_imu_overlap_s),
        _check_accel_gravity_range(
            imu_records,
            gravity_min_mps2=gravity_min_mps2,
            gravity_max_mps2=gravity_max_mps2,
        ),
        _check_imu_excitation(
            imu_records,
            min_accel_std_mps2=min_accel_std_mps2,
        ),
    ]

    report = {
        "session_dir": str(session_dir),
        "ready_for_p3a": all(check.ok for check in checks),
        "p3a_frame_convention": {
            "openvins_body_frame": imu_slot,
            "prototype_head_frame": "H := I",
            "prototype_output": "T_W_H := T_W_I",
            "final_head_frame_note": "After IMU-to-head extrinsic calibration, use T_W_H = T_W_I * T_I_H.",
        },
        "hardware_requirements": [
            "head_imu must be rigidly fixed to the headset/camera rig.",
            "The IMU axes should be marked before collecting P3 data.",
            "Do not use wrist_imu for P3a; it belongs to the wrist motion pipeline.",
        ],
        "thresholds": {
            "camera_id": camera_id,
            "imu_slot": imu_slot,
            "min_duration_s": min_duration_s,
            "gravity_min_mps2": gravity_min_mps2,
            "gravity_max_mps2": gravity_max_mps2,
            "min_image_imu_overlap_s": min_image_imu_overlap_s,
            "min_accel_std_mps2": min_accel_std_mps2,
            "min_imu_rate_hz": min_imu_rate_hz,
            "max_imu_dt_ms": max_imu_dt_ms,
            "max_imu_p99_dt_ms": max_imu_p99_dt_ms,
            "imu_time_mode": imu_time_mode,
            "imu_rate_hz": imu_rate_hz,
        },
        "readiness_window": window_summary,
        "checks": [asdict(check) for check in checks],
        "next_steps": _next_steps(checks),
    }
    return report


def prepare_p3_head_vio(
    session_dir: Path,
    *,
    output_dir: Path | None = None,
    config_dir: Path | None = None,
    cameras_path: Path = Path("configs/cameras.yaml"),
    template_config_dir: Path | None = Path("open_vins/config/euroc_mav"),
    kalibr_imucam_path: Path | None = None,
    camera_id: str | None = None,
    camera_ids: list[str] | None = None,
    imu_slot: str = "head_imu",
    fail_on_not_ready: bool = True,
    init_imu_thresh: float = 0.5,
    init_max_disparity: float = 10.0,
    init_dyn_use: bool = False,
    timeshift_cam_imu: float | None = None,
    calib_cam_timeoffset: bool = False,
    calib_cam_extrinsics: bool = False,
    imu_time_mode: str = "raw",
    imu_rate_hz: float = 200.0,
    start_offset_s: float = 0.0,
    max_duration_s: float | None = None,
    export_window: bool = False,
    export_imu_preroll_s: float = 0.0,
) -> dict:
    """Prepare one session for the OpenVINS head camera rig + head_imu path."""

    session_dir = session_dir.resolve()
    selected_camera_ids = _resolve_selected_camera_ids(camera_id=camera_id, camera_ids=camera_ids)
    output_dir = output_dir or Path("data/processed") / session_dir.name / "openvins_head"
    config_dir = config_dir or output_dir / "config"

    readiness = _check_head_vio_readiness_for_cameras(
        session_dir,
        camera_ids=selected_camera_ids,
        imu_slot=imu_slot,
        min_accel_std_mps2=init_imu_thresh,
        imu_time_mode=imu_time_mode,
        imu_rate_hz=imu_rate_hz,
        start_offset_s=start_offset_s,
        max_duration_s=max_duration_s,
    )
    if fail_on_not_ready and not readiness["ready_for_p3a"]:
        return {
            "session_dir": str(session_dir),
            "ready_for_p3a": False,
            "camera_ids": selected_camera_ids,
            "readiness": readiness,
            "steps": {
                "skipped": "Head VIO readiness failed. Fix the session before exporting OpenVINS inputs.",
            },
        }

    openvins_session = prepare_openvins_session(
        session_dir=session_dir,
        output_dir=output_dir,
        camera_ids=selected_camera_ids,
        imu_slot=imu_slot,
        imu_time_mode=imu_time_mode,
        imu_rate_hz=imu_rate_hz,
        export_window=export_window,
        export_start_offset_s=start_offset_s,
        export_max_duration_s=max_duration_s,
        export_imu_preroll_s=export_imu_preroll_s,
    )
    openvins_config = generate_openvins_config(
        cameras_path=cameras_path,
        output_dir=config_dir,
        camera_ids=selected_camera_ids,
        template_config_dir=template_config_dir,
        kalibr_imucam_path=kalibr_imucam_path,
        init_imu_thresh=init_imu_thresh,
        init_max_disparity=init_max_disparity,
        init_dyn_use=init_dyn_use,
        timeshift_cam_imu=timeshift_cam_imu,
        calib_cam_timeoffset=calib_cam_timeoffset,
        calib_cam_extrinsics=calib_cam_extrinsics,
    )

    summary = {
        "session_dir": str(session_dir),
        "ready_for_p3a": readiness["ready_for_p3a"],
        "camera_ids": selected_camera_ids,
        "readiness": readiness,
        "steps": {
            "openvins_session": openvins_session,
            "openvins_config": openvins_config,
            "rosbag2_next": {
                "command": (
                    "python scripts/write_openvins_rosbag2.py "
                    f"--prepared-dir {output_dir} "
                    f"--bag-dir {output_dir / 'rosbag2'} "
                    f"--frame-id {imu_slot}"
                )
            },
        },
        "warning": (
            "Head VIO currently treats headset frame H as the head IMU frame I_H unless calibrated "
            "IMU-camera and IMU-head extrinsics are provided."
        ),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "p3_head_vio_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def _resolve_selected_camera_ids(*, camera_id: str | None, camera_ids: list[str] | None) -> list[str]:
    if camera_ids:
        return [str(value) for value in camera_ids]
    if camera_id:
        return [str(camera_id)]
    return list(DEFAULT_HEAD_CAMERA_IDS)


def _check_head_vio_readiness_for_cameras(
    session_dir: Path,
    *,
    camera_ids: list[str],
    imu_slot: str,
    min_accel_std_mps2: float,
    imu_time_mode: str,
    imu_rate_hz: float,
    start_offset_s: float,
    max_duration_s: float | None,
) -> dict:
    reports = [
        check_head_vio_readiness(
            session_dir,
            camera_id=camera_id,
            imu_slot=imu_slot,
            min_accel_std_mps2=min_accel_std_mps2,
            imu_time_mode=imu_time_mode,
            imu_rate_hz=imu_rate_hz,
            start_offset_s=start_offset_s,
            max_duration_s=max_duration_s,
        )
        for camera_id in camera_ids
    ]
    return {
        "session_dir": str(session_dir.resolve()),
        "ready_for_p3a": all(report["ready_for_p3a"] for report in reports),
        "camera_ids": camera_ids,
        "camera_reports": reports,
        "p3a_frame_convention": reports[0]["p3a_frame_convention"] if reports else {},
        "hardware_requirements": reports[0]["hardware_requirements"] if reports else [],
        "next_steps": [
            step
            for report in reports
            for step in report.get("next_steps", [])
            if step
        ],
    }


def _resolve_cameras_dir(session_dir: Path) -> Path:
    if (session_dir / "cameras").is_dir():
        return session_dir / "cameras"
    return session_dir


def _load_camera_records(frames_path: Path, cameras_dir: Path, camera_id: str) -> list[dict]:
    if not frames_path.exists():
        return []
    records = []
    for record in _read_jsonl(frames_path):
        if record.get("camera_id") != camera_id:
            continue
        image_path = cameras_dir / str(record.get("image_path", ""))
        copied = dict(record)
        copied["_image_exists"] = image_path.exists()
        records.append(copied)
    records.sort(key=lambda item: int(item.get("timestamp_monotonic_ns", 0)))
    return records


def _load_imu_records(path: Path) -> list[dict]:
    if not path.exists():
        return []
    records = []
    for record in _read_jsonl(path):
        if "accel_mps2" in record and "gyro_radps" in record and "timestamp_monotonic_ns" in record:
            records.append(record)
    records.sort(key=lambda item: int(item["timestamp_monotonic_ns"]))
    return records


def _filter_readiness_window(
    *,
    frame_records: list[dict],
    imu_records: list[dict],
    start_offset_s: float,
    max_duration_s: float | None,
) -> tuple[list[dict], list[dict], dict]:
    if start_offset_s < 0:
        raise ValueError("--start-offset-s must be >= 0")
    if max_duration_s is not None and max_duration_s <= 0:
        raise ValueError("--max-duration-s must be > 0")
    if not frame_records:
        return frame_records, imu_records, {
            "enabled": True,
            "start_offset_s": start_offset_s,
            "max_duration_s": max_duration_s,
            "input_counts": {"images": len(frame_records), "imu_samples": len(imu_records)},
            "output_counts": {"images": len(frame_records), "imu_samples": len(imu_records)},
        }

    first_image_ns = min(int(record["timestamp_monotonic_ns"]) for record in frame_records)
    start_ns = first_image_ns + int(round(start_offset_s * 1_000_000_000))
    end_ns = None if max_duration_s is None else start_ns + int(round(max_duration_s * 1_000_000_000))

    def in_window(record: dict) -> bool:
        timestamp_ns = int(record["timestamp_monotonic_ns"])
        return timestamp_ns >= start_ns and (end_ns is None or timestamp_ns <= end_ns)

    filtered_frames = [record for record in frame_records if in_window(record)]
    filtered_imu = [record for record in imu_records if in_window(record)]
    return filtered_frames, filtered_imu, {
        "enabled": True,
        "start_offset_s": start_offset_s,
        "max_duration_s": max_duration_s,
        "start_monotonic_ns": start_ns,
        "end_monotonic_ns": end_ns,
        "input_counts": {"images": len(frame_records), "imu_samples": len(imu_records)},
        "output_counts": {"images": len(filtered_frames), "imu_samples": len(filtered_imu)},
    }


def _records_for_imu_timing_check(records: list[dict], *, imu_time_mode: str, imu_rate_hz: float) -> list[dict]:
    if imu_time_mode == "raw" or imu_time_mode == "resample-rate":
        return records
    if imu_time_mode != "reconstruct-rate":
        raise ValueError("--imu-time-mode must be one of: raw, resample-rate, reconstruct-rate")
    if imu_rate_hz <= 0:
        raise ValueError("--imu-rate-hz must be > 0")
    if len(records) < 2:
        return records
    period_ns = int(round(1_000_000_000 / imu_rate_hz))
    start_ns = int(records[0]["timestamp_monotonic_ns"])
    reconstructed = []
    for index, record in enumerate(records):
        copied = dict(record)
        copied["timestamp_monotonic_ns"] = start_ns + index * period_ns
        reconstructed.append(copied)
    return reconstructed


def _read_jsonl(path: Path) -> Iterable[dict]:
    with path.open("r", encoding="utf-8") as f:
        for line_number, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSONL in {path}:{line_number}: {exc}") from exc


def _check_camera_stream(records: list[dict], *, camera_id: str, min_duration_s: float) -> P3Check:
    timestamps = _timestamps(records)
    duration_s = _duration_s(timestamps)
    missing_images = sum(1 for record in records if not record.get("_image_exists"))
    monotonic = _is_strictly_monotonic(timestamps)
    ok = bool(records) and missing_images == 0 and monotonic and duration_s >= min_duration_s
    return P3Check(
        name="camera_stream",
        ok=ok,
        message=f"{camera_id}: frames={len(records)}, duration_s={duration_s:.3f}, missing_images={missing_images}",
        details={
            "camera_id": camera_id,
            "frame_count": len(records),
            "duration_s": duration_s,
            "missing_images": missing_images,
            "timestamp_monotonic": monotonic,
        },
    )


def _check_imu_stream(
    records: list[dict],
    *,
    imu_slot: str,
    min_duration_s: float,
    min_rate_hz: float,
    max_dt_ms: float,
    max_p99_dt_ms: float,
) -> P3Check:
    timestamps = _timestamps(records)
    duration_s = _duration_s(timestamps)
    monotonic = _is_strictly_monotonic(timestamps)
    dts_ms = [(curr - prev) / 1e6 for prev, curr in zip(timestamps, timestamps[1:]) if curr > prev]
    rate_hz = (len(timestamps) - 1) / duration_s if len(timestamps) >= 2 and duration_s > 0 else 0.0
    p99_dt_ms = _percentile(dts_ms, 99) if dts_ms else None
    max_observed_dt_ms = max(dts_ms) if dts_ms else None
    gaps_over_20ms = sum(dt > 20.0 for dt in dts_ms)
    ok = (
        bool(records)
        and monotonic
        and duration_s >= min_duration_s
        and rate_hz >= min_rate_hz
        and (max_observed_dt_ms is not None and max_observed_dt_ms <= max_dt_ms)
        and (p99_dt_ms is not None and p99_dt_ms <= max_p99_dt_ms)
    )
    return P3Check(
        name="imu_stream",
        ok=ok,
        message=(
            f"{imu_slot}: samples={len(records)}, duration_s={duration_s:.3f}, "
            f"rate_hz={rate_hz:.1f}, p99_dt_ms={p99_dt_ms}, max_dt_ms={max_observed_dt_ms}, "
            f"gaps_over_20ms={gaps_over_20ms}, monotonic={monotonic}"
        ),
        details={
            "imu_slot": imu_slot,
            "sample_count": len(records),
            "duration_s": duration_s,
            "rate_hz": rate_hz,
            "dt_ms_p99": p99_dt_ms,
            "dt_ms_max": max_observed_dt_ms,
            "gaps_over_20ms": gaps_over_20ms,
            "min_rate_hz": min_rate_hz,
            "max_dt_ms": max_dt_ms,
            "max_p99_dt_ms": max_p99_dt_ms,
            "timestamp_monotonic": monotonic,
        },
    )


def _check_image_imu_overlap(frame_records: list[dict], imu_records: list[dict], *, min_overlap_s: float) -> P3Check:
    image_range = _time_range(_timestamps(frame_records))
    imu_range = _time_range(_timestamps(imu_records))
    if image_range is None or imu_range is None:
        overlap_s = 0.0
    else:
        overlap_ns = max(0, min(image_range[1], imu_range[1]) - max(image_range[0], imu_range[0]))
        overlap_s = overlap_ns / 1e9
    ok = overlap_s >= min_overlap_s
    return P3Check(
        name="image_imu_time_overlap",
        ok=ok,
        message=f"overlap_s={overlap_s:.3f}",
        details={
            "overlap_s": overlap_s,
            "min_overlap_s": min_overlap_s,
            "image_range_monotonic_ns": list(image_range) if image_range else None,
            "imu_range_monotonic_ns": list(imu_range) if imu_range else None,
        },
    )


def _check_accel_gravity_range(
    records: list[dict],
    *,
    gravity_min_mps2: float,
    gravity_max_mps2: float,
) -> P3Check:
    magnitudes = [_norm3(record["accel_mps2"]) for record in records if len(record.get("accel_mps2", [])) == 3]
    mean = statistics.fmean(magnitudes) if magnitudes else None
    p05 = _percentile(magnitudes, 5)
    p95 = _percentile(magnitudes, 95)
    ok = mean is not None and gravity_min_mps2 <= mean <= gravity_max_mps2
    return P3Check(
        name="accel_gravity_magnitude",
        ok=ok,
        message=f"mean_accel_norm_mps2={_fmt(mean)}, p05={_fmt(p05)}, p95={_fmt(p95)}",
        details={
            "count": len(magnitudes),
            "mean_mps2": mean,
            "p05_mps2": p05,
            "p95_mps2": p95,
            "gravity_min_mps2": gravity_min_mps2,
            "gravity_max_mps2": gravity_max_mps2,
        },
    )


def _check_imu_excitation(records: list[dict], *, min_accel_std_mps2: float) -> P3Check:
    vectors = [record["accel_mps2"] for record in records if len(record.get("accel_mps2", [])) == 3]
    gyro_norms = [_norm3(record["gyro_radps"]) for record in records if len(record.get("gyro_radps", [])) == 3]
    accel_std = _vector_std(vectors)
    max_gyro = max(gyro_norms) if gyro_norms else None
    ok = accel_std is not None and accel_std >= min_accel_std_mps2
    return P3Check(
        name="imu_excitation",
        ok=ok,
        message=f"accel_vector_std_mps2={_fmt(accel_std)}, max_gyro_norm_radps={_fmt(max_gyro)}",
        details={
            "accel_vector_std_mps2": accel_std,
            "min_accel_std_mps2": min_accel_std_mps2,
            "max_gyro_norm_radps": max_gyro,
            "note": "OpenVINS static initialization waits for a quiet window followed by acceleration excitation.",
        },
    )


def _timestamps(records: list[dict]) -> list[int]:
    return [int(record["timestamp_monotonic_ns"]) for record in records if record.get("timestamp_monotonic_ns") is not None]


def _time_range(timestamps: list[int]) -> tuple[int, int] | None:
    if not timestamps:
        return None
    return min(timestamps), max(timestamps)


def _duration_s(timestamps: list[int]) -> float:
    time_range = _time_range(timestamps)
    if time_range is None:
        return 0.0
    return (time_range[1] - time_range[0]) / 1e9


def _is_strictly_monotonic(timestamps: list[int]) -> bool:
    return all(curr > prev for prev, curr in zip(timestamps, timestamps[1:]))


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * percentile / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[int(index)]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _norm3(values: list[float]) -> float:
    return math.sqrt(float(values[0]) ** 2 + float(values[1]) ** 2 + float(values[2]) ** 2)


def _vector_std(vectors: list[list[float]]) -> float | None:
    if len(vectors) < 2:
        return None
    means = [statistics.fmean(float(vector[index]) for vector in vectors) for index in range(3)]
    variance = 0.0
    for vector in vectors:
        variance += sum((float(vector[index]) - means[index]) ** 2 for index in range(3))
    return math.sqrt(variance / (len(vectors) - 1))


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = round((len(ordered) - 1) * percentile / 100.0)
    return ordered[index]


def _fmt(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.3f}"


def _next_steps(checks: list[P3Check]) -> list[str]:
    failed = {check.name for check in checks if not check.ok}
    steps = []
    if "camera_stream" in failed:
        steps.append("C0 image stream is not ready. Recheck dashboard recording and camera image files.")
    if "imu_stream" in failed:
        steps.append("head_imu stream is not ready. Reconnect the rigid-mounted head IMU and record again.")
    if "image_imu_time_overlap" in failed:
        steps.append("Camera and IMU time ranges do not overlap enough for VIO initialization.")
    if "accel_gravity_magnitude" in failed:
        steps.append("Head IMU acceleration magnitude is outside the expected gravity range; check units and sensor orientation.")
    if "imu_excitation" in failed:
        steps.append("Head IMU motion is too static for OpenVINS initialization. Record a new P3 session: keep still for 2-3s, then translate/rotate the rigid headset with visible acceleration.")
    if not steps:
        steps.append("P3a readiness passed. Run ROS-free OpenVINS for daily processing, or generate rosbag2 only for ROS2/RViz debugging.")
    return steps


def readiness_main() -> None:
    parser = argparse.ArgumentParser(description="Check whether a session is ready for P3a OpenVINS head cameras + head_imu.")
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--camera-id", default="C0")
    parser.add_argument("--imu-slot", default="head_imu")
    parser.add_argument("--min-accel-std-mps2", type=float, default=0.5)
    parser.add_argument("--imu-time-mode", choices=["raw", "resample-rate", "reconstruct-rate"], default="raw")
    parser.add_argument("--imu-rate-hz", type=float, default=200.0)
    parser.add_argument("--start-offset-s", type=float, default=0.0, help="Check readiness from this many seconds after the first selected camera frame.")
    parser.add_argument("--max-duration-s", type=float, help="Only check this many seconds after --start-offset-s.")
    parser.add_argument("--output", help="Optional JSON report path.")
    args = parser.parse_args()

    report = check_head_vio_readiness(
        Path(args.session_dir),
        camera_id=args.camera_id,
        imu_slot=args.imu_slot,
        min_accel_std_mps2=args.min_accel_std_mps2,
        imu_time_mode=args.imu_time_mode,
        imu_rate_hz=args.imu_rate_hz,
        start_offset_s=args.start_offset_s,
        max_duration_s=args.max_duration_s,
    )
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(0 if report["ready_for_p3a"] else 1)


def prepare_main() -> None:
    parser = argparse.ArgumentParser(description="Prepare OpenVINS head camera rig + head_imu inputs and config.")
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--output-dir", help="Default: data/processed/<session_name>/openvins_head")
    parser.add_argument("--config-dir", help="Default: <output-dir>/config")
    parser.add_argument("--cameras", default="configs/cameras.yaml")
    parser.add_argument("--template-config-dir", default="open_vins/config/euroc_mav", help="OpenVINS template config directory used for estimator_config.yaml.")
    parser.add_argument("--kalibr-imucam", help="Optional Kalibr camchain-imucam YAML with T_cam_imu/T_imu_cam and timeshift_cam_imu.")
    parser.add_argument(
        "--camera-id",
        action="append",
        dest="camera_ids",
        help="Camera ID to include. Repeat for multiple cameras. Default: C1,C2,C0,C3.",
    )
    parser.add_argument("--imu-slot", default="head_imu")
    parser.add_argument("--allow-not-ready", action="store_true")
    parser.add_argument("--init-imu-thresh", type=float, default=0.5)
    parser.add_argument("--init-max-disparity", type=float, default=10.0)
    parser.add_argument("--init-dyn-use", action="store_true", help="Use OpenVINS dynamic initializer when the static init window is moving.")
    parser.add_argument("--timeshift-cam-imu", type=float, help="Static camera-to-IMU offset in seconds. OpenVINS uses imu_time = camera_time + offset.")
    parser.add_argument("--calib-cam-timeoffset", action="store_true", help="Let OpenVINS estimate camera-IMU time offset online.")
    parser.add_argument("--calib-cam-extrinsics", action="store_true", help="Let OpenVINS estimate camera-IMU extrinsics online.")
    parser.add_argument("--imu-time-mode", choices=["raw", "resample-rate", "reconstruct-rate"], default="raw")
    parser.add_argument("--imu-rate-hz", type=float, default=200.0)
    parser.add_argument("--start-offset-s", type=float, default=0.0, help="Export/check from this many seconds after the first selected camera frame.")
    parser.add_argument("--max-duration-s", type=float, help="Export/check only this many seconds after --start-offset-s.")
    parser.add_argument("--imu-preroll-s", type=float, default=0.0, help="When exporting a window, keep this many seconds of IMU before the first exported image.")
    args = parser.parse_args()

    summary = prepare_p3_head_vio(
        session_dir=Path(args.session_dir),
        output_dir=Path(args.output_dir) if args.output_dir else None,
        config_dir=Path(args.config_dir) if args.config_dir else None,
        cameras_path=Path(args.cameras),
        template_config_dir=Path(args.template_config_dir) if args.template_config_dir else None,
        kalibr_imucam_path=Path(args.kalibr_imucam) if args.kalibr_imucam else None,
        camera_ids=args.camera_ids,
        imu_slot=args.imu_slot,
        fail_on_not_ready=not args.allow_not_ready,
        init_imu_thresh=args.init_imu_thresh,
        init_max_disparity=args.init_max_disparity,
        init_dyn_use=args.init_dyn_use,
        timeshift_cam_imu=args.timeshift_cam_imu,
        calib_cam_timeoffset=args.calib_cam_timeoffset,
        calib_cam_extrinsics=args.calib_cam_extrinsics,
        imu_time_mode=args.imu_time_mode,
        imu_rate_hz=args.imu_rate_hz,
        start_offset_s=args.start_offset_s,
        max_duration_s=args.max_duration_s,
        export_window=args.start_offset_s > 0 or args.max_duration_s is not None,
        export_imu_preroll_s=args.imu_preroll_s,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    raise SystemExit(0 if summary["ready_for_p3a"] or args.allow_not_ready else 1)
