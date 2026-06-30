from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

from packages.apriltag_ring_node.geometry import (
    RigidTransform,
    quat_wxyz_to_rotation_matrix,
    transform_to_dict,
)


def fuse_motion_session(
    *,
    session_dir: Path,
    output_root: Path,
    head_pose_path: Path | None = None,
    wrist_visual_path: Path | None = None,
    wrist_imu_path: Path | None = None,
    output_dir: Path | None = None,
    max_head_dt_ms: float = 50.0,
    max_wrist_imu_dt_ms: float = 20.0,
    use_wrist_imu_orientation: bool = True,
    visual_correction_alpha: float = 0.35,
) -> dict:
    session_dir = session_dir.resolve()
    output_root = output_root.resolve()
    head_pose_path = head_pose_path or _first_existing(
        [
            output_root / "head_pose.jsonl",
            output_root / "openvins_head" / "head_pose.jsonl",
            output_root / "openvins_session" / "head_pose.jsonl",
        ],
        label="head_pose.jsonl",
    )
    wrist_visual_path = wrist_visual_path or _first_existing(
        [output_root / "wrist_visual" / "wrist_visual_pose.jsonl", output_root / "wrist_visual_pose.jsonl"],
        label="wrist_visual_pose.jsonl",
    )
    wrist_imu_path = wrist_imu_path or session_dir / "imus" / "wrist_imu.jsonl"
    output_dir = (output_dir or output_root / "motion").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    head_records = sorted(
        [_pose_record(record, "T_W_H") for record in _read_jsonl(head_pose_path)],
        key=lambda record: record["timestamp_monotonic_ns"],
    )
    wrist_visual_records = sorted(
        [_pose_record(record, "T_H_B") for record in _read_jsonl(wrist_visual_path)],
        key=lambda record: record["timestamp_monotonic_ns"],
    )
    wrist_imu_available = wrist_imu_path.exists()
    wrist_imu_records = list(_read_jsonl(wrist_imu_path)) if wrist_imu_available else []

    if not head_records:
        raise RuntimeError(f"No head pose records found in {head_pose_path}")
    if not wrist_visual_records:
        raise RuntimeError(f"No wrist visual pose records found in {wrist_visual_path}")

    head_times = [record["timestamp_monotonic_ns"] for record in head_records]
    wrist_imu_times = [int(record["timestamp_monotonic_ns"]) for record in wrist_imu_records if record.get("timestamp_monotonic_ns") is not None]
    wrist_imu_records = sorted(
        [record for record in wrist_imu_records if record.get("timestamp_monotonic_ns") is not None],
        key=lambda record: int(record["timestamp_monotonic_ns"]),
    )
    wrist_imu_times = [int(record["timestamp_monotonic_ns"]) for record in wrist_imu_records]

    wrist_fused_path = output_dir / "wrist_fused_pose.jsonl"
    motion_frame_path = output_dir / "motion_frame.jsonl"
    head_dt_values_ms = []
    wrist_imu_dt_values_ms = []
    skipped = {"head_time_gap": 0}
    written = 0
    previous_fused_T_W_B: RigidTransform | None = None
    previous_timestamp_ns: int | None = None
    imu_propagated_frames = 0
    imu_fallback_frames = 0
    alpha = min(max(float(visual_correction_alpha), 0.0), 1.0)

    with wrist_fused_path.open("w", encoding="utf-8") as wrist_file, motion_frame_path.open("w", encoding="utf-8") as motion_file:
        for wrist_visual in wrist_visual_records:
            head_record, head_dt_ns = _nearest_by_time(head_records, head_times, wrist_visual["timestamp_monotonic_ns"])
            if head_record is None or abs(head_dt_ns) > max_head_dt_ms * 1_000_000.0:
                skipped["head_time_gap"] += 1
                continue
            imu_record, imu_dt_ns = _nearest_by_time(wrist_imu_records, wrist_imu_times, wrist_visual["timestamp_monotonic_ns"])

            T_W_H = head_record["transform"]
            T_H_B = wrist_visual["transform"]
            visual_T_W_B = T_W_H @ T_H_B
            timestamp_unix_ns = int(wrist_visual.get("timestamp_unix_ns") or head_record.get("timestamp_unix_ns") or 0)
            timestamp_monotonic_ns = int(wrist_visual["timestamp_monotonic_ns"])
            head_dt_ms = float(head_dt_ns) / 1e6
            imu_dt_ms = float(imu_dt_ns) / 1e6 if imu_record is not None else None
            head_dt_values_ms.append(abs(head_dt_ms))
            if imu_dt_ms is not None:
                wrist_imu_dt_values_ms.append(abs(imu_dt_ms))
            wrist_imu_ok = imu_dt_ms is not None and abs(imu_dt_ms) <= max_wrist_imu_dt_ms

            fusion_method = "visual_only"
            if (
                use_wrist_imu_orientation
                and previous_fused_T_W_B is not None
                and previous_timestamp_ns is not None
                and timestamp_monotonic_ns > previous_timestamp_ns
                and wrist_imu_records
            ):
                delta_rotation, used_imu = _integrate_wrist_gyro(
                    wrist_imu_records,
                    wrist_imu_times,
                    previous_timestamp_ns,
                    timestamp_monotonic_ns,
                )
                if used_imu:
                    predicted_rotation = previous_fused_T_W_B.rotation @ delta_rotation
                    fused_rotation = _slerp_rotation(predicted_rotation, visual_T_W_B.rotation, alpha)
                    T_W_B = RigidTransform(rotation=fused_rotation, translation=visual_T_W_B.translation)
                    fusion_method = "wrist_gyro_propagation_with_visual_correction"
                    imu_propagated_frames += 1
                else:
                    T_W_B = visual_T_W_B
                    fusion_method = "visual_only_no_imu_interval"
                    imu_fallback_frames += 1
            else:
                T_W_B = visual_T_W_B
                if use_wrist_imu_orientation and previous_fused_T_W_B is not None:
                    fusion_method = "visual_only_no_imu"
                    imu_fallback_frames += 1

            fused_T_H_B = T_W_H.inverse() @ T_W_B

            wrist_payload = {
                "timestamp_unix_ns": timestamp_unix_ns,
                "timestamp_monotonic_ns": timestamp_monotonic_ns,
                "timestamp_source": "wrist_visual_with_optional_wrist_imu",
                "tracking_state": 1,
                "T_H_B": transform_to_dict(fused_T_H_B),
                "T_W_B": transform_to_dict(T_W_B),
                "visual_T_H_B": transform_to_dict(T_H_B),
                "visual_T_W_B": transform_to_dict(visual_T_W_B),
                "alignment": {
                    "head_dt_ms": head_dt_ms,
                    "wrist_imu_dt_ms": imu_dt_ms,
                    "wrist_imu_ok": wrist_imu_ok,
                },
                "fusion": {
                    "method": fusion_method,
                    "visual_correction_alpha": alpha,
                    "uses_wrist_gyro": fusion_method == "wrist_gyro_propagation_with_visual_correction",
                    "translation_source": "wrist_visual",
                },
                "wrist_imu": _imu_payload(imu_record),
            }
            wrist_file.write(json.dumps(wrist_payload, separators=(",", ":")) + "\n")

            motion_payload = {
                "timestamp_us": timestamp_unix_ns // 1000 if timestamp_unix_ns else timestamp_monotonic_ns // 1000,
                "timestamp_unix_ns": timestamp_unix_ns,
                "timestamp_monotonic_ns": timestamp_monotonic_ns,
                "tracking_state": 1,
                "head": _rigid_body_state("world", "head", T_W_H),
                "wrist": _rigid_body_state("world", "wrist", T_W_B, imu_record),
                "relative": {
                    "frame_id": "head",
                    "child_frame_id": "wrist",
                    "T_H_B": transform_to_dict(fused_T_H_B),
                    "visual_T_H_B": transform_to_dict(T_H_B),
                },
            }
            motion_file.write(json.dumps(motion_payload, separators=(",", ":")) + "\n")
            written += 1
            previous_fused_T_W_B = T_W_B
            previous_timestamp_ns = timestamp_monotonic_ns

    summary = {
        "session_dir": str(session_dir),
        "output_dir": str(output_dir),
        "inputs": {
            "head_pose": str(head_pose_path),
            "wrist_visual_pose": str(wrist_visual_path),
            "wrist_imu": str(wrist_imu_path),
        },
        "outputs": {
            "wrist_fused_pose": str(wrist_fused_path),
            "motion_frame": str(motion_frame_path),
        },
        "counts": {
            "head_pose": len(head_records),
            "wrist_visual_pose": len(wrist_visual_records),
            "wrist_imu": len(wrist_imu_records),
            "motion_frames": written,
            "skipped": skipped,
        },
        "alignment": {
            "max_head_dt_ms": max(head_dt_values_ms) if head_dt_values_ms else None,
            "mean_head_dt_ms": float(np.mean(head_dt_values_ms)) if head_dt_values_ms else None,
            "max_wrist_imu_dt_ms": max(wrist_imu_dt_values_ms) if wrist_imu_dt_values_ms else None,
            "mean_wrist_imu_dt_ms": float(np.mean(wrist_imu_dt_values_ms)) if wrist_imu_dt_values_ms else None,
            "head_threshold_ms": max_head_dt_ms,
            "wrist_imu_threshold_ms": max_wrist_imu_dt_ms,
        },
        "wrist_imu_fusion": {
            "enabled": use_wrist_imu_orientation and wrist_imu_available,
            "requested": use_wrist_imu_orientation,
            "wrist_imu_available": wrist_imu_available,
            "method": "gyro_propagation_visual_correction",
            "visual_correction_alpha": alpha,
            "imu_propagated_frames": imu_propagated_frames,
            "imu_fallback_frames": imu_fallback_frames,
            "note": (
                "Gyro is integrated between AprilTag visual poses for orientation continuity when wrist_imu exists; "
                "AprilTag remains the absolute pose correction and translation source."
            ),
        },
    }
    if not wrist_imu_available:
        summary["wrist_imu_fusion"]["fallback_reason"] = f"Missing wrist IMU log: {wrist_imu_path}"
    (output_dir / "fusion_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def _pose_record(record: dict, key: str) -> dict:
    transform = _transform_from_record(record, key)
    return {
        **record,
        "timestamp_monotonic_ns": int(record["timestamp_monotonic_ns"]),
        "timestamp_unix_ns": int(record.get("timestamp_unix_ns") or 0),
        "transform": transform,
    }


def _transform_from_record(record: dict, key: str) -> RigidTransform:
    payload = record.get(key) or record.get("transform") or record.get("pose")
    if payload is None:
        raise ValueError(f"Pose record is missing {key}/transform/pose: {record}")
    if "matrix" in payload:
        return RigidTransform.from_matrix(payload["matrix"])
    position = payload.get("position") or payload.get("pos_w")
    quat = payload.get("orientation_wxyz") or payload.get("rot_w")
    if position is None or quat is None:
        raise ValueError(f"Pose payload needs matrix or position+orientation_wxyz: {payload}")
    return RigidTransform(
        rotation=quat_wxyz_to_rotation_matrix(quat),
        translation=np.asarray(position, dtype=np.float64).reshape(3),
    )


def _nearest_by_time(records: list[dict], timestamps: list[int], timestamp_ns: int) -> tuple[dict | None, int]:
    if not records:
        return None, 0
    index = bisect.bisect_left(timestamps, int(timestamp_ns))
    candidates = []
    if index < len(records):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    best_index = min(candidates, key=lambda idx: abs(timestamps[idx] - int(timestamp_ns)))
    return records[best_index], timestamps[best_index] - int(timestamp_ns)


def _imu_payload(record: dict | None) -> dict | None:
    if record is None:
        return None
    return {
        "sensor_id": record.get("sensor_id"),
        "timestamp_unix_ns": record.get("timestamp_unix_ns"),
        "timestamp_monotonic_ns": record.get("timestamp_monotonic_ns"),
        "accel_mps2": record.get("accel_mps2"),
        "gyro_radps": record.get("gyro_radps"),
        "quat_wxyz": record.get("quat_wxyz"),
        "euler_deg": record.get("euler_deg"),
    }


def _integrate_wrist_gyro(records: list[dict], timestamps: list[int], start_ns: int, end_ns: int) -> tuple[np.ndarray, bool]:
    if not records or end_ns <= start_ns:
        return np.eye(3, dtype=np.float64), False

    boundaries = [int(start_ns)]
    start_index = bisect.bisect_right(timestamps, int(start_ns))
    end_index = bisect.bisect_left(timestamps, int(end_ns))
    boundaries.extend(timestamps[start_index:end_index])
    boundaries.append(int(end_ns))

    rotation = np.eye(3, dtype=np.float64)
    used_imu = False
    for segment_start, segment_end in zip(boundaries[:-1], boundaries[1:]):
        if segment_end <= segment_start:
            continue
        imu_index = bisect.bisect_right(timestamps, segment_start) - 1
        if imu_index < 0:
            imu_index = 0
        gyro = records[imu_index].get("gyro_radps")
        if gyro is None or len(gyro) != 3:
            continue
        dt_s = (segment_end - segment_start) / 1_000_000_000.0
        rotation = rotation @ _rotation_from_rotvec(np.asarray(gyro, dtype=np.float64) * dt_s)
        used_imu = True
    return rotation, used_imu


def _rotation_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return np.eye(3, dtype=np.float64)
    axis = rotvec / angle
    x, y, z = axis
    skew = np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def _slerp_rotation(rotation_a: np.ndarray, rotation_b: np.ndarray, alpha: float) -> np.ndarray:
    quat_a = np.asarray(_rotation_matrix_to_quat_wxyz(rotation_a), dtype=np.float64)
    quat_b = np.asarray(_rotation_matrix_to_quat_wxyz(rotation_b), dtype=np.float64)
    quat = _slerp_quat(quat_a, quat_b, alpha)
    return quat_wxyz_to_rotation_matrix(quat)


def _slerp_quat(quat_a: np.ndarray, quat_b: np.ndarray, alpha: float) -> np.ndarray:
    a = np.asarray(quat_a, dtype=np.float64)
    b = np.asarray(quat_b, dtype=np.float64)
    a /= max(np.linalg.norm(a), 1e-12)
    b /= max(np.linalg.norm(b), 1e-12)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    alpha = min(max(float(alpha), 0.0), 1.0)
    if dot > 0.9995:
        result = a + alpha * (b - a)
        result /= max(np.linalg.norm(result), 1e-12)
        return result
    theta_0 = math.acos(max(min(dot, 1.0), -1.0))
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    scale_a = math.sin(theta_0 - theta) / sin_theta_0
    scale_b = math.sin(theta) / sin_theta_0
    result = scale_a * a + scale_b * b
    result /= max(np.linalg.norm(result), 1e-12)
    return result


def _rotation_matrix_to_quat_wxyz(rotation: np.ndarray) -> list[float]:
    # Local wrapper keeps this module independent from the AprilTag averaging helper internals.
    from packages.apriltag_ring_node.geometry import rotation_matrix_to_quat_wxyz

    return rotation_matrix_to_quat_wxyz(rotation)


def _rigid_body_state(frame_id: str, child_frame_id: str, transform: RigidTransform, imu_record: dict | None = None) -> dict:
    pose = transform_to_dict(transform)
    return {
        "frame_id": frame_id,
        "child_frame_id": child_frame_id,
        "position": pose["position"],
        "orientation_wxyz": pose["orientation_wxyz"],
        "linear_velocity": [0.0, 0.0, 0.0],
        "angular_velocity": [float(v) for v in imu_record.get("gyro_radps", [0.0, 0.0, 0.0])] if imu_record else [0.0, 0.0, 0.0],
        "linear_acceleration": [float(v) for v in imu_record.get("accel_mps2", [0.0, 0.0, 0.0])] if imu_record else [0.0, 0.0, 0.0],
        "tracking_state": 1,
    }


def _first_existing(paths: list[Path], *, label: str) -> Path:
    for path in paths:
        if path.exists():
            return path
    raise FileNotFoundError(f"Could not find {label}. Tried: {', '.join(str(path) for path in paths)}")


def _read_jsonl(path: Path) -> Iterable[dict]:
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def main() -> None:
    parser = argparse.ArgumentParser(description="Fuse head VIO, wrist AprilTag visual pose, and wrist IMU into motion JSONL.")
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--head-pose")
    parser.add_argument("--wrist-visual")
    parser.add_argument("--wrist-imu")
    parser.add_argument("--output-dir")
    parser.add_argument("--max-head-dt-ms", type=float, default=50.0)
    parser.add_argument("--max-wrist-imu-dt-ms", type=float, default=20.0)
    parser.add_argument("--disable-wrist-imu-orientation", action="store_true", help="Do not use wrist gyro for orientation propagation.")
    parser.add_argument(
        "--visual-correction-alpha",
        type=float,
        default=0.35,
        help="0 keeps gyro prediction, 1 snaps fully to AprilTag orientation. Default: 0.35.",
    )
    args = parser.parse_args()
    summary = fuse_motion_session(
        session_dir=Path(args.session_dir),
        output_root=Path(args.output_root),
        head_pose_path=Path(args.head_pose) if args.head_pose else None,
        wrist_visual_path=Path(args.wrist_visual) if args.wrist_visual else None,
        wrist_imu_path=Path(args.wrist_imu) if args.wrist_imu else None,
        output_dir=Path(args.output_dir) if args.output_dir else None,
        max_head_dt_ms=args.max_head_dt_ms,
        max_wrist_imu_dt_ms=args.max_wrist_imu_dt_ms,
        use_wrist_imu_orientation=not args.disable_wrist_imu_orientation,
        visual_correction_alpha=args.visual_correction_alpha,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
