from __future__ import annotations

import argparse
import bisect
import json
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
    wrist_imu_records = list(_read_jsonl(wrist_imu_path)) if wrist_imu_path.exists() else []

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

    with wrist_fused_path.open("w", encoding="utf-8") as wrist_file, motion_frame_path.open("w", encoding="utf-8") as motion_file:
        for wrist_visual in wrist_visual_records:
            head_record, head_dt_ns = _nearest_by_time(head_records, head_times, wrist_visual["timestamp_monotonic_ns"])
            if head_record is None or abs(head_dt_ns) > max_head_dt_ms * 1_000_000.0:
                skipped["head_time_gap"] += 1
                continue
            imu_record, imu_dt_ns = _nearest_by_time(wrist_imu_records, wrist_imu_times, wrist_visual["timestamp_monotonic_ns"])

            T_W_H = head_record["transform"]
            T_H_B = wrist_visual["transform"]
            T_W_B = T_W_H @ T_H_B
            timestamp_unix_ns = int(wrist_visual.get("timestamp_unix_ns") or head_record.get("timestamp_unix_ns") or 0)
            timestamp_monotonic_ns = int(wrist_visual["timestamp_monotonic_ns"])
            head_dt_ms = float(head_dt_ns) / 1e6
            imu_dt_ms = float(imu_dt_ns) / 1e6 if imu_record is not None else None
            head_dt_values_ms.append(abs(head_dt_ms))
            if imu_dt_ms is not None:
                wrist_imu_dt_values_ms.append(abs(imu_dt_ms))

            wrist_payload = {
                "timestamp_unix_ns": timestamp_unix_ns,
                "timestamp_monotonic_ns": timestamp_monotonic_ns,
                "timestamp_source": "wrist_visual",
                "tracking_state": 1,
                "T_H_B": transform_to_dict(T_H_B),
                "T_W_B": transform_to_dict(T_W_B),
                "alignment": {
                    "head_dt_ms": head_dt_ms,
                    "wrist_imu_dt_ms": imu_dt_ms,
                    "wrist_imu_ok": imu_dt_ms is not None and abs(imu_dt_ms) <= max_wrist_imu_dt_ms,
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
                    "T_H_B": transform_to_dict(T_H_B),
                },
            }
            motion_file.write(json.dumps(motion_payload, separators=(",", ":")) + "\n")
            written += 1

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
    }
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
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
