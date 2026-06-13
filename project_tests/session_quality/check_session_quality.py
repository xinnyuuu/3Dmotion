#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable


@dataclass
class Check:
    name: str
    ok: bool
    message: str
    details: dict


def check_session_quality(
    session_dir: Path,
    *,
    expected_cameras: list[str],
    max_group_span_us: float,
    max_abs_skew_us: float,
    min_complete_groups: int,
    min_duration_s: float,
    min_frames_per_camera: int,
    min_group_completion_ratio: float,
) -> dict:
    session_dir = session_dir.resolve()
    frames_path = session_dir / "cameras" / "frames.jsonl"
    imus_dir = session_dir / "imus"

    frames = list(_read_jsonl(frames_path))
    imu_files = sorted(imus_dir.glob("*.jsonl")) if imus_dir.exists() else []

    checks = [
        _check_imu_timestamp_monotonic(imu_files),
        _check_camera_group_stability(
            frames,
            expected_cameras=expected_cameras,
            min_complete_groups=min_complete_groups,
            min_group_completion_ratio=min_group_completion_ratio,
        ),
        _check_camera_skew(
            frames,
            max_group_span_us=max_group_span_us,
            max_abs_skew_us=max_abs_skew_us,
        ),
        _check_repeatable_short_log(
            session_dir,
            frames,
            expected_cameras=expected_cameras,
            min_duration_s=min_duration_s,
            min_frames_per_camera=min_frames_per_camera,
        ),
    ]
    overall_ok = all(check.ok for check in checks)

    report = {
        "session_dir": str(session_dir),
        "overall_ok": overall_ok,
        "thresholds": {
            "expected_cameras": expected_cameras,
            "max_group_span_us": max_group_span_us,
            "max_abs_skew_us": max_abs_skew_us,
            "min_complete_groups": min_complete_groups,
            "min_duration_s": min_duration_s,
            "min_frames_per_camera": min_frames_per_camera,
            "min_group_completion_ratio": min_group_completion_ratio,
        },
        "checks": [asdict(check) for check in checks],
        "next_steps": _next_steps(checks),
    }
    return report


def _check_imu_timestamp_monotonic(imu_files: list[Path]) -> Check:
    details = {}
    ok = bool(imu_files)
    messages = []

    for imu_path in imu_files:
        records = list(_read_jsonl(imu_path))
        timestamp_key = _first_timestamp_key(records)
        backward_count = 0
        duplicate_count = 0
        first_ts = None
        last_ts = None
        previous_ts = None

        for record in records:
            ts = record.get(timestamp_key) if timestamp_key else None
            if ts is None:
                continue
            ts = int(ts)
            if first_ts is None:
                first_ts = ts
            if previous_ts is not None:
                if ts < previous_ts:
                    backward_count += 1
                elif ts == previous_ts:
                    duplicate_count += 1
            previous_ts = ts
            last_ts = ts

        imu_ok = bool(records) and timestamp_key is not None and backward_count == 0
        ok = ok and imu_ok
        messages.append(f"{imu_path.stem}: count={len(records)}, backwards={backward_count}, duplicates={duplicate_count}")
        details[imu_path.stem] = {
            "file": str(imu_path),
            "timestamp_key": timestamp_key,
            "count": len(records),
            "first_timestamp": first_ts,
            "last_timestamp": last_ts,
            "backward_count": backward_count,
            "duplicate_count": duplicate_count,
            "ok": imu_ok,
        }

    if not imu_files:
        messages.append("no IMU JSONL files found")

    return Check(
        name="imu_timestamp_monotonic",
        ok=ok,
        message="; ".join(messages),
        details=details,
    )


def _check_camera_group_stability(
    frames: list[dict],
    *,
    expected_cameras: list[str],
    min_complete_groups: int,
    min_group_completion_ratio: float,
) -> Check:
    groups = _group_frames(frames)
    expected = set(expected_cameras)
    complete_groups = 0
    incomplete_examples = []
    duplicate_camera_groups = 0

    for group_id, group_frames in groups.items():
        cameras = [str(frame.get("camera_id")) for frame in group_frames]
        camera_set = set(cameras)
        if camera_set == expected and len(cameras) == len(expected):
            complete_groups += 1
        else:
            if len(incomplete_examples) < 10:
                incomplete_examples.append(
                    {
                        "group_id": group_id,
                        "cameras": sorted(camera_set),
                        "missing": sorted(expected - camera_set),
                        "extra": sorted(camera_set - expected),
                    }
                )
        if len(cameras) != len(camera_set):
            duplicate_camera_groups += 1

    group_ids = sorted(groups)
    missing_group_ids = _missing_group_ids(group_ids)
    completion_ratio = complete_groups / len(groups) if groups else 0.0
    ok = (
        bool(groups)
        and complete_groups >= min_complete_groups
        and completion_ratio >= min_group_completion_ratio
        and not missing_group_ids
        and duplicate_camera_groups == 0
    )

    return Check(
        name="camera_group_stability",
        ok=ok,
        message=(
            f"groups={len(groups)}, complete={complete_groups}, "
            f"completion_ratio={completion_ratio:.3f}, missing_group_ids={len(missing_group_ids)}, "
            f"duplicate_camera_groups={duplicate_camera_groups}"
        ),
        details={
            "group_count": len(groups),
            "complete_groups": complete_groups,
            "completion_ratio": completion_ratio,
            "expected_cameras": expected_cameras,
            "first_group_id": group_ids[0] if group_ids else None,
            "last_group_id": group_ids[-1] if group_ids else None,
            "missing_group_id_count": len(missing_group_ids),
            "missing_group_id_examples": missing_group_ids[:20],
            "duplicate_camera_groups": duplicate_camera_groups,
            "incomplete_examples": incomplete_examples,
        },
    )


def _check_camera_skew(
    frames: list[dict],
    *,
    max_group_span_us: float,
    max_abs_skew_us: float,
) -> Check:
    groups = _group_frames(frames)
    group_spans_us = []
    abs_skews_us = []

    for group_frames in groups.values():
        timestamps = [
            int(frame["timestamp_monotonic_ns"])
            for frame in group_frames
            if frame.get("timestamp_monotonic_ns") is not None
        ]
        if len(timestamps) >= 2:
            group_spans_us.append((max(timestamps) - min(timestamps)) / 1000.0)
        for frame in group_frames:
            if frame.get("skew_us") is not None:
                abs_skews_us.append(abs(float(frame["skew_us"])))

    max_span = max(group_spans_us) if group_spans_us else None
    p95_span = _percentile(group_spans_us, 95)
    max_abs_skew = max(abs_skews_us) if abs_skews_us else None
    p95_abs_skew = _percentile(abs_skews_us, 95)

    span_ok = max_span is not None and max_span <= max_group_span_us
    skew_ok = max_abs_skew is not None and max_abs_skew <= max_abs_skew_us
    ok = span_ok and skew_ok

    return Check(
        name="camera_skew",
        ok=ok,
        message=(
            f"max_group_span_us={_fmt(max_span)}, p95_group_span_us={_fmt(p95_span)}, "
            f"max_abs_skew_us={_fmt(max_abs_skew)}, p95_abs_skew_us={_fmt(p95_abs_skew)}"
        ),
        details={
            "group_span_us": _stats(group_spans_us),
            "abs_skew_us": _stats(abs_skews_us),
            "max_group_span_us_threshold": max_group_span_us,
            "max_abs_skew_us_threshold": max_abs_skew_us,
        },
    )


def _check_repeatable_short_log(
    session_dir: Path,
    frames: list[dict],
    *,
    expected_cameras: list[str],
    min_duration_s: float,
    min_frames_per_camera: int,
) -> Check:
    cameras_dir = session_dir / "cameras"
    camera_counts = Counter(str(frame.get("camera_id")) for frame in frames)
    missing_images = []
    timestamps = [
        int(frame["timestamp_monotonic_ns"])
        for frame in frames
        if frame.get("timestamp_monotonic_ns") is not None
    ]
    duration_s = (max(timestamps) - min(timestamps)) / 1e9 if len(timestamps) >= 2 else 0.0

    for frame in frames:
        image_path = frame.get("image_path")
        if not image_path or not (cameras_dir / str(image_path)).exists():
            missing_images.append(str(image_path))
            if len(missing_images) >= 20:
                break

    enough_duration = duration_s >= min_duration_s
    enough_frames = all(camera_counts.get(camera_id, 0) >= min_frames_per_camera for camera_id in expected_cameras)
    no_missing_images = not missing_images
    ok = bool(frames) and enough_duration and enough_frames and no_missing_images

    return Check(
        name="repeatable_short_log",
        ok=ok,
        message=(
            f"duration_s={duration_s:.3f}, frame_count={len(frames)}, "
            f"camera_counts={dict(sorted(camera_counts.items()))}, missing_images={len(missing_images)}"
        ),
        details={
            "duration_s": duration_s,
            "frame_count": len(frames),
            "camera_counts": dict(sorted(camera_counts.items())),
            "min_duration_s": min_duration_s,
            "min_frames_per_camera": min_frames_per_camera,
            "missing_image_examples": missing_images,
            "session_manifest_exists": (session_dir / "session_manifest.json").exists(),
            "session_summary_exists": (session_dir / "session_summary.json").exists(),
        },
    )


def _read_jsonl(path: Path) -> Iterable[dict]:
    if not path.exists():
        return
    with path.open("r", encoding="utf-8") as f:
        for line_number, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSONL in {path}:{line_number}: {exc}") from exc


def _first_timestamp_key(records: list[dict]) -> str | None:
    for key in ("timestamp_monotonic_ns", "timestamp_unix_ns"):
        if any(record.get(key) is not None for record in records):
            return key
    return None


def _group_frames(frames: list[dict]) -> dict[int, list[dict]]:
    groups = defaultdict(list)
    for frame in frames:
        group_id = frame.get("group_id")
        if group_id is None:
            continue
        groups[int(group_id)].append(frame)
    return dict(groups)


def _missing_group_ids(group_ids: list[int]) -> list[int]:
    if not group_ids:
        return []
    present = set(group_ids)
    return [group_id for group_id in range(group_ids[0], group_ids[-1] + 1) if group_id not in present]


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = round((len(ordered) - 1) * percentile / 100.0)
    return ordered[index]


def _stats(values: list[float]) -> dict:
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "p95": _percentile(values, 95),
        "max": max(values),
    }


def _fmt(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.1f}"


def _next_steps(checks: list[Check]) -> list[str]:
    failed = {check.name for check in checks if not check.ok}
    steps = []
    if "imu_timestamp_monotonic" in failed:
        steps.append("IMU timestamp 有倒退或缺失。先检查 BLE callback 写入逻辑和 timestamp 字段。")
    if "camera_group_stability" in failed:
        steps.append("camera group 不稳定。先确认四个 camera source 都正常打开，并检查 recorder 的 group_id 写入。")
    if "camera_skew" in failed:
        steps.append("四目 skew 超阈值。原型可先放宽阈值；若做几何融合，需要改进同步或降低分辨率/帧率。")
    if "repeatable_short_log" in failed:
        steps.append("短动作日志不够完整。建议重新采一段 5-10 秒数据，并确认每个 camera 文件夹持续写图。")
    if not steps:
        steps.append("这段 session 通过原型数据质量检查，可以继续做 AprilTag 处理、OpenVINS 准备和 wrist IMU 对齐。")
    return steps


def _parse_camera_ids(value: str) -> list[str]:
    camera_ids = [item.strip() for item in value.split(",") if item.strip()]
    if not camera_ids:
        raise argparse.ArgumentTypeError("expected at least one camera id")
    return camera_ids


def main() -> None:
    parser = argparse.ArgumentParser(description="Check collected 3DMotion session data against prototype quality gates.")
    parser.add_argument("--session-dir", required=True, help="Raw session directory, e.g. data/raw/session_YYYYMMDD_HHMMSS.")
    parser.add_argument("--expected-cameras", type=_parse_camera_ids, default=["C0", "C1", "C2", "C3"])
    parser.add_argument("--max-group-span-us", type=float, default=30000.0)
    parser.add_argument("--max-abs-skew-us", type=float, default=20000.0)
    parser.add_argument("--min-complete-groups", type=int, default=5)
    parser.add_argument("--min-duration-s", type=float, default=2.0)
    parser.add_argument("--min-frames-per-camera", type=int, default=15)
    parser.add_argument("--min-group-completion-ratio", type=float, default=0.90)
    parser.add_argument("--output", help="Optional JSON report path. Defaults to project_tests/reports/<session>_quality.json.")
    args = parser.parse_args()

    session_dir = Path(args.session_dir)
    report = check_session_quality(
        session_dir,
        expected_cameras=args.expected_cameras,
        max_group_span_us=args.max_group_span_us,
        max_abs_skew_us=args.max_abs_skew_us,
        min_complete_groups=args.min_complete_groups,
        min_duration_s=args.min_duration_s,
        min_frames_per_camera=args.min_frames_per_camera,
        min_group_completion_ratio=args.min_group_completion_ratio,
    )

    output_path = Path(args.output) if args.output else Path("project_tests") / "reports" / f"{session_dir.name}_quality.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"\nreport: {output_path}")
    sys.exit(0 if report["overall_ok"] else 1)


if __name__ == "__main__":
    main()
