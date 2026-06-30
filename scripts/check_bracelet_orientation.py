#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from packages.apriltag_ring_node.geometry import RigidTransform, average_transforms  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Check whether bracelet tag-to-wrist orientations agree across visible tags/cameras."
    )
    parser.add_argument("--candidates", help="Path to wrist_world_candidates.jsonl or wrist_visual_candidates.jsonl.")
    parser.add_argument(
        "--processed-dir",
        help="Directory containing wrist_world_candidates.jsonl or wrist_visual_candidates.jsonl.",
    )
    parser.add_argument("--min-candidates", type=int, default=2)
    parser.add_argument("--max-print", type=int, default=12)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    candidates_path = _resolve_candidates_path(args)
    records = _read_candidates(candidates_path)
    summary = analyze(records, min_candidates=args.min_candidates)
    summary["input"] = str(candidates_path)

    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        _print_summary(summary, max_print=args.max_print)


def _resolve_candidates_path(args: argparse.Namespace) -> Path:
    if args.candidates:
        return Path(args.candidates)
    if not args.processed_dir:
        raise SystemExit("Provide --candidates or --processed-dir.")
    root = Path(args.processed_dir)
    for name in ("wrist_world_candidates.jsonl", "wrist_visual_candidates.jsonl"):
        path = root / name
        if path.exists():
            return path
    raise SystemExit(f"No wrist candidate JSONL found in {root}")


def _read_candidates(path: Path) -> list[dict]:
    if not path.exists():
        raise FileNotFoundError(path)
    records = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            record = json.loads(line)
            transform_key = "T_W_B" if "T_W_B" in record else "T_H_B" if "T_H_B" in record else None
            if transform_key is None:
                continue
            records.append({**record, "_transform_key": transform_key})
    return records


def analyze(records: list[dict], *, min_candidates: int) -> dict:
    groups: dict[int, list[dict]] = defaultdict(list)
    for record in records:
        groups[int(record["group_id"])].append(record)

    residuals = []
    per_tag: dict[int, list[dict]] = defaultdict(list)
    per_camera: dict[str, list[dict]] = defaultdict(list)
    pairwise: dict[str, list[float]] = defaultdict(list)
    usable_group_count = 0

    for group_id, group_records in sorted(groups.items()):
        if len(group_records) < min_candidates:
            continue
        transforms = [(_record_transform(record), _candidate_weight(record)) for record in group_records]
        fused = average_transforms(transforms)
        usable_group_count += 1

        for record in group_records:
            transform = _record_transform(record)
            item = {
                "group_id": group_id,
                "tag_id": int(record.get("tag_id", -1)),
                "camera_id": str(record.get("camera_id", "")),
                "rotation_error_deg": _rotation_error_deg(transform.rotation, fused.rotation),
                "translation_error_m": float(np.linalg.norm(transform.translation - fused.translation)),
                "reprojection_error_px": float(record.get("reprojection_error_px", 0.0)),
            }
            residuals.append(item)
            per_tag[item["tag_id"]].append(item)
            per_camera[item["camera_id"]].append(item)

        for index, left in enumerate(group_records):
            left_tf = _record_transform(left)
            for right in group_records[index + 1 :]:
                right_tf = _record_transform(right)
                tag_pair = tuple(sorted((int(left.get("tag_id", -1)), int(right.get("tag_id", -1)))))
                pairwise[f"{tag_pair[0]}-{tag_pair[1]}"].append(
                    _rotation_error_deg(left_tf.rotation, right_tf.rotation)
                )

    overall_rotation = [item["rotation_error_deg"] for item in residuals]
    overall_translation = [item["translation_error_m"] for item in residuals]
    summary = {
        "candidate_count": len(records),
        "group_count": len(groups),
        "usable_group_count": usable_group_count,
        "multi_candidate_residual_count": len(residuals),
        "overall": {
            "rotation_error_deg": _stats(overall_rotation),
            "translation_error_m": _stats(overall_translation),
        },
        "per_tag": {
            str(tag_id): _residual_stats(items)
            for tag_id, items in sorted(per_tag.items())
        },
        "per_camera": {
            camera_id: _residual_stats(items)
            for camera_id, items in sorted(per_camera.items())
        },
        "pairwise_tag_rotation_deg": {
            pair: _stats(values)
            for pair, values in sorted(pairwise.items())
            if len(values) >= max(3, min_candidates)
        },
    }
    summary["interpretation"] = _interpret(summary)
    return summary


def _record_transform(record: dict) -> RigidTransform:
    value = record[record["_transform_key"]]
    return RigidTransform.from_matrix(value["matrix"])


def _candidate_weight(record: dict) -> float:
    return 1.0 / max(float(record.get("reprojection_error_px", 1.0)), 0.25)


def _rotation_error_deg(left: np.ndarray, right: np.ndarray) -> float:
    relative = left.T @ right
    cos_angle = (float(np.trace(relative)) - 1.0) * 0.5
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return math.degrees(math.acos(cos_angle))


def _stats(values: list[float]) -> dict:
    if not values:
        return {"count": 0}
    values_sorted = sorted(float(value) for value in values)
    return {
        "count": len(values_sorted),
        "median": statistics.median(values_sorted),
        "p90": _percentile(values_sorted, 90),
        "p95": _percentile(values_sorted, 95),
        "max": values_sorted[-1],
    }


def _residual_stats(items: list[dict]) -> dict:
    return {
        "count": len(items),
        "rotation_error_deg": _stats([item["rotation_error_deg"] for item in items]),
        "translation_error_m": _stats([item["translation_error_m"] for item in items]),
        "median_reprojection_error_px": statistics.median([item["reprojection_error_px"] for item in items])
        if items
        else None,
    }


def _percentile(values_sorted: list[float], percentile: float) -> float:
    if not values_sorted:
        return float("nan")
    index = int(round((len(values_sorted) - 1) * percentile / 100.0))
    return values_sorted[max(0, min(index, len(values_sorted) - 1))]


def _interpret(summary: dict) -> list[str]:
    notes = []
    rot = summary["overall"]["rotation_error_deg"]
    trans = summary["overall"]["translation_error_m"]
    if not rot.get("count"):
        return ["Not enough frames with multiple wrist candidates. Show at least two bracelet tags at once."]

    rot_med = float(rot["median"])
    rot_p95 = float(rot["p95"])
    trans_med = float(trans["median"])
    if rot_med < 12.0 and rot_p95 < 35.0:
        notes.append("Orientation looks broadly consistent across visible bracelet tags.")
    elif rot_med >= 45.0 or rot_p95 >= 90.0:
        notes.append("Orientation is very inconsistent. Check tag_order, ring_order_direction, and each tag-to-wrist rotation.")
    else:
        notes.append("Orientation is somewhat inconsistent. One or two tag transforms may be wrong.")

    if trans_med > 0.06:
        notes.append("Translation residual is large. Check center_offset_m / bracelet radius or tag-to-wrist translations.")
    else:
        notes.append("Translation residual is in a plausible range for the current placeholder bracelet geometry.")

    bad_tags = []
    for tag_id, stats in summary["per_tag"].items():
        tag_rot = stats["rotation_error_deg"]
        if tag_rot.get("count", 0) and float(tag_rot["median"]) > max(25.0, rot_med * 1.5):
            bad_tags.append(tag_id)
    if bad_tags:
        notes.append(f"Likely problematic tag ids: {', '.join(bad_tags)}.")
    return notes


def _print_summary(summary: dict, *, max_print: int) -> None:
    print(f"input: {summary['input']}")
    print(
        f"candidates={summary['candidate_count']} groups={summary['group_count']} "
        f"usable_multi_candidate_groups={summary['usable_group_count']}"
    )
    rot = summary["overall"]["rotation_error_deg"]
    trans = summary["overall"]["translation_error_m"]
    print(
        "overall rotation residual deg: "
        f"median={_fmt(rot, 'median')} p95={_fmt(rot, 'p95')} max={_fmt(rot, 'max')}"
    )
    print(
        "overall translation residual m: "
        f"median={_fmt(trans, 'median')} p95={_fmt(trans, 'p95')} max={_fmt(trans, 'max')}"
    )
    print("\nper tag:")
    rows = []
    for tag_id, stats in summary["per_tag"].items():
        rows.append(
            (
                tag_id,
                stats["count"],
                stats["rotation_error_deg"].get("median"),
                stats["rotation_error_deg"].get("p95"),
                stats["translation_error_m"].get("median"),
            )
        )
    for tag_id, count, rot_med, rot_p95, trans_med in rows[:max_print]:
        print(
            f"  tag {tag_id}: n={count} rot_med={_num(rot_med)} rot_p95={_num(rot_p95)} "
            f"trans_med={_num(trans_med, digits=3)}"
        )
    print("\ninterpretation:")
    for note in summary["interpretation"]:
        print(f"- {note}")


def _fmt(stats: dict, key: str) -> str:
    return _num(stats.get(key))


def _num(value, *, digits: int = 1) -> str:
    if value is None:
        return "--"
    return f"{float(value):.{digits}f}"


if __name__ == "__main__":
    main()
