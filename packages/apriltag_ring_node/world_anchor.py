from __future__ import annotations

import argparse
import bisect
import copy
import json
import math
import time
from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import yaml

from .config import BraceletConfig, CameraCalibration, load_bracelet_config, load_camera_calibrations
from .detector import (
    AprilTagDetector,
    TagDetection,
    TagPose,
    _is_omni_distortion,
    _pnp_points_and_distortion,
    estimate_tag_pose,
    reprojection_error,
    tag_object_points,
)
from .geometry import (
    RigidTransform,
    average_transforms,
    parse_transform,
    quat_wxyz_to_rotation_matrix,
    rotation_matrix_to_quat_wxyz,
    transform_to_dict,
)
from .hand import HAND_CONNECTIONS, HandDetection, HandKeypointDetector, hands_to_dicts
from .process_session import _scaled_camera_matrix, _tag_pose_to_wrist_pose


@dataclass
class WorldTag:
    tag_id: int
    tag_size_m: float
    T_W_T: RigidTransform


@dataclass
class WorldTagConfig:
    tag_family: str
    default_tag_size_m: float
    tags: dict[int, WorldTag]


@dataclass
class WorldCandidate:
    group_id: int
    camera_id: str
    tag_id: int | None
    source_tag_ids: list[int]
    source_corner_count: int
    timestamp_monotonic_ns: int
    timestamp_unix_ns: int
    method: str
    reprojection_error_px: float
    weight: float
    prediction_translation_error_m: float | None
    prediction_rotation_error_deg: float | None
    T_W_H: dict
    T_W_C: dict
    corners: list[list[float]]


@dataclass
class WorldBoardPose:
    T_W_C: RigidTransform
    T_W_H: RigidTransform
    reprojection_error_px: float
    source_tag_ids: list[int]
    source_corner_count: int
    corners: list[list[float]]
    method: str
    prediction_translation_error_m: float | None = None
    prediction_rotation_error_deg: float | None = None


@dataclass
class TrackerState:
    pose: RigidTransform | None = None
    timestamp_monotonic_ns: int | None = None
    velocity_world_mps: np.ndarray | None = None
    visual_updates: int = 0
    predicted_updates: int = 0


@dataclass
class HandTrackerState:
    hand: dict | None = None
    timestamp_monotonic_ns: int | None = None
    camera_id: str | None = None
    selected_updates: int = 0
    continuity_rejections: int = 0
    missing_updates: int = 0


@dataclass
class ImuSampleLite:
    timestamp_monotonic_ns: int
    gyro_radps: np.ndarray
    accel_mps2: np.ndarray | None = None


@dataclass
class WorldMotionFrame:
    timestamp_unix_ns: int
    timestamp_monotonic_ns: int
    timestamp_source: str
    group_id: int
    tracking_state: int
    source: dict
    head: dict
    wrist: dict | None
    relative: dict
    hands: list[dict] | None = None


def load_world_tag_config(path: Path) -> WorldTagConfig:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    family = str(data.get("tag_family", "tag36h11")).lower()
    default_size = float(data["tag_size_m"])
    tags = {}
    for raw_tag_id, cfg in (data.get("tags") or {}).items():
        tag_id = int(raw_tag_id)
        tags[tag_id] = WorldTag(
            tag_id=tag_id,
            tag_size_m=float(cfg.get("tag_size_m", default_size)),
            T_W_T=parse_transform(cfg.get("T_W_T"), default=RigidTransform.identity()),
        )
    if not tags:
        raise ValueError(f"No world tags configured in {path}")
    return WorldTagConfig(tag_family=family, default_tag_size_m=default_size, tags=tags)


def process_world_anchor_session(
    *,
    session_dir: Path,
    cameras_path: Path,
    world_tags_path: Path,
    bracelet_path: Path,
    output_dir: Path,
    max_reprojection_error_px: float = 8.0,
    require_wrist: bool = False,
    allow_single_tag: bool = True,
    use_head_imu: bool = True,
    use_wrist_imu: bool = True,
    allow_prediction_output: bool = False,
    max_prediction_gap_s: float = 1.0,
    max_position_prediction_gap_s: float = 0.25,
    single_tag_gate_translation_m: float = 0.12,
    single_tag_gate_rotation_deg: float = 35.0,
    allow_single_tag_bootstrap: bool = False,
    enable_hands: bool = False,
    max_hands: int = 2,
    hand_detection_confidence: float = 0.5,
    hand_tracking_confidence: float = 0.5,
    hand_model: Path | None = None,
    max_hand_skeletons_per_frame: int = 1,
    hand_continuity_gate_m: float = 0.12,
    hand_continuity_max_gap_s: float = 0.25,
    hand_multiview_only: bool = False,
    hand_allow_direct_singleview_fallback: bool = False,
    head_position_visual_alpha: float = 0.75,
    head_orientation_visual_alpha: float = 0.12,
    max_wrist_prediction_gap_s: float = 1.0,
    wrist_position_visual_alpha: float = 0.75,
    wrist_visual_correction_alpha: float = 0.35,
    wrist_prediction_gate_translation_m: float = 0.20,
    wrist_prediction_gate_rotation_deg: float = 60.0,
    wrist_static_gyro_thresh_radps: float = 0.18,
    wrist_static_accel_std_thresh_mps2: float = 0.25,
    multi_camera_gate_translation_m: float = 0.12,
    multi_camera_gate_rotation_deg: float = 25.0,
    multi_camera_median_gate_m: float = 0.08,
) -> dict:
    try:
        import cv2  # noqa: F401
    except ImportError as exc:
        raise RuntimeError("Install opencv-contrib-python before running AprilTag processing.") from exc

    raw_session_dir = session_dir
    session_dir = _resolve_camera_session_dir(session_dir)
    raw_session_dir = session_dir.parent if session_dir.name == "cameras" else raw_session_dir
    camera_calibrations = load_camera_calibrations(cameras_path)
    world_tags = load_world_tag_config(world_tags_path)
    bracelet = load_bracelet_config(bracelet_path)
    world_detector = AprilTagDetector(world_tags.tag_family)
    wrist_detector = AprilTagDetector(bracelet.tag_family)
    hand_detector = _make_hand_detector(
        enable_hands=enable_hands,
        max_hands=max_hands,
        hand_detection_confidence=hand_detection_confidence,
        hand_tracking_confidence=hand_tracking_confidence,
        hand_model=hand_model,
    )
    head_imu_samples = _read_imu_samples(raw_session_dir / "imus" / "head_imu.jsonl") if use_head_imu else []
    wrist_imu_samples = _read_imu_samples(raw_session_dir / "imus" / "wrist_imu.jsonl") if use_wrist_imu else []

    output_dir.mkdir(parents=True, exist_ok=True)
    frames_by_group = _read_frames(session_dir / "frames.jsonl")
    world_candidates_path = output_dir / "world_anchor_candidates.jsonl"
    wrist_candidates_path = output_dir / "wrist_world_candidates.jsonl"
    hand_skeletons_path = output_dir / "hand_skeletons.jsonl"
    camera_diagnostics_path = output_dir / "camera_diagnostics.jsonl"
    head_pose_path = output_dir / "head_pose.jsonl"
    motion_path = output_dir / "motion_frame.jsonl"
    world_candidates_tmp = world_candidates_path.with_name(world_candidates_path.name + ".tmp")
    wrist_candidates_tmp = wrist_candidates_path.with_name(wrist_candidates_path.name + ".tmp")
    hand_skeletons_tmp = hand_skeletons_path.with_name(hand_skeletons_path.name + ".tmp")
    camera_diagnostics_tmp = camera_diagnostics_path.with_name(camera_diagnostics_path.name + ".tmp")
    head_pose_tmp = head_pose_path.with_name(head_pose_path.name + ".tmp")
    motion_tmp = motion_path.with_name(motion_path.name + ".tmp")

    counts = defaultdict(int)
    camera_counts: dict[str, defaultdict[str, int]] = defaultdict(lambda: defaultdict(int))
    tracker = TrackerState()
    wrist_tracker = TrackerState()
    hand_tracker = HandTrackerState()
    with (
        world_candidates_tmp.open("w", encoding="utf-8") as world_file,
        wrist_candidates_tmp.open("w", encoding="utf-8") as wrist_file,
        hand_skeletons_tmp.open("w", encoding="utf-8") as hand_file,
        camera_diagnostics_tmp.open("w", encoding="utf-8") as diagnostics_file,
        head_pose_tmp.open("w", encoding="utf-8") as head_file,
        motion_tmp.open("w", encoding="utf-8") as motion_file,
    ):
        for group_id in sorted(frames_by_group):
            result = estimate_world_motion_frame(
                frame_records=frames_by_group[group_id],
                session_dir=session_dir,
                camera_calibrations=camera_calibrations,
                world_tags=world_tags,
                bracelet=bracelet,
                world_detector=world_detector,
                wrist_detector=wrist_detector,
                max_reprojection_error_px=max_reprojection_error_px,
                tracker=tracker,
                head_imu_samples=head_imu_samples,
                wrist_tracker=wrist_tracker,
                wrist_imu_samples=wrist_imu_samples,
                allow_single_tag=allow_single_tag,
                allow_prediction_output=allow_prediction_output,
                max_prediction_gap_s=max_prediction_gap_s,
                max_position_prediction_gap_s=max_position_prediction_gap_s,
                single_tag_gate_translation_m=single_tag_gate_translation_m,
                single_tag_gate_rotation_deg=single_tag_gate_rotation_deg,
                allow_single_tag_bootstrap=allow_single_tag_bootstrap,
                hand_detector=hand_detector,
                hand_tracker=hand_tracker,
                max_hand_skeletons_per_frame=max_hand_skeletons_per_frame,
                hand_continuity_gate_m=hand_continuity_gate_m,
                hand_continuity_max_gap_s=hand_continuity_max_gap_s,
                hand_multiview_only=hand_multiview_only,
                hand_allow_direct_singleview_fallback=hand_allow_direct_singleview_fallback,
                head_position_visual_alpha=head_position_visual_alpha,
                head_orientation_visual_alpha=head_orientation_visual_alpha,
                max_wrist_prediction_gap_s=max_wrist_prediction_gap_s,
                wrist_position_visual_alpha=wrist_position_visual_alpha,
                wrist_visual_correction_alpha=wrist_visual_correction_alpha,
                wrist_prediction_gate_translation_m=wrist_prediction_gate_translation_m,
                wrist_prediction_gate_rotation_deg=wrist_prediction_gate_rotation_deg,
                wrist_static_gyro_thresh_radps=wrist_static_gyro_thresh_radps,
                wrist_static_accel_std_thresh_mps2=wrist_static_accel_std_thresh_mps2,
                multi_camera_gate_translation_m=multi_camera_gate_translation_m,
                multi_camera_gate_rotation_deg=multi_camera_gate_rotation_deg,
                multi_camera_median_gate_m=multi_camera_median_gate_m,
            )
            for candidate in result["world_candidates"]:
                world_file.write(json.dumps(asdict(candidate), separators=(",", ":")) + "\n")
            for candidate in result["wrist_candidates"]:
                wrist_file.write(json.dumps(candidate, separators=(",", ":")) + "\n")
            for hand in result["hand_skeletons"]:
                hand_file.write(json.dumps(hand, separators=(",", ":")) + "\n")
            for diagnostic in result["camera_diagnostics"]:
                diagnostics_file.write(json.dumps(diagnostic, separators=(",", ":")) + "\n")
                camera_id = str(diagnostic["camera_id"])
                camera_counts[camera_id]["frames"] += 1
                camera_counts[camera_id]["world_detections"] += int(diagnostic["world_detection_count"])
                camera_counts[camera_id]["wrist_detections"] += int(diagnostic["wrist_detection_count"])
                if diagnostic.get("world_pose_used"):
                    camera_counts[camera_id]["world_pose_used"] += 1
                if str(diagnostic.get("world_method") or "").startswith("single_tag"):
                    camera_counts[camera_id]["single_tag_used"] += 1
                if diagnostic.get("rejected_reason"):
                    camera_counts[camera_id]["rejected"] += 1
            counts["groups"] += 1
            counts["world_candidates"] += len(result["world_candidates"])
            counts["wrist_candidates"] += len(result["wrist_candidates"])
            counts["hand_skeletons"] += len(result["hand_skeletons"])
            frame = result["motion_frame"]
            if frame is None:
                continue
            if require_wrist and frame.wrist is None:
                continue
            counts["motion_frames"] += 1
            head_file.write(json.dumps(_head_pose_record(frame), separators=(",", ":")) + "\n")
            motion_file.write(json.dumps(asdict(frame), separators=(",", ":")) + "\n")

    world_candidates_tmp.replace(world_candidates_path)
    wrist_candidates_tmp.replace(wrist_candidates_path)
    hand_skeletons_tmp.replace(hand_skeletons_path)
    camera_diagnostics_tmp.replace(camera_diagnostics_path)
    head_pose_tmp.replace(head_pose_path)
    motion_tmp.replace(motion_path)

    summary = {
        "session_dir": str(session_dir),
        "output_dir": str(output_dir),
        "outputs": {
            "world_anchor_candidates": str(world_candidates_path),
            "wrist_world_candidates": str(wrist_candidates_path),
            "hand_skeletons": str(hand_skeletons_path),
            "camera_diagnostics": str(camera_diagnostics_path),
            "head_pose": str(head_pose_path),
            "motion_frame": str(motion_path),
        },
        "counts": dict(counts),
        "camera_diagnostics": {camera_id: dict(values) for camera_id, values in sorted(camera_counts.items())},
        "tracker": {
            "allow_single_tag": allow_single_tag,
            "use_head_imu": bool(head_imu_samples),
            "use_wrist_imu": bool(wrist_imu_samples),
            "allow_prediction_output": allow_prediction_output,
            "head_imu_samples": len(head_imu_samples),
            "wrist_imu_samples": len(wrist_imu_samples),
            "max_prediction_gap_s": max_prediction_gap_s,
            "max_position_prediction_gap_s": max_position_prediction_gap_s,
            "single_tag_gate_translation_m": single_tag_gate_translation_m,
            "single_tag_gate_rotation_deg": single_tag_gate_rotation_deg,
            "allow_single_tag_bootstrap": allow_single_tag_bootstrap,
            "hands": {
                "enabled": enable_hands,
                "max_hands": max_hands,
                "max_hand_skeletons_per_frame": max_hand_skeletons_per_frame,
                "hand_continuity_gate_m": hand_continuity_gate_m,
                "hand_continuity_max_gap_s": hand_continuity_max_gap_s,
                "hand_multiview_only": hand_multiview_only,
                "hand_allow_direct_singleview_fallback": hand_allow_direct_singleview_fallback,
                "hand_detection_confidence": hand_detection_confidence,
                "hand_tracking_confidence": hand_tracking_confidence,
                "hand_model": str(hand_model) if hand_model else None,
                "projection_note": "All camera hand detections are written to hand_skeletons.jsonl for diagnostics, but motion_frame/RViz keeps only the best skeleton(s) per frame. Hand landmarks use multi-view ray triangulation when possible, then guided single-view fallback from recent multi-view state. They are approximate world points, not a bone-length constrained metric hand model.",
                "selected_updates": hand_tracker.selected_updates,
                "continuity_rejections": hand_tracker.continuity_rejections,
                "missing_updates": hand_tracker.missing_updates,
            },
            "visual_updates": tracker.visual_updates,
            "predicted_updates": tracker.predicted_updates,
            "wrist_visual_updates": wrist_tracker.visual_updates,
            "wrist_predicted_updates": wrist_tracker.predicted_updates,
            "head_fusion": {
                "method": "imu_gyro_prediction_with_aprilgrid_visual_update",
                "head_position_visual_alpha": head_position_visual_alpha,
                "head_orientation_visual_alpha": head_orientation_visual_alpha,
                "max_position_prediction_gap_s": max_position_prediction_gap_s,
                "multi_camera_gate_translation_m": multi_camera_gate_translation_m,
                "multi_camera_gate_rotation_deg": multi_camera_gate_rotation_deg,
                "multi_camera_median_gate_m": multi_camera_median_gate_m,
                "note": "Inspired by OpenVINS' propagate-then-update structure: IMU gyro carries short-term orientation; AprilGrid visual candidates correct absolute position and slow orientation drift.",
            },
            "wrist_fusion": {
                "method": "wrist_gyro_prediction_with_apriltag_visual_update",
                "max_wrist_prediction_gap_s": max_wrist_prediction_gap_s,
                "wrist_position_visual_alpha": wrist_position_visual_alpha,
                "wrist_visual_correction_alpha": wrist_visual_correction_alpha,
                "wrist_prediction_gate_translation_m": wrist_prediction_gate_translation_m,
                "wrist_prediction_gate_rotation_deg": wrist_prediction_gate_rotation_deg,
                "wrist_static_gyro_thresh_radps": wrist_static_gyro_thresh_radps,
                "wrist_static_accel_std_thresh_mps2": wrist_static_accel_std_thresh_mps2,
                "note": "Wrist gyro carries short-term wrist orientation; wrist AprilTags correct absolute wrist pose and are gated against the predicted state.",
            },
        },
        "world_tags": {
            "path": str(world_tags_path),
            "tag_family": world_tags.tag_family,
            "tag_ids": sorted(world_tags.tags),
        },
        "bracelet": {
            "path": str(bracelet_path),
            "tag_family": bracelet.tag_family,
            "tag_ids": sorted(bracelet.tag_to_wrist),
        },
        "next_commands": {
            "rviz_replay": (
                "cd ros2_ws\n"
                "source /opt/ros/humble/setup.bash\n"
                "source install/setup.bash\n"
                "ros2 launch vimas_motion_bringup motion_replay_rviz.launch.py "
                f"motion_jsonl:={motion_path.resolve()}"
            )
        },
    }
    summary_path = output_dir / "world_anchor_summary.json"
    summary_tmp = summary_path.with_name(summary_path.name + ".tmp")
    summary_tmp.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary_tmp.replace(summary_path)
    if hand_detector:
        hand_detector.close()
    return summary


def estimate_world_motion_frame(
    *,
    frame_records: list[dict],
    session_dir: Path,
    camera_calibrations: dict[str, CameraCalibration],
    world_tags: WorldTagConfig,
    bracelet: BraceletConfig,
    world_detector: AprilTagDetector,
    wrist_detector: AprilTagDetector,
    max_reprojection_error_px: float,
    tracker: TrackerState | None = None,
    head_imu_samples: list[ImuSampleLite] | None = None,
    wrist_tracker: TrackerState | None = None,
    wrist_imu_samples: list[ImuSampleLite] | None = None,
    allow_single_tag: bool = True,
    allow_prediction_output: bool = False,
    max_prediction_gap_s: float = 1.0,
    max_position_prediction_gap_s: float = 0.25,
    single_tag_gate_translation_m: float = 0.12,
    single_tag_gate_rotation_deg: float = 35.0,
    allow_single_tag_bootstrap: bool = False,
    hand_detector: HandKeypointDetector | None = None,
    hand_tracker: HandTrackerState | None = None,
    max_hand_skeletons_per_frame: int = 1,
    hand_continuity_gate_m: float = 0.12,
    hand_continuity_max_gap_s: float = 0.25,
    hand_multiview_only: bool = False,
    hand_allow_direct_singleview_fallback: bool = False,
    head_position_visual_alpha: float = 0.75,
    head_orientation_visual_alpha: float = 0.12,
    max_wrist_prediction_gap_s: float = 1.0,
    wrist_position_visual_alpha: float = 0.75,
    wrist_visual_correction_alpha: float = 0.35,
    wrist_prediction_gate_translation_m: float = 0.20,
    wrist_prediction_gate_rotation_deg: float = 60.0,
    wrist_static_gyro_thresh_radps: float = 0.18,
    wrist_static_accel_std_thresh_mps2: float = 0.25,
    multi_camera_gate_translation_m: float = 0.12,
    multi_camera_gate_rotation_deg: float = 25.0,
    multi_camera_median_gate_m: float = 0.08,
) -> dict:
    import cv2

    world_candidates: list[WorldCandidate] = []
    world_weighted: list[tuple[RigidTransform, float]] = []
    world_pose_weighted: list[tuple[WorldBoardPose, float]] = []
    wrist_weighted_head: list[tuple[RigidTransform, float]] = []
    wrist_world_candidates: list[dict] = []
    hand_skeletons: list[dict] = []
    camera_diagnostics: list[dict] = []
    timestamps_mono = [int(record["timestamp_monotonic_ns"]) for record in frame_records]
    timestamps_unix = [int(record["timestamp_unix_ns"]) for record in frame_records]
    frame_timestamp_mono = int(sum(timestamps_mono) / len(timestamps_mono))
    frame_timestamp_unix = int(sum(timestamps_unix) / len(timestamps_unix))
    predicted_head = _predict_tracker_pose(
        tracker,
        frame_timestamp_mono,
        head_imu_samples or [],
        max_prediction_gap_s=max_prediction_gap_s,
    )
    prediction_dt_s = _tracker_prediction_dt_s(tracker, frame_timestamp_mono) if predicted_head is not None else None
    prediction_has_imu = _has_imu_samples_between(
        head_imu_samples or [],
        tracker.timestamp_monotonic_ns if tracker is not None else None,
        frame_timestamp_mono,
    ) if predicted_head is not None else False
    predicted_wrist = _predict_tracker_pose(
        wrist_tracker,
        frame_timestamp_mono,
        wrist_imu_samples or [],
        max_prediction_gap_s=max_wrist_prediction_gap_s,
        use_velocity=False,
    )
    wrist_prediction_dt_s = _tracker_prediction_dt_s(wrist_tracker, frame_timestamp_mono) if predicted_wrist is not None else None
    wrist_prediction_has_imu = _has_imu_samples_between(
        wrist_imu_samples or [],
        wrist_tracker.timestamp_monotonic_ns if wrist_tracker is not None else None,
        frame_timestamp_mono,
    ) if predicted_wrist is not None else False
    wrist_motion_stats = _imu_motion_stats(
        wrist_imu_samples or [],
        wrist_tracker.timestamp_monotonic_ns if wrist_tracker is not None else None,
        frame_timestamp_mono,
    )
    wrist_imu_static = _imu_stats_are_static(
        wrist_motion_stats,
        gyro_thresh_radps=wrist_static_gyro_thresh_radps,
        accel_std_thresh_mps2=wrist_static_accel_std_thresh_mps2,
    )

    for frame_record in frame_records:
        camera_id = frame_record["camera_id"]
        diagnostic = {
            "group_id": int(frame_record["group_id"]),
            "camera_id": camera_id,
            "timestamp_monotonic_ns": int(frame_record["timestamp_monotonic_ns"]),
            "timestamp_unix_ns": int(frame_record["timestamp_unix_ns"]),
            "world_detection_count": 0,
            "wrist_detection_count": 0,
            "world_pose_used": False,
            "world_method": None,
            "world_reprojection_error_px": None,
            "world_source_tag_ids": [],
            "rejected_reason": None,
        }
        calibration = camera_calibrations.get(camera_id)
        if calibration is None:
            diagnostic["rejected_reason"] = "missing_camera_calibration"
            camera_diagnostics.append(diagnostic)
            continue
        image_path = session_dir / frame_record["image_path"]
        image = cv2.imread(str(image_path))
        if image is None:
            diagnostic["rejected_reason"] = "missing_image"
            camera_diagnostics.append(diagnostic)
            continue
        frame_width = int(frame_record.get("width") or image.shape[1])
        frame_height = int(frame_record.get("height") or image.shape[0])
        camera_matrix = _scaled_camera_matrix(calibration, frame_width, frame_height)

        world_detections = [
            detection for detection in world_detector.detect(image) if detection.tag_id in world_tags.tags
        ]
        wrist_detections = [
            detection for detection in wrist_detector.detect(image) if not bracelet.tag_to_wrist or detection.tag_id in bracelet.tag_to_wrist
        ]
        diagnostic["world_detection_count"] = len(world_detections)
        diagnostic["wrist_detection_count"] = len(wrist_detections)

        camera_world_poses: list[WorldBoardPose] = []
        camera_wrist_poses: list[tuple[RigidTransform, float]] = []
        world_pose, rejected_reason = _estimate_world_pose(
            world_detections,
            world_tags,
            calibration,
            camera_matrix,
            predicted_head=predicted_head,
            allow_single_tag=allow_single_tag,
            single_tag_gate_translation_m=single_tag_gate_translation_m,
            single_tag_gate_rotation_deg=single_tag_gate_rotation_deg,
            allow_single_tag_bootstrap=allow_single_tag_bootstrap,
        )
        if world_pose is not None and world_pose.reprojection_error_px <= max_reprojection_error_px:
            weight = float(world_pose.source_corner_count) / max(world_pose.reprojection_error_px, 0.25)
            if world_pose.method.startswith("single_tag"):
                weight *= 0.25
            world_weighted.append((world_pose.T_W_H, weight))
            world_pose_weighted.append((world_pose, weight))
            camera_world_poses.append(world_pose)
            diagnostic["world_pose_used"] = True
            diagnostic["world_method"] = world_pose.method
            diagnostic["world_reprojection_error_px"] = world_pose.reprojection_error_px
            diagnostic["world_source_tag_ids"] = world_pose.source_tag_ids
            world_candidates.append(
                WorldCandidate(
                    group_id=int(frame_record["group_id"]),
                    camera_id=camera_id,
                    tag_id=None,
                    source_tag_ids=world_pose.source_tag_ids,
                    source_corner_count=world_pose.source_corner_count,
                    timestamp_monotonic_ns=int(frame_record["timestamp_monotonic_ns"]),
                    timestamp_unix_ns=int(frame_record["timestamp_unix_ns"]),
                    method=world_pose.method,
                    reprojection_error_px=world_pose.reprojection_error_px,
                    weight=weight,
                    prediction_translation_error_m=world_pose.prediction_translation_error_m,
                    prediction_rotation_error_deg=world_pose.prediction_rotation_error_deg,
                    T_W_H=transform_to_dict(world_pose.T_W_H),
                    T_W_C=transform_to_dict(world_pose.T_W_C),
                    corners=world_pose.corners,
                )
            )
        else:
            diagnostic["rejected_reason"] = rejected_reason or (
                "high_reprojection_error" if world_pose is not None else "no_world_pose"
            )

        for detection in wrist_detections:
            tag_pose = _safe_estimate_pose(detection, bracelet.tag_size_m, calibration, camera_matrix)
            if tag_pose is None or tag_pose.reprojection_error_px > max_reprojection_error_px:
                continue
            T_H_B = _tag_pose_to_wrist_pose(tag_pose, calibration, bracelet)
            weight = 1.0 / max(tag_pose.reprojection_error_px, 0.25)
            wrist_weighted_head.append((T_H_B, weight))
            if camera_world_poses:
                T_C_B = _tag_pose_to_camera_wrist_pose(tag_pose, bracelet)
                camera_wrist_poses.append((T_C_B, weight))
                for world_pose in camera_world_poses:
                    T_W_B = world_pose.T_W_C @ T_C_B
                    wrist_world_candidates.append(
                        {
                            "group_id": int(frame_record["group_id"]),
                            "camera_id": camera_id,
                            "tag_id": detection.tag_id,
                            "timestamp_monotonic_ns": int(frame_record["timestamp_monotonic_ns"]),
                            "timestamp_unix_ns": int(frame_record["timestamp_unix_ns"]),
                            "reprojection_error_px": tag_pose.reprojection_error_px,
                            "world_reprojection_error_px": world_pose.reprojection_error_px,
                            "world_source_tag_ids": world_pose.source_tag_ids,
                            "T_W_B": transform_to_dict(T_W_B),
                            "corners": detection.corners,
                        }
                    )
        if hand_detector is not None and camera_world_poses and camera_wrist_poses:
            hand_skeletons.extend(
                _estimate_hand_skeletons(
                    hand_detector=hand_detector,
                    image=image,
                    frame_record=frame_record,
                    camera_matrix=camera_matrix,
                    calibration=calibration,
                    world_pose=camera_world_poses[0],
                    T_C_B=average_transforms(camera_wrist_poses),
                )
            )
        camera_diagnostics.append(diagnostic)

    used_prediction_only = False
    if world_weighted:
        visual_T_W_H, head_fusion = _robust_fuse_world_head_pose(
            world_pose_weighted,
            predicted_head=predicted_head,
            prediction_translation_gate_m=multi_camera_gate_translation_m,
            prediction_rotation_gate_deg=multi_camera_gate_rotation_deg,
            median_translation_gate_m=multi_camera_median_gate_m,
        )
        T_W_H, correction_debug = _apply_imu_aided_head_correction(
            visual_T_W_H,
            predicted_head,
            prediction_dt_s=prediction_dt_s,
            prediction_has_imu=prediction_has_imu,
            position_visual_alpha=head_position_visual_alpha,
            orientation_visual_alpha=head_orientation_visual_alpha,
            max_position_prediction_gap_s=max_position_prediction_gap_s,
        )
        head_fusion.update(correction_debug)
        _update_tracker_from_visual(tracker, T_W_H, frame_timestamp_mono)
    elif predicted_head is not None and allow_prediction_output:
        T_W_H = predicted_head
        used_prediction_only = True
        head_fusion = {
            "mode": "prediction_only",
            "visual_candidate_count": 0,
            "used_visual_candidate_count": 0,
        }
        if tracker is not None:
            tracker.predicted_updates += 1
    else:
        return {
            "world_candidates": world_candidates,
            "wrist_candidates": wrist_world_candidates,
            "hand_skeletons": hand_skeletons,
            "camera_diagnostics": camera_diagnostics,
            "motion_frame": None,
        }
    T_H_B = average_transforms(wrist_weighted_head) if wrist_weighted_head else None
    visual_T_W_B = None
    wrist_visual_source = None
    if wrist_world_candidates:
        visual_T_W_B = average_transforms(
            [
                (
                    _dict_to_transform(candidate["T_W_B"]),
                    1.0
                    / max(
                        float(candidate["reprojection_error_px"]) + float(candidate["world_reprojection_error_px"]),
                        0.25,
                    ),
                )
                for candidate in wrist_world_candidates
            ]
        )
        wrist_visual_source = "board_camera_wrist"
    elif T_H_B is not None:
        visual_T_W_B = T_W_H @ T_H_B
        wrist_visual_source = "head_relative_fallback"
    if visual_T_W_B is not None:
        T_W_B, wrist_fusion = _apply_wrist_imu_aided_correction(
            visual_T_W_B,
            predicted_wrist,
            prediction_dt_s=wrist_prediction_dt_s,
            prediction_has_imu=wrist_prediction_has_imu,
            position_visual_alpha=wrist_position_visual_alpha,
            visual_correction_alpha=wrist_visual_correction_alpha,
            gate_translation_m=wrist_prediction_gate_translation_m,
            gate_rotation_deg=wrist_prediction_gate_rotation_deg,
            wrist_imu_static=wrist_imu_static,
        )
        wrist_fusion["visual_source"] = wrist_visual_source
        wrist_fusion["wrist_imu_motion"] = wrist_motion_stats
        if wrist_fusion["mode"] == "visual_update" and wrist_tracker is not None:
            _update_tracker_from_visual(wrist_tracker, T_W_B, frame_timestamp_mono)
        elif wrist_tracker is not None and predicted_wrist is not None:
            _update_tracker_from_prediction(wrist_tracker, T_W_B, frame_timestamp_mono)
    elif predicted_wrist is not None:
        T_W_B = predicted_wrist
        wrist_fusion = {
            "mode": "prediction_only",
            "prediction_available": True,
            "prediction_dt_s": wrist_prediction_dt_s,
            "position_prediction_used": False,
            "translation_source": "last_visual_hold",
            "orientation_prediction_used": wrist_prediction_has_imu,
            "wrist_imu_static": wrist_imu_static,
            "wrist_imu_motion": wrist_motion_stats,
        }
        if wrist_tracker is not None:
            _update_tracker_from_prediction(wrist_tracker, T_W_B, frame_timestamp_mono)
    else:
        T_W_B = None
        wrist_fusion = {
            "mode": "unavailable",
            "prediction_available": False,
            "visual_available": False,
        }
    triangulated_hands = _triangulate_multiview_hand_skeletons(hand_skeletons, T_W_B)
    guided_hands: list[dict] = []
    if triangulated_hands:
        hand_candidates_for_selection = triangulated_hands
    elif hand_multiview_only:
        hand_candidates_for_selection = []
    elif _hand_tracker_recently_used_multiview(hand_tracker, frame_timestamp_mono, hand_continuity_max_gap_s):
        guided_hands = _guide_singleview_hands_from_tracker(
            hand_skeletons,
            T_W_B,
            hand_tracker,
            frame_timestamp_mono,
            max_gap_s=hand_continuity_max_gap_s,
        )
        hand_candidates_for_selection = guided_hands
    elif hand_allow_direct_singleview_fallback:
        hand_candidates_for_selection = hand_skeletons
    else:
        hand_candidates_for_selection = []
    selected_hands = _select_hand_skeletons(
        hand_candidates_for_selection,
        T_W_B,
        max_count=max_hand_skeletons_per_frame,
        hand_tracker=hand_tracker,
        timestamp_monotonic_ns=frame_timestamp_mono,
        continuity_gate_m=hand_continuity_gate_m,
        continuity_max_gap_s=hand_continuity_max_gap_s,
    )

    frame = WorldMotionFrame(
        timestamp_unix_ns=frame_timestamp_unix,
        timestamp_monotonic_ns=frame_timestamp_mono,
        timestamp_source="camera_group_average",
        group_id=int(frame_records[0]["group_id"]),
        tracking_state=3 if used_prediction_only else (1 if T_W_B is not None else 2),
        source={
            "world_candidate_count": len(world_candidates),
            "wrist_candidate_count": len(wrist_world_candidates) or len(wrist_weighted_head),
            "hand_skeleton_count": len(hand_skeletons),
            "triangulated_hand_skeleton_count": len(triangulated_hands),
            "guided_hand_skeleton_count": len(guided_hands),
            "selected_hand_skeleton_count": len(selected_hands),
            "selected_hand_method": selected_hands[0].get("source", {}).get("method") if selected_hands else None,
            "head_fusion": head_fusion,
            "wrist_fusion": wrist_fusion,
            "world_tag_ids": sorted({tag_id for candidate in world_candidates for tag_id in candidate.source_tag_ids}),
            "prediction_only": used_prediction_only,
            "head_imu_prediction": predicted_head is not None,
            "wrist_imu_prediction": predicted_wrist is not None,
        },
        head=_rigid_body_state("world", "head", T_W_H),
        wrist=_rigid_body_state("world", "wrist", T_W_B) if T_W_B is not None else None,
        relative={
            "T_H_B": transform_to_dict(T_H_B) if T_H_B is not None else None,
            "method": "desktop_world_apriltag_head_relative_wrist",
        },
        hands=selected_hands,
    )
    return {
        "world_candidates": world_candidates,
        "wrist_candidates": wrist_world_candidates,
        "hand_skeletons": hand_skeletons + triangulated_hands + guided_hands,
        "camera_diagnostics": camera_diagnostics,
        "motion_frame": frame,
    }


def run_live_world_anchor(
    *,
    sources: list[str],
    cameras_path: Path,
    world_tags_path: Path,
    bracelet_path: Path,
    fps: float,
    width: int | None,
    height: int | None,
    output_jsonl: Path | None,
    max_reprojection_error_px: float,
    show: bool,
    ros_publish: bool = False,
    fixed_frame: str = "world",
    head_frame: str = "head",
    wrist_frame: str = "wrist",
    max_path_length: int = 5000,
    warmup_frames: int = 5,
    max_camera_failures: int = 10,
    enable_hands: bool = False,
    max_hands: int = 2,
    hand_detection_confidence: float = 0.5,
    hand_tracking_confidence: float = 0.5,
    hand_model: Path | None = None,
    max_hand_skeletons_per_frame: int = 1,
    hand_continuity_gate_m: float = 0.12,
    hand_continuity_max_gap_s: float = 0.25,
    hand_multiview_only: bool = False,
    hand_allow_direct_singleview_fallback: bool = False,
    head_imu_live_jsonl: Path | None = None,
    wrist_imu_live_jsonl: Path | None = None,
    live_imu_buffer_s: float = 3.0,
    max_prediction_gap_s: float = 1.0,
    max_wrist_prediction_gap_s: float = 1.0,
    head_position_visual_alpha: float = 0.75,
    head_orientation_visual_alpha: float = 0.12,
    max_position_prediction_gap_s: float = 0.25,
    wrist_position_visual_alpha: float = 0.75,
    wrist_visual_correction_alpha: float = 0.35,
    wrist_prediction_gate_translation_m: float = 0.20,
    wrist_prediction_gate_rotation_deg: float = 60.0,
    wrist_static_gyro_thresh_radps: float = 0.18,
    wrist_static_accel_std_thresh_mps2: float = 0.25,
    multi_camera_gate_translation_m: float = 0.12,
    multi_camera_gate_rotation_deg: float = 25.0,
    multi_camera_median_gate_m: float = 0.08,
) -> None:
    import cv2

    from packages.quad_camera_capture.capture import parse_sources

    camera_sources = parse_sources(sources)
    camera_calibrations = load_camera_calibrations(cameras_path)
    world_tags = load_world_tag_config(world_tags_path)
    bracelet = load_bracelet_config(bracelet_path)
    world_detector = AprilTagDetector(world_tags.tag_family)
    wrist_detector = AprilTagDetector(bracelet.tag_family)
    hand_detector = _make_hand_detector(
        enable_hands=enable_hands,
        max_hands=max_hands,
        hand_detection_confidence=hand_detection_confidence,
        hand_tracking_confidence=hand_tracking_confidence,
        hand_model=hand_model,
    )
    caps = []
    camera_failures = defaultdict(int)
    disabled_cameras: set[str] = set()
    for source in camera_sources:
        cap = cv2.VideoCapture(source.source, cv2.CAP_V4L2)
        if source.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*source.fourcc[:4]))
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 700)
        if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
            cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 700)
        if not cap.isOpened():
            cap.release()
            print(f"{source.camera_id}: failed to open {source.source}")
            continue
        for _ in range(max(0, warmup_frames)):
            cap.grab()
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"{source.camera_id}: opened {source.source} {actual_width}x{actual_height} fps={actual_fps:.1f}")
        caps.append((source, cap))
    if not caps:
        raise RuntimeError("Could not open any camera source.")

    output_file = output_jsonl.open("a", encoding="utf-8") if output_jsonl else None
    ros_publisher = _RosMotionPublisher(
        fixed_frame=fixed_frame,
        head_frame=head_frame,
        wrist_frame=wrist_frame,
        max_path_length=max_path_length,
    ) if ros_publish else None
    period = 1.0 / max(fps, 1e-6)
    group_id = 0
    tracker = TrackerState()
    wrist_tracker = TrackerState()
    hand_tracker = HandTrackerState()
    head_imu_tailer = _LiveImuJsonlTailer(head_imu_live_jsonl) if head_imu_live_jsonl else None
    wrist_imu_tailer = _LiveImuJsonlTailer(wrist_imu_live_jsonl) if wrist_imu_live_jsonl else None
    head_imu_samples: list[ImuSampleLite] = []
    wrist_imu_samples: list[ImuSampleLite] = []
    if show:
        print("OpenCV live preview enabled: 3DMotion desktop AprilTag world map")
    else:
        print("Live preview window disabled. Add --show to open the OpenCV top-down map.")
    if head_imu_tailer:
        print(f"Live head IMU enabled: {head_imu_live_jsonl}")
    if wrist_imu_tailer:
        print(f"Live wrist IMU enabled: {wrist_imu_live_jsonl}")
    try:
        while True:
            start = time.monotonic()
            frame_records = []
            temp_dir = Path("/tmp/3dmotion_live_world_anchor")
            temp_dir.mkdir(parents=True, exist_ok=True)
            active_caps = [(source, cap) for source, cap in caps if source.camera_id not in disabled_cameras]
            for source, cap in active_caps:
                if not cap.grab():
                    camera_failures[source.camera_id] += 1
                    if camera_failures[source.camera_id] >= max_camera_failures:
                        disabled_cameras.add(source.camera_id)
                        print(f"\n{source.camera_id}: disabled after {camera_failures[source.camera_id]} grab failures")
            for source, cap in caps:
                if source.camera_id in disabled_cameras:
                    continue
                ok, frame = cap.retrieve()
                if not ok:
                    camera_failures[source.camera_id] += 1
                    if camera_failures[source.camera_id] >= max_camera_failures:
                        disabled_cameras.add(source.camera_id)
                        print(f"\n{source.camera_id}: disabled after {camera_failures[source.camera_id]} retrieve failures")
                    continue
                camera_failures[source.camera_id] = 0
                image_path = temp_dir / f"{source.camera_id}.jpg"
                if not cv2.imwrite(str(image_path), frame):
                    continue
                frame_records.append(
                    {
                        "group_id": group_id,
                        "camera_id": source.camera_id,
                        "timestamp_unix_ns": time.time_ns(),
                        "timestamp_monotonic_ns": time.monotonic_ns(),
                        "image_path": image_path.name,
                        "width": frame.shape[1],
                        "height": frame.shape[0],
                    }
                )
            if not frame_records:
                print("no live camera frames", end="\r", flush=True)
                group_id += 1
                sleep_s = period - (time.monotonic() - start)
                if sleep_s > 0:
                    time.sleep(sleep_s)
                continue
            if head_imu_tailer:
                head_imu_samples.extend(head_imu_tailer.read_new())
                prune_before_ns = frame_records[0]["timestamp_monotonic_ns"] - int(max(live_imu_buffer_s, max_prediction_gap_s + 0.5) * 1e9)
                if head_imu_samples and head_imu_samples[0].timestamp_monotonic_ns < prune_before_ns:
                    head_imu_samples = [
                        sample for sample in head_imu_samples if sample.timestamp_monotonic_ns >= prune_before_ns
                    ]
            if wrist_imu_tailer:
                wrist_imu_samples.extend(wrist_imu_tailer.read_new())
                prune_before_ns = frame_records[0]["timestamp_monotonic_ns"] - int(max(live_imu_buffer_s, max_wrist_prediction_gap_s + 0.5) * 1e9)
                if wrist_imu_samples and wrist_imu_samples[0].timestamp_monotonic_ns < prune_before_ns:
                    wrist_imu_samples = [
                        sample for sample in wrist_imu_samples if sample.timestamp_monotonic_ns >= prune_before_ns
                    ]
            result = estimate_world_motion_frame(
                    frame_records=frame_records,
                    session_dir=temp_dir,
                    camera_calibrations=camera_calibrations,
                    world_tags=world_tags,
                    bracelet=bracelet,
                    world_detector=world_detector,
                    wrist_detector=wrist_detector,
                    max_reprojection_error_px=max_reprojection_error_px,
                    tracker=tracker,
                    head_imu_samples=head_imu_samples,
                    wrist_tracker=wrist_tracker,
                    wrist_imu_samples=wrist_imu_samples,
                    hand_detector=hand_detector,
                    hand_tracker=hand_tracker,
                    max_hand_skeletons_per_frame=max_hand_skeletons_per_frame,
                    hand_continuity_gate_m=hand_continuity_gate_m,
                    hand_continuity_max_gap_s=hand_continuity_max_gap_s,
                    hand_multiview_only=hand_multiview_only,
                    hand_allow_direct_singleview_fallback=hand_allow_direct_singleview_fallback,
                    max_prediction_gap_s=max_prediction_gap_s,
                    max_wrist_prediction_gap_s=max_wrist_prediction_gap_s,
                    head_position_visual_alpha=head_position_visual_alpha,
                    head_orientation_visual_alpha=head_orientation_visual_alpha,
                    max_position_prediction_gap_s=max_position_prediction_gap_s,
                    wrist_position_visual_alpha=wrist_position_visual_alpha,
                    wrist_visual_correction_alpha=wrist_visual_correction_alpha,
                    wrist_prediction_gate_translation_m=wrist_prediction_gate_translation_m,
                    wrist_prediction_gate_rotation_deg=wrist_prediction_gate_rotation_deg,
                    wrist_static_gyro_thresh_radps=wrist_static_gyro_thresh_radps,
                    wrist_static_accel_std_thresh_mps2=wrist_static_accel_std_thresh_mps2,
                    multi_camera_gate_translation_m=multi_camera_gate_translation_m,
                    multi_camera_gate_rotation_deg=multi_camera_gate_rotation_deg,
                    multi_camera_median_gate_m=multi_camera_median_gate_m,
                )
            frame = result["motion_frame"]
            if frame is not None:
                _print_live_status(frame)
                if show:
                    show = _try_draw_live_map(cv2, frame)
                if ros_publisher:
                    ros_publisher.publish(frame)
                if output_file:
                    output_file.write(json.dumps(asdict(frame), separators=(",", ":")) + "\n")
                    output_file.flush()
            else:
                print("world tag not visible", end="\r", flush=True)
                if show:
                    show = _try_draw_live_map(cv2, None)
            group_id += 1
            sleep_s = period - (time.monotonic() - start)
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        if output_file:
            output_file.close()
        if ros_publisher:
            ros_publisher.close()
        if hand_detector:
            hand_detector.close()
        for _source, cap in caps:
            cap.release()
        if show:
            try:
                cv2.destroyWindow("3DMotion desktop AprilTag world map")
            except cv2.error:
                pass


class _RosMotionPublisher:
    def __init__(
        self,
        *,
        fixed_frame: str,
        head_frame: str,
        wrist_frame: str,
        max_path_length: int,
    ) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import Point, PoseStamped, TransformStamped
            from nav_msgs.msg import Path as PathMsg
            from tf2_ros import TransformBroadcaster
            from visualization_msgs.msg import Marker, MarkerArray
        except ImportError as exc:
            raise RuntimeError(
                "ROS publishing requires ROS2 Python packages. Run after `source /opt/ros/humble/setup.bash`."
            ) from exc

        self.rclpy = rclpy
        self.PoseStamped = PoseStamped
        self.Point = Point
        self.TransformStamped = TransformStamped
        self.PathMsg = PathMsg
        self.Marker = Marker
        self.MarkerArray = MarkerArray
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True
        else:
            self._owns_rclpy = False

        self.node = rclpy.create_node("world_anchor_live_visualizer")
        self.fixed_frame = fixed_frame
        self.head_frame = head_frame
        self.wrist_frame = wrist_frame
        self.max_path_length = max_path_length
        self.head_pose_pub = self.node.create_publisher(PoseStamped, "/motion/head_pose", 10)
        self.wrist_pose_pub = self.node.create_publisher(PoseStamped, "/motion/wrist_pose", 10)
        self.head_path_pub = self.node.create_publisher(PathMsg, "/motion/head_path", 10)
        self.wrist_path_pub = self.node.create_publisher(PathMsg, "/motion/wrist_path", 10)
        self.hand_marker_pub = self.node.create_publisher(MarkerArray, "/motion/hand_skeleton", 10)
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.head_path = PathMsg()
        self.head_path.header.frame_id = fixed_frame
        self.wrist_path = PathMsg()
        self.wrist_path.header.frame_id = fixed_frame
        self.node.get_logger().info("Publishing live world-anchor motion to /motion/* and /tf")

    def publish(self, frame: WorldMotionFrame) -> None:
        stamp = self.node.get_clock().now().to_msg()
        head_pose = self._pose_msg(frame.head, stamp)
        self.head_pose_pub.publish(head_pose)
        self._append_path(self.head_path, head_pose)
        self.head_path_pub.publish(self.head_path)
        self.tf_broadcaster.sendTransform(self._transform_msg(self.fixed_frame, self.head_frame, frame.head, stamp))

        if frame.wrist is not None:
            wrist_pose = self._pose_msg(frame.wrist, stamp)
            self.wrist_pose_pub.publish(wrist_pose)
            self._append_path(self.wrist_path, wrist_pose)
            self.wrist_path_pub.publish(self.wrist_path)
            self.tf_broadcaster.sendTransform(self._transform_msg(self.fixed_frame, self.wrist_frame, frame.wrist, stamp))
        self.hand_marker_pub.publish(self._hand_markers(frame.hands or [], stamp))
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self) -> None:
        self.node.destroy_node()
        if self._owns_rclpy:
            self.rclpy.shutdown()

    def _pose_msg(self, state: dict, stamp):
        msg = self.PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.fixed_frame
        position = state["position"]
        quat = state["orientation_wxyz"]
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = float(quat[0])
        msg.pose.orientation.x = float(quat[1])
        msg.pose.orientation.y = float(quat[2])
        msg.pose.orientation.z = float(quat[3])
        return msg

    def _transform_msg(self, frame_id: str, child_frame_id: str, state: dict, stamp):
        msg = self.TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        position = state["position"]
        quat = state["orientation_wxyz"]
        msg.transform.translation.x = float(position[0])
        msg.transform.translation.y = float(position[1])
        msg.transform.translation.z = float(position[2])
        msg.transform.rotation.w = float(quat[0])
        msg.transform.rotation.x = float(quat[1])
        msg.transform.rotation.y = float(quat[2])
        msg.transform.rotation.z = float(quat[3])
        return msg

    def _append_path(self, path, pose) -> None:
        path.header.stamp = pose.header.stamp
        path.header.frame_id = self.fixed_frame
        path.poses.append(pose)
        if self.max_path_length > 0 and len(path.poses) > self.max_path_length:
            path.poses = path.poses[-self.max_path_length :]

    def _hand_markers(self, hands: list[dict], stamp):
        array = self.MarkerArray()
        clear = self.Marker()
        clear.action = self.Marker.DELETEALL
        array.markers.append(clear)
        marker_id = 1
        for hand in hands:
            points_by_index = {
                int(item["index"]): item["world"]
                for item in hand.get("landmarks", [])
                if item.get("world") is not None
            }
            if not points_by_index:
                continue

            lines = self.Marker()
            lines.header.frame_id = self.fixed_frame
            lines.header.stamp = stamp
            lines.ns = "hand_skeleton"
            lines.id = marker_id
            marker_id += 1
            lines.type = self.Marker.LINE_LIST
            lines.action = self.Marker.ADD
            lines.scale.x = 0.006
            lines.color.r = 0.1
            lines.color.g = 0.8
            lines.color.b = 1.0
            lines.color.a = 1.0
            for start, end in hand.get("connections", []):
                if start in points_by_index and end in points_by_index:
                    lines.points.append(self._point(points_by_index[start]))
                    lines.points.append(self._point(points_by_index[end]))
            array.markers.append(lines)

            joints = self.Marker()
            joints.header.frame_id = self.fixed_frame
            joints.header.stamp = stamp
            joints.ns = "hand_joints"
            joints.id = marker_id
            marker_id += 1
            joints.type = self.Marker.SPHERE_LIST
            joints.action = self.Marker.ADD
            joints.scale.x = 0.018
            joints.scale.y = 0.018
            joints.scale.z = 0.018
            joints.color.r = 1.0
            joints.color.g = 0.85
            joints.color.b = 0.15
            joints.color.a = 1.0
            for index in sorted(points_by_index):
                joints.points.append(self._point(points_by_index[index]))
            array.markers.append(joints)
        return array

    def _point(self, values: list[float]):
        point = self.Point()
        point.x = float(values[0])
        point.y = float(values[1])
        point.z = float(values[2])
        return point


def _estimate_world_board_pose(
    detections: list[TagDetection],
    world_tags: WorldTagConfig,
    calibration: CameraCalibration,
    camera_matrix: np.ndarray,
) -> WorldBoardPose | None:
    if not detections:
        return None

    import cv2

    object_points_by_tag = []
    image_points_by_tag = []
    corners = []
    source_tag_ids = []
    for detection in detections:
        tag = world_tags.tags.get(detection.tag_id)
        if tag is None:
            continue
        tag_points = tag_object_points(tag.tag_size_m)
        world_points = (tag.T_W_T.rotation @ tag_points.T).T + tag.T_W_T.translation
        object_points_by_tag.append(world_points)
        tag_image_points = np.asarray(detection.corners, dtype=np.float64).reshape(4, 2)
        image_points_by_tag.append(tag_image_points)
        corners.extend(tag_image_points.tolist())
        source_tag_ids.append(detection.tag_id)

    if len(object_points_by_tag) < 2:
        return None

    object_points = np.vstack(object_points_by_tag).astype(np.float64)
    image_points = np.vstack(image_points_by_tag).astype(np.float64)
    if len(object_points) < 4:
        return None

    pnp_image_points, pnp_dist_coeffs = _pnp_points_and_distortion(
        image_points,
        camera_matrix,
        calibration.distortion,
        calibration.distortion_model,
        calibration.xi if calibration.uses_omni_projection else None,
    )
    uses_omni = _is_omni_distortion(
        calibration.distortion_model,
        calibration.xi if calibration.uses_omni_projection else None,
    )
    pnp_camera_matrix = np.eye(3, dtype=np.float64) if uses_omni else camera_matrix

    solve_flags = [
        getattr(cv2, "SOLVEPNP_IPPE", cv2.SOLVEPNP_ITERATIVE),
        cv2.SOLVEPNP_ITERATIVE,
        cv2.SOLVEPNP_EPNP,
    ]
    candidates: list[tuple[float, np.ndarray, np.ndarray]] = []
    for flag in solve_flags:
        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                pnp_image_points,
                pnp_camera_matrix,
                pnp_dist_coeffs,
                flags=flag,
            )
        except cv2.error:
            continue
        if not success:
            continue
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
        if float(tvec[2, 0]) <= 0:
            continue
        error = reprojection_error(
            object_points,
            image_points,
            rvec,
            tvec,
            camera_matrix,
            calibration.distortion,
            calibration.distortion_model,
            calibration.xi if calibration.uses_omni_projection else None,
        )
        candidates.append((error, rvec, tvec))

    if not candidates:
        return None

    error, rvec, tvec = min(candidates, key=lambda item: item[0])
    rotation, _ = cv2.Rodrigues(rvec)
    T_C_W = RigidTransform(rotation=rotation, translation=tvec.reshape(3))
    T_W_C = T_C_W.inverse()
    T_W_H = T_W_C @ calibration.T_H_C.inverse()
    return WorldBoardPose(
        T_W_C=T_W_C,
        T_W_H=T_W_H,
        reprojection_error_px=error,
        source_tag_ids=sorted(set(source_tag_ids)),
        source_corner_count=int(len(object_points)),
        corners=corners,
        method="board_pnp",
    )


def _estimate_world_pose(
    detections: list[TagDetection],
    world_tags: WorldTagConfig,
    calibration: CameraCalibration,
    camera_matrix: np.ndarray,
    *,
    predicted_head: RigidTransform | None,
    allow_single_tag: bool,
    single_tag_gate_translation_m: float,
    single_tag_gate_rotation_deg: float,
    allow_single_tag_bootstrap: bool,
) -> tuple[WorldBoardPose | None, str | None]:
    if len(detections) >= 2:
        board_pose = _estimate_world_board_pose(detections, world_tags, calibration, camera_matrix)
        if board_pose is not None:
            if predicted_head is not None:
                _attach_prediction_error(board_pose, predicted_head)
            return board_pose, None
        return None, "board_pnp_failed"

    if not detections:
        return None, "no_world_tags"
    if not allow_single_tag:
        return None, "single_tag_disabled"

    candidates = _estimate_single_world_tag_candidates(detections[0], world_tags, calibration, camera_matrix)
    if not candidates:
        return None, "single_tag_pnp_failed"
    for candidate in candidates:
        if predicted_head is not None:
            _attach_prediction_error(candidate, predicted_head)

    if predicted_head is None:
        if not allow_single_tag_bootstrap:
            return None, "single_tag_needs_prediction"
        selected = min(candidates, key=lambda candidate: candidate.reprojection_error_px)
        selected.method = "single_tag_bootstrap"
        return selected, None

    gated = [
        candidate
        for candidate in candidates
        if (candidate.prediction_translation_error_m or 0.0) <= single_tag_gate_translation_m
        and (candidate.prediction_rotation_error_deg or 0.0) <= single_tag_gate_rotation_deg
    ]
    if not gated:
        return None, "single_tag_prediction_gate"

    def score(candidate: WorldBoardPose) -> float:
        return (
            candidate.reprojection_error_px
            + 20.0 * float(candidate.prediction_translation_error_m or 0.0)
            + 0.02 * float(candidate.prediction_rotation_error_deg or 0.0)
        )

    return min(gated, key=score), None


def _estimate_single_world_tag_candidates(
    detection: TagDetection,
    world_tags: WorldTagConfig,
    calibration: CameraCalibration,
    camera_matrix: np.ndarray,
) -> list[WorldBoardPose]:
    tag = world_tags.tags.get(detection.tag_id)
    if tag is None:
        return []

    import cv2

    object_points = tag_object_points(tag.tag_size_m)
    image_points = np.asarray(detection.corners, dtype=np.float64).reshape(4, 2)
    pnp_image_points, pnp_dist_coeffs = _pnp_points_and_distortion(
        image_points,
        camera_matrix,
        calibration.distortion,
        calibration.distortion_model,
        calibration.xi if calibration.uses_omni_projection else None,
    )
    uses_omni = _is_omni_distortion(
        calibration.distortion_model,
        calibration.xi if calibration.uses_omni_projection else None,
    )
    pnp_camera_matrix = np.eye(3, dtype=np.float64) if uses_omni else camera_matrix

    try:
        ok, rvecs, tvecs = cv2.solvePnPGeneric(
            object_points,
            pnp_image_points,
            pnp_camera_matrix,
            pnp_dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )[:3]
    except cv2.error:
        ok, rvecs, tvecs = False, [], []

    if not ok:
        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                pnp_image_points,
                pnp_camera_matrix,
                pnp_dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except cv2.error:
            success, rvec, tvec = False, None, None
        rvecs = [rvec] if success else []
        tvecs = [tvec] if success else []

    candidates = []
    for rvec, tvec in zip(rvecs, tvecs):
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
        if float(tvec[2, 0]) <= 0:
            continue
        error = reprojection_error(
            object_points,
            image_points,
            rvec,
            tvec,
            camera_matrix,
            calibration.distortion,
            calibration.distortion_model,
            calibration.xi if calibration.uses_omni_projection else None,
        )
        rotation, _ = cv2.Rodrigues(rvec)
        T_C_T = RigidTransform(rotation=rotation, translation=tvec.reshape(3))
        T_W_C = tag.T_W_T @ T_C_T.inverse()
        T_W_H = T_W_C @ calibration.T_H_C.inverse()
        candidates.append(
            WorldBoardPose(
                T_W_C=T_W_C,
                T_W_H=T_W_H,
                reprojection_error_px=error,
                source_tag_ids=[detection.tag_id],
                source_corner_count=4,
                corners=image_points.tolist(),
                method="single_tag",
            )
        )
    return candidates


def _attach_prediction_error(candidate: WorldBoardPose, predicted_head: RigidTransform) -> None:
    candidate.prediction_translation_error_m = float(np.linalg.norm(candidate.T_W_H.translation - predicted_head.translation))
    candidate.prediction_rotation_error_deg = _rotation_error_deg(candidate.T_W_H.rotation, predicted_head.rotation)


def _robust_fuse_world_head_pose(
    candidates: list[tuple[WorldBoardPose, float]],
    *,
    predicted_head: RigidTransform | None,
    prediction_translation_gate_m: float,
    prediction_rotation_gate_deg: float,
    median_translation_gate_m: float,
) -> tuple[RigidTransform, dict]:
    if not candidates:
        raise ValueError("No world head candidates to fuse.")

    original_count = len(candidates)
    gated = candidates
    prediction_rejected = 0
    if predicted_head is not None:
        prediction_gated = [
            (pose, weight)
            for pose, weight in candidates
            if (
                pose.prediction_translation_error_m is None
                or pose.prediction_translation_error_m <= prediction_translation_gate_m
            )
            and (
                pose.prediction_rotation_error_deg is None
                or pose.prediction_rotation_error_deg <= prediction_rotation_gate_deg
            )
        ]
        if prediction_gated:
            prediction_rejected = len(candidates) - len(prediction_gated)
            gated = prediction_gated

    median_rejected = 0
    if len(gated) >= 3 and median_translation_gate_m > 0:
        positions = np.stack([pose.T_W_H.translation for pose, _weight in gated], axis=0)
        median_position = np.median(positions, axis=0)
        median_gated = [
            (pose, weight)
            for pose, weight in gated
            if float(np.linalg.norm(pose.T_W_H.translation - median_position)) <= median_translation_gate_m
        ]
        if median_gated:
            median_rejected = len(gated) - len(median_gated)
            gated = median_gated

    fused = average_transforms([(pose.T_W_H, weight) for pose, weight in gated])
    return fused, {
        "mode": "visual_update",
        "visual_candidate_count": original_count,
        "used_visual_candidate_count": len(gated),
        "prediction_rejected": prediction_rejected,
        "median_rejected": median_rejected,
        "prediction_available": predicted_head is not None,
    }


def _apply_imu_aided_head_correction(
    visual_pose: RigidTransform,
    predicted_head: RigidTransform | None,
    *,
    prediction_dt_s: float | None,
    prediction_has_imu: bool,
    position_visual_alpha: float,
    orientation_visual_alpha: float,
    max_position_prediction_gap_s: float,
) -> tuple[RigidTransform, dict]:
    if predicted_head is None:
        return visual_pose, {
            "prediction_dt_s": None,
            "position_prediction_used": False,
            "orientation_prediction_used": False,
        }
    pos_alpha = float(np.clip(position_visual_alpha, 0.0, 1.0))
    ori_alpha = float(np.clip(orientation_visual_alpha, 0.0, 1.0))
    max_pos_gap = max(float(max_position_prediction_gap_s), 0.0)
    position_prediction_used = prediction_dt_s is not None and prediction_dt_s <= max_pos_gap
    orientation_prediction_used = bool(prediction_has_imu)
    if position_prediction_used:
        translation = predicted_head.translation * (1.0 - pos_alpha) + visual_pose.translation * pos_alpha
    else:
        translation = visual_pose.translation.copy()
        pos_alpha = 1.0
    if orientation_prediction_used:
        rotation = _slerp_rotation(predicted_head.rotation, visual_pose.rotation, ori_alpha)
    else:
        rotation = visual_pose.rotation.copy()
        ori_alpha = 1.0
    return RigidTransform(rotation=rotation, translation=translation), {
        "prediction_dt_s": prediction_dt_s,
        "position_visual_alpha_used": pos_alpha,
        "orientation_visual_alpha_used": ori_alpha,
        "position_prediction_used": position_prediction_used,
        "orientation_prediction_used": orientation_prediction_used,
        "max_position_prediction_gap_s": max_position_prediction_gap_s,
    }


def _apply_wrist_imu_aided_correction(
    visual_pose: RigidTransform,
    predicted_wrist: RigidTransform | None,
    *,
    prediction_dt_s: float | None,
    prediction_has_imu: bool,
    position_visual_alpha: float,
    visual_correction_alpha: float,
    gate_translation_m: float,
    gate_rotation_deg: float,
    wrist_imu_static: bool,
) -> tuple[RigidTransform, dict]:
    if predicted_wrist is None:
        return visual_pose, {
            "mode": "visual_update",
            "prediction_available": False,
            "visual_available": True,
            "translation_source": "wrist_visual",
            "orientation_source": "wrist_visual",
            "wrist_imu_static": wrist_imu_static,
        }

    translation_error_m = float(np.linalg.norm(visual_pose.translation - predicted_wrist.translation))
    rotation_error_deg = _rotation_error_deg(visual_pose.rotation, predicted_wrist.rotation)
    gate_translation_m = max(float(gate_translation_m), 0.0)
    gate_rotation_deg = max(float(gate_rotation_deg), 0.0)
    translation_rejected = gate_translation_m > 0.0 and translation_error_m > gate_translation_m
    rotation_rejected = gate_rotation_deg > 0.0 and rotation_error_deg > gate_rotation_deg

    pos_alpha = _adaptive_visual_alpha(
        position_visual_alpha,
        residual=translation_error_m,
        soft_gate=gate_translation_m,
        min_alpha=0.20,
    )
    static_translation_hold = bool(wrist_imu_static)
    if static_translation_hold:
        pos_alpha = min(pos_alpha, 0.05)
    translation = predicted_wrist.translation * (1.0 - pos_alpha) + visual_pose.translation * pos_alpha
    translation_source = "wrist_static_imu_hold_with_visual_nudge" if static_translation_hold else "wrist_visual_correction"

    alpha = _adaptive_visual_alpha(
        visual_correction_alpha,
        residual=rotation_error_deg,
        soft_gate=gate_rotation_deg,
        min_alpha=0.08,
    )
    if prediction_has_imu:
        rotation = _slerp_rotation(predicted_wrist.rotation, visual_pose.rotation, alpha)
        orientation_source = "wrist_gyro_prediction_with_visual_correction"
    else:
        rotation = visual_pose.rotation.copy()
        orientation_source = "wrist_visual"

    mode = "visual_update"
    if translation_rejected or rotation_rejected:
        mode = "gated_visual_update"
    if static_translation_hold:
        mode = f"static_hold_{mode}"
    return RigidTransform(rotation=rotation, translation=translation), {
        "mode": mode,
        "prediction_available": True,
        "visual_available": True,
        "prediction_dt_s": prediction_dt_s,
        "prediction_has_imu": prediction_has_imu,
        "translation_error_m": translation_error_m,
        "rotation_error_deg": rotation_error_deg,
        "translation_rejected": translation_rejected,
        "rotation_rejected": rotation_rejected,
        "wrist_imu_static": wrist_imu_static,
        "static_translation_hold": static_translation_hold,
        "position_visual_alpha_used": pos_alpha,
        "visual_correction_alpha": alpha,
        "translation_source": translation_source,
        "orientation_source": orientation_source,
    }


def _adaptive_visual_alpha(
    base_alpha: float,
    *,
    residual: float,
    soft_gate: float,
    min_alpha: float,
) -> float:
    base = float(np.clip(base_alpha, 0.0, 1.0))
    floor = float(np.clip(min_alpha, 0.0, base))
    gate = max(float(soft_gate), 1e-9)
    ratio = max(float(residual), 0.0) / gate
    if ratio <= 1.0:
        return base
    if ratio >= 2.0:
        return floor
    return base + (floor - base) * (ratio - 1.0)


def _slerp_quat_wxyz(quat_a: list[float], quat_b: list[float], alpha: float) -> list[float]:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    qa = _normalize_quat_wxyz(quat_a)
    qb = _normalize_quat_wxyz(quat_b)
    dot = float(np.dot(qa, qb))
    if dot < 0.0:
        qb = -qb
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        q = qa * (1.0 - alpha) + qb * alpha
        return _normalize_quat_wxyz(q).tolist()
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    q = qa * s0 + qb * s1
    return _normalize_quat_wxyz(q).tolist()


def _normalize_quat_wxyz(values) -> np.ndarray:
    quat = np.asarray(values, dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        return np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return quat / norm


def _slerp_rotation(rotation_a: np.ndarray, rotation_b: np.ndarray, alpha: float) -> np.ndarray:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    if alpha <= 0.0:
        return rotation_a.copy()
    if alpha >= 1.0:
        return rotation_b.copy()

    qa = np.asarray(rotation_matrix_to_quat_wxyz(rotation_a), dtype=np.float64)
    qb = np.asarray(rotation_matrix_to_quat_wxyz(rotation_b), dtype=np.float64)
    if float(np.dot(qa, qb)) < 0.0:
        qb = -qb
    dot = float(np.clip(np.dot(qa, qb), -1.0, 1.0))
    if dot > 0.9995:
        q = qa + alpha * (qb - qa)
        q /= max(float(np.linalg.norm(q)), 1e-12)
        return quat_wxyz_to_rotation_matrix(q)

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    sin_theta = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    q = s0 * qa + s1 * qb
    q /= max(float(np.linalg.norm(q)), 1e-12)
    return quat_wxyz_to_rotation_matrix(q)


def _pose_to_transform(tag_pose: TagPose) -> RigidTransform:
    import cv2

    rotation, _ = cv2.Rodrigues(tag_pose.rvec)
    return RigidTransform(rotation=rotation, translation=tag_pose.tvec.reshape(3))


def _predict_tracker_pose(
    tracker: TrackerState | None,
    timestamp_monotonic_ns: int,
    imu_samples: list[ImuSampleLite],
    *,
    max_prediction_gap_s: float,
    use_velocity: bool = True,
) -> RigidTransform | None:
    if tracker is None or tracker.pose is None or tracker.timestamp_monotonic_ns is None:
        return None
    dt_s = (timestamp_monotonic_ns - tracker.timestamp_monotonic_ns) / 1e9
    if dt_s < 0.0 or dt_s > max_prediction_gap_s:
        return None

    translation = tracker.pose.translation.copy()
    if use_velocity and tracker.velocity_world_mps is not None:
        translation = translation + tracker.velocity_world_mps * dt_s
    rotation = _integrate_head_gyro(
        tracker.pose.rotation,
        imu_samples,
        tracker.timestamp_monotonic_ns,
        timestamp_monotonic_ns,
    )
    return RigidTransform(rotation=rotation, translation=translation)


def _tracker_prediction_dt_s(tracker: TrackerState | None, timestamp_monotonic_ns: int) -> float | None:
    if tracker is None or tracker.timestamp_monotonic_ns is None:
        return None
    return (timestamp_monotonic_ns - tracker.timestamp_monotonic_ns) / 1e9


def _has_imu_samples_between(
    imu_samples: list[ImuSampleLite],
    start_ns: int | None,
    end_ns: int,
) -> bool:
    if not imu_samples or start_ns is None or end_ns <= start_ns:
        return False
    timestamps = [sample.timestamp_monotonic_ns for sample in imu_samples]
    start_index = bisect.bisect_left(timestamps, start_ns)
    end_index = bisect.bisect_right(timestamps, end_ns)
    return start_index < end_index


def _imu_motion_stats(
    imu_samples: list[ImuSampleLite],
    start_ns: int | None,
    end_ns: int,
) -> dict:
    if not imu_samples or start_ns is None or end_ns <= start_ns:
        return {
            "sample_count": 0,
            "max_gyro_norm_radps": None,
            "mean_gyro_norm_radps": None,
            "accel_vector_std_mps2": None,
        }
    timestamps = [sample.timestamp_monotonic_ns for sample in imu_samples]
    start_index = bisect.bisect_left(timestamps, start_ns)
    end_index = bisect.bisect_right(timestamps, end_ns)
    window = imu_samples[start_index:end_index]
    if not window:
        return {
            "sample_count": 0,
            "max_gyro_norm_radps": None,
            "mean_gyro_norm_radps": None,
            "accel_vector_std_mps2": None,
        }
    gyro_norms = np.asarray([np.linalg.norm(sample.gyro_radps) for sample in window], dtype=np.float64)
    accel_values = [sample.accel_mps2 for sample in window if sample.accel_mps2 is not None]
    accel_std = None
    if len(accel_values) >= 3:
        accel = np.stack(accel_values, axis=0)
        accel_std = float(np.linalg.norm(np.std(accel, axis=0)))
    return {
        "sample_count": len(window),
        "max_gyro_norm_radps": float(np.max(gyro_norms)),
        "mean_gyro_norm_radps": float(np.mean(gyro_norms)),
        "accel_vector_std_mps2": accel_std,
    }


def _imu_stats_are_static(
    stats: dict,
    *,
    gyro_thresh_radps: float,
    accel_std_thresh_mps2: float,
) -> bool:
    if int(stats.get("sample_count") or 0) < 3:
        return False
    mean_gyro = stats.get("mean_gyro_norm_radps")
    max_gyro = stats.get("max_gyro_norm_radps")
    if mean_gyro is None or float(mean_gyro) > float(gyro_thresh_radps):
        return False
    if max_gyro is not None and float(max_gyro) > max(0.8, float(gyro_thresh_radps) * 4.0):
        return False
    accel_std = stats.get("accel_vector_std_mps2")
    if accel_std is not None and float(accel_std) > float(accel_std_thresh_mps2):
        return False
    return True


def _update_tracker_from_visual(tracker: TrackerState | None, pose: RigidTransform, timestamp_monotonic_ns: int) -> None:
    if tracker is None:
        return
    if tracker.pose is not None and tracker.timestamp_monotonic_ns is not None:
        dt_s = (timestamp_monotonic_ns - tracker.timestamp_monotonic_ns) / 1e9
        if 1e-4 <= dt_s <= 0.5:
            velocity = (pose.translation - tracker.pose.translation) / dt_s
            speed = float(np.linalg.norm(velocity))
            if speed <= 1.0:
                tracker.velocity_world_mps = velocity
            elif tracker.velocity_world_mps is None:
                tracker.velocity_world_mps = np.zeros(3, dtype=np.float64)
    elif tracker.velocity_world_mps is None:
        tracker.velocity_world_mps = np.zeros(3, dtype=np.float64)
    tracker.pose = pose
    tracker.timestamp_monotonic_ns = timestamp_monotonic_ns
    tracker.visual_updates += 1


def _update_tracker_from_prediction(tracker: TrackerState | None, pose: RigidTransform, timestamp_monotonic_ns: int) -> None:
    if tracker is None:
        return
    tracker.pose = pose
    tracker.timestamp_monotonic_ns = timestamp_monotonic_ns
    tracker.predicted_updates += 1


def _integrate_head_gyro(
    rotation_world_head: np.ndarray,
    imu_samples: list[ImuSampleLite],
    start_ns: int,
    end_ns: int,
) -> np.ndarray:
    if not imu_samples or end_ns <= start_ns:
        return rotation_world_head.copy()
    timestamps = [sample.timestamp_monotonic_ns for sample in imu_samples]
    start_index = bisect.bisect_left(timestamps, start_ns)
    end_index = bisect.bisect_right(timestamps, end_ns)
    if start_index >= end_index:
        return rotation_world_head.copy()

    rotation = rotation_world_head.copy()
    prev_ns = start_ns
    for sample in imu_samples[start_index:end_index]:
        curr_ns = min(sample.timestamp_monotonic_ns, end_ns)
        dt_s = max(0.0, min((curr_ns - prev_ns) / 1e9, 0.02))
        if dt_s > 0.0:
            rotation = rotation @ _so3_exp(sample.gyro_radps * dt_s)
        prev_ns = curr_ns
    tail_dt_s = max(0.0, min((end_ns - prev_ns) / 1e9, 0.02))
    if tail_dt_s > 0.0 and start_index < end_index:
        rotation = rotation @ _so3_exp(imu_samples[end_index - 1].gyro_radps * tail_dt_s)
    return rotation


def _so3_exp(rotation_vector: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rotation_vector))
    if theta < 1e-12:
        return np.eye(3, dtype=np.float64)
    axis = rotation_vector / theta
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)
    return np.eye(3, dtype=np.float64) + math.sin(theta) * skew + (1.0 - math.cos(theta)) * (skew @ skew)


def _rotation_error_deg(a: np.ndarray, b: np.ndarray) -> float:
    relative = a.T @ b
    cos_angle = (float(np.trace(relative)) - 1.0) * 0.5
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return math.degrees(math.acos(cos_angle))


def _tag_pose_to_camera_wrist_pose(tag_pose: TagPose, bracelet: BraceletConfig) -> RigidTransform:
    T_C_T = _pose_to_transform(tag_pose)
    T_T_B = bracelet.tag_to_wrist.get(
        tag_pose.detection.tag_id,
        RigidTransform(rotation=np.eye(3, dtype=np.float64), translation=np.array([0.0, 0.0, -bracelet.center_offset_m])),
    )
    return T_C_T @ T_T_B


def _safe_estimate_pose(
    detection,
    tag_size_m: float,
    calibration: CameraCalibration,
    camera_matrix: np.ndarray,
) -> TagPose | None:
    try:
        return estimate_tag_pose(
            detection,
            tag_size_m,
            camera_matrix,
            calibration.distortion,
            calibration.distortion_model,
            calibration.xi if calibration.uses_omni_projection else None,
        )
    except Exception:
        return None


def _make_hand_detector(
    *,
    enable_hands: bool,
    max_hands: int,
    hand_detection_confidence: float,
    hand_tracking_confidence: float,
    hand_model: Path | None,
) -> HandKeypointDetector | None:
    if not enable_hands:
        return None
    return HandKeypointDetector(
        max_num_hands=max_hands,
        min_detection_confidence=hand_detection_confidence,
        min_tracking_confidence=hand_tracking_confidence,
        model_path=str(hand_model) if hand_model else None,
    )


def _select_hand_skeletons(
    hand_skeletons: list[dict],
    T_W_B: RigidTransform | None,
    max_count: int = 1,
    *,
    hand_tracker: HandTrackerState | None = None,
    timestamp_monotonic_ns: int | None = None,
    continuity_gate_m: float = 0.12,
    continuity_max_gap_s: float = 0.25,
) -> list[dict]:
    if max_count <= 0 or not hand_skeletons:
        if hand_tracker is not None:
            hand_tracker.missing_updates += 1
        return []
    if T_W_B is None:
        if hand_tracker is not None:
            hand_tracker.missing_updates += 1
        return []

    wrist_position = T_W_B.translation if T_W_B is not None else None
    plausible_hands = [
        hand for hand in hand_skeletons if _hand_skeleton_is_plausible(hand, wrist_position)
    ]
    if not plausible_hands:
        if hand_tracker is not None:
            hand_tracker.missing_updates += 1
        return []

    previous_hand = hand_tracker.hand if hand_tracker is not None else None
    previous_timestamp_ns = hand_tracker.timestamp_monotonic_ns if hand_tracker is not None else None
    previous_is_recent = (
        previous_hand is not None
        and previous_timestamp_ns is not None
        and timestamp_monotonic_ns is not None
        and 0.0 <= (timestamp_monotonic_ns - previous_timestamp_ns) / 1e9 <= continuity_max_gap_s
    )
    continuity_gate_m = max(float(continuity_gate_m), 0.0)

    def score(hand: dict) -> tuple[float, float, float, float]:
        hand_score = float(hand.get("score") or 0.0)
        if wrist_position is None:
            wrist_distance = 0.0
        else:
            wrist_landmark = next((item for item in hand.get("landmarks", []) if int(item.get("index", -1)) == 0), None)
            if wrist_landmark is None or wrist_landmark.get("world") is None:
                wrist_distance = 1e6
            else:
                wrist_distance = float(np.linalg.norm(np.asarray(wrist_landmark["world"], dtype=np.float64) - wrist_position))
        continuity_distance = 0.0
        camera_switch = 0.0
        if previous_is_recent and previous_hand is not None:
            continuity_distance = _hand_skeleton_rms_distance(hand, previous_hand)
            if hand_tracker is not None and hand_tracker.camera_id and hand.get("camera_id") != hand_tracker.camera_id:
                camera_switch = 0.02
        hand["selection_score"] = {
            "wrist_distance_m": wrist_distance,
            "continuity_rms_m": continuity_distance if previous_is_recent else None,
            "camera_switch_penalty_m": camera_switch,
            "mediapipe_score": hand_score,
        }
        return (continuity_distance * 2.0 + wrist_distance + camera_switch, wrist_distance, -hand_score, camera_switch)

    ranked = sorted(plausible_hands, key=score)
    if previous_is_recent and continuity_gate_m > 0.0:
        continuous = []
        for hand in ranked:
            rms = float(hand.get("selection_score", {}).get("continuity_rms_m") or 0.0)
            if rms <= continuity_gate_m:
                continuous.append(hand)
            else:
                hand["rejected_reason"] = "hand_continuity_jump"
                hand["continuity_rms_m"] = rms
        if continuous:
            ranked = continuous
        else:
            if hand_tracker is not None:
                hand_tracker.continuity_rejections += 1
                hand_tracker.missing_updates += 1
            return []

    selected = ranked[:max_count]
    for hand in selected:
        hand["selected_for_motion_frame"] = True
        hand["selection_source"] = "continuity_tracker" if previous_is_recent else "wrist_distance"
    if hand_tracker is not None:
        if selected:
            hand_tracker.hand = copy.deepcopy(selected[0])
            hand_tracker.timestamp_monotonic_ns = timestamp_monotonic_ns
            hand_tracker.camera_id = str(selected[0].get("camera_id") or "")
            hand_tracker.selected_updates += 1
        else:
            hand_tracker.missing_updates += 1
    return selected


def _hand_skeleton_rms_distance(hand: dict, previous_hand: dict) -> float:
    points = _hand_points_by_index(hand)
    previous_points = _hand_points_by_index(previous_hand)
    shared = sorted(set(points) & set(previous_points))
    if not shared:
        return 1e6
    diffs = [float(np.linalg.norm(points[index] - previous_points[index])) for index in shared]
    return float(np.sqrt(np.mean(np.square(diffs))))


def _hand_points_by_index(hand: dict) -> dict[int, np.ndarray]:
    return {
        int(item["index"]): np.asarray(item["world"], dtype=np.float64)
        for item in hand.get("landmarks", [])
        if item.get("world") is not None
    }


def _hand_tracker_recently_used_multiview(
    hand_tracker: HandTrackerState | None,
    timestamp_monotonic_ns: int,
    max_gap_s: float,
) -> bool:
    if hand_tracker is None or hand_tracker.hand is None or hand_tracker.timestamp_monotonic_ns is None:
        return False
    method = (hand_tracker.hand.get("source") or {}).get("method")
    if method not in {"multiview_triangulated", "singleview_guided_by_multiview"}:
        return False
    gap_s = (timestamp_monotonic_ns - hand_tracker.timestamp_monotonic_ns) / 1e9
    return 0.0 <= gap_s <= max_gap_s


def _guide_singleview_hands_from_tracker(
    hand_skeletons: list[dict],
    T_W_B: RigidTransform | None,
    hand_tracker: HandTrackerState | None,
    timestamp_monotonic_ns: int,
    *,
    max_gap_s: float = 0.25,
    max_candidate_rms_m: float = 0.14,
    max_landmark_step_m: float = 0.045,
    update_alpha: float = 0.65,
) -> list[dict]:
    if T_W_B is None or hand_tracker is None or hand_tracker.hand is None or hand_tracker.timestamp_monotonic_ns is None:
        return []
    gap_s = (timestamp_monotonic_ns - hand_tracker.timestamp_monotonic_ns) / 1e9
    if gap_s < 0.0 or gap_s > max_gap_s:
        return []

    wrist_position = T_W_B.translation
    previous_hand = hand_tracker.hand
    previous_points = _hand_points_by_index(previous_hand)
    if len(previous_points) < 8:
        return []

    candidates = []
    for hand in hand_skeletons:
        if (hand.get("source") or {}).get("method") == "multiview_triangulated":
            continue
        if not _hand_skeleton_is_plausible(hand, wrist_position, max_wrist_distance_m=0.35):
            continue
        points = _hand_points_by_index(hand)
        if len(points) < 8:
            continue
        aligned_points, wrist_anchor_delta = _align_hand_wrist_to_position(points, wrist_position)
        shared = sorted(set(aligned_points) & set(previous_points))
        if len(shared) < 8:
            continue
        innovation_rms = float(
            np.sqrt(np.mean([np.linalg.norm(aligned_points[index] - previous_points[index]) ** 2 for index in shared]))
        )
        if innovation_rms > max_candidate_rms_m:
            hand["rejected_reason"] = "singleview_guided_innovation_too_large"
            hand["singleview_guided_innovation_rms_m"] = innovation_rms
            continue
        score = (
            innovation_rms,
            float(np.linalg.norm(wrist_anchor_delta)),
            -float(hand.get("score") or 0.0),
        )
        candidates.append((score, hand, aligned_points, innovation_rms, wrist_anchor_delta))
    if not candidates:
        return []

    _score, hand, aligned_points, innovation_rms, wrist_anchor_delta = sorted(candidates, key=lambda item: item[0])[0]
    guided_landmarks = []
    raw_landmarks = {
        int(item["index"]): item
        for item in hand.get("landmarks", [])
        if item.get("world") is not None
    }
    for index in sorted(aligned_points):
        target = aligned_points[index]
        previous = previous_points.get(index)
        if previous is None:
            guided = target
            step_norm = None
        else:
            step = target - previous
            step_norm = float(np.linalg.norm(step))
            if step_norm > max_landmark_step_m:
                step = step * (max_landmark_step_m / max(step_norm, 1e-12))
            guided = previous + step * float(np.clip(update_alpha, 0.0, 1.0))
        raw = raw_landmarks.get(index, {})
        guided_landmarks.append(
            {
                "index": int(index),
                "image_xy": raw.get("image_xy"),
                "normalized_xyz": raw.get("normalized_xyz"),
                "world": guided.astype(float).tolist(),
                "singleview_world_raw": raw.get("world"),
                "singleview_world_aligned": target.astype(float).tolist(),
                "singleview_step_norm_m": step_norm,
                "camera_id": hand.get("camera_id"),
            }
        )

    guided_hand = {
        "group_id": hand.get("group_id"),
        "camera_id": str(hand.get("camera_id") or ""),
        "timestamp_monotonic_ns": hand.get("timestamp_monotonic_ns"),
        "timestamp_unix_ns": hand.get("timestamp_unix_ns"),
        "hand_index": hand.get("hand_index", 0),
        "handedness": hand.get("handedness", "Unknown"),
        "score": float(hand.get("score") or 0.0),
        "source": {
            "method": "singleview_guided_by_multiview",
            "weak_observation_method": (hand.get("source") or {}).get("method"),
            "guided_from_previous_method": (previous_hand.get("source") or {}).get("method"),
            "guide_gap_s": gap_s,
            "innovation_rms_m": innovation_rms,
            "max_candidate_rms_m": max_candidate_rms_m,
            "max_landmark_step_m": max_landmark_step_m,
            "update_alpha": update_alpha,
            "wrist_anchor_delta_m": wrist_anchor_delta.astype(float).tolist(),
        },
        "landmarks": guided_landmarks,
        "connections": [list(pair) for pair in HAND_CONNECTIONS],
    }
    if not _hand_skeleton_is_plausible(guided_hand, wrist_position, max_wrist_distance_m=0.35):
        return []
    return [guided_hand]


def _align_hand_wrist_to_position(
    points: dict[int, np.ndarray],
    wrist_position: np.ndarray,
) -> tuple[dict[int, np.ndarray], np.ndarray]:
    if 0 not in points:
        return points, np.zeros(3, dtype=np.float64)
    delta = wrist_position - points[0]
    return {index: point + delta for index, point in points.items()}, delta


def _triangulate_multiview_hand_skeletons(
    hand_skeletons: list[dict],
    T_W_B: RigidTransform | None,
    *,
    min_cameras: int = 2,
    max_ray_residual_m: float = 0.08,
    min_ray_angle_deg: float = 2.0,
) -> list[dict]:
    if T_W_B is None or len(hand_skeletons) < min_cameras:
        return []
    wrist_position = T_W_B.translation
    per_camera = _best_hand_observation_per_camera(hand_skeletons, wrist_position)
    if len(per_camera) < min_cameras:
        return []

    cameras_used = sorted(per_camera)
    selected_observations = [per_camera[camera_id] for camera_id in cameras_used]
    landmarks = []
    residuals = []
    obs_counts = []
    for landmark_index in range(21):
        observations = []
        for hand in selected_observations:
            item = next(
                (landmark for landmark in hand.get("landmarks", []) if int(landmark.get("index", -1)) == landmark_index),
                None,
            )
            if item is None:
                continue
            origin = item.get("ray_world_origin")
            direction = item.get("ray_world_direction")
            if len(origin or []) != 3 or len(direction or []) != 3:
                continue
            observations.append(
                {
                    "camera_id": hand.get("camera_id"),
                    "origin": np.asarray(origin, dtype=np.float64),
                    "direction": _normalize_vector(np.asarray(direction, dtype=np.float64)),
                    "image_xy": item.get("image_xy"),
                }
            )
        if len(observations) < min_cameras:
            continue
        angle_deg = _max_ray_angle_deg([obs["direction"] for obs in observations])
        if angle_deg < min_ray_angle_deg:
            continue
        point, residual_m = _least_squares_ray_intersection(observations)
        if point is None or residual_m > max_ray_residual_m:
            continue
        residuals.append(residual_m)
        obs_counts.append(len(observations))
        landmarks.append(
            {
                "index": landmark_index,
                "world": point.astype(float).tolist(),
                "triangulation_residual_m": residual_m,
                "ray_angle_deg": angle_deg,
                "observation_count": len(observations),
                "observations": [
                    {
                        "camera_id": str(obs["camera_id"]),
                        "image_xy": obs["image_xy"],
                    }
                    for obs in observations
                ],
            }
        )
    if len(landmarks) < 8:
        return []

    scores = [float(hand.get("score") or 0.0) for hand in selected_observations]
    handedness = _majority_label([str(hand.get("handedness") or "Unknown") for hand in selected_observations])
    timestamp_ns_values = [int(hand["timestamp_monotonic_ns"]) for hand in selected_observations if hand.get("timestamp_monotonic_ns") is not None]
    timestamp_unix_values = [int(hand["timestamp_unix_ns"]) for hand in selected_observations if hand.get("timestamp_unix_ns") is not None]
    group_ids = [int(hand["group_id"]) for hand in selected_observations if hand.get("group_id") is not None]
    hand = {
        "group_id": int(round(float(np.mean(group_ids)))) if group_ids else 0,
        "camera_id": "+".join(cameras_used),
        "timestamp_monotonic_ns": int(round(float(np.mean(timestamp_ns_values)))) if timestamp_ns_values else None,
        "timestamp_unix_ns": int(round(float(np.mean(timestamp_unix_values)))) if timestamp_unix_values else None,
        "hand_index": 0,
        "handedness": handedness,
        "score": float(np.mean(scores)) if scores else 0.0,
        "source": {
            "method": "multiview_triangulated",
            "input_method": "mediapipe_2d_rays",
            "cameras_used": cameras_used,
            "landmark_count": len(landmarks),
            "min_cameras": min_cameras,
            "max_ray_residual_m": max_ray_residual_m,
            "median_ray_residual_m": float(np.median(residuals)) if residuals else None,
            "max_used_ray_residual_m": float(np.max(residuals)) if residuals else None,
            "median_observation_count": float(np.median(obs_counts)) if obs_counts else None,
        },
        "landmarks": landmarks,
        "connections": [list(pair) for pair in HAND_CONNECTIONS],
    }
    if not _hand_skeleton_is_plausible(hand, wrist_position, max_wrist_distance_m=0.35):
        return []
    return [hand]


def _best_hand_observation_per_camera(hand_skeletons: list[dict], wrist_position: np.ndarray) -> dict[str, dict]:
    best: dict[str, tuple[tuple[float, float], dict]] = {}
    for hand in hand_skeletons:
        camera_id = str(hand.get("camera_id") or "")
        if not camera_id:
            continue
        if not _hand_skeleton_is_plausible(hand, wrist_position):
            continue
        wrist_landmark = next((item for item in hand.get("landmarks", []) if int(item.get("index", -1)) == 0), None)
        if wrist_landmark is None or wrist_landmark.get("world") is None:
            continue
        wrist_distance = float(np.linalg.norm(np.asarray(wrist_landmark["world"], dtype=np.float64) - wrist_position))
        score = float(hand.get("score") or 0.0)
        ranking = (wrist_distance, -score)
        if camera_id not in best or ranking < best[camera_id][0]:
            best[camera_id] = (ranking, hand)
    return {camera_id: hand for camera_id, (_ranking, hand) in best.items()}


def _least_squares_ray_intersection(observations: list[dict]) -> tuple[np.ndarray | None, float]:
    a = np.zeros((3, 3), dtype=np.float64)
    b = np.zeros(3, dtype=np.float64)
    eye = np.eye(3, dtype=np.float64)
    for obs in observations:
        direction = _normalize_vector(obs["direction"])
        origin = obs["origin"]
        projection = eye - np.outer(direction, direction)
        a += projection
        b += projection @ origin
    try:
        point = np.linalg.solve(a, b)
    except np.linalg.LinAlgError:
        point = np.linalg.lstsq(a, b, rcond=None)[0]
    distances = []
    for obs in observations:
        direction = _normalize_vector(obs["direction"])
        origin = obs["origin"]
        distances.append(float(np.linalg.norm(np.cross(direction, point - origin))))
    return point, float(np.sqrt(np.mean(np.square(distances)))) if distances else 1e6


def _max_ray_angle_deg(directions: list[np.ndarray]) -> float:
    max_angle = 0.0
    for i, direction_a in enumerate(directions):
        for direction_b in directions[i + 1 :]:
            dot = float(np.dot(_normalize_vector(direction_a), _normalize_vector(direction_b)))
            dot = min(1.0, max(-1.0, dot))
            max_angle = max(max_angle, math.degrees(math.acos(abs(dot))))
    return max_angle


def _normalize_vector(values: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        return np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return values / norm


def _majority_label(labels: list[str]) -> str:
    counts: dict[str, int] = defaultdict(int)
    for label in labels:
        counts[label] += 1
    if not counts:
        return "Unknown"
    return sorted(counts.items(), key=lambda item: (-item[1], item[0]))[0][0]


def _hand_skeleton_is_plausible(
    hand: dict,
    wrist_position: np.ndarray | None,
    *,
    max_wrist_distance_m: float = 0.25,
    max_bone_length_m: float = 0.16,
    max_extent_m: float = 0.35,
) -> bool:
    points_by_index = {
        int(item["index"]): np.asarray(item["world"], dtype=np.float64)
        for item in hand.get("landmarks", [])
        if item.get("world") is not None
    }
    if len(points_by_index) < 8:
        hand["rejected_reason"] = "too_few_hand_landmarks"
        return False
    if wrist_position is not None and 0 in points_by_index:
        distance = float(np.linalg.norm(points_by_index[0] - wrist_position))
        if distance > max_wrist_distance_m:
            hand["rejected_reason"] = "hand_wrist_far_from_wrist_pose"
            hand["wrist_distance_m"] = distance
            return False
    values = np.stack(list(points_by_index.values()), axis=0)
    extent = float(np.max(np.linalg.norm(values - values.mean(axis=0), axis=1)) * 2.0)
    if extent > max_extent_m:
        hand["rejected_reason"] = "hand_extent_too_large"
        hand["extent_m"] = extent
        return False
    for start, end in HAND_CONNECTIONS:
        if start in points_by_index and end in points_by_index:
            length = float(np.linalg.norm(points_by_index[start] - points_by_index[end]))
            if length > max_bone_length_m:
                hand["rejected_reason"] = "hand_bone_too_long"
                hand["bone_length_m"] = length
                hand["bone"] = [int(start), int(end)]
                return False
    return True


def _estimate_hand_skeletons(
    *,
    hand_detector: HandKeypointDetector,
    image: np.ndarray,
    frame_record: dict,
    camera_matrix: np.ndarray,
    calibration: CameraCalibration,
    world_pose: WorldBoardPose,
    T_C_B: RigidTransform,
) -> list[dict]:
    hands = hand_detector.detect(image)
    if not hands:
        return []

    height, width = image.shape[:2]
    wrist_depth_m = float(T_C_B.translation[2])
    if wrist_depth_m <= 0.0:
        return []

    result = []
    for hand_index, hand in enumerate(hands):
        landmarks = []
        for keypoint in hand.keypoints:
            pixel = np.array([keypoint.x * width, keypoint.y * height], dtype=np.float64)
            ray = _pixel_to_camera_ray(
                pixel,
                camera_matrix,
                calibration.distortion,
                calibration.distortion_model,
                calibration.xi if calibration.uses_omni_projection else None,
            )
            if abs(float(ray[2])) < 1e-9:
                continue
            point_camera = ray * (wrist_depth_m / float(ray[2]))
            point_world = world_pose.T_W_C.rotation @ point_camera + world_pose.T_W_C.translation
            ray_world_direction = _normalize_vector(world_pose.T_W_C.rotation @ ray)
            ray_world_origin = world_pose.T_W_C.translation
            landmarks.append(
                {
                    "index": int(keypoint.index),
                    "image_xy": [float(pixel[0]), float(pixel[1])],
                    "normalized_xyz": [float(keypoint.x), float(keypoint.y), float(keypoint.z)],
                    "world": point_world.astype(float).tolist(),
                    "camera": point_camera.astype(float).tolist(),
                    "ray_world_origin": ray_world_origin.astype(float).tolist(),
                    "ray_world_direction": ray_world_direction.astype(float).tolist(),
                    "visibility": keypoint.visibility,
                }
            )
        if landmarks:
            result.append(
                {
                    "group_id": int(frame_record["group_id"]),
                    "camera_id": str(frame_record["camera_id"]),
                    "timestamp_monotonic_ns": int(frame_record["timestamp_monotonic_ns"]),
                    "timestamp_unix_ns": int(frame_record["timestamp_unix_ns"]),
                    "hand_index": hand_index,
                    "handedness": hand.handedness,
                    "score": float(hand.score),
                    "source": {
                        "method": "mediapipe_2d_backprojected_to_wrist_depth",
                        "world_method": world_pose.method,
                        "world_source_tag_ids": world_pose.source_tag_ids,
                        "wrist_depth_m": wrist_depth_m,
                    },
                    "landmarks": landmarks,
                    "connections": [list(pair) for pair in HAND_CONNECTIONS],
                }
            )
    return result


def _pixel_to_camera_ray(
    pixel: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    distortion_model: str,
    xi: float | None,
) -> np.ndarray:
    import cv2

    image_points = np.asarray(pixel, dtype=np.float64).reshape(1, 1, 2)
    if _is_omni_distortion(distortion_model, xi):
        undistorted = cv2.omnidir.undistortPoints(
            image_points,
            camera_matrix,
            dist_coeffs.reshape(-1, 1),
            np.asarray([float(xi)], dtype=np.float64),
            np.eye(3, dtype=np.float64),
        )
        x, y = undistorted.reshape(2)
    elif distortion_model.lower() in {"equidistant", "opencv_fisheye", "fisheye"}:
        undistorted = cv2.fisheye.undistortPoints(
            image_points,
            camera_matrix,
            dist_coeffs.reshape(-1, 1),
        )
        x, y = undistorted.reshape(2)
    else:
        undistorted = cv2.undistortPoints(
            image_points,
            camera_matrix,
            dist_coeffs.reshape(-1, 1),
        )
        x, y = undistorted.reshape(2)
    ray = np.array([float(x), float(y), 1.0], dtype=np.float64)
    return ray / max(float(np.linalg.norm(ray)), 1e-12)


def _dict_to_transform(value: dict) -> RigidTransform:
    return RigidTransform.from_matrix(value["matrix"])


def _rigid_body_state(frame_id: str, child_frame_id: str, transform: RigidTransform) -> dict:
    result = transform_to_dict(transform)
    result["frame_id"] = frame_id
    result["child_frame_id"] = child_frame_id
    return result


def _head_pose_record(frame: WorldMotionFrame) -> dict:
    return {
        "timestamp_unix_ns": frame.timestamp_unix_ns,
        "timestamp_monotonic_ns": frame.timestamp_monotonic_ns,
        "timestamp_source": frame.timestamp_source,
        "tracking_state": frame.tracking_state,
        "T_W_H": {
            "position": frame.head["position"],
            "orientation_wxyz": frame.head["orientation_wxyz"],
            "matrix": frame.head["matrix"],
        },
    }


def _read_frames(path: Path) -> dict[int, list[dict]]:
    if not path.exists():
        raise FileNotFoundError(f"Missing frame manifest: {path}")
    frames = defaultdict(list)
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            record = json.loads(line)
            frames[int(record["group_id"])].append(record)
    return dict(frames)


def _read_imu_samples(path: Path) -> list[ImuSampleLite]:
    if not path.exists():
        return []
    samples: list[ImuSampleLite] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            record = json.loads(line)
            sample = _imu_record_to_sample(record)
            if sample is not None:
                samples.append(sample)
    samples.sort(key=lambda sample: sample.timestamp_monotonic_ns)
    return samples


class _LiveImuJsonlTailer:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.offset = 0
        self.partial = ""

    def read_new(self) -> list[ImuSampleLite]:
        if not self.path.exists():
            return []
        try:
            size = self.path.stat().st_size
            if size < self.offset:
                self.offset = 0
                self.partial = ""
            with self.path.open("r", encoding="utf-8") as f:
                f.seek(self.offset)
                chunk = f.read()
                self.offset = f.tell()
        except OSError:
            return []
        if not chunk:
            return []

        text = self.partial + chunk
        if text.endswith("\n"):
            lines = text.splitlines()
            self.partial = ""
        else:
            lines = text.splitlines()
            self.partial = lines.pop() if lines else text

        samples = []
        for line in lines:
            if not line.strip():
                continue
            try:
                sample = _imu_record_to_sample(json.loads(line))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                continue
            if sample is not None:
                samples.append(sample)
        return samples


def _imu_record_to_sample(record: dict) -> ImuSampleLite | None:
    gyro = record.get("gyro_radps")
    if len(gyro or []) != 3:
        return None
    accel = record.get("accel_mps2")
    return ImuSampleLite(
        timestamp_monotonic_ns=int(record["timestamp_monotonic_ns"]),
        gyro_radps=np.asarray(gyro, dtype=np.float64),
        accel_mps2=np.asarray(accel, dtype=np.float64) if len(accel or []) == 3 else None,
    )


def _resolve_camera_session_dir(session_dir: Path) -> Path:
    return session_dir / "cameras" if (session_dir / "cameras" / "frames.jsonl").exists() else session_dir


def _print_live_status(frame: WorldMotionFrame) -> None:
    head = frame.head["position"]
    wrist = frame.wrist["position"] if frame.wrist else None
    if wrist:
        text = (
            f"head W=({head[0]:+.2f},{head[1]:+.2f},{head[2]:+.2f}) "
            f"wrist W=({wrist[0]:+.2f},{wrist[1]:+.2f},{wrist[2]:+.2f}) "
            f"sources W:{frame.source['world_candidate_count']} B:{frame.source['wrist_candidate_count']}"
        )
    else:
        text = (
            f"head W=({head[0]:+.2f},{head[1]:+.2f},{head[2]:+.2f}) "
            f"wrist not visible sources W:{frame.source['world_candidate_count']}"
        )
    print(text, end="\r", flush=True)


def _try_draw_live_map(cv2, frame: WorldMotionFrame | None) -> bool:
    try:
        _draw_live_map(cv2, frame)
        return True
    except cv2.error as exc:
        print(f"\nOpenCV live preview disabled: {exc}")
        return False


def _draw_live_map(cv2, frame: WorldMotionFrame | None) -> None:
    canvas = np.full((700, 700, 3), 245, dtype=np.uint8)
    origin = np.array([350.0, 350.0], dtype=np.float64)
    scale = 120.0
    cv2.line(canvas, (0, 350), (700, 350), (220, 220, 220), 1)
    cv2.line(canvas, (350, 0), (350, 700), (220, 220, 220), 1)
    cv2.putText(canvas, "W origin / desktop tag", (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 40, 40), 2)
    cv2.circle(canvas, tuple(origin.astype(int)), 6, (0, 0, 0), -1)
    if frame is None:
        cv2.putText(canvas, "world tag not visible", (210, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 80, 220), 2)
    else:
        _draw_body(cv2, canvas, origin, scale, frame.head, (40, 120, 240), "head")
        if frame.wrist:
            _draw_body(cv2, canvas, origin, scale, frame.wrist, (70, 170, 60), "wrist")
            head_xy = _map_point(origin, scale, frame.head["position"])
            wrist_xy = _map_point(origin, scale, frame.wrist["position"])
            cv2.line(canvas, tuple(head_xy), tuple(wrist_xy), (120, 120, 120), 2)
        cv2.putText(
            canvas,
            f"W:{frame.source['world_candidate_count']} B:{frame.source['wrist_candidate_count']}",
            (20, 670),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (40, 40, 40),
            2,
        )
    cv2.imshow("3DMotion desktop AprilTag world map", canvas)
    cv2.waitKey(1)


def _draw_body(cv2, canvas: np.ndarray, origin: np.ndarray, scale: float, state: dict, color: tuple[int, int, int], label: str) -> None:
    xy = _map_point(origin, scale, state["position"])
    cv2.circle(canvas, tuple(xy), 9, color, -1)
    cv2.putText(canvas, label, tuple((xy + np.array([12, -8])).tolist()), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


def _map_point(origin: np.ndarray, scale: float, position: list[float]) -> np.ndarray:
    p = np.asarray(position, dtype=np.float64)
    xy = origin + np.array([p[0], -p[1]], dtype=np.float64) * scale
    return np.clip(xy, 0, 699).astype(int)


def main() -> None:
    parser = argparse.ArgumentParser(description="Use desktop AprilTags as world anchors for head/wrist pose.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    offline = subparsers.add_parser("offline", help="Process a recorded camera session.")
    offline.add_argument("--session-dir", required=True)
    offline.add_argument("--cameras", default="configs/cameras.yaml")
    offline.add_argument("--world-tags", default="configs/world_tags.yaml")
    offline.add_argument("--bracelet", default="configs/bracelet.yaml")
    offline.add_argument("--output-dir", default="data/processed/world_anchor")
    offline.add_argument("--max-reprojection-error-px", type=float, default=8.0)
    offline.add_argument("--require-wrist", action="store_true")
    offline.add_argument("--no-single-tag", action="store_true", help="Disable single world-tag fallback.")
    offline.add_argument("--no-head-imu", action="store_true", help="Do not read imus/head_imu.jsonl for short-term prediction.")
    offline.add_argument("--no-wrist-imu", action="store_true", help="Do not read imus/wrist_imu.jsonl for wrist orientation prediction.")
    offline.add_argument("--allow-prediction-output", action="store_true", help="Write short-term head_imu/velocity predicted frames when no world tag is visible.")
    offline.add_argument("--max-prediction-gap-s", type=float, default=1.0, help="Maximum gap for IMU/velocity prediction. Position prediction is further limited by --max-position-prediction-gap-s.")
    offline.add_argument("--max-wrist-prediction-gap-s", type=float, default=1.0)
    offline.add_argument("--single-tag-gate-translation-m", type=float, default=0.12)
    offline.add_argument("--single-tag-gate-rotation-deg", type=float, default=35.0)
    offline.add_argument("--single-tag-bootstrap", action="store_true", help="Allow one visible world-grid tag to initialize tracking before any board/PnP prediction exists. This is convenient but can jump on planar ambiguity.")
    offline.add_argument("--head-position-visual-alpha", type=float, default=0.75, help="Visual correction alpha for head position. 1.0 uses raw visual position, lower blends with predicted velocity.")
    offline.add_argument("--head-orientation-visual-alpha", type=float, default=0.12, help="Visual correction alpha for head orientation. Lower values let head_imu gyro dominate short-term motion.")
    offline.add_argument("--max-position-prediction-gap-s", type=float, default=0.25, help="Only blend velocity-predicted head position across gaps up to this duration.")
    offline.add_argument("--wrist-position-visual-alpha", type=float, default=0.75)
    offline.add_argument("--wrist-visual-correction-alpha", type=float, default=0.35)
    offline.add_argument("--wrist-prediction-gate-translation-m", type=float, default=0.20)
    offline.add_argument("--wrist-prediction-gate-rotation-deg", type=float, default=60.0)
    offline.add_argument("--wrist-static-gyro-thresh-radps", type=float, default=0.18)
    offline.add_argument("--wrist-static-accel-std-thresh-mps2", type=float, default=0.25)
    offline.add_argument("--multi-camera-gate-translation-m", type=float, default=0.12, help="Reject camera head candidates farther than this from IMU prediction.")
    offline.add_argument("--multi-camera-gate-rotation-deg", type=float, default=25.0, help="Reject camera head candidates whose orientation is farther than this from IMU prediction.")
    offline.add_argument("--multi-camera-median-gate-m", type=float, default=0.08, help="Reject multi-camera translation outliers farther than this from the per-frame median.")
    offline.add_argument("--hands", action="store_true", help="Detect MediaPipe hand skeletons and back-project them to approximate world points.")
    offline.add_argument("--max-hands", type=int, default=2)
    offline.add_argument("--max-hand-skeletons-per-frame", type=int, default=1, help="Maximum selected hand skeletons written to motion_frame/RViz. All candidates still go to hand_skeletons.jsonl.")
    offline.add_argument("--hand-continuity-gate-m", type=float, default=0.12, help="Reject selected hand candidates whose landmark RMS jump from the previous selected hand exceeds this distance.")
    offline.add_argument("--hand-continuity-max-gap-s", type=float, default=0.25, help="Only apply hand continuity gating when the previous selected hand is newer than this gap.")
    offline.add_argument("--hand-multiview-only", action="store_true", help="Only write hand skeletons reconstructed from at least two cameras; do not fall back to single-camera wrist-depth backprojection.")
    offline.add_argument("--hand-allow-direct-singleview-fallback", action="store_true", help="Allow raw single-camera wrist-depth hand output when no recent multiview/guided hand exists.")
    offline.add_argument("--hand-detection-confidence", type=float, default=0.5)
    offline.add_argument("--hand-tracking-confidence", type=float, default=0.5)
    offline.add_argument("--hand-model", help="Optional MediaPipe Tasks hand_landmarker.task path.")

    live = subparsers.add_parser("live", help="Run live camera processing and print/persist poses.")
    live.add_argument("--source", action="append", required=True, help="Camera source, e.g. C1:/dev/video0. Repeat.")
    live.add_argument("--cameras", default="configs/cameras.yaml")
    live.add_argument("--world-tags", default="configs/world_tags.yaml")
    live.add_argument("--bracelet", default="configs/bracelet.yaml")
    live.add_argument("--fps", type=float, default=10.0)
    live.add_argument("--width", type=int)
    live.add_argument("--height", type=int)
    live.add_argument("--output-jsonl")
    live.add_argument("--max-reprojection-error-px", type=float, default=8.0)
    live.add_argument("--show", action="store_true", help="Show a simple live top-down OpenCV map.")
    live.add_argument("--ros-publish", action="store_true", help="Publish live poses, paths, and TF to ROS2 /motion/* topics.")
    live.add_argument("--fixed-frame", default="world")
    live.add_argument("--head-frame", default="head")
    live.add_argument("--wrist-frame", default="wrist")
    live.add_argument("--max-path-length", type=int, default=5000)
    live.add_argument("--head-imu-live-jsonl", help="Live head_imu JSONL stream written by the dashboard backend.")
    live.add_argument("--wrist-imu-live-jsonl", help="Live wrist_imu JSONL stream written by the dashboard backend.")
    live.add_argument("--live-imu-buffer-s", type=float, default=3.0)
    live.add_argument("--max-prediction-gap-s", type=float, default=1.0)
    live.add_argument("--max-wrist-prediction-gap-s", type=float, default=1.0)
    live.add_argument("--head-position-visual-alpha", type=float, default=0.75)
    live.add_argument("--head-orientation-visual-alpha", type=float, default=0.12)
    live.add_argument("--max-position-prediction-gap-s", type=float, default=0.25)
    live.add_argument("--wrist-position-visual-alpha", type=float, default=0.75)
    live.add_argument("--wrist-visual-correction-alpha", type=float, default=0.35)
    live.add_argument("--wrist-prediction-gate-translation-m", type=float, default=0.20)
    live.add_argument("--wrist-prediction-gate-rotation-deg", type=float, default=60.0)
    live.add_argument("--wrist-static-gyro-thresh-radps", type=float, default=0.18)
    live.add_argument("--wrist-static-accel-std-thresh-mps2", type=float, default=0.25)
    live.add_argument("--multi-camera-gate-translation-m", type=float, default=0.12)
    live.add_argument("--multi-camera-gate-rotation-deg", type=float, default=25.0)
    live.add_argument("--multi-camera-median-gate-m", type=float, default=0.08)
    live.add_argument("--hands", action="store_true", help="Detect MediaPipe hand skeletons and publish/output approximate world landmarks.")
    live.add_argument("--max-hands", type=int, default=2)
    live.add_argument("--max-hand-skeletons-per-frame", type=int, default=1, help="Maximum selected hand skeletons published to RViz. All candidates still go to output JSONL.")
    live.add_argument("--hand-continuity-gate-m", type=float, default=0.12)
    live.add_argument("--hand-continuity-max-gap-s", type=float, default=0.25)
    live.add_argument("--hand-multiview-only", action="store_true")
    live.add_argument("--hand-allow-direct-singleview-fallback", action="store_true")
    live.add_argument("--hand-detection-confidence", type=float, default=0.5)
    live.add_argument("--hand-tracking-confidence", type=float, default=0.5)
    live.add_argument("--hand-model", help="Optional MediaPipe Tasks hand_landmarker.task path.")

    args = parser.parse_args()
    if args.command == "offline":
        summary = process_world_anchor_session(
            session_dir=Path(args.session_dir),
            cameras_path=Path(args.cameras),
            world_tags_path=Path(args.world_tags),
            bracelet_path=Path(args.bracelet),
            output_dir=Path(args.output_dir),
            max_reprojection_error_px=args.max_reprojection_error_px,
            require_wrist=args.require_wrist,
            allow_single_tag=not args.no_single_tag,
            use_head_imu=not args.no_head_imu,
            use_wrist_imu=not args.no_wrist_imu,
            allow_prediction_output=args.allow_prediction_output,
            max_prediction_gap_s=args.max_prediction_gap_s,
            max_position_prediction_gap_s=args.max_position_prediction_gap_s,
            max_wrist_prediction_gap_s=args.max_wrist_prediction_gap_s,
            single_tag_gate_translation_m=args.single_tag_gate_translation_m,
            single_tag_gate_rotation_deg=args.single_tag_gate_rotation_deg,
            allow_single_tag_bootstrap=args.single_tag_bootstrap,
            enable_hands=args.hands,
            max_hands=args.max_hands,
            max_hand_skeletons_per_frame=args.max_hand_skeletons_per_frame,
            hand_continuity_gate_m=args.hand_continuity_gate_m,
            hand_continuity_max_gap_s=args.hand_continuity_max_gap_s,
            hand_multiview_only=args.hand_multiview_only,
            hand_allow_direct_singleview_fallback=args.hand_allow_direct_singleview_fallback,
            hand_detection_confidence=args.hand_detection_confidence,
            hand_tracking_confidence=args.hand_tracking_confidence,
            hand_model=Path(args.hand_model) if args.hand_model else None,
            head_position_visual_alpha=args.head_position_visual_alpha,
            head_orientation_visual_alpha=args.head_orientation_visual_alpha,
            wrist_position_visual_alpha=args.wrist_position_visual_alpha,
            wrist_visual_correction_alpha=args.wrist_visual_correction_alpha,
            wrist_prediction_gate_translation_m=args.wrist_prediction_gate_translation_m,
            wrist_prediction_gate_rotation_deg=args.wrist_prediction_gate_rotation_deg,
            wrist_static_gyro_thresh_radps=args.wrist_static_gyro_thresh_radps,
            wrist_static_accel_std_thresh_mps2=args.wrist_static_accel_std_thresh_mps2,
            multi_camera_gate_translation_m=args.multi_camera_gate_translation_m,
            multi_camera_gate_rotation_deg=args.multi_camera_gate_rotation_deg,
            multi_camera_median_gate_m=args.multi_camera_median_gate_m,
        )
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        run_live_world_anchor(
            sources=args.source,
            cameras_path=Path(args.cameras),
            world_tags_path=Path(args.world_tags),
            bracelet_path=Path(args.bracelet),
            fps=args.fps,
            width=args.width,
            height=args.height,
            output_jsonl=Path(args.output_jsonl) if args.output_jsonl else None,
            max_reprojection_error_px=args.max_reprojection_error_px,
            show=args.show,
            ros_publish=args.ros_publish,
            fixed_frame=args.fixed_frame,
            head_frame=args.head_frame,
            wrist_frame=args.wrist_frame,
            max_path_length=args.max_path_length,
            head_imu_live_jsonl=Path(args.head_imu_live_jsonl) if args.head_imu_live_jsonl else None,
            wrist_imu_live_jsonl=Path(args.wrist_imu_live_jsonl) if args.wrist_imu_live_jsonl else None,
            live_imu_buffer_s=args.live_imu_buffer_s,
            max_prediction_gap_s=args.max_prediction_gap_s,
            max_wrist_prediction_gap_s=args.max_wrist_prediction_gap_s,
            enable_hands=args.hands,
            max_hands=args.max_hands,
            max_hand_skeletons_per_frame=args.max_hand_skeletons_per_frame,
            hand_continuity_gate_m=args.hand_continuity_gate_m,
            hand_continuity_max_gap_s=args.hand_continuity_max_gap_s,
            hand_multiview_only=args.hand_multiview_only,
            hand_allow_direct_singleview_fallback=args.hand_allow_direct_singleview_fallback,
            hand_detection_confidence=args.hand_detection_confidence,
            hand_tracking_confidence=args.hand_tracking_confidence,
            hand_model=Path(args.hand_model) if args.hand_model else None,
            head_position_visual_alpha=args.head_position_visual_alpha,
            head_orientation_visual_alpha=args.head_orientation_visual_alpha,
            max_position_prediction_gap_s=args.max_position_prediction_gap_s,
            wrist_position_visual_alpha=args.wrist_position_visual_alpha,
            wrist_visual_correction_alpha=args.wrist_visual_correction_alpha,
            wrist_prediction_gate_translation_m=args.wrist_prediction_gate_translation_m,
            wrist_prediction_gate_rotation_deg=args.wrist_prediction_gate_rotation_deg,
            wrist_static_gyro_thresh_radps=args.wrist_static_gyro_thresh_radps,
            wrist_static_accel_std_thresh_mps2=args.wrist_static_accel_std_thresh_mps2,
            multi_camera_gate_translation_m=args.multi_camera_gate_translation_m,
            multi_camera_gate_rotation_deg=args.multi_camera_gate_rotation_deg,
            multi_camera_median_gate_m=args.multi_camera_median_gate_m,
        )


if __name__ == "__main__":
    main()
