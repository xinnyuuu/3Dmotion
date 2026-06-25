from __future__ import annotations

import argparse
import json
from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from .config import BraceletConfig, CameraCalibration, load_bracelet_config, load_camera_calibrations
from .detector import AprilTagDetector, TagPose, estimate_tag_pose
from .geometry import RigidTransform, average_transforms, transform_to_dict


@dataclass
class VisualCandidate:
    group_id: int
    camera_id: str
    tag_id: int
    timestamp_monotonic_ns: int
    timestamp_unix_ns: int
    reprojection_error_px: float
    weight: float
    T_H_B: dict
    corners: list[list[float]]


@dataclass
class WristVisualPose:
    group_id: int
    timestamp_monotonic_ns: int
    timestamp_unix_ns: int
    timestamp_source: str
    source_count: int
    source_camera_ids: list[str]
    source_tag_ids: list[int]
    mean_reprojection_error_px: float
    T_H_B: dict


def process_session(
    session_dir: Path,
    cameras_path: Path,
    bracelet_path: Path,
    output_dir: Path,
    max_reprojection_error_px: float = 8.0,
) -> None:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("Install opencv-contrib-python before running AprilTag processing.") from exc

    camera_calibrations = load_camera_calibrations(cameras_path)
    if not camera_calibrations:
        raise RuntimeError(f"No calibrated cameras found in {cameras_path}. Fill intrinsics and T_H_C first.")
    bracelet = load_bracelet_config(bracelet_path)
    detector = AprilTagDetector(bracelet.tag_family)

    output_dir.mkdir(parents=True, exist_ok=True)
    candidates_path = output_dir / "wrist_visual_candidates.jsonl"
    fused_path = output_dir / "wrist_visual_pose.jsonl"
    frames_by_group = _read_frames(session_dir / "frames.jsonl")

    with candidates_path.open("w", encoding="utf-8") as candidates_file, fused_path.open("w", encoding="utf-8") as fused_file:
        for group_id in sorted(frames_by_group):
            candidates: list[VisualCandidate] = []
            weighted_transforms: list[tuple[RigidTransform, float]] = []
            frame_records = frames_by_group[group_id]
            for frame_record in frame_records:
                camera_id = frame_record["camera_id"]
                calibration = camera_calibrations.get(camera_id)
                if calibration is None:
                    continue
                if not calibration.supported_opencv_projection:
                    raise RuntimeError(
                        f"Camera {camera_id} uses projection_model={calibration.projection_model!r}. "
                        "This AprilTag processor currently supports pinhole/radtan and fisheye/equidistant only. "
                        "Use the OpenCV fisheye fallback config, or add a projection adapter for this Kalibr model."
                    )
                image_path = session_dir / frame_record["image_path"]
                image = cv2.imread(str(image_path))
                if image is None:
                    continue
                frame_width = int(frame_record.get("width") or image.shape[1])
                frame_height = int(frame_record.get("height") or image.shape[0])
                camera_matrix = _scaled_camera_matrix(calibration, frame_width, frame_height)
                for tag_pose in _estimate_frame_tags(detector, image, bracelet, calibration, camera_matrix):
                    if tag_pose.reprojection_error_px > max_reprojection_error_px:
                        continue
                    T_H_B = _tag_pose_to_wrist_pose(tag_pose, calibration, bracelet)
                    weight = 1.0 / max(tag_pose.reprojection_error_px, 0.25)
                    candidate = VisualCandidate(
                        group_id=group_id,
                        camera_id=camera_id,
                        tag_id=tag_pose.detection.tag_id,
                        timestamp_monotonic_ns=int(frame_record["timestamp_monotonic_ns"]),
                        timestamp_unix_ns=int(frame_record["timestamp_unix_ns"]),
                        reprojection_error_px=tag_pose.reprojection_error_px,
                        weight=weight,
                        T_H_B=transform_to_dict(T_H_B),
                        corners=tag_pose.detection.corners,
                    )
                    candidates.append(candidate)
                    weighted_transforms.append((T_H_B, weight))
                    candidates_file.write(json.dumps(asdict(candidate), separators=(",", ":")) + "\n")

            if not weighted_transforms:
                continue
            fused = average_transforms(weighted_transforms)
            timestamp_monotonic_ns = int(sum(c.timestamp_monotonic_ns for c in candidates) / len(candidates))
            timestamp_unix_ns = int(sum(c.timestamp_unix_ns for c in candidates) / len(candidates))
            pose = WristVisualPose(
                group_id=group_id,
                timestamp_monotonic_ns=timestamp_monotonic_ns,
                timestamp_unix_ns=timestamp_unix_ns,
                timestamp_source="camera_group_average",
                source_count=len(candidates),
                source_camera_ids=sorted({candidate.camera_id for candidate in candidates}),
                source_tag_ids=sorted({candidate.tag_id for candidate in candidates}),
                mean_reprojection_error_px=float(np.mean([candidate.reprojection_error_px for candidate in candidates])),
                T_H_B=transform_to_dict(fused),
            )
            fused_file.write(json.dumps(asdict(pose), separators=(",", ":")) + "\n")


def _estimate_frame_tags(
    detector: AprilTagDetector,
    image: np.ndarray,
    bracelet: BraceletConfig,
    calibration: CameraCalibration,
    camera_matrix: np.ndarray,
) -> list[TagPose]:
    poses = []
    for detection in detector.detect(image):
        if bracelet.tag_to_wrist and detection.tag_id not in bracelet.tag_to_wrist:
            continue
        poses.append(
            estimate_tag_pose(
                detection,
                bracelet.tag_size_m,
                camera_matrix,
                calibration.distortion,
                calibration.distortion_model,
                calibration.xi if calibration.uses_omni_projection else None,
            )
        )
    return poses


def _scaled_camera_matrix(calibration: CameraCalibration, width: int, height: int) -> np.ndarray:
    if calibration.image_size is None:
        return calibration.intrinsics
    calibration_width, calibration_height = calibration.image_size
    if calibration_width == width and calibration_height == height:
        return calibration.intrinsics
    sx = width / float(calibration_width)
    sy = height / float(calibration_height)
    scaled = calibration.intrinsics.copy()
    scaled[0, 0] *= sx
    scaled[0, 2] *= sx
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled


def _tag_pose_to_wrist_pose(
    tag_pose: TagPose,
    calibration: CameraCalibration,
    bracelet: BraceletConfig,
) -> RigidTransform:
    import cv2

    rotation, _ = cv2.Rodrigues(tag_pose.rvec)
    T_C_T = RigidTransform(rotation=rotation, translation=tag_pose.tvec.reshape(3))
    T_T_B = bracelet.tag_to_wrist.get(
        tag_pose.detection.tag_id,
        RigidTransform(rotation=np.eye(3, dtype=np.float64), translation=np.array([0.0, 0.0, -bracelet.center_offset_m])),
    )
    return calibration.T_H_C @ T_C_T @ T_T_B


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


def main() -> None:
    parser = argparse.ArgumentParser(description="Process recorded quad-camera frames into wrist AprilTag visual poses.")
    parser.add_argument("--session-dir", required=True, help="Camera directory containing frames.jsonl, e.g. data/raw/session_xxx/cameras.")
    parser.add_argument("--cameras", default="configs/cameras.yaml", help="Camera calibration YAML.")
    parser.add_argument("--bracelet", default="configs/bracelet.yaml", help="Bracelet geometry YAML.")
    parser.add_argument("--output-dir", default="data/processed/wrist_visual", help="Output directory for JSONL pose files.")
    parser.add_argument("--max-reprojection-error-px", type=float, default=8.0)
    args = parser.parse_args()

    process_session(
        session_dir=Path(args.session_dir),
        cameras_path=Path(args.cameras),
        bracelet_path=Path(args.bracelet),
        output_dir=Path(args.output_dir),
        max_reprojection_error_px=args.max_reprojection_error_px,
    )


if __name__ == "__main__":
    main()
