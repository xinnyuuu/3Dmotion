from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .geometry import RigidTransform, parse_transform, rotation_y


@dataclass
class CameraCalibration:
    camera_id: str
    image_size: tuple[int, int] | None
    intrinsics: np.ndarray
    distortion: np.ndarray
    T_H_C: RigidTransform


@dataclass
class BraceletConfig:
    tag_family: str
    tag_size_m: float
    center_offset_m: float
    tag_order: list[int]
    tag_to_wrist: dict[int, RigidTransform]
    fallback_orientation_mode: str


def load_camera_calibrations(path: Path) -> dict[str, CameraCalibration]:
    data = _load_yaml(path)
    defaults = data.get("camera_defaults", {})
    cameras = data.get("cameras", {})
    result = {}
    for camera_id, cfg in cameras.items():
        intrinsics = cfg.get("intrinsics") or defaults.get("intrinsics")
        distortion = cfg.get("distortion") or defaults.get("distortion")
        image_size = cfg.get("image_size") or defaults.get("image_size")
        if intrinsics is None:
            continue
        result[camera_id] = CameraCalibration(
            camera_id=camera_id,
            image_size=tuple(int(v) for v in image_size) if image_size is not None else None,
            intrinsics=np.asarray(intrinsics, dtype=np.float64).reshape(3, 3),
            distortion=np.asarray(distortion if distortion is not None else [0, 0, 0, 0, 0], dtype=np.float64).reshape(-1, 1),
            T_H_C=parse_transform(cfg.get("T_H_C"), default=RigidTransform.identity()),
        )
    return result


def load_bracelet_config(path: Path) -> BraceletConfig:
    data = _load_yaml(path)
    tag_family = str(data.get("tag_family", "tag36h11")).lower()
    tag_size_m = float(data["tag_size_m"])
    center_offset_m = _resolve_center_offset(data)
    tag_order = [int(tag_id) for tag_id in data.get("tag_order", [])]
    fallback_orientation_mode = str(data.get("fallback_orientation_mode", "tag_orientation"))
    explicit = data.get("tag_to_wrist_transforms") or {}
    tag_to_wrist = {}
    if explicit:
        for tag_id, transform in explicit.items():
            tag_to_wrist[int(tag_id)] = parse_transform(transform)
    elif tag_order:
        offset_sign = int(data.get("ring_offset_sign", -1))
        order_direction = int(data.get("ring_order_direction", 1))
        for index, tag_id in enumerate(tag_order):
            # This fallback mirrors the existing AprilTag prototype convention:
            # local +/-Z points from the visible tag face toward the bracelet center,
            # and neighboring faces rotate around the local Y axis.
            angle = order_direction * index * (2.0 * np.pi / len(tag_order))
            tag_to_wrist[tag_id] = RigidTransform(
                rotation=rotation_y(angle),
                translation=np.array([0.0, 0.0, offset_sign * center_offset_m], dtype=np.float64),
            )
    return BraceletConfig(
        tag_family=tag_family,
        tag_size_m=tag_size_m,
        center_offset_m=center_offset_m,
        tag_order=tag_order,
        tag_to_wrist=tag_to_wrist,
        fallback_orientation_mode=fallback_orientation_mode,
    )


def _resolve_center_offset(data: dict) -> float:
    if data.get("center_offset_m") is not None:
        return float(data["center_offset_m"])
    if data.get("flat_to_flat_m") is not None:
        return float(data["flat_to_flat_m"]) / 2.0
    if data.get("side_m") is not None:
        return float(data["side_m"]) * np.sqrt(3.0) / 2.0
    raise ValueError("Bracelet config needs center_offset_m, flat_to_flat_m, or side_m.")


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}
