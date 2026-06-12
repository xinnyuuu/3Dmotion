from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class RigidTransform:
    rotation: np.ndarray
    translation: np.ndarray

    @classmethod
    def identity(cls) -> "RigidTransform":
        return cls(rotation=np.eye(3, dtype=np.float64), translation=np.zeros(3, dtype=np.float64))

    @classmethod
    def from_matrix(cls, matrix: list[list[float]] | np.ndarray) -> "RigidTransform":
        arr = np.asarray(matrix, dtype=np.float64)
        if arr.shape != (4, 4):
            raise ValueError("Transform matrix must be 4x4.")
        return cls(rotation=arr[:3, :3], translation=arr[:3, 3])

    @classmethod
    def from_xyz_ypr_deg(cls, values: list[float]) -> "RigidTransform":
        if len(values) != 6:
            raise ValueError("Expected [x, y, z, yaw_deg, pitch_deg, roll_deg].")
        x, y, z, yaw_deg, pitch_deg, roll_deg = [float(v) for v in values]
        return cls(
            rotation=rotation_from_ypr(math.radians(yaw_deg), math.radians(pitch_deg), math.radians(roll_deg)),
            translation=np.array([x, y, z], dtype=np.float64),
        )

    def as_matrix(self) -> np.ndarray:
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = self.rotation
        matrix[:3, 3] = self.translation
        return matrix

    def inverse(self) -> "RigidTransform":
        rotation = self.rotation.T
        translation = -rotation @ self.translation
        return RigidTransform(rotation=rotation, translation=translation)

    def __matmul__(self, other: "RigidTransform") -> "RigidTransform":
        return RigidTransform(
            rotation=self.rotation @ other.rotation,
            translation=self.rotation @ other.translation + self.translation,
        )


def parse_transform(value, default: RigidTransform | None = None) -> RigidTransform:
    if value is None:
        if default is not None:
            return default
        raise ValueError("Missing transform.")
    if isinstance(value, str):
        values = [float(item) for item in value.replace(",", " ").split()]
        return RigidTransform.from_xyz_ypr_deg(values)
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape == (4, 4):
        return RigidTransform.from_matrix(arr)
    if arr.shape == (6,):
        return RigidTransform.from_xyz_ypr_deg(arr.tolist())
    raise ValueError("Transform must be a 4x4 matrix or [x, y, z, yaw_deg, pitch_deg, roll_deg].")


def rotation_from_ypr(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    return rz @ ry @ rx


def rotation_y(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def rotation_matrix_to_quat_wxyz(rotation: np.ndarray) -> list[float]:
    m = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    quat = np.array([qw, qx, qy, qz], dtype=np.float64)
    quat /= max(np.linalg.norm(quat), 1e-12)
    return quat.tolist()


def quat_wxyz_to_rotation_matrix(quat: list[float] | np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=np.float64)
    q /= max(np.linalg.norm(q), 1e-12)
    qw, qx, qy, qz = q
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def average_transforms(candidates: list[tuple[RigidTransform, float]]) -> RigidTransform:
    if not candidates:
        raise ValueError("No transform candidates to average.")
    weights = np.array([max(weight, 1e-9) for _pose, weight in candidates], dtype=np.float64)
    weights /= weights.sum()
    translation = sum(pose.translation * weight for (pose, _), weight in zip(candidates, weights))

    # Markley quaternion average for stable sign handling.
    accumulator = np.zeros((4, 4), dtype=np.float64)
    for (pose, _), weight in zip(candidates, weights):
        q = np.asarray(rotation_matrix_to_quat_wxyz(pose.rotation), dtype=np.float64)
        if q[0] < 0:
            q = -q
        accumulator += weight * np.outer(q, q)
    eigenvalues, eigenvectors = np.linalg.eigh(accumulator)
    quat = eigenvectors[:, int(np.argmax(eigenvalues))]
    if quat[0] < 0:
        quat = -quat
    return RigidTransform(rotation=quat_wxyz_to_rotation_matrix(quat), translation=translation)


def transform_to_dict(transform: RigidTransform) -> dict:
    return {
        "position": transform.translation.astype(float).tolist(),
        "orientation_wxyz": rotation_matrix_to_quat_wxyz(transform.rotation),
        "matrix": transform.as_matrix().astype(float).tolist(),
    }

