from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class TagDetection:
    tag_id: int
    family: str
    corners: list[list[float]]
    center: list[float]
    backend: str


@dataclass
class TagPose:
    detection: TagDetection
    rvec: np.ndarray
    tvec: np.ndarray
    reprojection_error_px: float


class AprilTagDetector:
    def __init__(self, family: str) -> None:
        self.family = family.lower()
        self.backend = "opencv-aruco"
        self._detector = None
        self._dictionary = None

    def _ensure_detector(self):
        if self._detector is not None or self._dictionary is not None:
            return
        import cv2

        dictionary = _opencv_apriltag_dictionary(cv2, self.family)
        if hasattr(cv2.aruco, "DetectorParameters"):
            params = cv2.aruco.DetectorParameters()
        else:
            params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detector = cv2.aruco.ArucoDetector(dictionary, params)
        else:
            self._dictionary = (dictionary, params)

    def detect(self, image: np.ndarray) -> list[TagDetection]:
        import cv2

        self._ensure_detector()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        if self._detector is not None:
            corners, ids, _rejected = self._detector.detectMarkers(gray)
        else:
            dictionary, params = self._dictionary
            corners, ids, _rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        if ids is None:
            return []
        detections = []
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            pts = marker_corners.reshape(4, 2).astype(float)
            detections.append(
                TagDetection(
                    tag_id=int(marker_id),
                    family=self.family,
                    corners=pts.tolist(),
                    center=pts.mean(axis=0).tolist(),
                    backend=self.backend,
                )
            )
        return detections


def estimate_tag_pose(
    detection: TagDetection,
    tag_size_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    distortion_model: str = "radtan",
    xi: float | None = None,
) -> TagPose:
    import cv2

    object_points = tag_object_points(tag_size_m)
    image_points = np.asarray(detection.corners, dtype=np.float64).reshape(4, 2)
    pnp_image_points, pnp_dist_coeffs = _pnp_points_and_distortion(
        image_points,
        camera_matrix,
        dist_coeffs,
        distortion_model,
        xi,
    )
    pnp_camera_matrix = np.eye(3, dtype=np.float64) if _is_omni_distortion(distortion_model, xi) else camera_matrix
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

    candidates = []
    if ok:
        for rvec, tvec in zip(rvecs, tvecs):
            rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
            tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
            if float(tvec[2, 0]) <= 0:
                continue
            candidates.append(
                TagPose(
                    detection=detection,
                    rvec=rvec,
                    tvec=tvec,
                    reprojection_error_px=reprojection_error(
                        object_points,
                        image_points,
                        rvec,
                        tvec,
                        camera_matrix,
                        dist_coeffs,
                        distortion_model,
                        xi,
                    ),
                )
            )
    if not candidates:
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            pnp_image_points,
            pnp_camera_matrix,
            pnp_dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            success, rvec, tvec = cv2.solvePnP(object_points, pnp_image_points, pnp_camera_matrix, pnp_dist_coeffs)
        if not success:
            raise RuntimeError(f"solvePnP failed for tag {detection.tag_id}.")
        candidates.append(
            TagPose(
                detection=detection,
                rvec=np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                tvec=np.asarray(tvec, dtype=np.float64).reshape(3, 1),
                reprojection_error_px=reprojection_error(
                    object_points,
                    image_points,
                    rvec,
                    tvec,
                    camera_matrix,
                    dist_coeffs,
                    distortion_model,
                    xi,
                ),
            )
        )
    return min(candidates, key=lambda pose: pose.reprojection_error_px)


def tag_object_points(tag_size_m: float) -> np.ndarray:
    half = float(tag_size_m) / 2.0
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def reprojection_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    distortion_model: str = "radtan",
    xi: float | None = None,
) -> float:
    import cv2

    if _is_omni_distortion(distortion_model, xi):
        _require_omnidir(cv2)
        projected, _ = cv2.omnidir.projectPoints(
            object_points.reshape(-1, 1, 3),
            rvec,
            tvec,
            camera_matrix,
            float(xi),
            dist_coeffs.reshape(-1, 1),
        )
    elif _is_fisheye_distortion(distortion_model):
        projected, _ = cv2.fisheye.projectPoints(
            object_points.reshape(-1, 1, 3),
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs.reshape(-1, 1),
        )
    else:
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    return float(np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1).mean())


def _pnp_points_and_distortion(
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    distortion_model: str,
    xi: float | None,
) -> tuple[np.ndarray, np.ndarray]:
    if _is_omni_distortion(distortion_model, xi):
        import cv2

        _require_omnidir(cv2)
        undistorted = cv2.omnidir.undistortPoints(
            image_points.reshape(-1, 1, 2),
            camera_matrix,
            dist_coeffs.reshape(-1, 1),
            _omnidir_xi(xi),
            np.eye(3, dtype=np.float64),
        )
        return undistorted.reshape(-1, 2), np.zeros((4, 1), dtype=np.float64)

    if not _is_fisheye_distortion(distortion_model):
        return image_points, dist_coeffs

    import cv2

    undistorted = cv2.fisheye.undistortPoints(
        image_points.reshape(-1, 1, 2),
        camera_matrix,
        dist_coeffs.reshape(-1, 1),
        P=camera_matrix,
    )
    return undistorted.reshape(-1, 2), np.zeros((4, 1), dtype=np.float64)


def _is_fisheye_distortion(distortion_model: str) -> bool:
    return distortion_model.lower() in {"equidistant", "opencv_fisheye", "fisheye"}


def _is_omni_distortion(distortion_model: str, xi: float | None) -> bool:
    return xi is not None and distortion_model.lower() in {"radtan", "mei", "omni", "omnidir"}


def _omnidir_xi(xi: float | None) -> np.ndarray:
    if xi is None:
        raise ValueError("Omnidirectional projection requires xi.")
    return np.asarray([float(xi)], dtype=np.float64)


def _require_omnidir(cv2) -> None:
    if not hasattr(cv2, "omnidir"):
        raise RuntimeError(
            "This camera uses Mei/omni intrinsics with xi, but cv2.omnidir is unavailable. "
            "Install an OpenCV contrib build with omnidir support, or rectify the images to a virtual pinhole camera first."
        )


def _opencv_apriltag_dictionary(cv2, family: str):
    names = {
        "tag16h5": "DICT_APRILTAG_16h5",
        "tag25h9": "DICT_APRILTAG_25h9",
        "tag36h10": "DICT_APRILTAG_36h10",
        "tag36h11": "DICT_APRILTAG_36h11",
    }
    if family not in names:
        raise ValueError(f"Unsupported AprilTag family: {family}")
    dict_id = getattr(cv2.aruco, names[family])
    return cv2.aruco.getPredefinedDictionary(dict_id)
