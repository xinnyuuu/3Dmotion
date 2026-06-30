from __future__ import annotations

from dataclasses import asdict, dataclass
import os
from pathlib import Path

import cv2
import numpy as np


@dataclass
class HandKeypoint:
    index: int
    x: float
    y: float
    z: float
    visibility: float | None = None


@dataclass
class HandDetection:
    handedness: str
    score: float
    keypoints: list[HandKeypoint]


class HandKeypointDetector:
    def __init__(
        self,
        max_num_hands: int = 2,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
        model_path: str | None = None,
    ) -> None:
        os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
        try:
            import mediapipe as mp
        except ImportError as exc:
            raise RuntimeError("MediaPipe is required for hand keypoint detection. Install mediapipe first.") from exc

        self.mp = mp
        self.backend = "solutions" if hasattr(mp, "solutions") else "tasks"
        self.hands = None
        self.landmarker = None

        if self.backend == "solutions":
            self.hands = mp.solutions.hands.Hands(
                static_image_mode=False,
                max_num_hands=max_num_hands,
                model_complexity=1,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence,
            )
        else:
            if not model_path:
                default_model = Path("models/hand_landmarker.task")
                model_path = str(default_model) if default_model.exists() else None
            if not model_path or not Path(model_path).exists():
                raise RuntimeError(
                    "This MediaPipe install uses the Tasks API and requires a hand landmarker model. "
                    "Pass --hand-model models/hand_landmarker.task."
                )
            from mediapipe.tasks.python import BaseOptions, vision

            options = vision.HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=model_path, delegate=BaseOptions.Delegate.CPU),
                running_mode=vision.RunningMode.IMAGE,
                num_hands=max_num_hands,
                min_hand_detection_confidence=min_detection_confidence,
                min_hand_presence_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence,
            )
            self.landmarker = vision.HandLandmarker.create_from_options(options)

    def detect(self, frame_bgr: np.ndarray) -> list[HandDetection]:
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        if self.backend == "solutions":
            return self._detect_with_solutions(rgb)
        return self._detect_with_tasks(rgb)

    def _detect_with_solutions(self, rgb: np.ndarray) -> list[HandDetection]:
        rgb.flags.writeable = False
        results = self.hands.process(rgb)
        if not results.multi_hand_landmarks:
            return []

        detections: list[HandDetection] = []
        handedness_list = results.multi_handedness or []
        for hand_idx, landmarks in enumerate(results.multi_hand_landmarks):
            handedness = "Unknown"
            score = 0.0
            if hand_idx < len(handedness_list):
                classification = handedness_list[hand_idx].classification[0]
                handedness = classification.label
                score = float(classification.score)

            keypoints = [
                HandKeypoint(
                    index=i,
                    x=float(lm.x),
                    y=float(lm.y),
                    z=float(lm.z),
                    visibility=float(lm.visibility) if hasattr(lm, "visibility") else None,
                )
                for i, lm in enumerate(landmarks.landmark)
            ]
            detections.append(HandDetection(handedness=handedness, score=score, keypoints=keypoints))
        return detections

    def _detect_with_tasks(self, rgb: np.ndarray) -> list[HandDetection]:
        image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        results = self.landmarker.detect(image)
        if not results.hand_landmarks:
            return []

        detections: list[HandDetection] = []
        handedness_list = results.handedness or []
        for hand_idx, landmarks in enumerate(results.hand_landmarks):
            handedness = "Unknown"
            score = 0.0
            if hand_idx < len(handedness_list) and handedness_list[hand_idx]:
                classification = handedness_list[hand_idx][0]
                handedness = classification.category_name
                score = float(classification.score)
            keypoints = [
                HandKeypoint(
                    index=i,
                    x=float(lm.x),
                    y=float(lm.y),
                    z=float(lm.z),
                    visibility=None,
                )
                for i, lm in enumerate(landmarks)
            ]
            detections.append(HandDetection(handedness=handedness, score=score, keypoints=keypoints))
        return detections

    def close(self) -> None:
        if self.hands is not None:
            self.hands.close()
        if self.landmarker is not None:
            self.landmarker.close()


def hands_to_dicts(hands: list[HandDetection]) -> list[dict]:
    return [asdict(hand) for hand in hands]


HAND_CONNECTIONS = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (0, 5),
    (5, 6),
    (6, 7),
    (7, 8),
    (5, 9),
    (9, 10),
    (10, 11),
    (11, 12),
    (9, 13),
    (13, 14),
    (14, 15),
    (15, 16),
    (13, 17),
    (0, 17),
    (17, 18),
    (18, 19),
    (19, 20),
)
