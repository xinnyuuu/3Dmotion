from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass
class CameraSource:
    camera_id: str
    source: int | str


@dataclass
class FrameRecord:
    group_id: int
    camera_id: str
    timestamp_unix_ns: int
    timestamp_monotonic_ns: int
    timestamp_source: str
    skew_us: float
    image_path: str
    width: int
    height: int


class QuadCameraCapture:
    """Near-synchronous OpenCV capture for four unsynchronized cameras."""

    def __init__(
        self,
        sources: list[CameraSource],
        output_dir: Path,
        target_fps: float = 30.0,
        width: int | None = None,
        height: int | None = None,
    ) -> None:
        if len(sources) != 4:
            raise ValueError("QuadCameraCapture expects exactly four sources.")
        if target_fps <= 0:
            raise ValueError("target_fps must be positive.")
        self.sources = sources
        self.output_dir = output_dir
        self.target_fps = target_fps
        self.width = width
        self.height = height

    def run(self, duration_s: float | None = None) -> None:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("Install opencv-python to capture camera frames.") from exc

        self.output_dir.mkdir(parents=True, exist_ok=True)
        manifest_path = self.output_dir / "frames.jsonl"
        captures = []
        for source in self.sources:
            cap = cv2.VideoCapture(source.source)
            if not cap.isOpened():
                raise RuntimeError(f"Could not open camera {source.camera_id}: {source.source}")
            if self.width is not None:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            if self.height is not None:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.target_fps)
            captures.append((source, cap))

        frame_period = 1.0 / self.target_fps
        next_frame_time = time.monotonic()
        start = time.monotonic()
        group_id = 0
        try:
            while duration_s is None or time.monotonic() - start < duration_s:
                for _source, cap in captures:
                    cap.grab()

                retrieved = []
                for source, cap in captures:
                    ok, frame = cap.retrieve()
                    now_unix_ns = time.time_ns()
                    now_mono_ns = time.monotonic_ns()
                    if not ok:
                        continue
                    retrieved.append((source, frame, now_unix_ns, now_mono_ns))

                if retrieved:
                    group_center_ns = int(sum(item[3] for item in retrieved) / len(retrieved))
                    records = []
                    for source, frame, unix_ns, mono_ns in retrieved:
                        camera_dir = self.output_dir / source.camera_id
                        camera_dir.mkdir(parents=True, exist_ok=True)
                        image_name = f"{group_id:08d}.jpg"
                        image_path = camera_dir / image_name
                        cv2.imwrite(str(image_path), frame)
                        height, width = frame.shape[:2]
                        records.append(
                            FrameRecord(
                                group_id=group_id,
                                camera_id=source.camera_id,
                                timestamp_unix_ns=unix_ns,
                                timestamp_monotonic_ns=mono_ns,
                                timestamp_source="host_retrieve",
                                skew_us=(mono_ns - group_center_ns) / 1000.0,
                                image_path=str(image_path.relative_to(self.output_dir)),
                                width=width,
                                height=height,
                            )
                        )
                    with manifest_path.open("a", encoding="utf-8") as f:
                        for record in records:
                            f.write(json.dumps(asdict(record), separators=(",", ":")) + "\n")
                    group_id += 1

                next_frame_time += frame_period
                sleep_s = next_frame_time - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_frame_time = time.monotonic()
        finally:
            for _source, cap in captures:
                cap.release()


def parse_sources(values: list[str]) -> list[CameraSource]:
    sources = []
    for index, value in enumerate(values):
        if ":" in value:
            camera_id, raw_source = value.split(":", 1)
        else:
            camera_id, raw_source = f"C{index}", value
        try:
            source: int | str = int(raw_source)
        except ValueError:
            source = raw_source
        sources.append(CameraSource(camera_id=camera_id, source=source))
    return sources


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture four camera streams with host timestamps.")
    parser.add_argument("--source", action="append", required=True, help="Camera source, e.g. C0:0. Repeat four times.")
    parser.add_argument("--output-dir", default="data/raw/quad_camera_session", help="Output directory.")
    parser.add_argument("--fps", type=float, default=30.0, help="Target capture FPS, typically 15-30.")
    parser.add_argument("--duration-s", type=float, default=None, help="Optional capture duration.")
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    args = parser.parse_args()

    capture = QuadCameraCapture(
        sources=parse_sources(args.source),
        output_dir=Path(args.output_dir),
        target_fps=args.fps,
        width=args.width,
        height=args.height,
    )
    capture.run(duration_s=args.duration_s)


if __name__ == "__main__":
    main()

