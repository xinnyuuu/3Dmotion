from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from .capture import CameraSource


def probe_camera_sources(
    sources: list[CameraSource],
    output_dir: Path,
    width: int | None = None,
    height: int | None = None,
    fps: float | None = None,
    warmup_frames: int = 5,
) -> dict:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("Install opencv-python to probe camera frames.") from exc

    output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for source in sources:
        result = {
            "camera_id": source.camera_id,
            "source": str(source.source),
            "fourcc": source.fourcc,
            "ok": False,
            "error": None,
            "image_path": None,
            "width": None,
            "height": None,
        }
        cap = cv2.VideoCapture(source.source, cv2.CAP_V4L2)
        try:
            if source.fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*source.fourcc[:4]))
            if width is not None:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height is not None:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if fps is not None:
                cap.set(cv2.CAP_PROP_FPS, fps)
            if not cap.isOpened():
                result["error"] = "open_failed"
                results.append(result)
                continue

            frame = None
            for _ in range(max(1, warmup_frames)):
                ok, candidate = cap.read()
                if ok:
                    frame = candidate
                time.sleep(0.02)
            if frame is None:
                result["error"] = "read_failed"
                results.append(result)
                continue

            image_path = output_dir / f"{source.camera_id}_probe.jpg"
            if not cv2.imwrite(str(image_path), frame):
                result["error"] = "write_failed"
                results.append(result)
                continue

            frame_height, frame_width = frame.shape[:2]
            result.update(
                {
                    "ok": True,
                    "image_path": str(image_path),
                    "width": int(frame_width),
                    "height": int(frame_height),
                }
            )
            results.append(result)
        finally:
            cap.release()

    summary = {
        "output_dir": str(output_dir),
        "ok": any(result["ok"] for result in results),
        "results": results,
    }
    (output_dir / "camera_preflight_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def parse_source_specs(values: list[str], fourcc: str | None = None) -> list[CameraSource]:
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
        sources.append(CameraSource(camera_id=camera_id, source=source, fourcc=fourcc))
    return sources


def main() -> None:
    parser = argparse.ArgumentParser(description="Probe selected cameras by opening each source and writing one test frame.")
    parser.add_argument("--source", action="append", required=True, help="Camera source, e.g. C0:/dev/video0. Repeat for multiple cameras.")
    parser.add_argument("--format", dest="fourcc", default=None, help="Optional FOURCC, e.g. MJPG or YUYV.")
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--output-dir", default="data/raw/camera_preflight", help="Directory for probe images and summary.")
    args = parser.parse_args()

    summary = probe_camera_sources(
        sources=parse_source_specs(args.source, fourcc=args.fourcc),
        output_dir=Path(args.output_dir),
        width=args.width,
        height=args.height,
        fps=args.fps,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
