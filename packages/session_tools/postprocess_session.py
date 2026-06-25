from __future__ import annotations

import argparse
import json
from pathlib import Path

from packages.apriltag_ring_node.process_session import process_session as process_apriltag_session
from packages.head_vio_bridge.openvins_config import generate_openvins_config
from packages.head_vio_bridge.openvins_session import prepare_openvins_session
from packages.session_tools.motion_fusion import fuse_motion_session
from packages.session_tools.validate_session import validate_session


def postprocess_session(
    session_dir: Path,
    output_root: Path | None,
    cameras_path: Path,
    bracelet_path: Path,
    camera_ids: list[str] | None = None,
    run_apriltag: bool = False,
    run_openvins: bool = False,
    generate_config: bool = False,
    fuse_motion: bool = False,
    head_pose_path: Path | None = None,
) -> dict:
    session_dir = session_dir.resolve()
    if output_root is None:
        output_root = Path("data/processed") / session_dir.name
    output_root.mkdir(parents=True, exist_ok=True)
    summary = {
        "session_dir": str(session_dir),
        "output_root": str(output_root.resolve()),
        "validation": validate_session(session_dir),
        "steps": {},
    }
    if not summary["validation"]["ok_for_camera_replay"]:
        summary["steps"]["skipped"] = "Camera replay is not ready. Fix capture before running AprilTag/OpenVINS processing."
        _write_summary(output_root, summary)
        return summary

    if run_apriltag:
        apriltag_dir = output_root / "wrist_visual"
        process_apriltag_session(
            session_dir=session_dir / "cameras",
            cameras_path=cameras_path,
            bracelet_path=bracelet_path,
            output_dir=apriltag_dir,
        )
        summary["steps"]["apriltag"] = {
            "output_dir": str(apriltag_dir),
            "wrist_visual_candidates": str(apriltag_dir / "wrist_visual_candidates.jsonl"),
            "wrist_visual_pose": str(apriltag_dir / "wrist_visual_pose.jsonl"),
        }

    if run_openvins:
        openvins_dir = output_root / "openvins_session"
        summary["steps"]["openvins_session"] = prepare_openvins_session(
            session_dir=session_dir,
            output_dir=openvins_dir,
            camera_ids=camera_ids,
            imu_slot="head_imu",
        )

    if generate_config:
        config_dir = output_root / "openvins_config"
        summary["steps"]["openvins_config"] = generate_openvins_config(
            cameras_path=cameras_path,
            output_dir=config_dir,
            camera_ids=camera_ids,
        )

    if fuse_motion:
        summary["steps"]["motion_fusion"] = fuse_motion_session(
            session_dir=session_dir,
            output_root=output_root,
            head_pose_path=head_pose_path,
        )

    _write_summary(output_root, summary)
    return summary


def _write_summary(output_root: Path, summary: dict) -> None:
    (output_root / "postprocess_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate and optionally postprocess one recorded 3DMotion session.")
    parser.add_argument("--session-dir", required=True, help="Dashboard session directory, e.g. data/raw/session_YYYYMMDD_HHMMSS.")
    parser.add_argument("--output-root", default=None, help="Output root for all generated files. Default: data/processed/<session_name>.")
    parser.add_argument("--cameras", default="configs/cameras.yaml", help="Camera calibration YAML.")
    parser.add_argument("--bracelet", default="configs/bracelet.yaml", help="Bracelet geometry YAML.")
    parser.add_argument("--camera-id", action="append", dest="camera_ids", help="Camera ID for OpenVINS export/config. Repeat for multiple cameras. Default: C1,C2,C0,C3.")
    parser.add_argument("--apriltag", action="store_true", help="Run offline wrist AprilTag processing.")
    parser.add_argument("--openvins", action="store_true", help="Prepare OpenVINS image/IMU JSONL streams.")
    parser.add_argument("--openvins-config", action="store_true", help="Generate first-pass OpenVINS config files.")
    parser.add_argument("--fuse-motion", action="store_true", help="Fuse head_pose.jsonl, wrist_visual_pose.jsonl, and wrist_imu.jsonl.")
    parser.add_argument("--head-pose", help="Optional head_pose.jsonl path for --fuse-motion.")
    args = parser.parse_args()

    summary = postprocess_session(
        session_dir=Path(args.session_dir),
        output_root=Path(args.output_root) if args.output_root else None,
        cameras_path=Path(args.cameras),
        bracelet_path=Path(args.bracelet),
        camera_ids=args.camera_ids,
        run_apriltag=args.apriltag,
        run_openvins=args.openvins,
        generate_config=args.openvins_config,
        fuse_motion=args.fuse_motion,
        head_pose_path=Path(args.head_pose) if args.head_pose else None,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
