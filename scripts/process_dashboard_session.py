#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from packages.session_tools.postprocess_session import postprocess_session
from packages.session_tools.motion_frame_filter import filter_motion_frame


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Postprocess a dashboard recording with the current AprilGrid world-anchor workflow."
    )
    parser.add_argument(
        "session_dir",
        nargs="?",
        help="Dashboard session directory. Default: latest data/raw/session_* with cameras/frames.jsonl.",
    )
    parser.add_argument("--session-root", default="data/raw")
    parser.add_argument("--output-root", help="Default: data/processed/<session_name>.")
    parser.add_argument("--cameras", default="configs/cameras.yaml")
    parser.add_argument("--world-tags", default="configs/world_tags.yaml")
    parser.add_argument("--bracelet", default="configs/bracelet.yaml")
    parser.add_argument("--hands", action="store_true", help="Also detect and publish approximate hand skeletons.")
    parser.add_argument("--no-hands", action="store_true", help="Alias for the default; kept for explicit scripts.")
    parser.add_argument("--hand-multiview-only", action="store_true", help="Only output hand skeletons reconstructed from at least two cameras.")
    parser.add_argument("--hand-allow-direct-singleview-fallback", action="store_true", help="Allow raw single-camera wrist-depth hand output when no recent multiview/guided hand exists.")
    parser.add_argument("--filter", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--filter-accel-noise-mps2", type=float, default=0.8)
    parser.add_argument("--filter-measurement-noise-m", type=float, default=0.015)
    parser.add_argument("--filter-gate-mahalanobis", type=float, default=25.0)
    parser.add_argument("--print-rviz", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    session_dir = Path(args.session_dir) if args.session_dir else _latest_session(Path(args.session_root))
    output_root = Path(args.output_root) if args.output_root else None
    manifest = _read_json(session_dir / "session_manifest.json")
    if manifest:
        camera = manifest.get("camera") or {}
        print("Dashboard camera mapping:")
        print(json.dumps(camera.get("sources") or {}, ensure_ascii=False, indent=2))
        if camera.get("tile_configs"):
            print("Dashboard camera config:")
            print(json.dumps(camera["tile_configs"], ensure_ascii=False, indent=2))

    summary = postprocess_session(
        session_dir=session_dir,
        output_root=output_root,
        cameras_path=Path(args.cameras),
        bracelet_path=Path(args.bracelet),
        world_tags_path=Path(args.world_tags),
        run_world_anchor=True,
        enable_hands=bool(args.hands and not args.no_hands),
        hand_multiview_only=args.hand_multiview_only,
        hand_allow_direct_singleview_fallback=args.hand_allow_direct_singleview_fallback,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    world_anchor = summary.get("steps", {}).get("world_anchor") or {}
    motion_frame = world_anchor.get("motion_frame")
    if args.filter and motion_frame:
        filter_summary = filter_motion_frame(
            Path(motion_frame),
            Path(motion_frame).with_name("motion_frame_filtered.jsonl"),
            accel_noise_mps2=args.filter_accel_noise_mps2,
            measurement_noise_m=args.filter_measurement_noise_m,
            gate_mahalanobis=args.filter_gate_mahalanobis,
        )
        motion_frame = filter_summary["output"]
        print("\nFiltered motion:")
        print(json.dumps(filter_summary, ensure_ascii=False, indent=2))
    if args.print_rviz and motion_frame:
        print("\nRViz replay:")
        motion_path = Path(motion_frame).resolve()
        ros2_ws = REPO_ROOT / "ros2_ws"
        print(
            "bash -lc "
            + _shell_quote(
                f"cd {ros2_ws} && "
                "source /opt/ros/humble/setup.bash && "
                "source install/setup.bash && "
                "export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 && "
                "ros2 launch vimas_motion_bringup motion_replay_rviz.launch.py "
                f"motion_jsonl:={motion_path}"
            )
        )


def _latest_session(root: Path) -> Path:
    candidates = [
        path
        for path in root.glob("session_*")
        if (path / "cameras" / "frames.jsonl").exists()
    ]
    if not candidates:
        raise FileNotFoundError(f"No dashboard sessions with cameras/frames.jsonl under {root}")
    return max(candidates, key=lambda path: path.stat().st_mtime_ns)


def _read_json(path: Path) -> dict:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _shell_quote(value: str) -> str:
    return "'" + str(value).replace("'", "'\"'\"'") + "'"


if __name__ == "__main__":
    main()
