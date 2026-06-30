#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from packages.head_vio_bridge.openvins_session import replay_openvins_session_ros2


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Replay prepared OpenVINS image/IMU JSONL streams directly to ROS2 without writing rosbag2."
    )
    parser.add_argument("--prepared-dir", required=True, help="Directory containing images.jsonl and imu.jsonl.")
    parser.add_argument("--frame-id", default="head_imu", help="Frame ID for IMU messages. Image frame IDs use camera IDs.")
    parser.add_argument("--max-duration-s", type=float, help="Optional debug replay duration in seconds.")
    parser.add_argument("--start-offset-s", type=float, default=0.0, help="Skip this many seconds from the beginning before replay.")
    parser.add_argument("--image-stride", type=int, default=1, help="Publish every Nth image while keeping all IMU samples.")
    parser.add_argument(
        "--rate",
        type=float,
        default=0.0,
        help="Replay speed multiplier. 1.0 is recorded time; 0 publishes as fast as possible. Default: 0.",
    )
    parser.add_argument("--start-delay-s", type=float, default=2.0, help="Wait for subscribers before publishing. Default: 2.")
    parser.add_argument("--progress-every", type=int, default=500, help="Print replay progress every N events. Use 0 to disable.")
    parser.add_argument("--head-pose-topic", default="/ov_msckf/poseimu", help="OpenVINS pose topic to record.")
    parser.add_argument("--head-pose-path", help="Output head_pose.jsonl path. Default: <prepared-dir>/head_pose.jsonl.")
    parser.add_argument(
        "--reliability",
        choices=("reliable", "best_effort"),
        default="reliable",
        help="Publisher QoS reliability for image and IMU topics. OpenVINS normally requires reliable. Default: reliable.",
    )
    args = parser.parse_args()

    try:
        summary = replay_openvins_session_ros2(
            prepared_dir=Path(args.prepared_dir),
            frame_id=args.frame_id,
            max_duration_s=args.max_duration_s,
            start_offset_s=args.start_offset_s,
            image_stride=args.image_stride,
            rate=args.rate,
            start_delay_s=args.start_delay_s,
            progress_every=args.progress_every,
            head_pose_topic=args.head_pose_topic,
            head_pose_path=Path(args.head_pose_path) if args.head_pose_path else None,
            reliability=args.reliability,
        )
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        raise SystemExit(str(exc)) from exc
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
