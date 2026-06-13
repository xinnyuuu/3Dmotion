#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from packages.head_vio_bridge.openvins_session import write_openvins_rosbag2


def main() -> None:
    parser = argparse.ArgumentParser(description="Write prepared OpenVINS image/IMU JSONL streams to rosbag2.")
    parser.add_argument("--prepared-dir", required=True, help="Directory containing images.jsonl and imu.jsonl.")
    parser.add_argument("--bag-dir", default="data/processed/openvins_session/rosbag2", help="Output rosbag2 directory.")
    parser.add_argument("--frame-id", default="headset", help="Frame ID for IMU messages. Image frame IDs use camera IDs.")
    args = parser.parse_args()

    try:
        summary = write_openvins_rosbag2(
            prepared_dir=Path(args.prepared_dir),
            bag_dir=Path(args.bag_dir),
            frame_id=args.frame_id,
        )
    except Exception as exc:
        raise SystemExit(str(exc)) from exc
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
