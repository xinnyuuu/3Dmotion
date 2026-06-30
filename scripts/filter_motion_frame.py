#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from packages.session_tools.motion_frame_filter import filter_motion_frame


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a filtered derivative of motion_frame.jsonl.")
    parser.add_argument("motion_jsonl", help="Input motion_frame.jsonl.")
    parser.add_argument("--output", help="Default: motion_frame_filtered.jsonl next to input.")
    parser.add_argument("--accel-noise-mps2", type=float, default=0.8)
    parser.add_argument("--measurement-noise-m", type=float, default=0.015)
    parser.add_argument("--gate-mahalanobis", type=float, default=25.0)
    parser.add_argument("--hand-alpha", type=float, default=0.35)
    parser.add_argument("--hand-hold-s", type=float, default=0.25)
    args = parser.parse_args()

    input_path = Path(args.motion_jsonl)
    output_path = Path(args.output) if args.output else input_path.with_name("motion_frame_filtered.jsonl")
    summary = filter_motion_frame(
        input_path,
        output_path,
        accel_noise_mps2=args.accel_noise_mps2,
        measurement_noise_m=args.measurement_noise_m,
        gate_mahalanobis=args.gate_mahalanobis,
        hand_alpha=args.hand_alpha,
        hand_hold_s=args.hand_hold_s,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
