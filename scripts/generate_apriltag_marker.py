#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a printable AprilTag marker image using OpenCV aruco.")
    parser.add_argument("--family", default="tag36h11", choices=["tag16h5", "tag25h9", "tag36h10", "tag36h11"])
    parser.add_argument("--tag-id", type=int, default=100)
    parser.add_argument("--pixels", type=int, default=800)
    parser.add_argument("--border-bits", type=int, default=1)
    parser.add_argument("--output", default="data/processed/world_tag_100.png")
    args = parser.parse_args()

    import cv2

    dictionary_name = {
        "tag16h5": "DICT_APRILTAG_16h5",
        "tag25h9": "DICT_APRILTAG_25h9",
        "tag36h10": "DICT_APRILTAG_36h10",
        "tag36h11": "DICT_APRILTAG_36h11",
    }[args.family]
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    image = cv2.aruco.generateImageMarker(dictionary, args.tag_id, args.pixels, borderBits=args.border_bits)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), image):
        raise RuntimeError(f"Failed to write marker image: {output}")
    print(output)


if __name__ == "__main__":
    main()
