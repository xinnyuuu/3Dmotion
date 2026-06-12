#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from packages.quad_camera_capture.v4l2 import find_capture_devices, get_camera_config


def main() -> None:
    parser = argparse.ArgumentParser(description="List V4L2 camera devices and supported formats.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON.")
    parser.add_argument("--configs", action="store_true", help="Include supported formats, sizes, and FPS.")
    args = parser.parse_args()

    devices = find_capture_devices()
    rows = []
    for device in devices:
        row = device.as_dict()
        if args.configs:
            row["configs"] = get_camera_config(device.path)
        rows.append(row)

    if args.json:
        print(json.dumps(rows, ensure_ascii=False, indent=2))
        return

    if not rows:
        print("No V4L2 capture devices found.")
        return
    for row in rows:
        print(f"{row['path']}\t{row['name']}\t{row['bus']}")
        if args.configs:
            for fmt in row.get("configs", []):
                sizes = ", ".join(
                    f"{size['width']}x{size['height']}@{('/'.join(map(str, size['fps'])) or '?')}fps"
                    for size in fmt.get("sizes", [])
                )
                print(f"  {fmt['format'].strip() or fmt['format']}: {sizes}")


if __name__ == "__main__":
    main()

