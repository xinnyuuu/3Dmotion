#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


PAPER_SIZES_MM = {
    "A4": (210.0, 297.0),
    "LETTER": (215.9, 279.4),
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a printable AprilTag grid and matching world_tags.yaml.")
    parser.add_argument("--family", default="tag36h11", choices=["tag16h5", "tag25h9", "tag36h10", "tag36h11"])
    parser.add_argument("--start-id", type=int, default=100)
    parser.add_argument("--rows", type=int, default=3)
    parser.add_argument("--cols", type=int, default=4)
    parser.add_argument("--tag-size-mm", type=float, default=40.0)
    parser.add_argument("--gap-mm", type=float, default=12.0)
    parser.add_argument("--paper", default="A4", choices=sorted(PAPER_SIZES_MM))
    parser.add_argument("--landscape", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--border-bits", type=int, default=1)
    parser.add_argument("--output", default="data/processed/world_tag_grid_a4.png")
    parser.add_argument("--config", default="configs/world_tags.yaml")
    parser.add_argument("--no-labels", action="store_true", help="Do not print small tag id labels under markers.")
    args = parser.parse_args()

    if args.rows <= 0 or args.cols <= 0:
        raise ValueError("--rows and --cols must be positive")
    if args.tag_size_mm <= 0 or args.gap_mm < 0:
        raise ValueError("--tag-size-mm must be positive and --gap-mm must be non-negative")

    import cv2

    page_w_mm, page_h_mm = PAPER_SIZES_MM[args.paper]
    if args.landscape:
        page_w_mm, page_h_mm = page_h_mm, page_w_mm
    px_per_mm = args.dpi / 25.4
    page_w_px = int(round(page_w_mm * px_per_mm))
    page_h_px = int(round(page_h_mm * px_per_mm))
    tag_px = int(round(args.tag_size_mm * px_per_mm))
    gap_px = int(round(args.gap_mm * px_per_mm))

    grid_w_px = args.cols * tag_px + (args.cols - 1) * gap_px
    grid_h_px = args.rows * tag_px + (args.rows - 1) * gap_px
    if grid_w_px > page_w_px or grid_h_px > page_h_px:
        raise ValueError(
            f"Grid does not fit {args.paper}: grid={grid_w_px}x{grid_h_px}px page={page_w_px}x{page_h_px}px"
        )

    dictionary = _opencv_apriltag_dictionary(cv2, args.family)
    max_id = int(dictionary.bytesList.shape[0]) - 1
    last_id = args.start_id + args.rows * args.cols - 1
    if last_id > max_id:
        raise ValueError(f"{args.family} supports ids 0..{max_id}, but requested through id {last_id}")

    canvas = np.full((page_h_px, page_w_px), 255, dtype=np.uint8)
    x0 = (page_w_px - grid_w_px) // 2
    y0 = (page_h_px - grid_h_px) // 2
    font = cv2.FONT_HERSHEY_SIMPLEX
    tag_ids = []
    for row in range(args.rows):
        for col in range(args.cols):
            tag_id = args.start_id + row * args.cols + col
            tag_ids.append(tag_id)
            marker = cv2.aruco.generateImageMarker(dictionary, tag_id, tag_px, borderBits=args.border_bits)
            x = x0 + col * (tag_px + gap_px)
            y = y0 + row * (tag_px + gap_px)
            canvas[y : y + tag_px, x : x + tag_px] = marker
            if not args.no_labels:
                cv2.putText(
                    canvas,
                    str(tag_id),
                    (x + max(2, tag_px // 20), y + tag_px + max(18, tag_px // 12)),
                    font,
                    max(0.45, tag_px / 360.0),
                    0,
                    max(1, tag_px // 180),
                    cv2.LINE_AA,
                )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), canvas):
        raise RuntimeError(f"Failed to write grid image: {output}")

    config = Path(args.config)
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text(
        _world_tags_yaml(
            family=args.family,
            tag_size_m=args.tag_size_mm / 1000.0,
            rows=args.rows,
            cols=args.cols,
            start_id=args.start_id,
            pitch_m=(args.tag_size_mm + args.gap_mm) / 1000.0,
            paper=args.paper,
            landscape=args.landscape,
        ),
        encoding="utf-8",
    )
    print(f"image: {output}")
    print(f"config: {config}")
    print(f"ids: {tag_ids[0]}..{tag_ids[-1]}")


def _opencv_apriltag_dictionary(cv2, family: str):
    name = {
        "tag16h5": "DICT_APRILTAG_16h5",
        "tag25h9": "DICT_APRILTAG_25h9",
        "tag36h10": "DICT_APRILTAG_36h10",
        "tag36h11": "DICT_APRILTAG_36h11",
    }[family]
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def _world_tags_yaml(
    *,
    family: str,
    tag_size_m: float,
    rows: int,
    cols: int,
    start_id: int,
    pitch_m: float,
    paper: str,
    landscape: bool,
) -> str:
    lines = [
        f"tag_family: {family}",
        f"tag_size_m: {tag_size_m:.9f}",
        "",
        "# Generated by scripts/generate_apriltag_grid.py",
        f"# paper: {paper} {'landscape' if landscape else 'portrait'}",
        f"# grid: {rows} rows x {cols} cols",
        "# World frame W: origin at grid center, +X right on paper, +Y up on paper, +Z out of paper.",
        "# T_W_T maps points from each tag frame T into world frame W.",
        "tags:",
    ]
    for row in range(rows):
        for col in range(cols):
            tag_id = start_id + row * cols + col
            x = (col - (cols - 1) / 2.0) * pitch_m
            y = ((rows - 1) / 2.0 - row) * pitch_m
            lines.extend(
                [
                    f"  {tag_id}:",
                    "    T_W_T:",
                    f"      - [1.0, 0.0, 0.0, {x:.9f}]",
                    f"      - [0.0, 1.0, 0.0, {y:.9f}]",
                    "      - [0.0, 0.0, 1.0, 0.0]",
                    "      - [0.0, 0.0, 0.0, 1.0]",
                ]
            )
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    main()
