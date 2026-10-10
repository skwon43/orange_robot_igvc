#!/usr/bin/env python3
"""Export the deterministic Gazebo lane centerline as a metric CSV file."""

from __future__ import annotations

import argparse
import csv
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path


LINE_RE = re.compile(r"^white_line_(inner|outer)_(\d+)$")


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        default=root / "src/orange_gazebo/models/orange_igvc_2026/model.sdf",
        help="SDF model containing white_line_inner_* and white_line_outer_* links.",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--offset-x", type=float, default=0.0)
    parser.add_argument("--offset-y", type=float, default=0.0)
    return parser.parse_args()


def pose_xy(link: ET.Element) -> tuple[float, float]:
    pose = link.find("pose")
    if pose is None or not pose.text:
        raise ValueError(f"Link {link.get('name')} has no pose")
    values = [float(value) for value in pose.text.split()]
    if len(values) != 6:
        raise ValueError(f"Link {link.get('name')} has an invalid pose")
    return values[0], values[1]


def collect_centers(model: ET.Element) -> list[tuple[int, float, float, float]]:
    pairs: dict[int, dict[str, ET.Element]] = {}
    for link in model.findall("link"):
        match = LINE_RE.match(link.get("name", ""))
        if match:
            side, index = match.groups()
            pairs.setdefault(int(index), {})[side] = link

    centers = []
    for index, pair in sorted(pairs.items()):
        if set(pair) != {"inner", "outer"}:
            raise ValueError(f"Lane boundary pair {index} is incomplete")
        inner_x, inner_y = pose_xy(pair["inner"])
        outer_x, outer_y = pose_xy(pair["outer"])
        centers.append(
            (
                index,
                0.5 * (inner_x + outer_x),
                0.5 * (inner_y + outer_y),
                math.hypot(outer_x - inner_x, outer_y - inner_y),
            )
        )
    if len(centers) < 3:
        raise ValueError("At least three complete lane boundary pairs are required")
    return centers


def main() -> int:
    args = parse_args()
    root = ET.parse(args.model).getroot()
    model = root.find("model")
    if model is None:
        raise ValueError("SDF has no top-level model")
    centers = collect_centers(model)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["index", "x_m", "y_m", "heading_rad", "lane_width_m"],
        )
        writer.writeheader()
        for position, (index, x, y, width) in enumerate(centers):
            previous = centers[position - 1]
            following = centers[(position + 1) % len(centers)]
            heading = math.atan2(following[2] - previous[2], following[1] - previous[1])
            writer.writerow(
                {
                    "index": index,
                    "x_m": f"{x + args.offset_x:.6f}",
                    "y_m": f"{y + args.offset_y:.6f}",
                    "heading_rad": f"{heading:.9f}",
                    "lane_width_m": f"{width:.6f}",
                }
            )
    print(f"Wrote {len(centers)} centerline samples to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
