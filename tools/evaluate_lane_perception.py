#!/usr/bin/env python3
"""Measure metric lane-center and heading error against a ground-truth CSV."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--predictions", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--label", default="unnamed-run")
    return parser.parse_args()


def load_ground_truth(path: Path) -> list[tuple[float, float, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    required = {"x_m", "y_m", "heading_rad"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError("Ground truth requires x_m, y_m, and heading_rad columns")
    points = [
        (float(row["x_m"]), float(row["y_m"]), float(row["heading_rad"]))
        for row in rows
    ]
    if len(points) < 3:
        raise ValueError("Ground truth needs at least three centerline points")
    return points


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def closest_segment(
    point: tuple[float, float], points: list[tuple[float, float, float]]
) -> tuple[float, float]:
    best: tuple[float, float] | None = None
    px, py = point
    for index, start in enumerate(points):
        end = points[(index + 1) % len(points)]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            continue
        t = max(0.0, min(1.0, ((px - start[0]) * dx + (py - start[1]) * dy) / length_sq))
        projected_x = start[0] + t * dx
        projected_y = start[1] + t * dy
        length = math.sqrt(length_sq)
        lateral_error = ((px - projected_x) * -dy + (py - projected_y) * dx) / length
        heading_delta = normalize_angle(end[2] - start[2])
        heading = normalize_angle(start[2] + t * heading_delta)
        candidate = (lateral_error, heading)
        if best is None or abs(lateral_error) < abs(best[0]):
            best = candidate
    if best is None:
        raise ValueError("Ground-truth centerline has no valid segments")
    return best


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def main() -> int:
    args = parse_args()
    centerline = load_ground_truth(args.ground_truth)
    lateral_errors: list[float] = []
    heading_errors: list[float] = []

    with args.predictions.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    required = {"timestamp_ns", "predicted_center_x_m", "predicted_center_y_m"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(
            "Predictions CSV requires timestamp_ns, predicted_center_x_m, and "
            "predicted_center_y_m columns"
        )

    for row in rows:
        lateral_error, reference_heading = closest_segment(
            (float(row["predicted_center_x_m"]), float(row["predicted_center_y_m"])),
            centerline,
        )
        lateral_errors.append(lateral_error)
        prediction_heading = row.get("predicted_heading_rad", "").strip()
        if prediction_heading:
            heading_errors.append(
                normalize_angle(float(prediction_heading) - reference_heading)
            )

    absolute_lateral = [abs(value) for value in lateral_errors]
    absolute_heading = [abs(value) for value in heading_errors]
    report = {
        "label": args.label,
        "samples": len(lateral_errors),
        "lateral_error_m": {
            "mean_abs": sum(absolute_lateral) / len(absolute_lateral),
            "rms": math.sqrt(sum(value * value for value in lateral_errors) / len(lateral_errors)),
            "p95_abs": percentile(absolute_lateral, 0.95),
            "max_abs": max(absolute_lateral),
        },
        "heading_error_rad": None,
    }
    if absolute_heading:
        report["heading_error_rad"] = {
            "mean_abs": sum(absolute_heading) / len(absolute_heading),
            "p95_abs": percentile(absolute_heading, 0.95),
            "max_abs": max(absolute_heading),
        }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
