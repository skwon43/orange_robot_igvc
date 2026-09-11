#!/usr/bin/env python3
"""Generate a deterministic IGVC 2026-normalized Gazebo course.

The checked-in orange_igvc model remains the reference topology. This tool:
  * globally scales the course centerline footprint,
  * reconstructs paired inner/outer white boundaries at a legal lane width,
  * sets boundary tape width to the 2026 nominal 3 in,
  * preserves physical obstacle and ramp sizes,
  * remaps construction barrels relative to the local course centerline,
  * writes a separate orange_igvc_2026 model.

Random variation is intentionally deferred. The centerline / local-lane
parameterization in this generator is the foundation for future seeded
lane and obstacle variation.
"""

from __future__ import annotations

import argparse
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

GLOBAL_SCALE_DEFAULT = 0.86
LANE_WIDTH_DEFAULT_M = 4.0
LINE_WIDTH_DEFAULT_M = 0.0762
GROUND_SIZE_DEFAULT_M = (39.0, 33.0)
LEGAL_LANE_WIDTH_M = (3.048, 6.096)
BARREL_PREFIX = "construction_barrel"
LINE_RE = re.compile(r"^white_line_(inner|outer)_(\d+)$")


@dataclass
class Pose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    @classmethod
    def parse(cls, text: str) -> "Pose":
        values = [float(v) for v in text.split()]
        if len(values) != 6:
            raise ValueError(f"Expected 6-value SDF pose, got: {text!r}")
        return cls(*values)

    def format(self) -> str:
        return (
            f"{self.x:.9f} {self.y:.9f} {self.z:.9f} "
            f"{self.roll:.9f} {self.pitch:.9f} {self.yaw:.9f}"
        )


@dataclass
class BoundaryPair:
    index: int
    inner: ET.Element
    outer: ET.Element
    inner_pose: Pose
    outer_pose: Pose

    @property
    def center(self) -> tuple[float, float]:
        return (
            0.5 * (self.inner_pose.x + self.outer_pose.x),
            0.5 * (self.inner_pose.y + self.outer_pose.y),
        )

    @property
    def half_width(self) -> float:
        dx = self.outer_pose.x - self.inner_pose.x
        dy = self.outer_pose.y - self.inner_pose.y
        return 0.5 * math.hypot(dx, dy)

    @property
    def outward_unit(self) -> tuple[float, float]:
        dx = self.outer_pose.x - self.inner_pose.x
        dy = self.outer_pose.y - self.inner_pose.y
        norm = math.hypot(dx, dy)
        if norm <= 1e-9:
            raise ValueError(f"Boundary pair {self.index} has zero separation")
        return dx / norm, dy / norm


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    default_input = (
        repo_root
        / "src/orange_gazebo/models/orange_igvc/model.sdf"
    )
    default_output_dir = (
        repo_root
        / "src/orange_gazebo/models/orange_igvc_2026"
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=default_input)
    parser.add_argument("--output-dir", type=Path, default=default_output_dir)
    parser.add_argument("--global-scale", type=float, default=GLOBAL_SCALE_DEFAULT)
    parser.add_argument("--lane-width", type=float, default=LANE_WIDTH_DEFAULT_M)
    parser.add_argument("--line-width", type=float, default=LINE_WIDTH_DEFAULT_M)
    parser.add_argument(
        "--ground-size",
        nargs=2,
        type=float,
        metavar=("X_M", "Y_M"),
        default=GROUND_SIZE_DEFAULT_M,
    )
    return parser.parse_args()


def get_pose(link: ET.Element) -> Pose | None:
    pose_el = link.find("pose")
    if pose_el is None or not pose_el.text:
        return None
    return Pose.parse(pose_el.text)


def set_pose(link: ET.Element, pose: Pose) -> None:
    pose_el = link.find("pose")
    if pose_el is None:
        pose_el = ET.SubElement(link, "pose")
    pose_el.text = pose.format()


def iter_links(model: ET.Element) -> Iterable[ET.Element]:
    return model.findall("link")


def collect_boundary_pairs(model: ET.Element) -> list[BoundaryPair]:
    by_index: dict[int, dict[str, ET.Element]] = {}
    for link in iter_links(model):
        name = link.get("name", "")
        match = LINE_RE.match(name)
        if not match:
            continue
        side, index_str = match.groups()
        by_index.setdefault(int(index_str), {})[side] = link

    if not by_index:
        raise ValueError("No white_line_inner/outer boundary links were found")

    pairs: list[BoundaryPair] = []
    for index in sorted(by_index):
        record = by_index[index]
        if set(record) != {"inner", "outer"}:
            raise ValueError(
                f"Boundary index {index} is incomplete: {sorted(record)}"
            )
        inner_pose = get_pose(record["inner"])
        outer_pose = get_pose(record["outer"])
        if inner_pose is None or outer_pose is None:
            raise ValueError(f"Boundary index {index} is missing a pose")
        pairs.append(
            BoundaryPair(
                index=index,
                inner=record["inner"],
                outer=record["outer"],
                inner_pose=inner_pose,
                outer_pose=outer_pose,
            )
        )

    return pairs


def bbox_center(points: Iterable[tuple[float, float]]) -> tuple[float, float]:
    pts = list(points)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return 0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys))


def scale_point(
    p: tuple[float, float],
    origin: tuple[float, float],
    scale: float,
) -> tuple[float, float]:
    return (
        origin[0] + scale * (p[0] - origin[0]),
        origin[1] + scale * (p[1] - origin[1]),
    )


def set_line_box_dimensions(
    link: ET.Element,
    global_scale: float,
    line_width: float,
) -> None:
    for size_el in link.findall(".//box/size"):
        if not size_el.text:
            continue
        values = [float(v) for v in size_el.text.split()]
        if len(values) != 3:
            continue
        values[0] *= global_scale
        values[1] = line_width
        size_el.text = " ".join(f"{v:.9f}" for v in values)


def rebuild_boundaries(
    pairs: list[BoundaryPair],
    origin: tuple[float, float],
    global_scale: float,
    lane_width: float,
    line_width: float,
) -> list[tuple[float, float]]:
    half_target = 0.5 * lane_width
    new_centers: list[tuple[float, float]] = []

    for pair in pairs:
        center_new = scale_point(pair.center, origin, global_scale)
        nx, ny = pair.outward_unit

        inner_new = (
            center_new[0] - half_target * nx,
            center_new[1] - half_target * ny,
        )
        outer_new = (
            center_new[0] + half_target * nx,
            center_new[1] + half_target * ny,
        )

        ip = pair.inner_pose
        op = pair.outer_pose
        set_pose(
            pair.inner,
            Pose(inner_new[0], inner_new[1], ip.z, ip.roll, ip.pitch, ip.yaw),
        )
        set_pose(
            pair.outer,
            Pose(outer_new[0], outer_new[1], op.z, op.roll, op.pitch, op.yaw),
        )
        set_line_box_dimensions(pair.inner, global_scale, line_width)
        set_line_box_dimensions(pair.outer, global_scale, line_width)
        new_centers.append(center_new)

    return new_centers


def closest_projection_on_polyline(
    point: tuple[float, float],
    points: list[tuple[float, float]],
) -> tuple[int, float, tuple[float, float], tuple[float, float], float]:
    """Return segment index, t, projection, left normal, and distance."""
    if len(points) < 2:
        raise ValueError("Polyline requires at least two points")

    best = None
    px, py = point

    for i in range(len(points)):
        a = points[i]
        b = points[(i + 1) % len(points)]
        vx = b[0] - a[0]
        vy = b[1] - a[1]
        length2 = vx * vx + vy * vy
        if length2 <= 1e-12:
            continue

        t = ((px - a[0]) * vx + (py - a[1]) * vy) / length2
        t = max(0.0, min(1.0, t))
        qx = a[0] + t * vx
        qy = a[1] + t * vy
        dx = px - qx
        dy = py - qy
        distance = math.hypot(dx, dy)

        inv_len = 1.0 / math.sqrt(length2)
        normal = (-vy * inv_len, vx * inv_len)

        candidate = (distance, i, t, (qx, qy), normal)
        if best is None or candidate[0] < best[0]:
            best = candidate

    if best is None:
        raise ValueError("Polyline contains no non-degenerate segments")

    distance, i, t, projection, normal = best
    return i, t, projection, normal, distance


def remap_barrels(
    model: ET.Element,
    pairs: list[BoundaryPair],
    old_centers: list[tuple[float, float]],
    new_centers: list[tuple[float, float]],
    target_lane_width: float,
) -> int:
    old_half_widths = [pair.half_width for pair in pairs]
    new_half_width = 0.5 * target_lane_width
    count = 0

    for link in iter_links(model):
        if not link.get("name", "").startswith(BARREL_PREFIX):
            continue
        pose = get_pose(link)
        if pose is None:
            continue

        i, t, projection, normal, _ = closest_projection_on_polyline(
            (pose.x, pose.y), old_centers
        )
        next_i = (i + 1) % len(old_centers)

        old_half = (
            (1.0 - t) * old_half_widths[i] + t * old_half_widths[next_i]
        )
        if old_half <= 1e-9:
            raise ValueError(f"Invalid old lane width near barrel {link.get('name')}")

        lateral = (
            (pose.x - projection[0]) * normal[0]
            + (pose.y - projection[1]) * normal[1]
        )
        normalized_lateral = lateral / old_half

        qa = new_centers[i]
        qb = new_centers[next_i]
        q_new = (
            qa[0] + t * (qb[0] - qa[0]),
            qa[1] + t * (qb[1] - qa[1]),
        )

        new_lateral = normalized_lateral * new_half_width
        pose.x = q_new[0] + new_lateral * normal[0]
        pose.y = q_new[1] + new_lateral * normal[1]
        set_pose(link, pose)
        count += 1

    return count


def scale_other_link_positions(
    model: ET.Element,
    origin: tuple[float, float],
    scale: float,
) -> int:
    """Scale XY location of non-ground, non-boundary, non-barrel links."""
    count = 0
    for link in iter_links(model):
        name = link.get("name", "")
        if name == "ground" or LINE_RE.match(name) or name.startswith(BARREL_PREFIX):
            continue
        pose = get_pose(link)
        if pose is None:
            continue
        pose.x, pose.y = scale_point((pose.x, pose.y), origin, scale)
        set_pose(link, pose)
        count += 1
    return count


def set_ground_size(model: ET.Element, x_m: float, y_m: float) -> None:
    ground = model.find("./link[@name='ground']")
    if ground is None:
        return
    for size_el in ground.findall(".//plane/size"):
        size_el.text = f"{x_m:.6f} {y_m:.6f}"


def footprint_of_boundaries(
    pairs: list[BoundaryPair],
) -> tuple[float, float, float, float]:
    points: list[tuple[float, float]] = []
    for pair in pairs:
        for link in (pair.inner, pair.outer):
            pose = get_pose(link)
            if pose is not None:
                points.append((pose.x, pose.y))
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), max(xs), min(ys), max(ys)


def write_model_config(output_dir: Path) -> None:
    config = """<?xml version="1.0"?>
<model>
  <name>orange_igvc_2026</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Orange Robot IGVC</name>
  </author>
  <description>Deterministic 2026 IGVC-normalized AutoNav course.</description>
</model>
"""
    (output_dir / "model.config").write_text(config, encoding="utf-8")


def main() -> None:
    args = parse_args()

    if args.global_scale <= 0:
        raise SystemExit("--global-scale must be positive")
    legal_min, legal_max = LEGAL_LANE_WIDTH_M
    if not (legal_min <= args.lane_width <= legal_max):
        raise SystemExit(
            f"--lane-width must be within the 2026 legal range "
            f"[{legal_min:.3f}, {legal_max:.3f}] m"
        )
    if args.line_width <= 0:
        raise SystemExit("--line-width must be positive")

    tree = ET.parse(args.input)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise SystemExit("Input SDF does not contain a top-level <model>")

    pairs = collect_boundary_pairs(model)
    old_centers = [pair.center for pair in pairs]
    origin = bbox_center(old_centers)

    new_centers = rebuild_boundaries(
        pairs=pairs,
        origin=origin,
        global_scale=args.global_scale,
        lane_width=args.lane_width,
        line_width=args.line_width,
    )

    barrel_count = remap_barrels(
        model=model,
        pairs=pairs,
        old_centers=old_centers,
        new_centers=new_centers,
        target_lane_width=args.lane_width,
    )
    other_count = scale_other_link_positions(model, origin, args.global_scale)
    set_ground_size(model, args.ground_size[0], args.ground_size[1])
    model.set("name", "orange_igvc_2026")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_sdf = args.output_dir / "model.sdf"
    ET.indent(tree, space="  ")
    tree.write(output_sdf, encoding="utf-8", xml_declaration=True)
    write_model_config(args.output_dir)

    xmin, xmax, ymin, ymax = footprint_of_boundaries(pairs)
    widths = []
    for pair in pairs:
        ip = get_pose(pair.inner)
        op = get_pose(pair.outer)
        assert ip is not None and op is not None
        widths.append(math.hypot(op.x - ip.x, op.y - ip.y))

    print("Generated deterministic IGVC 2026 course")
    print(f"  input:             {args.input}")
    print(f"  output:            {output_sdf}")
    print(f"  boundary pairs:    {len(pairs)}")
    print(f"  remapped barrels:  {barrel_count}")
    print(f"  scaled other links:{other_count}")
    print(f"  global scale:      {args.global_scale:.3f}")
    print(f"  lane width:        {min(widths):.3f} .. {max(widths):.3f} m")
    print(f"  tape width:        {args.line_width:.4f} m")
    print(
        f"  boundary footprint:{xmax - xmin:.3f} x {ymax - ymin:.3f} m "
        f"(x=[{xmin:.3f}, {xmax:.3f}], y=[{ymin:.3f}, {ymax:.3f}])"
    )
    print(
        "  note: random lane / obstacle variation is intentionally disabled "
        "for this baseline"
    )


if __name__ == "__main__":
    main()
