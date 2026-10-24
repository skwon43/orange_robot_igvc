"""Small dependency-free geometry helpers shared by lane ROS nodes."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class LaneEstimate:
    """Centerline pose in the local ground frame at a fixed lookahead."""

    x_m: float
    y_m: float
    heading_rad: float
    lane_width_m: float
    left_points: int
    right_points: int
    boundary_mode: str = "both"


def rotate_vector(
    vector: tuple[float, float, float], quaternion: tuple[float, float, float, float]
) -> tuple[float, float, float]:
    """Rotate a vector by an x/y/z/w quaternion without ROS dependencies."""

    x, y, z, w = quaternion
    vx, vy, vz = vector
    dot_uv = x * vx + y * vy + z * vz
    dot_uu = x * x + y * y + z * z
    cross_x = y * vz - z * vy
    cross_y = z * vx - x * vz
    cross_z = x * vy - y * vx
    return (
        2.0 * dot_uv * x + (w * w - dot_uu) * vx + 2.0 * w * cross_x,
        2.0 * dot_uv * y + (w * w - dot_uu) * vy + 2.0 * w * cross_y,
        2.0 * dot_uv * z + (w * w - dot_uu) * vz + 2.0 * w * cross_z,
    )


def multiply_quaternions(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Return the rotation formed by applying second and then first."""

    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def intersect_ground_plane(
    origin: tuple[float, float, float], direction: tuple[float, float, float]
) -> tuple[float, float] | None:
    """Intersect a ray with z=0 in its destination frame."""

    if abs(direction[2]) < 1e-9:
        return None
    scale = -origin[2] / direction[2]
    if scale <= 0.0:
        return None
    return (origin[0] + scale * direction[0], origin[1] + scale * direction[1])


def _fit_line(points: list[tuple[float, float]]) -> tuple[float, float] | None:
    if len(points) < 2:
        return None
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    denominator = sum((point[0] - mean_x) ** 2 for point in points)
    if denominator < 1e-9:
        return None
    slope = sum(
        (point[0] - mean_x) * (point[1] - mean_y) for point in points
    ) / denominator
    return slope, mean_y - slope * mean_x


def fit_line_y_from_x(
    points: Iterable[tuple[float, float]], min_points: int
) -> tuple[float, float] | None:
    """Fit y=ax+b and reject a single residual-based outlier pass."""

    candidates = list(points)
    if len(candidates) < min_points:
        return None
    initial = _fit_line(candidates)
    if initial is None:
        return None
    slope, intercept = initial
    residuals = [abs(y - (slope * x + intercept)) for x, y in candidates]
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    if rms <= 1e-9:
        return initial
    inlier_limit = max(0.05, 2.5 * rms)
    inliers = [
        point for point, residual in zip(candidates, residuals) if residual <= inlier_limit
    ]
    if len(inliers) < min_points:
        return None
    return _fit_line(inliers)


def estimate_lane_center(
    left_points: Iterable[tuple[float, float]],
    right_points: Iterable[tuple[float, float]],
    lookahead_m: float,
    min_lane_width_m: float,
    max_lane_width_m: float,
    min_points_per_side: int,
) -> LaneEstimate | None:
    """Estimate a local centerline from the left and right lane boundaries."""

    return estimate_lane_center_from_boundaries(
        left_points,
        right_points,
        lookahead_m,
        min_lane_width_m,
        max_lane_width_m,
        min_points_per_side,
        nominal_lane_width_m=None,
        allow_single_boundary=False,
    )


def estimate_lane_center_from_boundaries(
    left_points: Iterable[tuple[float, float]],
    right_points: Iterable[tuple[float, float]],
    lookahead_m: float,
    min_lane_width_m: float,
    max_lane_width_m: float,
    min_points_per_side: int,
    nominal_lane_width_m: float | None,
    allow_single_boundary: bool,
) -> LaneEstimate | None:
    """Estimate the centerline, optionally inferring it from one boundary.

    One-boundary recovery is deliberately opt-in: it uses the configured nominal
    lane width and is less certain than a measurement from both boundaries.
    """

    left = list(left_points)
    right = list(right_points)
    left_fit = fit_line_y_from_x(left, min_points_per_side)
    right_fit = fit_line_y_from_x(right, min_points_per_side)
    if left_fit is None and right_fit is None:
        return None

    if left_fit is not None and right_fit is not None:
        left_y = left_fit[0] * lookahead_m + left_fit[1]
        right_y = right_fit[0] * lookahead_m + right_fit[1]
        lane_width = left_y - right_y
        if not min_lane_width_m <= lane_width <= max_lane_width_m:
            return None
        center_y = (left_y + right_y) * 0.5
        heading = math.atan((left_fit[0] + right_fit[0]) * 0.5)
        boundary_mode = "both"
    elif not allow_single_boundary or nominal_lane_width_m is None:
        return None
    elif left_fit is not None:
        left_y = left_fit[0] * lookahead_m + left_fit[1]
        lane_width = nominal_lane_width_m
        center_y = left_y - lane_width * 0.5
        heading = math.atan(left_fit[0])
        boundary_mode = "left_inferred"
    else:
        assert right_fit is not None
        right_y = right_fit[0] * lookahead_m + right_fit[1]
        lane_width = nominal_lane_width_m
        center_y = right_y + lane_width * 0.5
        heading = math.atan(right_fit[0])
        boundary_mode = "right_inferred"

    return LaneEstimate(
        x_m=lookahead_m,
        y_m=center_y,
        heading_rad=heading,
        lane_width_m=lane_width,
        left_points=len(left),
        right_points=len(right),
        boundary_mode=boundary_mode,
    )


def blend_lane_estimates(
    previous: LaneEstimate, current: LaneEstimate, current_weight: float
) -> LaneEstimate:
    """Blend centerline poses without wrapping errors around +/- pi."""

    weight = min(1.0, max(0.0, current_weight))
    previous_weight = 1.0 - weight
    heading = math.atan2(
        previous_weight * math.sin(previous.heading_rad)
        + weight * math.sin(current.heading_rad),
        previous_weight * math.cos(previous.heading_rad)
        + weight * math.cos(current.heading_rad),
    )
    return LaneEstimate(
        x_m=previous_weight * previous.x_m + weight * current.x_m,
        y_m=previous_weight * previous.y_m + weight * current.y_m,
        heading_rad=heading,
        lane_width_m=(
            previous_weight * previous.lane_width_m + weight * current.lane_width_m
        ),
        left_points=current.left_points,
        right_points=current.right_points,
        boundary_mode=current.boundary_mode,
    )
