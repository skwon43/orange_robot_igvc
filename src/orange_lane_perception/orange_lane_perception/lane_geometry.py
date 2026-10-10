"""Small dependency-free geometry helpers shared by lane ROS nodes."""

from __future__ import annotations

from dataclasses import dataclass
import math
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

    left = list(left_points)
    right = list(right_points)
    left_fit = fit_line_y_from_x(left, min_points_per_side)
    right_fit = fit_line_y_from_x(right, min_points_per_side)
    if left_fit is None or right_fit is None:
        return None

    left_y = left_fit[0] * lookahead_m + left_fit[1]
    right_y = right_fit[0] * lookahead_m + right_fit[1]
    lane_width = left_y - right_y
    if not min_lane_width_m <= lane_width <= max_lane_width_m:
        return None

    return LaneEstimate(
        x_m=lookahead_m,
        y_m=(left_y + right_y) * 0.5,
        heading_rad=math.atan((left_fit[0] + right_fit[0]) * 0.5),
        lane_width_m=lane_width,
        left_points=len(left),
        right_points=len(right),
    )
