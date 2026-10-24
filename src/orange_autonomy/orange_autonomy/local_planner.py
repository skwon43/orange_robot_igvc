"""Dependency-free local planning and mission helpers for the ROS controller."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class ScanPoint:
    angle_rad: float
    distance_m: float


@dataclass(frozen=True)
class PlannerConfig:
    max_heading_rad: float = 0.85
    candidate_count: int = 19
    obstacle_lookahead_m: float = 4.0
    corridor_half_width_m: float = 0.55
    nominal_lane_width_m: float = 4.0
    lane_constraint_horizon_m: float = 1.5
    stop_distance_m: float = 0.75
    slowdown_distance_m: float = 2.50
    lane_weight: float = 4.0
    clearance_weight: float = 5.0
    turn_weight: float = 0.35
    switching_weight: float = 0.45
    lateral_gain: float = 1.0
    heading_gain: float = 0.8
    max_linear_speed_mps: float = 0.85
    min_linear_speed_mps: float = 0.12
    max_angular_speed_rps: float = 0.90
    angular_gain: float = 1.5
    blocked_turn_speed_rps: float = 0.35

    def validate(self) -> None:
        if self.max_heading_rad <= 0.0:
            raise ValueError("max_heading_rad must be positive")
        if self.candidate_count < 3 or self.candidate_count % 2 == 0:
            raise ValueError("candidate_count must be an odd integer >= 3")
        if not 0.0 < self.stop_distance_m < self.slowdown_distance_m:
            raise ValueError(
                "stop_distance_m must be below slowdown_distance_m")
        if self.obstacle_lookahead_m <= self.slowdown_distance_m:
            raise ValueError(
                "obstacle_lookahead_m must exceed slowdown_distance_m")
        if self.corridor_half_width_m <= 0.0:
            raise ValueError("corridor_half_width_m must be positive")
        if self.nominal_lane_width_m <= 2.0 * self.corridor_half_width_m:
            raise ValueError(
                "nominal_lane_width_m is too narrow for the corridor")
        if self.lane_constraint_horizon_m <= 0.0:
            raise ValueError("lane_constraint_horizon_m must be positive")
        if self.max_linear_speed_mps <= 0.0 or self.max_angular_speed_rps <= 0.0:
            raise ValueError("maximum speeds must be positive")


@dataclass(frozen=True)
class PlanDecision:
    desired_heading_rad: float
    selected_heading_rad: float
    selected_clearance_m: float
    forward_clearance_m: float
    score: float
    blocked: bool
    mode: str


@dataclass(frozen=True)
class MotionCommand:
    linear_mps: float
    angular_rps: float


@dataclass(frozen=True)
class MissionProgress:
    distance_travelled_m: float
    distance_from_start_m: float
    departed_start: bool
    finished: bool


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def scan_points_from_ranges(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> list[ScanPoint]:
    """Convert finite, in-range LaserScan samples into polar points."""

    points = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance):
            continue
        if distance < range_min or distance > range_max:
            continue
        points.append(ScanPoint(angle_min + index * angle_increment, distance))
    return points


def lane_target_heading(
    lane_x_m: float,
    lane_y_m: float,
    lane_heading_rad: float,
    config: PlannerConfig,
) -> float:
    """Combine pure-pursuit lateral correction with the measured lane tangent."""

    lookahead = max(0.10, lane_x_m)
    lateral_heading = math.atan2(lane_y_m, lookahead)
    target = (
        config.lateral_gain * lateral_heading
        + config.heading_gain * lane_heading_rad
    )
    return clamp(target, -config.max_heading_rad, config.max_heading_rad)


def corridor_clearance(
    points: Iterable[ScanPoint],
    heading_rad: float,
    config: PlannerConfig,
) -> float:
    """Return free distance inside a robot-width straight candidate corridor."""

    clearance = config.obstacle_lookahead_m
    for point in points:
        relative_angle = normalize_angle(point.angle_rad - heading_rad)
        along_path = point.distance_m * math.cos(relative_angle)
        if along_path <= 0.0 or along_path > config.obstacle_lookahead_m:
            continue
        across_path = abs(point.distance_m * math.sin(relative_angle))
        if across_path <= config.corridor_half_width_m:
            clearance = min(clearance, along_path)
    return clearance


def choose_local_heading(
    points: Iterable[ScanPoint],
    desired_heading_rad: float,
    previous_heading_rad: float,
    config: PlannerConfig,
    lane_center_y_at_horizon_m: float = 0.0,
) -> PlanDecision:
    """Score collision-free local headings and choose the lowest-cost route."""

    config.validate()
    scan_points = list(points)
    maximum = config.max_heading_rad
    step = 2.0 * maximum / float(config.candidate_count - 1)
    candidates = [-maximum + index *
                  step for index in range(config.candidate_count)]
    candidates.append(clamp(desired_heading_rad, -maximum, maximum))
    candidates = sorted(set(round(value, 9) for value in candidates))

    usable_lane_half_width = (
        0.5 * config.nominal_lane_width_m - config.corridor_half_width_m
    )
    evaluations: list[tuple[float, float, float, bool]] = []
    for candidate in candidates:
        clearance = corridor_clearance(scan_points, candidate, config)
        clearance_ratio = clamp(
            (clearance - config.stop_distance_m)
            / (config.obstacle_lookahead_m - config.stop_distance_m),
            0.0,
            1.0,
        )
        lane_error = normalize_angle(candidate - desired_heading_rad) / maximum
        switching_error = normalize_angle(
            candidate - previous_heading_rad) / maximum
        cost = (
            config.lane_weight * lane_error * lane_error
            + config.clearance_weight * (1.0 - clearance_ratio) ** 2
            + config.turn_weight * (candidate / maximum) ** 2
            + config.switching_weight * switching_error * switching_error
        )
        candidate_y = config.lane_constraint_horizon_m * math.sin(candidate)
        stays_in_lane = (
            abs(candidate_y - lane_center_y_at_horizon_m) <= usable_lane_half_width
        )
        evaluations.append((cost, candidate, clearance, stays_in_lane))

    feasible = [
        evaluation
        for evaluation in evaluations
        if evaluation[2] >= config.stop_distance_m and evaluation[3]
    ]
    blocked = not feasible
    if feasible:
        cost, selected, clearance, _ = min(feasible, key=lambda item: item[0])
    else:
        in_lane = [evaluation for evaluation in evaluations if evaluation[3]]
        cost, selected, clearance, _ = min(
            in_lane or evaluations,
            key=lambda item: (-item[2], item[0]),
        )

    forward_clearance = corridor_clearance(scan_points, 0.0, config)
    avoiding = (
        abs(normalize_angle(selected - desired_heading_rad)) > 0.75 * step
        or forward_clearance < config.slowdown_distance_m
    )
    mode = "blocked" if blocked else ("avoiding" if avoiding else "tracking")
    return PlanDecision(
        desired_heading_rad=desired_heading_rad,
        selected_heading_rad=selected,
        selected_clearance_m=clearance,
        forward_clearance_m=forward_clearance,
        score=cost,
        blocked=blocked,
        mode=mode,
    )


def command_from_decision(
    decision: PlanDecision,
    confidence: float,
    config: PlannerConfig,
) -> MotionCommand:
    """Convert a selected local route to bounded differential-drive velocity."""

    if decision.blocked or decision.forward_clearance_m < config.stop_distance_m:
        turn_sign = -1.0 if decision.selected_heading_rad < 0.0 else 1.0
        turn_speed = max(
            config.blocked_turn_speed_rps,
            abs(config.angular_gain * decision.selected_heading_rad),
        )
        return MotionCommand(
            0.0,
            turn_sign * min(config.max_angular_speed_rps, turn_speed),
        )

    clearance_scale = clamp(
        (decision.selected_clearance_m - config.stop_distance_m)
        / (config.slowdown_distance_m - config.stop_distance_m),
        0.0,
        1.0,
    )
    heading_scale = max(
        0.25,
        1.0 - abs(decision.selected_heading_rad) / config.max_heading_rad,
    )
    confidence_scale = 0.35 + 0.65 * clamp(confidence, 0.0, 1.0)
    linear = (
        config.max_linear_speed_mps
        * clearance_scale
        * heading_scale
        * confidence_scale
    )
    if linear > 0.0:
        linear = max(config.min_linear_speed_mps, linear)
    angular = clamp(
        config.angular_gain * decision.selected_heading_rad,
        -config.max_angular_speed_rps,
        config.max_angular_speed_rps,
    )
    return MotionCommand(linear, angular)


class LoopMissionTracker:
    """Detect completion of a closed course using odometry only."""

    def __init__(
        self,
        minimum_completion_distance_m: float,
        departure_radius_m: float,
        finish_radius_m: float,
        maximum_odom_step_m: float,
    ) -> None:
        if minimum_completion_distance_m <= 0.0:
            raise ValueError("minimum_completion_distance_m must be positive")
        if not 0.0 < finish_radius_m < departure_radius_m:
            raise ValueError(
                "finish_radius_m must be below departure_radius_m")
        self.minimum_completion_distance_m = minimum_completion_distance_m
        self.departure_radius_m = departure_radius_m
        self.finish_radius_m = finish_radius_m
        self.maximum_odom_step_m = maximum_odom_step_m
        self.reset()

    def reset(self) -> None:
        self.start: tuple[float, float] | None = None
        self.previous: tuple[float, float] | None = None
        self.distance_travelled_m = 0.0
        self.departed_start = False
        self.finished = False

    def update(self, x_m: float, y_m: float) -> MissionProgress:
        position = (x_m, y_m)
        if self.start is None:
            self.start = position
            self.previous = position

        assert self.start is not None
        if self.previous is not None:
            step = math.hypot(x_m - self.previous[0], y_m - self.previous[1])
            if step <= self.maximum_odom_step_m:
                self.distance_travelled_m += step
        self.previous = position

        distance_from_start = math.hypot(
            x_m - self.start[0], y_m - self.start[1]
        )
        self.departed_start = (
            self.departed_start or distance_from_start >= self.departure_radius_m
        )
        self.finished = self.finished or (
            self.departed_start
            and self.distance_travelled_m >= self.minimum_completion_distance_m
            and distance_from_start <= self.finish_radius_m
        )
        return MissionProgress(
            self.distance_travelled_m,
            distance_from_start,
            self.departed_start,
            self.finished,
        )
