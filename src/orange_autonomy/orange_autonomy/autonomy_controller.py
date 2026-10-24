#!/usr/bin/env python3
"""ROS 2 closed-loop lane following, obstacle avoidance, and mission control."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from .local_planner import (LoopMissionTracker, MissionProgress, MotionCommand,
                            PlannerConfig, ScanPoint, choose_local_heading,
                            clamp, command_from_decision, lane_target_heading,
                            scan_points_from_ranges)


class AutonomyController(Node):
    """Select a safe lane-guided local route and publish differential drive commands."""

    def __init__(self) -> None:
        super().__init__("autonomy_controller")
        self._declare_parameters()

        self.enabled = bool(self.get_parameter("enabled").value)
        self.lane_frame = str(self.get_parameter("lane_frame").value)
        self.lane_timeout_sec = float(
            self.get_parameter("lane_timeout_sec").value)
        self.scan_timeout_sec = float(
            self.get_parameter("scan_timeout_sec").value)
        self.odom_timeout_sec = float(
            self.get_parameter("odom_timeout_sec").value)
        self.minimum_scan_valid_fraction = float(
            self.get_parameter("minimum_scan_valid_fraction").value
        )
        self.scan_angle_offset_rad = float(
            self.get_parameter("scan_angle_offset_rad").value
        )
        self.control_frequency_hz = float(
            self.get_parameter("control_frequency_hz").value
        )
        self.linear_acceleration_mps2 = float(
            self.get_parameter("linear_acceleration_mps2").value
        )
        self.linear_deceleration_mps2 = float(
            self.get_parameter("linear_deceleration_mps2").value
        )
        self.angular_acceleration_rps2 = float(
            self.get_parameter("angular_acceleration_rps2").value
        )
        self.status_publish_period_sec = float(
            self.get_parameter("status_publish_period_sec").value
        )
        self.summary_path = Path(str(self.get_parameter("summary_path").value))
        self.reference_course_length_m = float(
            self.get_parameter("reference_course_length_m").value
        )
        self.planner_config = self._planner_config()
        self.planner_config.validate()
        self._validate_parameters()

        self.mission = LoopMissionTracker(
            float(self.get_parameter("minimum_completion_distance_m").value),
            float(self.get_parameter("departure_radius_m").value),
            float(self.get_parameter("finish_radius_m").value),
            float(self.get_parameter("maximum_odom_step_m").value),
        )
        self.progress = MissionProgress(0.0, 0.0, False, False)

        self.lane_pose: PoseStamped | None = None
        self.scan_points: list[ScanPoint] = []
        self.scan_valid = False
        self.lane_received_sec: float | None = None
        self.scan_received_sec: float | None = None
        self.odom_received_sec: float | None = None
        self.lane_confidence = 1.0
        self.lane_boundary_mode = "unknown"
        self.previous_selected_heading = 0.0
        self.last_command = MotionCommand(0.0, 0.0)
        self.last_control_sec = self._now_sec()
        self.last_status_sec = -math.inf
        self.finished_announced = False
        self.run_start_sec: float | None = None
        self.minimum_forward_clearance_m = math.inf
        self.near_stop_active = False
        self.near_stop_events = 0
        self.mode_cycles = {"tracking": 0, "avoiding": 0, "blocked": 0}

        lane_topic = str(self.get_parameter("lane_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        diagnostics_topic = str(self.get_parameter(
            "lane_diagnostics_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self.create_subscription(
            PoseStamped, lane_topic, self._lane_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            DiagnosticArray,
            diagnostics_topic,
            self._lane_diagnostics_callback,
            10,
        )
        self.command_publisher = self.create_publisher(
            Twist, cmd_vel_topic, 10)
        self.status_publisher = self.create_publisher(
            DiagnosticArray, "/autonomy/diagnostics", 10
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.finished_publisher = self.create_publisher(
            Bool, "/autonomy/finished", latched_qos
        )
        self.create_service(
            SetBool, "/autonomy/set_enabled", self._set_enabled)
        self.create_service(
            Trigger, "/autonomy/reset_mission", self._reset_mission)
        self.timer = self.create_timer(
            1.0 / self.control_frequency_hz, self._control_callback
        )
        self._publish_finished(False)
        self.get_logger().info(
            "Autonomy controller ready; motion starts after lane, scan, and odometry are valid"
        )

    def _declare_parameters(self) -> None:
        parameters: dict[str, Any] = {
            "enabled": True,
            "lane_topic": "/perception/lane_center",
            "lane_diagnostics_topic": "/perception/lane_diagnostics",
            "scan_topic": "/hokuyo_scan",
            "odom_topic": "/odom",
            "cmd_vel_topic": "/cmd_vel",
            "lane_frame": "base_footprint",
            "control_frequency_hz": 10.0,
            "lane_timeout_sec": 0.80,
            "scan_timeout_sec": 0.50,
            "odom_timeout_sec": 0.75,
            "minimum_scan_valid_fraction": 0.50,
            "scan_angle_offset_rad": 0.0,
            "linear_acceleration_mps2": 0.45,
            "linear_deceleration_mps2": 1.20,
            "angular_acceleration_rps2": 1.80,
            "status_publish_period_sec": 0.50,
            "summary_path": "/tmp/autonomy_run_summary.json",
            "reference_course_length_m": 92.167,
            "minimum_completion_distance_m": 70.0,
            "departure_radius_m": 5.0,
            "finish_radius_m": 2.5,
            "maximum_odom_step_m": 1.0,
            "max_heading_rad": 0.85,
            "candidate_count": 19,
            "obstacle_lookahead_m": 4.0,
            "corridor_half_width_m": 0.55,
            "nominal_lane_width_m": 4.0,
            "lane_constraint_horizon_m": 1.5,
            "stop_distance_m": 0.75,
            "slowdown_distance_m": 2.50,
            "lane_weight": 4.0,
            "clearance_weight": 5.0,
            "turn_weight": 0.35,
            "switching_weight": 0.45,
            "lateral_gain": 1.0,
            "heading_gain": 0.8,
            "max_linear_speed_mps": 0.85,
            "min_linear_speed_mps": 0.12,
            "max_angular_speed_rps": 0.90,
            "angular_gain": 1.5,
            "blocked_turn_speed_rps": 0.35,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _planner_config(self) -> PlannerConfig:
        return PlannerConfig(
            max_heading_rad=float(self.get_parameter("max_heading_rad").value),
            candidate_count=int(self.get_parameter("candidate_count").value),
            obstacle_lookahead_m=float(
                self.get_parameter("obstacle_lookahead_m").value
            ),
            corridor_half_width_m=float(
                self.get_parameter("corridor_half_width_m").value
            ),
            nominal_lane_width_m=float(
                self.get_parameter("nominal_lane_width_m").value
            ),
            lane_constraint_horizon_m=float(
                self.get_parameter("lane_constraint_horizon_m").value
            ),
            stop_distance_m=float(self.get_parameter("stop_distance_m").value),
            slowdown_distance_m=float(
                self.get_parameter("slowdown_distance_m").value
            ),
            lane_weight=float(self.get_parameter("lane_weight").value),
            clearance_weight=float(
                self.get_parameter("clearance_weight").value),
            turn_weight=float(self.get_parameter("turn_weight").value),
            switching_weight=float(
                self.get_parameter("switching_weight").value),
            lateral_gain=float(self.get_parameter("lateral_gain").value),
            heading_gain=float(self.get_parameter("heading_gain").value),
            max_linear_speed_mps=float(
                self.get_parameter("max_linear_speed_mps").value
            ),
            min_linear_speed_mps=float(
                self.get_parameter("min_linear_speed_mps").value
            ),
            max_angular_speed_rps=float(
                self.get_parameter("max_angular_speed_rps").value
            ),
            angular_gain=float(self.get_parameter("angular_gain").value),
            blocked_turn_speed_rps=float(
                self.get_parameter("blocked_turn_speed_rps").value
            ),
        )

    def _validate_parameters(self) -> None:
        if self.control_frequency_hz <= 0.0:
            raise ValueError("control_frequency_hz must be positive")
        if min(self.lane_timeout_sec, self.scan_timeout_sec, self.odom_timeout_sec) <= 0:
            raise ValueError("sensor timeouts must be positive")
        if not 0.0 <= self.minimum_scan_valid_fraction <= 1.0:
            raise ValueError("minimum_scan_valid_fraction must be in [0, 1]")
        if min(
            self.linear_acceleration_mps2,
            self.linear_deceleration_mps2,
            self.angular_acceleration_rps2,
        ) <= 0.0:
            raise ValueError("acceleration limits must be positive")
        if self.reference_course_length_m <= 0.0:
            raise ValueError("reference_course_length_m must be positive")

    def _lane_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id != self.lane_frame:
            self.get_logger().warning(
                f"Ignoring lane pose in {message.header.frame_id!r}; expected {self.lane_frame!r}"
            )
            return
        self.lane_pose = message
        self.lane_received_sec = self._now_sec()

    def _scan_callback(self, message: LaserScan) -> None:
        self.scan_points = scan_points_from_ranges(
            message.ranges,
            message.angle_min + self.scan_angle_offset_rad,
            message.angle_increment,
            message.range_min,
            message.range_max,
        )
        valid_samples = sum(
            1
            for distance in message.ranges
            if not math.isnan(distance) and distance >= message.range_min
        )
        valid_fraction = valid_samples / max(1, len(message.ranges))
        self.scan_valid = bool(message.ranges) and (
            valid_fraction >= self.minimum_scan_valid_fraction
        )
        self.scan_received_sec = self._now_sec()

    def _odom_callback(self, message: Odometry) -> None:
        self.odom_received_sec = self._now_sec()
        if not self.enabled or self.progress.finished:
            return
        position = message.pose.pose.position
        self.progress = self.mission.update(position.x, position.y)
        if self.progress.finished and not self.finished_announced:
            self.finished_announced = True
            self._publish_finished(True)
            self._write_run_summary()
            self.get_logger().info(
                f"Course complete after {self.progress.distance_travelled_m:.1f} m"
            )

    def _lane_diagnostics_callback(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "orange_lane_perception/lane_detector":
                continue
            values = {item.key: item.value for item in status.values}
            if values.get("detection_valid") != "true":
                continue
            try:
                self.lane_confidence = clamp(
                    float(values.get("detection_confidence", "1.0")), 0.0, 1.0
                )
            except ValueError:
                self.lane_confidence = 1.0
            self.lane_boundary_mode = values.get("boundary_mode", "unknown")

    def _control_callback(self) -> None:
        now_sec = self._now_sec()
        delta_sec = clamp(now_sec - self.last_control_sec, 0.001, 0.50)
        self.last_control_sec = now_sec

        stop_reason = self._stop_reason(now_sec)
        if stop_reason is not None:
            self._publish_immediate_stop()
            self._publish_status(now_sec, "stopped", stop_reason, {})
            return

        assert self.lane_pose is not None
        lane_position = self.lane_pose.pose.position
        lane_heading = self._yaw(self.lane_pose)
        desired_heading = lane_target_heading(
            lane_position.x,
            lane_position.y,
            lane_heading,
            self.planner_config,
        )
        decision = choose_local_heading(
            self.scan_points,
            desired_heading,
            self.previous_selected_heading,
            self.planner_config,
            self._lane_center_at_constraint_horizon(
                lane_position.x, lane_position.y, lane_heading
            ),
        )
        target = command_from_decision(
            decision, self.lane_confidence, self.planner_config
        )
        self.minimum_forward_clearance_m = min(
            self.minimum_forward_clearance_m, decision.forward_clearance_m
        )
        near_stop = decision.forward_clearance_m < self.planner_config.stop_distance_m
        if near_stop and not self.near_stop_active:
            self.near_stop_events += 1
        self.near_stop_active = near_stop
        self.mode_cycles[decision.mode] += 1
        command = self._slew_command(target, delta_sec)
        if decision.blocked or (
            decision.forward_clearance_m < self.planner_config.stop_distance_m
        ):
            command = MotionCommand(0.0, command.angular_rps)
        nearest_scan_point = min(
            self.scan_points, key=lambda point: point.distance_m, default=None
        )
        if self.run_start_sec is None and (
            abs(command.linear_mps) > 1e-6 or abs(command.angular_rps) > 1e-6
        ):
            self.run_start_sec = now_sec
        self._publish_command(command)
        self.previous_selected_heading = decision.selected_heading_rad
        self._publish_status(
            now_sec,
            decision.mode,
            "local route selected",
            {
                "desired_heading_rad": f"{decision.desired_heading_rad:.3f}",
                "selected_heading_rad": f"{decision.selected_heading_rad:.3f}",
                "selected_clearance_m": f"{decision.selected_clearance_m:.3f}",
                "forward_clearance_m": f"{decision.forward_clearance_m:.3f}",
                "route_score": f"{decision.score:.3f}",
                "lane_confidence": f"{self.lane_confidence:.3f}",
                "lane_boundary_mode": self.lane_boundary_mode,
                "scan_point_count": str(len(self.scan_points)),
                "nearest_scan_distance_m": (
                    f"{nearest_scan_point.distance_m:.3f}"
                    if nearest_scan_point is not None
                    else "none"
                ),
                "nearest_scan_angle_rad": (
                    f"{nearest_scan_point.angle_rad:.3f}"
                    if nearest_scan_point is not None
                    else "none"
                ),
            },
        )

    def _stop_reason(self, now_sec: float) -> str | None:
        if not self.enabled:
            return "autonomy disabled"
        if self.progress.finished:
            return "course complete"
        required = (
            ("lane", self.lane_pose, self.lane_received_sec, self.lane_timeout_sec),
            ("scan", self.scan_valid, self.scan_received_sec, self.scan_timeout_sec),
            ("odometry", self.odom_received_sec,
             self.odom_received_sec, self.odom_timeout_sec),
        )
        for name, value, received_sec, timeout_sec in required:
            if value is None or value is False or received_sec is None:
                return f"waiting for valid {name}"
            if now_sec - received_sec > timeout_sec:
                return f"stale {name} input"
        return None

    def _lane_center_at_constraint_horizon(
        self, lane_x_m: float, lane_y_m: float, lane_heading_rad: float
    ) -> float:
        remaining_x = self.planner_config.lane_constraint_horizon_m - lane_x_m
        return lane_y_m + remaining_x * math.tan(
            clamp(lane_heading_rad, -0.75, 0.75)
        )

    def _slew_command(self, target: MotionCommand, delta_sec: float) -> MotionCommand:
        linear_limit = (
            self.linear_acceleration_mps2
            if target.linear_mps >= self.last_command.linear_mps
            else self.linear_deceleration_mps2
        ) * delta_sec
        angular_limit = self.angular_acceleration_rps2 * delta_sec
        return MotionCommand(
            self.last_command.linear_mps
            + clamp(
                target.linear_mps - self.last_command.linear_mps,
                -linear_limit,
                linear_limit,
            ),
            self.last_command.angular_rps
            + clamp(
                target.angular_rps - self.last_command.angular_rps,
                -angular_limit,
                angular_limit,
            ),
        )

    def _publish_command(self, command: MotionCommand) -> None:
        message = Twist()
        message.linear.x = command.linear_mps
        message.angular.z = command.angular_rps
        self.command_publisher.publish(message)
        self.last_command = command

    def _publish_immediate_stop(self) -> None:
        self._publish_command(MotionCommand(0.0, 0.0))

    def _publish_status(
        self,
        now_sec: float,
        state: str,
        text: str,
        details: dict[str, Any],
    ) -> None:
        if now_sec - self.last_status_sec < self.status_publish_period_sec:
            return
        self.last_status_sec = now_sec
        status = DiagnosticStatus()
        status.name = "orange_autonomy/controller"
        status.hardware_id = "orange_robot"
        status.level = (
            DiagnosticStatus.OK
            if state in {"tracking", "avoiding", "finished"}
            else DiagnosticStatus.WARN
        )
        status.message = text
        values: dict[str, Any] = {
            "state": state,
            "enabled": str(self.enabled).lower(),
            "linear_command_mps": f"{self.last_command.linear_mps:.3f}",
            "angular_command_rps": f"{self.last_command.angular_rps:.3f}",
            "distance_travelled_m": f"{self.progress.distance_travelled_m:.3f}",
            "distance_from_start_m": f"{self.progress.distance_from_start_m:.3f}",
            "departed_start": str(self.progress.departed_start).lower(),
            "finished": str(self.progress.finished).lower(),
            **details,
        }
        status.values = [
            KeyValue(key=key, value=str(value)) for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_publisher.publish(array)

    def _set_enabled(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if request.data and self.progress.finished:
            response.success = False
            response.message = "reset the mission before re-enabling autonomy"
            return response
        self.enabled = request.data
        if not self.enabled:
            self._publish_immediate_stop()
        response.success = True
        response.message = "autonomy enabled" if self.enabled else "autonomy disabled"
        return response

    def _reset_mission(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        self._publish_immediate_stop()
        self.mission.reset()
        self.progress = MissionProgress(0.0, 0.0, False, False)
        self.finished_announced = False
        self.previous_selected_heading = 0.0
        self.run_start_sec = None
        self.minimum_forward_clearance_m = math.inf
        self.near_stop_active = False
        self.near_stop_events = 0
        self.mode_cycles = {"tracking": 0, "avoiding": 0, "blocked": 0}
        self._publish_finished(False)
        response.success = True
        response.message = "mission reset; start pose will be captured from the next odometry"
        return response

    def _publish_finished(self, finished: bool) -> None:
        message = Bool()
        message.data = finished
        self.finished_publisher.publish(message)

    def _write_run_summary(self) -> None:
        now_sec = self._now_sec()
        elapsed_sec = max(0.0, now_sec - (self.run_start_sec or now_sec))
        distance_m = self.progress.distance_travelled_m
        summary = {
            "completed": self.progress.finished,
            "elapsed_sec": round(elapsed_sec, 3),
            "distance_travelled_m": round(distance_m, 3),
            "average_speed_mps": round(distance_m / elapsed_sec, 3)
            if elapsed_sec > 0.0
            else 0.0,
            "reference_course_length_m": self.reference_course_length_m,
            "path_efficiency_ratio": round(
                min(1.0, self.reference_course_length_m / distance_m), 4
            )
            if distance_m > 0.0
            else 0.0,
            "minimum_forward_clearance_m": round(
                self.minimum_forward_clearance_m, 3
            )
            if math.isfinite(self.minimum_forward_clearance_m)
            else None,
            "near_stop_events": self.near_stop_events,
            "control_cycles": self.mode_cycles,
        }
        try:
            self.summary_path.parent.mkdir(parents=True, exist_ok=True)
            self.summary_path.write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            self.get_logger().info(f"Wrote run summary to {self.summary_path}")
        except OSError as error:
            self.get_logger().error(f"Could not write run summary: {error}")

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _yaw(message: PoseStamped) -> float:
        orientation = message.pose.orientation
        return math.atan2(
            2.0 * (orientation.w * orientation.z +
                   orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y +
                         orientation.z * orientation.z),
        )

    def destroy_node(self) -> bool:
        self._publish_immediate_stop()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: AutonomyController | None = None
    try:
        node = AutonomyController()
        rclpy.spin(node)
    except (ValueError, RuntimeError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"autonomy_controller: {error}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
