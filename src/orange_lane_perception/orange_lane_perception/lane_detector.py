#!/usr/bin/env python3
"""Project white lane boundaries from camera pixels into a local ROS ground frame."""

from __future__ import annotations

import math
from typing import Any

import cv2
from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .lane_geometry import estimate_lane_center, intersect_ground_plane, rotate_vector


class LaneDetector(Node):
    """A transparent baseline detector for the fixed IGVC white tape course."""

    def __init__(self) -> None:
        super().__init__("lane_detector")
        self.declare_parameter("image_topic", "/camera_front/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_front/color/camera_info")
        self.declare_parameter("output_topic", "/perception/lane_center")
        self.declare_parameter("diagnostics_topic", "/perception/lane_diagnostics")
        self.declare_parameter("debug_mask_topic", "/perception/lane_mask")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("roi_top_fraction", 0.35)
        self.declare_parameter("projection_stride_px", 8)
        self.declare_parameter("min_lookahead_m", 0.40)
        self.declare_parameter("max_lookahead_m", 3.00)
        self.declare_parameter("center_lookahead_m", 1.50)
        self.declare_parameter("min_lane_width_m", 2.50)
        self.declare_parameter("max_lane_width_m", 6.50)
        self.declare_parameter("min_points_per_side", 12)
        self.declare_parameter("white_threshold", 180)

        self.base_frame = self._string_parameter("base_frame")
        self.roi_top_fraction = self._float_parameter("roi_top_fraction")
        self.projection_stride_px = self._int_parameter("projection_stride_px")
        self.min_lookahead_m = self._float_parameter("min_lookahead_m")
        self.max_lookahead_m = self._float_parameter("max_lookahead_m")
        self.center_lookahead_m = self._float_parameter("center_lookahead_m")
        self.min_lane_width_m = self._float_parameter("min_lane_width_m")
        self.max_lane_width_m = self._float_parameter("max_lane_width_m")
        self.min_points_per_side = self._int_parameter("min_points_per_side")
        self.white_threshold = self._int_parameter("white_threshold")
        self._validate_parameters()

        self.camera_model: tuple[float, float, float, float] | None = None
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.image_subscription = self.create_subscription(
            Image,
            self._string_parameter("image_topic"),
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self._string_parameter("camera_info_topic"),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.lane_publisher = self.create_publisher(
            PoseStamped, self._string_parameter("output_topic"), 10
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, self._string_parameter("diagnostics_topic"), 10
        )
        self.mask_publisher = self.create_publisher(
            Image, self._string_parameter("debug_mask_topic"), 10
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _float_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _int_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _validate_parameters(self) -> None:
        if not 0.0 <= self.roi_top_fraction < 1.0:
            raise ValueError("roi_top_fraction must be in [0, 1)")
        if self.projection_stride_px < 1 or self.min_points_per_side < 2:
            raise ValueError("projection_stride_px must be >= 1 and min_points_per_side >= 2")
        if not 0.0 < self.min_lookahead_m < self.max_lookahead_m:
            raise ValueError("lookahead range is invalid")
        if not self.min_lookahead_m <= self.center_lookahead_m <= self.max_lookahead_m:
            raise ValueError("center_lookahead_m must fall within the projection range")
        if not 0.0 < self.min_lane_width_m <= self.max_lane_width_m:
            raise ValueError("lane-width range is invalid")
        if not 0 <= self.white_threshold <= 255:
            raise ValueError("white_threshold must be between 0 and 255")

    def camera_info_callback(self, message: CameraInfo) -> None:
        fx, fy, cx, cy = message.k[0], message.k[4], message.k[2], message.k[5]
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning("Ignoring CameraInfo with a non-positive focal length")
            return
        self.camera_model = (fx, fy, cx, cy)

    def image_callback(self, message: Image) -> None:
        if self.camera_model is None:
            self._publish_status(message, False, "waiting for CameraInfo", {})
            return
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            self._publish_status(message, False, f"cv_bridge conversion failed: {error}", {})
            return

        mask = self._white_mask(bgr_image)
        self._publish_mask(message, mask)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.10),
            )
        except TransformException as error:
            self._publish_status(message, False, f"missing TF to {self.base_frame}: {error}", {})
            return

        left_points, right_points, projected_count = self._project_lane_pixels(mask, transform)
        estimate = estimate_lane_center(
            left_points,
            right_points,
            self.center_lookahead_m,
            self.min_lane_width_m,
            self.max_lane_width_m,
            self.min_points_per_side,
        )
        details: dict[str, Any] = {
            "mask_pixels": int(np.count_nonzero(mask)),
            "projected_points": projected_count,
            "left_points": len(left_points),
            "right_points": len(right_points),
        }
        if estimate is None:
            self._publish_status(message, False, "insufficient or implausible lane boundaries", details)
            return

        pose = PoseStamped()
        pose.header.stamp = message.header.stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = estimate.x_m
        pose.pose.position.y = estimate.y_m
        pose.pose.orientation.z = math.sin(estimate.heading_rad * 0.5)
        pose.pose.orientation.w = math.cos(estimate.heading_rad * 0.5)
        self.lane_publisher.publish(pose)
        details["lane_width_m"] = estimate.lane_width_m
        details["heading_rad"] = estimate.heading_rad
        self._publish_status(message, True, "lane detected", details)

    def _white_mask(self, bgr_image: np.ndarray) -> np.ndarray:
        mask = cv2.inRange(
            bgr_image,
            np.array([self.white_threshold] * 3, dtype=np.uint8),
            np.array([255, 255, 255], dtype=np.uint8),
        )
        roi_start = int(mask.shape[0] * self.roi_top_fraction)
        mask[:roi_start, :] = 0
        kernel = np.ones((3, 3), dtype=np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    def _project_lane_pixels(
        self, mask: np.ndarray, transform: Any
    ) -> tuple[list[tuple[float, float]], list[tuple[float, float]], int]:
        fx, fy, cx, cy = self.camera_model or (0.0, 0.0, 0.0, 0.0)
        rows, columns = np.nonzero(mask)
        translation = transform.transform.translation
        rotation_msg = transform.transform.rotation
        origin = (translation.x, translation.y, translation.z)
        rotation = (rotation_msg.x, rotation_msg.y, rotation_msg.z, rotation_msg.w)
        left_points: list[tuple[float, float]] = []
        right_points: list[tuple[float, float]] = []
        projected_count = 0

        for pixel_y, pixel_x in zip(
            rows[:: self.projection_stride_px], columns[:: self.projection_stride_px]
        ):
            optical_ray = (
                (float(pixel_x) - cx) / fx,
                (float(pixel_y) - cy) / fy,
                1.0,
            )
            ground_point = intersect_ground_plane(origin, rotate_vector(optical_ray, rotation))
            if ground_point is None:
                continue
            x_m, y_m = ground_point
            if not self.min_lookahead_m <= x_m <= self.max_lookahead_m:
                continue
            projected_count += 1
            if y_m >= 0.0:
                left_points.append(ground_point)
            else:
                right_points.append(ground_point)
        return left_points, right_points, projected_count

    def _publish_mask(self, image: Image, mask: np.ndarray) -> None:
        message = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        message.header = image.header
        self.mask_publisher.publish(message)

    def _publish_status(
        self, image: Image, valid: bool, text: str, details: dict[str, Any]
    ) -> None:
        status = DiagnosticStatus()
        status.name = "orange_lane_perception/lane_detector"
        status.hardware_id = "lane-camera"
        status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        status.message = text
        values = {"detection_valid": str(valid).lower(), **details}
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        array = DiagnosticArray()
        array.header = image.header
        array.status = [status]
        self.diagnostics_publisher.publish(array)


def main() -> None:
    rclpy.init()
    node: LaneDetector | None = None
    try:
        node = LaneDetector()
        rclpy.spin(node)
    except (ValueError, RuntimeError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"lane_detector: {error}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
