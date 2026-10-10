#!/usr/bin/env python3
"""Record lane-center PoseStamped messages as evaluator-compatible CSV rows."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .lane_geometry import multiply_quaternions, rotate_vector


class LanePredictionRecorder(Node):
    """Transform local lane estimates into odom and stream them to a CSV file."""

    def __init__(self) -> None:
        super().__init__("lane_prediction_recorder")
        self.declare_parameter("input_topic", "/perception/lane_center")
        self.declare_parameter("output_frame", "odom")
        self.declare_parameter("output_path", "/tmp/lane_predictions.csv")
        self.output_frame = self.get_parameter("output_frame").value
        self.output_path = Path(self.get_parameter("output_path").value)
        if not self.output_frame:
            raise ValueError("output_frame must not be empty")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.stream,
            fieldnames=[
                "timestamp_ns",
                "predicted_center_x_m",
                "predicted_center_y_m",
                "predicted_heading_rad",
            ],
        )
        self.writer.writeheader()
        self.stream.flush()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False)
        self.subscription = self.create_subscription(
            PoseStamped,
            self.get_parameter("input_topic").value,
            self.pose_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Recording lane predictions to {self.output_path}")

    def pose_callback(self, message: PoseStamped) -> None:
        try:
            transformed = self._transform_to_output_frame(message)
        except TransformException as error:
            self.get_logger().warning(
                f"Skipping lane sample without TF: {error}")
            return

        timestamp_ns = message.header.stamp.sec * \
            1_000_000_000 + message.header.stamp.nanosec
        if timestamp_ns == 0:
            timestamp_ns = self.get_clock().now().nanoseconds
        self.writer.writerow(
            {
                "timestamp_ns": timestamp_ns,
                "predicted_center_x_m": f"{transformed.pose.position.x:.6f}",
                "predicted_center_y_m": f"{transformed.pose.position.y:.6f}",
                "predicted_heading_rad": f"{self._yaw(transformed):.6f}",
            }
        )
        self.stream.flush()

    def _transform_to_output_frame(self, message: PoseStamped) -> PoseStamped:
        if message.header.frame_id == self.output_frame:
            return message
        transform = self.tf_buffer.lookup_transform(
            self.output_frame,
            message.header.frame_id,
            Time.from_msg(message.header.stamp),
            timeout=Duration(seconds=0.10),
        )
        translation = transform.transform.translation
        rotation_message = transform.transform.rotation
        rotation = (
            rotation_message.x,
            rotation_message.y,
            rotation_message.z,
            rotation_message.w,
        )
        point = rotate_vector(
            (message.pose.position.x, message.pose.position.y, message.pose.position.z),
            rotation,
        )
        source_orientation = message.pose.orientation
        orientation = multiply_quaternions(
            rotation,
            (
                source_orientation.x,
                source_orientation.y,
                source_orientation.z,
                source_orientation.w,
            ),
        )
        transformed = PoseStamped()
        transformed.header.stamp = message.header.stamp
        transformed.header.frame_id = self.output_frame
        transformed.pose.position.x = point[0] + translation.x
        transformed.pose.position.y = point[1] + translation.y
        transformed.pose.position.z = point[2] + translation.z
        transformed.pose.orientation.x = orientation[0]
        transformed.pose.orientation.y = orientation[1]
        transformed.pose.orientation.z = orientation[2]
        transformed.pose.orientation.w = orientation[3]
        return transformed

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
        self.stream.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: LanePredictionRecorder | None = None
    try:
        node = LanePredictionRecorder()
        rclpy.spin(node)
    except (OSError, ValueError, RuntimeError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"lane_prediction_recorder: {error}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
