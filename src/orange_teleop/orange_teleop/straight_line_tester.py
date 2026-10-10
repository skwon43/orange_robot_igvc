import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class StraightLineTester(Node):
    def __init__(self):
        super().__init__("straight_line_tester")
        self.declare_parameter("target_speed", 0.5)
        self.declare_parameter("target_duration", 10.0)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")

        self.target_speed = float(self.get_parameter("target_speed").value)
        self.target_duration = float(
            self.get_parameter("target_duration").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        if self.target_duration <= 0.0 or publish_rate <= 0.0:
            raise ValueError(
                "target_duration and publish_rate must be positive")

        self.publisher_ = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            10,
        )
        self.started_at = None
        self.start_position = None
        self.last_position = None
        self.finished = False
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Straight-line test: {self.target_speed:.3f} m/s for "
            f"{self.target_duration:.3f} s; ideal displacement "
            f"{self.target_speed * self.target_duration:.3f} m"
        )

    def odom_callback(self, message):
        position = message.pose.pose.position
        current_position = (position.x, position.y)
        if self.start_position is None:
            self.start_position = current_position
        self.last_position = current_position

    def timer_callback(self):
        now = self.get_clock().now()
        if self.started_at is None:
            if now.nanoseconds == 0:
                return
            self.started_at = now
        elapsed = (now - self.started_at).nanoseconds / 1e9
        message = Twist()
        if elapsed < self.target_duration:
            message.linear.x = self.target_speed
        else:
            self.publisher_.publish(message)
            self.finished = True
            self.timer.cancel()
            self.report_result()
            return
        self.publisher_.publish(message)

    def report_result(self):
        if self.start_position is None or self.last_position is None:
            self.get_logger().warning("Test ended without receiving odometry")
            return

        delta_x = self.last_position[0] - self.start_position[0]
        delta_y = self.last_position[1] - self.start_position[1]
        displacement = math.hypot(delta_x, delta_y)
        expected = self.target_speed * self.target_duration
        self.get_logger().info(
            f"Test complete: ideal={expected:.3f} m, "
            f"odometry displacement={displacement:.3f} m "
            f"(dx={delta_x:.3f} m, dy={delta_y:.3f} m)"
        )

    def stop(self):
        self.publisher_.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineTester()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted; publishing stop command")
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
