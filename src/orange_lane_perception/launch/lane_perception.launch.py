"""Launch the lane baseline and a CSV recorder for quantitative comparison."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    output_topic = LaunchConfiguration("output_topic")
    output_path = LaunchConfiguration("output_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "image_topic", default_value="/camera_front/color/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera_front/color/camera_info"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/perception/lane_center"),
            DeclareLaunchArgument(
                "output_path", default_value="/tmp/lane_predictions.csv"),
            Node(
                package="orange_lane_perception",
                executable="lane_detector",
                name="lane_detector",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "image_topic": image_topic,
                        "camera_info_topic": camera_info_topic,
                        "output_topic": output_topic,
                    }
                ],
            ),
            Node(
                package="orange_lane_perception",
                executable="lane_prediction_recorder",
                name="lane_prediction_recorder",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": output_topic,
                        "output_path": output_path,
                    }
                ],
            ),
        ]
    )
