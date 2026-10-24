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
    mask_mode = LaunchConfiguration("mask_mode")
    roi_top_fraction = LaunchConfiguration("roi_top_fraction")
    roi_bottom_fraction = LaunchConfiguration("roi_bottom_fraction")
    white_threshold = LaunchConfiguration("white_threshold")
    allow_single_boundary = LaunchConfiguration("allow_single_boundary")
    temporal_alpha = LaunchConfiguration("temporal_alpha")
    min_publish_confidence = LaunchConfiguration("min_publish_confidence")

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
            DeclareLaunchArgument("mask_mode", default_value="fixed_white"),
            DeclareLaunchArgument("roi_top_fraction", default_value="0.35"),
            DeclareLaunchArgument("roi_bottom_fraction", default_value="1.0"),
            DeclareLaunchArgument("white_threshold", default_value="180"),
            DeclareLaunchArgument("allow_single_boundary",
                                  default_value="false"),
            DeclareLaunchArgument("temporal_alpha", default_value="1.0"),
            DeclareLaunchArgument(
                "min_publish_confidence", default_value="0.0"),
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
                        "mask_mode": mask_mode,
                        "roi_top_fraction": roi_top_fraction,
                        "roi_bottom_fraction": roi_bottom_fraction,
                        "white_threshold": white_threshold,
                        "allow_single_boundary": allow_single_boundary,
                        "temporal_alpha": temporal_alpha,
                        "min_publish_confidence": min_publish_confidence,
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
