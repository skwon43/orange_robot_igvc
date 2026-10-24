"""Run the deterministic course, robust lane perception, and autonomy stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_simulation = LaunchConfiguration("start_simulation")
    autostart = LaunchConfiguration("autostart")
    mask_mode = LaunchConfiguration("mask_mode")
    roi_top_fraction = LaunchConfiguration("roi_top_fraction")
    roi_bottom_fraction = LaunchConfiguration("roi_bottom_fraction")
    white_threshold = LaunchConfiguration("white_threshold")
    output_path = LaunchConfiguration("output_path")

    simulation = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("orange_gazebo"), "launch",
                 "orange_igvc_baseline.launch.xml"]
            )
        ),
        condition=IfCondition(start_simulation),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("orange_lane_perception"),
                    "launch",
                    "lane_perception.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "mask_mode": mask_mode,
            "roi_top_fraction": roi_top_fraction,
            "roi_bottom_fraction": roi_bottom_fraction,
            "white_threshold": white_threshold,
            "allow_single_boundary": "true",
            "temporal_alpha": "0.35",
            "min_publish_confidence": "0.35",
            "output_path": output_path,
        }.items(),
    )
    controller = Node(
        package="orange_autonomy",
        executable="autonomy_controller",
        name="autonomy_controller",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("orange_autonomy"),
                 "config", "autonomy.yaml"]
            ),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "enabled": ParameterValue(autostart, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_simulation", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("mask_mode", default_value="fixed_white"),
            DeclareLaunchArgument("roi_top_fraction", default_value="0.0"),
            DeclareLaunchArgument("roi_bottom_fraction", default_value="0.70"),
            DeclareLaunchArgument("white_threshold", default_value="145"),
            DeclareLaunchArgument(
                "output_path", default_value="/tmp/lane_predictions_autonomous.csv"
            ),
            simulation,
            perception,
            controller,
        ]
    )
