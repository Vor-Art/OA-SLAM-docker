from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("oaslam_ros2_wrapper"))
    default_params = package_share / "config" / "wrapper.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="Path to the ROS2 wrapper parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use ROS time from /clock.",
            ),
            DeclareLaunchArgument(
                "output_folder",
                default_value="",
                description="Directory to save CameraTrajectory.txt in TUM format. Empty to disable.",
            ),
            Node(
                package="oaslam_ros2_wrapper",
                executable="oaslam_wrapper_node",
                name="oaslam_wrapper",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "output_folder": LaunchConfiguration("output_folder"),
                    },
                ],
            ),
        ]
    )
