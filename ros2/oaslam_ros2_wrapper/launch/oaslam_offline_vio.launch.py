"""Launch file for the offline VIO node.

Reads a ROS2 bag (sqlite3) directly and processes frames sequentially
through the OA-SLAM pipeline.  No pub/sub — behaves like a standalone binary.

All parameters (bag_path, output_folder, topics, SLAM settings, etc.) are
read from the YAML params file.  Only the params file path and use_sim_time
are accepted as launch arguments.

Usage:
  ros2 launch oaslam_ros2_wrapper oaslam_offline_vio.launch.py \
      params_file:=/path/to/offline_vio.yaml

  # Via docker compose (set OASLAM_ROS2_LAUNCH_FILE and OASLAM_ROS2_PARAMS_FILE):
  OASLAM_ROS2_LAUNCH_FILE=oaslam_offline_vio.launch.py \
  OASLAM_ROS2_PARAMS_FILE=/opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/offline_vio.yaml \
    docker compose --profile ros2 up oa-slam-ros2
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("oaslam_ros2_wrapper"))
    default_params = package_share / "config" / "offline_vio.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="Path to the offline VIO parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulated time (accepted for compatibility).",
            ),
            Node(
                package="oaslam_ros2_wrapper",
                executable="oaslam_offline_vio_node",
                name="oaslam_offline_vio_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                ],
            ),
        ]
    )
