"""Launch file for the offline VIO node.

Reads a ROS2 bag (sqlite3) directly and processes frames sequentially
through the OA-SLAM pipeline.  No pub/sub — behaves like a standalone binary.

Usage:
  ros2 launch oaslam_ros2_wrapper oaslam_offline_vio.launch.py \
      bag_path:=/path/to/rosbag \
      output_folder:=/path/to/output

  # With custom config:
  ros2 launch oaslam_ros2_wrapper oaslam_offline_vio.launch.py \
      bag_path:=/path/to/rosbag \
      output_folder:=/path/to/output \
      params_file:=/path/to/custom_config.yaml
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
                "bag_path",
                default_value="",
                description="Path to the rosbag directory (sqlite3 format). REQUIRED.",
            ),
            DeclareLaunchArgument(
                "output_folder",
                default_value="",
                description="Directory to save CameraTrajectory.txt in TUM format. Empty to disable.",
            ),
            Node(
                package="oaslam_ros2_wrapper",
                executable="oaslam_offline_vio_node",
                name="oaslam_offline_vio_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "bag_path": LaunchConfiguration("bag_path"),
                        "output_folder": LaunchConfiguration("output_folder"),
                    },
                ],
            ),
        ]
    )
