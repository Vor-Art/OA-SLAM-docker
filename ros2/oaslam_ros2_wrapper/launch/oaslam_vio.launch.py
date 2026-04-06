from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("oaslam_ros2_wrapper"))
    default_params = package_share / "config" / "wrapper_vio.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="Path to the VIO wrapper parameter file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use ROS time from /clock (set true for rosbag playback).",
            ),
            DeclareLaunchArgument(
                "rgb_topic",
                default_value="/camera/color/image_raw",
                description="RGB image topic from D435i.",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/aligned_depth_to_color/image_raw",
                description="Aligned depth image topic from D435i.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/camera/imu",
                description="IMU topic from D435i.",
            ),
            Node(
                package="oaslam_ros2_wrapper",
                executable="oaslam_vio_node",
                name="oaslam_vio_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("rgb_topic", LaunchConfiguration("rgb_topic")),
                    ("depth_topic", LaunchConfiguration("depth_topic")),
                    ("imu_topic", LaunchConfiguration("imu_topic")),
                ],
            ),
        ]
    )
