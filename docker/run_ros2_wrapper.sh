#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /opt/oaslam_ros2_ws/install/setup.bash

PARAMS_FILE="${OASLAM_ROS2_PARAMS_FILE:-/opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/wrapper.yaml}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"

exec ros2 launch oaslam_ros2_wrapper oaslam_wrapper.launch.py \
  params_file:="${PARAMS_FILE}" \
  use_sim_time:="${USE_SIM_TIME}"
