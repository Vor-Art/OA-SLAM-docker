#!/usr/bin/env bash
set -eo pipefail

# ROS setup scripts may read unset shell variables internally.
set +u
source /opt/ros/humble/setup.bash
source /opt/oaslam_ros2_ws/install/setup.bash
set -u

export HOME="${HOME:-/home/oaslam}"
export ROS_HOME="${ROS_HOME:-${HOME}/.ros}"
mkdir -p "${HOME}" "${ROS_HOME}"
export LD_LIBRARY_PATH="/opt/OA-SLAM/Thirdparty/DBoW2/lib:/opt/OA-SLAM/Thirdparty/g2o/lib:/opt/OA-SLAM/lib:${LD_LIBRARY_PATH:-}"

PARAMS_FILE="${OASLAM_ROS2_PARAMS_FILE:-/opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/wrapper.yaml}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"

exec ros2 launch oaslam_ros2_wrapper oaslam_wrapper.launch.py \
  params_file:="${PARAMS_FILE}" \
  use_sim_time:="${USE_SIM_TIME}"
