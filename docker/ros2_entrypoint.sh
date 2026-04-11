#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export ROS_WS="${ROS_WS:-/opt/oaslam_ros2_ws}"
export OASLAM_CPP_INSTALL_PREFIX="${OASLAM_CPP_INSTALL_PREFIX:-/opt/oaslam_artifacts/cpp/install}"

if [[ ! -f "${ROS_WS}/install/setup.bash" ]]; then
  echo "ROS 2 workspace is not built. Run ./docker/run.sh rebuild-ros2 first." >&2
  exit 1
fi

# ROS setup scripts may read unset shell variables internally.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROS_WS}/install/setup.bash"
set -u

export HOME="${HOME:-/root}"
export ROS_HOME="${ROS_HOME:-${HOME}/.ros}"
mkdir -p "${HOME}" "${ROS_HOME}"
export LD_LIBRARY_PATH="${OASLAM_CPP_INSTALL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

LAUNCH_FILE="${OASLAM_ROS2_LAUNCH_FILE:-oaslam_vio.launch.py}"
PARAMS_FILE="${OASLAM_ROS2_PARAMS_FILE:-/opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/online_vio.yaml}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"

exec ros2 launch oaslam_ros2_wrapper "${LAUNCH_FILE}" \
  params_file:="${PARAMS_FILE}" \
  use_sim_time:="${USE_SIM_TIME}"
