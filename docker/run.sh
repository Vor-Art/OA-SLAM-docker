#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="docker/docker-compose.yml"
ENV_FILE="docker/.env"
SERVICE="oa-slam"

prepare_proxy_env() {
  local http_proxy_value="${HTTP_PROXY:-${http_proxy:-}}"
  local https_proxy_value="${HTTPS_PROXY:-${https_proxy:-}}"
  local ftp_proxy_value="${FTP_PROXY:-${ftp_proxy:-}}"
  local all_proxy_value="${ALL_PROXY:-${all_proxy:-}}"
  local no_proxy_value="${NO_PROXY:-${no_proxy:-}}"

  export HTTP_PROXY="$http_proxy_value"
  export HTTPS_PROXY="$https_proxy_value"
  export FTP_PROXY="$ftp_proxy_value"
  export ALL_PROXY="$all_proxy_value"
  export NO_PROXY="$no_proxy_value"

  export http_proxy="$HTTP_PROXY"
  export https_proxy="$HTTPS_PROXY"
  export ftp_proxy="$FTP_PROXY"
  export all_proxy="$ALL_PROXY"
  export no_proxy="$NO_PROXY"

  if [[ -n "$HTTP_PROXY$HTTPS_PROXY$ALL_PROXY" ]]; then
    echo "Docker proxy settings:"
    [[ -n "$HTTP_PROXY" ]] && echo "  HTTP_PROXY=$HTTP_PROXY"
    [[ -n "$HTTPS_PROXY" ]] && echo "  HTTPS_PROXY=$HTTPS_PROXY"
    [[ -n "$ALL_PROXY" ]] && echo "  ALL_PROXY=$ALL_PROXY"
  fi
}

run_compose() {
  prepare_proxy_env
  (cd "$PROJECT_DIR" && docker compose --env-file "$ENV_FILE" -f "$COMPOSE_FILE" "$@")
}

is_running() {
  docker ps --format '{{.Names}}' | grep -q "^${SERVICE}$"
}

build_image() {
  prepare_proxy_env
  (cd "$PROJECT_DIR" && docker compose --progress plain --env-file "$ENV_FILE" -f "$COMPOSE_FILE" build "$SERVICE")
}

start_container() {
  run_compose up -d "$SERVICE"
}

stop_container() {
  run_compose down
}

enter_shell() {
  docker exec -it "$SERVICE" bash
}

build_cpp() {
  docker exec -it "$SERVICE" bash -lc '
    set -euo pipefail
    BUILD_DIR="${OASLAM_CPP_BUILD_DIR:-/opt/OA-SLAM/build}"
    INSTALL_PREFIX="${OASLAM_CPP_INSTALL_PREFIX:-/opt/oaslam_artifacts/cpp/install}"
    JOBS="${NPROC:-$(nproc)}"
    ORBSLAM3_THIRDPARTY_JOBS="${OASLAM_ORBSLAM3_THIRDPARTY_JOBS:-10}"
    ORBSLAM3_INTERNAL_JOBS="${OASLAM_ORBSLAM3_INTERNAL_JOBS:-6}"

    cmake -S /opt/OA-SLAM -B "${BUILD_DIR}" \
      -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
      -DCUDAToolkit_ROOT=/usr/local/cuda

    cmake --build "${BUILD_DIR}" --parallel "${ORBSLAM3_THIRDPARTY_JOBS}" --target orbslam3_g2o orbslam3_DBoW2
    cmake --build "${BUILD_DIR}" --parallel "${ORBSLAM3_INTERNAL_JOBS}" --target oaslam_orbslam3_internal
    cmake --build "${BUILD_DIR}" --parallel "${JOBS}" -- --output-sync=target
    cmake --install "${BUILD_DIR}"
  '
}

build_ros2() {
  docker exec -it "$SERVICE" bash -lc '
    set -euo pipefail
    export ROS_DISTRO="${ROS_DISTRO:-humble}"
    export ROS_WS="${ROS_WS:-/opt/oaslam_ros2_ws}"
    export OASLAM_CPP_INSTALL_PREFIX="${OASLAM_CPP_INSTALL_PREFIX:-/opt/oaslam_artifacts/cpp/install}"
    export CMAKE_PREFIX_PATH="${OASLAM_CPP_INSTALL_PREFIX}:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="${OASLAM_CPP_INSTALL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

    if [[ ! -x "${OASLAM_CPP_INSTALL_PREFIX}/bin/oa-slam" ]]; then
      echo "OA-SLAM C++ artifacts are missing. Run ./docker/run.sh rebuild-cpp first." >&2
      exit 1
    fi

    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u

    cd "${ROS_WS}"
    colcon build \
      --merge-install \
      --packages-select cv_bridge oaslam_ros2_wrapper \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/usr/local/lib/cmake/opencv4
  '
}

run_mapping() {
  local args=(python3 /opt/OA-SLAM/docker/oa_slam_start.py)
  if [[ -n "${1:-}" ]]; then
    args+=("$1")
  fi
  docker exec -it "$SERVICE" "${args[@]}"
}

run_ros2() {
  local exec_args=(-it)
  if [[ -n "${1:-}" ]]; then
    exec_args+=(-e "OASLAM_ROS2_LAUNCH_FILE=$1")
  fi
  if [[ -n "${2:-}" ]]; then
    exec_args+=(-e "OASLAM_ROS2_PARAMS_FILE=$2")
  fi
  docker exec "${exec_args[@]}" "$SERVICE" /bin/bash /opt/OA-SLAM/docker/ros2_wrapper_start.sh
}

status() {
  docker ps -a --filter "name=^${SERVICE}$"
}

require_running() {
  if ! is_running; then
    echo "Container '$SERVICE' is not running. Start it with: ./docker/run.sh start"
    exit 1
  fi
}

require_cpp_built() {
  if ! docker exec "$SERVICE" test -x /opt/oaslam_artifacts/cpp/install/bin/oa-slam; then
    echo "OA-SLAM C++ artifacts are missing. Build them with: ./docker/run.sh rebuild-cpp"
    exit 1
  fi
}

require_ros2_built() {
  if ! docker exec "$SERVICE" test -f /opt/oaslam_ros2_ws/install/setup.bash; then
    echo "ROS 2 artifacts are missing. Build them with: ./docker/run.sh rebuild-ros2"
    exit 1
  fi
}

case "${1:-help}" in
  build)
    build_image
    ;;
  start)
    start_container
    ;;
  stop)
    stop_container
    ;;
  restart)
    stop_container
    start_container
    ;;
  shell)
    require_running
    enter_shell
    ;;
  rebuild)
    require_running
    build_cpp
    build_ros2
    ;;
  rebuild-cpp)
    require_running
    build_cpp
    ;;
  rebuild-ros2)
    require_running
    build_ros2
    ;;
  run)
    require_running
    require_cpp_built
    run_mapping "${2:-}"
    ;;
  run-ros2)
    require_running
    require_ros2_built
    run_ros2 "${2:-}" "${3:-}"
    ;;
  logs)
    run_compose logs -f "$SERVICE"
    ;;
  status)
    status
    ;;
  help|*)
    echo "Usage: $0 {build|start|stop|restart|shell|rebuild|rebuild-cpp|rebuild-ros2|run [config]|run-ros2 [launch] [params]|logs|status}"
    echo ""
    echo "Examples:"
    echo "  $0 build"
    echo "  $0 start"
    echo "  $0 rebuild"
    echo "  $0 rebuild-cpp"
    echo "  $0 rebuild-ros2"
    echo "  $0 run"
    echo "  $0 run docker/run_config.yaml"
    echo "  $0 run-ros2 oaslam_wrapper.launch.py /opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/wrapper.yaml"
    ;;
esac
