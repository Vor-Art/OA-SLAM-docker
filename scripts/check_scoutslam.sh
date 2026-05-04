#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_PARAMS="/opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/my_records.yaml"
PARAMS_FILE="${1:-${SCOUTSLAM_PARAMS_FILE:-${DEFAULT_PARAMS}}}"
SKIP_OFFLINE_RUN="${SCOUTSLAM_SKIP_OFFLINE_RUN:-0}"
HEADLESS="${SCOUTSLAM_HEADLESS:-1}"
TEMP_PARAMS_HOST=""

cd "${ROOT_DIR}"

color_enabled() {
  if [[ "${SCOUTSLAM_COLOR:-}" =~ ^(0|false|never|off)$ ]]; then
    return 1
  fi
  if [[ "${SCOUTSLAM_COLOR:-}" =~ ^(1|true|always|on)$ ]]; then
    return 0
  fi
  [[ -z "${NO_COLOR:-}" && "${TERM:-}" != "dumb" ]]
}

paint() {
  local code="$1"
  local text="$2"
  if color_enabled; then
    printf '\033[%sm%s\033[0m' "${code}" "${text}"
  else
    printf '%s' "${text}"
  fi
}

section() {
  printf '\n%s %s\n' "$(paint '1;36' '[ScoutSLAM]')" "$(paint '1;37' "$1")"
}

ok() {
  printf '%s %s\n' "$(paint '1;32' 'OK')" "$1"
}

warn() {
  printf '%s %s\n' "$(paint '1;33' 'WARN')" "$1"
}

fail() {
  printf '%s %s\n' "$(paint '1;31' 'FAIL')" "$1" >&2
}

cleanup() {
  if [[ -n "${TEMP_PARAMS_HOST}" ]]; then
    rm -f "${TEMP_PARAMS_HOST}"
  fi
}
trap cleanup EXIT

require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    fail "missing required command: ${command_name}"
    exit 1
  fi
}

mapped_local_path() {
  local container_path="$1"
  if [[ "${container_path}" == /opt/OA-SLAM/* ]]; then
    printf '%s/%s\n' "${ROOT_DIR}" "${container_path#/opt/OA-SLAM/}"
  fi
}

require_command docker

if [[ ! -x docker/run.sh ]]; then
  fail "missing executable Docker wrapper: docker/run.sh"
  exit 1
fi

local_params="$(mapped_local_path "${PARAMS_FILE}")"
if [[ -n "${local_params}" && ! -f "${local_params}" ]]; then
  fail "configured params file is not present in the checkout: ${local_params}"
  exit 1
fi

RUN_PARAMS_FILE="${PARAMS_FILE}"
if [[ "${HEADLESS}" == "1" && -n "${local_params}" ]]; then
  mkdir -p ros2/oaslam_ros2_wrapper/config/.generated
  TEMP_PARAMS_HOST="$(mktemp "${ROOT_DIR}/ros2/oaslam_ros2_wrapper/config/.generated/scoutslam_params.XXXXXX.yaml")"
  sed -E 's/^([[:space:]]*use_viewer:[[:space:]]*)true([[:space:]]*(#.*)?)$/\1false\2/' \
    "${local_params}" > "${TEMP_PARAMS_HOST}"
  RUN_PARAMS_FILE="/opt/OA-SLAM/${TEMP_PARAMS_HOST#"${ROOT_DIR}/"}"
fi

section "container status"
./docker/run.sh status

section "build C++ artifacts"
./docker/run.sh rebuild-cpp
ok "C++ artifacts built"

section "build ROS 2 artifacts"
./docker/run.sh rebuild-ros2
ok "ROS 2 artifacts built"

if [[ "${SKIP_OFFLINE_RUN}" == "1" ]]; then
  warn "offline run disabled by SCOUTSLAM_SKIP_OFFLINE_RUN=1"
  exit 0
fi

section "offline run"
printf '%s %s\n' "$(paint '36' 'params=')" "${RUN_PARAMS_FILE}"
if [[ "${RUN_PARAMS_FILE}" != "${PARAMS_FILE}" ]]; then
  printf '%s %s\n' "$(paint '36' 'headless_source=')" "${PARAMS_FILE}"
fi
RUN_LOG="$(mktemp -t scoutslam_offline.XXXXXX.log)"
if ! ./docker/run.sh run-offline "${RUN_PARAMS_FILE}" 2>&1 | tee "${RUN_LOG}"; then
  fail "offline run command failed. log=${RUN_LOG}"
  exit 1
fi

if grep -E "\\[FATAL\\]|process has died|Offline VIO failed|Traceback|Exception" "${RUN_LOG}" >/dev/null; then
  fail "offline run reported a fatal node failure. log=${RUN_LOG}"
  exit 1
fi

ok "ScoutSLAM check passed"
