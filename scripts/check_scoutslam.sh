#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_PARAMS="/opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/my_records.yaml"
PARAMS_FILE="${1:-${SCOUTSLAM_PARAMS_FILE:-${DEFAULT_PARAMS}}}"
SKIP_OFFLINE_RUN="${SCOUTSLAM_SKIP_OFFLINE_RUN:-0}"
HEADLESS="${SCOUTSLAM_HEADLESS:-1}"
TEMP_PARAMS_HOST=""

cd "${ROOT_DIR}"

cleanup() {
  if [[ -n "${TEMP_PARAMS_HOST}" ]]; then
    rm -f "${TEMP_PARAMS_HOST}"
  fi
}
trap cleanup EXIT

require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    printf 'Missing required command: %s\n' "${command_name}" >&2
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
  printf 'Missing executable Docker wrapper: docker/run.sh\n' >&2
  exit 1
fi

local_params="$(mapped_local_path "${PARAMS_FILE}")"
if [[ -n "${local_params}" && ! -f "${local_params}" ]]; then
  printf 'Configured params file is not present in the checkout: %s\n' "${local_params}" >&2
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

printf '== ScoutSLAM container status ==\n'
./docker/run.sh status

printf '\n== Build C++ artifacts ==\n'
./docker/run.sh rebuild-cpp

printf '\n== Build ROS 2 artifacts ==\n'
./docker/run.sh rebuild-ros2

if [[ "${SKIP_OFFLINE_RUN}" == "1" ]]; then
  printf '\nSKIP: offline run disabled by SCOUTSLAM_SKIP_OFFLINE_RUN=1\n'
  exit 0
fi

printf '\n== Offline run ==\n'
printf 'Params: %s\n' "${RUN_PARAMS_FILE}"
if [[ "${RUN_PARAMS_FILE}" != "${PARAMS_FILE}" ]]; then
  printf 'Headless params copy generated from: %s\n' "${PARAMS_FILE}"
fi
RUN_LOG="$(mktemp -t scoutslam_offline.XXXXXX.log)"
if ! ./docker/run.sh run-offline "${RUN_PARAMS_FILE}" 2>&1 | tee "${RUN_LOG}"; then
  printf 'Offline run command failed. Log: %s\n' "${RUN_LOG}" >&2
  exit 1
fi

if grep -E "\\[FATAL\\]|process has died|Offline VIO failed|Traceback|Exception" "${RUN_LOG}" >/dev/null; then
  printf 'Offline run reported a fatal node failure. Log: %s\n' "${RUN_LOG}" >&2
  exit 1
fi

printf '\nScoutSLAM check passed.\n'
