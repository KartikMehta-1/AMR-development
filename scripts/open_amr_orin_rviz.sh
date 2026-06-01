#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${AMR_ORIN_RVIZ_IMAGE:-amr/ros2-humble-rviz-nuc:amd64}"
CONTAINER_NAME="${AMR_ORIN_RVIZ_CONTAINER:-amr_orin_rviz}"
ORIN_HOST="${ORIN_HOST:-orin}"
ORIN_CONTAINER_NAME="${AMR_ORIN_HW_CONTAINER:-amr_orin_hw}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_ONLY_VALUE="${ROS_LOCALHOST_ONLY:-0}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ORIN_IP="${ORIN_IP:-192.168.1.20}"
DISPLAY_VALUE="${DISPLAY:-:0}"
LOCAL_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RVIZ_CONFIG="${AMR_ORIN_RVIZ_CONFIG:-/workspaces/AMR-development/ros_ws/src/amr_description/config/orin_rviz.rviz}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_cmd docker
require_cmd ssh
require_cmd xhost

if ! ssh "${ORIN_HOST}" "docker inspect -f '{{.State.Running}}' '${ORIN_CONTAINER_NAME}' 2>/dev/null" | grep -qx true; then
  echo "Orin hardware container ${ORIN_CONTAINER_NAME} is not running on ${ORIN_HOST}." >&2
  echo "Start the supervised Orin hardware launch before opening RViz." >&2
  exit 1
fi

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "Docker image ${IMAGE_NAME} not found. Build it with:" >&2
  echo "  docker build -f docker/nuc-humble-rviz/Dockerfile -t ${IMAGE_NAME} ." >&2
  exit 1
fi

xhost +SI:localuser:root >/dev/null

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
docker run -d --name "${CONTAINER_NAME}" --net=host \
  -e DISPLAY="${DISPLAY_VALUE}" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY_VALUE}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION_VALUE}" \
  -e AMR_CYCLONEDDS_PEER="${ORIN_IP}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${LOCAL_REPO}:/workspaces/AMR-development:rw" \
  -w /workspaces/AMR-development/ros_ws \
  "${IMAGE_NAME}" \
  bash -lc "while sleep 3600; do :; done" >/dev/null

echo "Checking ROS graph/data visibility from NUC RViz container..."
docker exec "${CONTAINER_NAME}" /entrypoint.sh bash -lc "
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 2
  ros2 topic info -v /scan || true
  timeout 6 ros2 topic echo /scan --once --qos-reliability best_effort >/tmp/amr_orin_rviz_scan_sample.txt 2>&1 || true
  if grep -q 'ranges:' /tmp/amr_orin_rviz_scan_sample.txt; then
    echo 'NUC container received /scan sample.'
  else
    echo 'WARNING: NUC container did not receive /scan sample yet. RViz may open without live Orin data.'
    cat /tmp/amr_orin_rviz_scan_sample.txt
  fi
"

docker exec -e DISPLAY="${DISPLAY_VALUE}" -e QT_X11_NO_MITSHM=1 "${CONTAINER_NAME}" \
  /entrypoint.sh bash -lc "rviz2 -d '${RVIZ_CONFIG}'" &

echo "RViz launch requested from container ${CONTAINER_NAME}."
