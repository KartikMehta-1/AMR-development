#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${AMR_CAMERA_PREVIEW_IMAGE:-amr/ros2-humble-rviz-nuc:amd64}"
CONTAINER_NAME="${AMR_CAMERA_PREVIEW_CONTAINER:-amr_camera_preview}"
ORIN_IP="${ORIN_IP:-192.168.1.20}"
DISPLAY_VALUE="${DISPLAY:-:0}"
LOCAL_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v docker >/dev/null 2>&1; then
  echo "Missing required command: docker" >&2
  exit 1
fi

if ! command -v xhost >/dev/null 2>&1; then
  echo "Missing required command: xhost" >&2
  exit 1
fi

xhost +SI:localuser:root >/dev/null
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker run --rm --name "${CONTAINER_NAME}" --net=host \
  -e DISPLAY="${DISPLAY_VALUE}" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e AMR_CYCLONEDDS_PEER="${ORIN_IP}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${LOCAL_REPO}:/workspaces/AMR-development:ro" \
  "${IMAGE_NAME}" \
  python3 -u /workspaces/AMR-development/scripts/amr_camera_preview_viewer.py
