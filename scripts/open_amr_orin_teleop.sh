#!/usr/bin/env bash
set -euo pipefail

ORIN_HOST="${ORIN_HOST:-orin}"
CONTAINER_NAME="${CONTAINER_NAME:-amr_orin_hw}"
AMR_TELEOP_SPEED="${AMR_TELEOP_SPEED:-0.05}"
AMR_TELEOP_TURN="${AMR_TELEOP_TURN:-0.15}"
AMR_TELEOP_TOPIC="${AMR_TELEOP_TOPIC:-/diff_drive_controller/cmd_vel_unstamped}"

if ! ssh "${ORIN_HOST}" "docker inspect -f '{{.State.Running}}' '${CONTAINER_NAME}' 2>/dev/null" | grep -qx true; then
  echo "Container '${CONTAINER_NAME}' is not running on ${ORIN_HOST}." >&2
  echo "Start the supervised Orin hardware launch first, then run this teleop helper again." >&2
  exit 1
fi

read -r -d '' CONTAINER_CMD <<'EOF' || true
set -eo pipefail

container_env() {
  tr '\0' '\n' < /proc/1/environ 2>/dev/null | awk -F= -v key="$1" '$1 == key {print substr($0, length(key) + 2); exit}'
}

export RMW_IMPLEMENTATION="$(container_env RMW_IMPLEMENTATION)"
if [ -z "${RMW_IMPLEMENTATION}" ]; then
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi

if [ "${RMW_IMPLEMENTATION}" = "rmw_fastrtps_cpp" ]; then
  FASTDDS_PROFILE="$(container_env FASTRTPS_DEFAULT_PROFILES_FILE)"
  if [ -z "${FASTDDS_PROFILE}" ] || [ ! -f "${FASTDDS_PROFILE}" ]; then
    FASTDDS_PROFILE="$(ls /tmp/fastdds_*.xml 2>/dev/null | head -1 || true)"
  fi
  if [ -z "${FASTDDS_PROFILE}" ] || [ ! -f "${FASTDDS_PROFILE}" ]; then
    echo "No FastDDS profile found in /tmp. Start the Orin hardware launch first." >&2
    exit 1
  fi
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_PROFILE}"
elif [ "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp" ]; then
  CYCLONEDDS_PROFILE="$(container_env CYCLONEDDS_URI)"
  if [ -z "${CYCLONEDDS_PROFILE}" ]; then
    CYCLONEDDS_PROFILE="$(ls /tmp/cyclonedds_*.xml 2>/dev/null | head -1 | sed 's#^#file://#' || true)"
  fi
  if [ -n "${CYCLONEDDS_PROFILE}" ]; then
    export CYCLONEDDS_URI="${CYCLONEDDS_PROFILE}"
  fi
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# ROS setup files read optional environment variables that may be unset, so do
# not enable nounset in this shell.
source /opt/ros/humble/setup.bash
source /opt/ros/driver_ws/install/setup.bash
source /workspaces/AMR-development/ros_ws/install/setup.bash

ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "Teleop topic: ${AMR_TELEOP_TOPIC}"
ros2 topic info -v "${AMR_TELEOP_TOPIC}" || true
echo
echo "Use i/j/k/l/u/o/m/,/. for motion. Any other key stops. Ctrl-C exits."
echo

exec python3 /workspaces/AMR-development/scripts/amr_teleop_keyboard.py \
  --speed "${AMR_TELEOP_SPEED}" \
  --turn "${AMR_TELEOP_TURN}" \
  --topic "${AMR_TELEOP_TOPIC}"
EOF

ssh -t "${ORIN_HOST}" \
  "docker exec -e TERM=xterm -e AMR_TELEOP_SPEED='${AMR_TELEOP_SPEED}' -e AMR_TELEOP_TURN='${AMR_TELEOP_TURN}' -e AMR_TELEOP_TOPIC='${AMR_TELEOP_TOPIC}' -it '${CONTAINER_NAME}' bash -lc $(printf '%q' "${CONTAINER_CMD}")"
