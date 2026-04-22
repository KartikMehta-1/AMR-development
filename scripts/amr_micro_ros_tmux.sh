#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="${AMR_TMUX_SESSION:-amr_bench}"
CONTAINER_NAME="${AMR_CONTAINER_NAME:-amr_foxy}"
IMAGE_NAME="${AMR_IMAGE_NAME:-amr/ros2-foxy-jetson:arm64}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_ONLY_VALUE="${ROS_LOCALHOST_ONLY:-0}"
ROS_WS_HOST_PATH="${AMR_ROS_WS_HOST_PATH:-$HOME/AMR-development/ros_ws}"
AGENT_BAUD="${AMR_AGENT_BAUD:-460800}"

default_agent_dev() {
  local dev
  for dev in /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_*; do
    if [[ -e "${dev}" ]]; then
      printf '%s\n' "${dev}"
      return
    fi
  done
  printf '%s\n' "/dev/ttyACM0"
}

AGENT_DEV="${AMR_AGENT_DEV:-$(default_agent_dev)}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_cmd docker
require_cmd tmux

host_arch="$(uname -m)"

fail_with_usage() {
  cat >&2 <<EOF
${1}

This script is the Jetson-local monitor. It expects to run on the Jetson with:
- container: ${CONTAINER_NAME}
- image: ${IMAGE_NAME}

If you are on the laptop/dev PC, use:
  ./scripts/open_amr_monitor.sh

If you intentionally want to override the image/container, set:
  AMR_IMAGE_NAME=...
  AMR_CONTAINER_NAME=...
EOF
  exit 1
}

if [[ "${host_arch}" != "aarch64" && "${host_arch}" != "arm64" ]]; then
  fail_with_usage "Unsupported host architecture '${host_arch}' for ${0}."
fi

ensure_container() {
  local running
  running="$(docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}')"
  if [[ "${running}" == "${CONTAINER_NAME}" ]]; then
    return
  fi

  if docker ps -a --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
    docker rm -f "${CONTAINER_NAME}" >/dev/null
  fi

  if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    fail_with_usage "Docker image '${IMAGE_NAME}' is not available locally."
  fi

  docker run -d --name "${CONTAINER_NAME}" --net=host --privileged --runtime nvidia \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
    -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY_VALUE}" \
    -v "${ROS_WS_HOST_PATH}:/workspaces/ros_ws" \
    "${IMAGE_NAME}" \
    bash -lc "ros2 run micro_ros_agent micro_ros_agent serial --dev ${AGENT_DEV} -b ${AGENT_BAUD} >/tmp/micro_ros_agent.log 2>&1 & while sleep 3600; do :; done" >/dev/null
}

pane_cmd() {
  local command_text="$1"
  printf 'docker exec -e TERM=xterm -i %q /entrypoint.sh bash -lc %q' "${CONTAINER_NAME}" "${command_text}"
}

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  exec tmux attach -t "${SESSION_NAME}"
fi

ensure_container

tmux new-session -d -s "${SESSION_NAME}" -n overview
tmux send-keys -t "${SESSION_NAME}:overview.0" "$(pane_cmd "tail -f /tmp/micro_ros_agent.log")" C-m

tmux split-window -h -t "${SESSION_NAME}:overview.0"
tmux send-keys -t "${SESSION_NAME}:overview.1" "$(pane_cmd "ros2 topic echo /amr/safety_state std_msgs/msg/UInt32 --qos-reliability best_effort")" C-m

tmux split-window -v -t "${SESSION_NAME}:overview.1"
tmux send-keys -t "${SESSION_NAME}:overview.2" "$(pane_cmd "ros2 topic echo /amr/fault_mask std_msgs/msg/Int32 --qos-reliability best_effort")" C-m

tmux split-window -v -t "${SESSION_NAME}:overview.0"
tmux send-keys -t "${SESSION_NAME}:overview.3" "$(pane_cmd "ros2 topic echo /amr/wheel_state sensor_msgs/msg/JointState --qos-reliability best_effort")" C-m

tmux select-layout -t "${SESSION_NAME}:overview" tiled

tmux new-window -t "${SESSION_NAME}" -n currents
tmux send-keys -t "${SESSION_NAME}:currents.0" "$(pane_cmd "ros2 topic echo /amr/current_left_ma std_msgs/msg/Int32 --qos-reliability best_effort")" C-m

tmux split-window -h -t "${SESSION_NAME}:currents.0"
tmux send-keys -t "${SESSION_NAME}:currents.1" "$(pane_cmd "ros2 topic echo /amr/current_right_ma std_msgs/msg/Int32 --qos-reliability best_effort")" C-m

tmux split-window -v -t "${SESSION_NAME}:currents.0"
tmux send-keys -t "${SESSION_NAME}:currents.2" "$(pane_cmd "ros2 topic echo /amr/current_left_adc std_msgs/msg/UInt32 --qos-reliability best_effort")" C-m

tmux split-window -v -t "${SESSION_NAME}:currents.1"
tmux send-keys -t "${SESSION_NAME}:currents.3" "$(pane_cmd "ros2 topic echo /amr/current_right_adc std_msgs/msg/UInt32 --qos-reliability best_effort")" C-m

tmux split-window -v -t "${SESSION_NAME}:currents.2"
tmux send-keys -t "${SESSION_NAME}:currents.4" "$(pane_cmd "ros2 topic echo /amr/current_left_zero std_msgs/msg/UInt32 --qos-reliability best_effort")" C-m

tmux split-window -v -t "${SESSION_NAME}:currents.3"
tmux send-keys -t "${SESSION_NAME}:currents.5" "$(pane_cmd "ros2 topic echo /amr/current_right_zero std_msgs/msg/UInt32 --qos-reliability best_effort")" C-m

tmux select-layout -t "${SESSION_NAME}:currents" tiled

tmux new-window -t "${SESSION_NAME}" -n drive
tmux send-keys -t "${SESSION_NAME}:drive.0" "$(pane_cmd "ros2 topic echo /amr/duty_cmd_left std_msgs/msg/Float32 --qos-reliability best_effort")" C-m

tmux split-window -h -t "${SESSION_NAME}:drive.0"
tmux send-keys -t "${SESSION_NAME}:drive.1" "$(pane_cmd "ros2 topic echo /amr/duty_cmd_right std_msgs/msg/Float32 --qos-reliability best_effort")" C-m

tmux split-window -v -t "${SESSION_NAME}:drive.0"
tmux send-keys -t "${SESSION_NAME}:drive.2" "$(pane_cmd "printf '%s\n%s\n%s\n%s\n%s\n%s\n' \
  'safe reset:' \
  'ros2 topic pub --once /amr/enable std_msgs/msg/Bool \"{data: false}\"' \
  'ros2 topic pub --once /amr/clear_fault std_msgs/msg/Empty \"{}\"' \
  'left +0.5:' \
  'ros2 topic pub --once /amr/wheel_cmd_right std_msgs/msg/Float32 \"{data: 0.0}\"' \
  'ros2 topic pub -r 10 /amr/wheel_cmd_left std_msgs/msg/Float32 \"{data: 0.5}\"' ; bash")" C-m

tmux split-window -v -t "${SESSION_NAME}:drive.1"
tmux send-keys -t "${SESSION_NAME}:drive.3" "$(pane_cmd "bash")" C-m

tmux select-layout -t "${SESSION_NAME}:drive" tiled
tmux select-window -t "${SESSION_NAME}:overview"

exec tmux attach -t "${SESSION_NAME}"
