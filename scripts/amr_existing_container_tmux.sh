#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="${AMR_TMUX_SESSION:-amr_bench}"
CONTAINER_NAME="${AMR_CONTAINER_NAME:-amr_foxy}"
SESSION_WIDTH="${AMR_TMUX_WIDTH:-240}"
SESSION_HEIGHT="${AMR_TMUX_HEIGHT:-70}"
RECREATE_SESSION="${AMR_RECREATE_SESSION:-1}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_cmd docker
require_cmd tmux

fail_with_usage() {
  cat >&2 <<EOF
${1}

This script only attaches monitoring panes to an already-running container.
It does not create, restart, or replace Docker containers.

Expected running container:
  ${CONTAINER_NAME}

Typical flow:
  1. Start the hardware stack first:
     docker run --rm -it --net=host --privileged --runtime nvidia \\
       --name ${CONTAINER_NAME} \\
       amr/ros2-foxy-jetson:arm64 \\
       ros2 launch amr_description hardware.launch.py ...

  2. In another terminal on the same machine, run:
     ./scripts/amr_existing_container_tmux.sh
EOF
  exit 1
}

ensure_running_container() {
  local running
  running="$(docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}')"
  if [[ "${running}" != "${CONTAINER_NAME}" ]]; then
    fail_with_usage "Container '${CONTAINER_NAME}' is not currently running."
  fi
}

pane_cmd() {
  local command_text="$1"
  printf 'docker exec -e TERM=xterm -i %q /entrypoint.sh bash -lc %q' "${CONTAINER_NAME}" "${command_text}"
}

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  if [[ "${RECREATE_SESSION}" == "1" ]]; then
    tmux kill-session -t "${SESSION_NAME}"
  else
    exec tmux attach -t "${SESSION_NAME}"
  fi
fi

ensure_running_container

tmux new-session -d -x "${SESSION_WIDTH}" -y "${SESSION_HEIGHT}" -s "${SESSION_NAME}" -n overview \
  "$(pane_cmd "while true; do printf '\033[2J\033[H'; printf '== launch_status ==\n\n'; printf 'container: ${CONTAINER_NAME} (existing)\n\n'; printf 'nodes:\n'; ros2 node list 2>/dev/null || true; printf '\nAMR topics:\n'; ros2 topic list 2>/dev/null | grep '^/amr/' || true; sleep 3; done")"

overview_main="$(tmux display-message -p -t "${SESSION_NAME}:overview.0" '#{pane_id}')"
overview_safety="$(tmux split-window -h -P -F '#{pane_id}' -t "${overview_main}" "$(pane_cmd "ros2 topic echo /amr/safety_state std_msgs/msg/UInt32 --qos-reliability best_effort")")"
tmux split-window -v -P -F '#{pane_id}' -t "${overview_safety}" "$(pane_cmd "ros2 topic echo /amr/fault_mask std_msgs/msg/Int32 --qos-reliability best_effort")" >/dev/null
tmux split-window -v -P -F '#{pane_id}' -t "${overview_main}" "$(pane_cmd "ros2 topic echo /amr/wheel_state sensor_msgs/msg/JointState --qos-reliability best_effort")" >/dev/null

tmux select-layout -t "${SESSION_NAME}:overview" tiled

tmux new-window -d -t "${SESSION_NAME}" -n currents \
  "$(pane_cmd "ros2 topic echo /amr/current_left_ma std_msgs/msg/Int32 --qos-reliability best_effort")"

currents_left="$(tmux display-message -p -t "${SESSION_NAME}:currents.0" '#{pane_id}')"
currents_right="$(tmux split-window -h -P -F '#{pane_id}' -t "${currents_left}" "$(pane_cmd "ros2 topic echo /amr/current_right_ma std_msgs/msg/Int32 --qos-reliability best_effort")")"
currents_left_adc="$(tmux split-window -v -P -F '#{pane_id}' -t "${currents_left}" "$(pane_cmd "ros2 topic echo /amr/current_left_adc std_msgs/msg/UInt32 --qos-reliability best_effort")")"
currents_right_adc="$(tmux split-window -v -P -F '#{pane_id}' -t "${currents_right}" "$(pane_cmd "ros2 topic echo /amr/current_right_adc std_msgs/msg/UInt32 --qos-reliability best_effort")")"
tmux split-window -v -P -F '#{pane_id}' -t "${currents_left_adc}" "$(pane_cmd "ros2 topic echo /amr/current_left_zero std_msgs/msg/UInt32 --qos-reliability best_effort")" >/dev/null
tmux split-window -v -P -F '#{pane_id}' -t "${currents_right_adc}" "$(pane_cmd "ros2 topic echo /amr/current_right_zero std_msgs/msg/UInt32 --qos-reliability best_effort")" >/dev/null

tmux select-layout -t "${SESSION_NAME}:currents" tiled

tmux new-window -d -t "${SESSION_NAME}" -n drive \
  "$(pane_cmd "ros2 topic echo /amr/duty_cmd_left std_msgs/msg/Float32 --qos-reliability best_effort")"

drive_left="$(tmux display-message -p -t "${SESSION_NAME}:drive.0" '#{pane_id}')"
drive_right="$(tmux split-window -h -P -F '#{pane_id}' -t "${drive_left}" "$(pane_cmd "ros2 topic echo /amr/duty_cmd_right std_msgs/msg/Float32 --qos-reliability best_effort")")"
drive_shell="$(tmux split-window -v -P -F '#{pane_id}' -t "${drive_left}" "$(pane_cmd "printf '%s\n%s\n%s\n%s\n' \
  'safe reset:' \
  'ros2 topic pub --once /amr/enable std_msgs/msg/Bool \"{data: false}\"' \
  'ros2 topic pub --once /amr/clear_fault std_msgs/msg/Empty \"{}\"' \
  'teleop monitor shell ready' ; bash")")"
tmux split-window -v -P -F '#{pane_id}' -t "${drive_right}" "$(pane_cmd "bash")" >/dev/null

tmux select-layout -t "${SESSION_NAME}:drive" tiled
tmux select-window -t "${SESSION_NAME}:overview"
tmux select-pane -t "${drive_shell}"

exec tmux attach -t "${SESSION_NAME}"
