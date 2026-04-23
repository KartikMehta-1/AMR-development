#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${AMR_DEVPC_CONTAINER:-amr_devpc}"
SESSION_NAME="${AMR_DEVPC_SESSION:-amr_devpc}"
LOCAL_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISPLAY_VALUE="${DISPLAY:-:0}"
DEFAULT_TMUX_WIDTH="$(tput cols 2>/dev/null || printf '180')"
DEFAULT_TMUX_HEIGHT="$(tput lines 2>/dev/null || printf '48')"
SESSION_WIDTH="${AMR_DEVPC_WIDTH:-${DEFAULT_TMUX_WIDTH}}"
SESSION_HEIGHT="${AMR_DEVPC_HEIGHT:-${DEFAULT_TMUX_HEIGHT}}"
RECREATE_SESSION="${AMR_RECREATE_SESSION:-1}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_cmd docker
require_cmd xhost
require_cmd tmux

container_cmd() {
  local command_text="$1"
  printf 'docker exec -e TERM=xterm -e DISPLAY=%q -e QT_X11_NO_MITSHM=1 -it %q bash -lc %q' \
    "${DISPLAY_VALUE}" "${CONTAINER_NAME}" "${command_text}"
}

xhost +local:root >/dev/null 2>&1 || true

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
docker run -d --name "${CONTAINER_NAME}" --net=host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  -e DISPLAY="${DISPLAY_VALUE}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${LOCAL_REPO}:/workspaces/AMR-development" \
  amr/ros2-foxy-devpc:amd64 \
  bash -lc "while sleep 3600; do :; done" >/dev/null

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  if [[ "${RECREATE_SESSION}" == "1" ]]; then
    tmux kill-session -t "${SESSION_NAME}"
  else
    exec tmux attach -t "${SESSION_NAME}"
  fi
fi

tmux new-session -d -x "${SESSION_WIDTH}" -y "${SESSION_HEIGHT}" -s "${SESSION_NAME}" -n slam \
  "$(container_cmd "source /opt/ros/foxy/setup.bash; export LIBGL_ALWAYS_SOFTWARE=1; rviz2 -d /workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz")"

rviz_pane="$(tmux display-message -p -t "${SESSION_NAME}:slam.0" '#{pane_id}')"
slam_pane="$(tmux split-window -h -p 40 -P -F '#{pane_id}' -t "${rviz_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false params_file:=/workspaces/AMR-development/ros_ws/src/amr_description/config/slam_toolbox_online_async.yaml")")"
teleop_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${slam_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; python3 /workspaces/AMR-development/scripts/amr_teleop_keyboard.py --speed 0.2 --turn 0.15 --topic /diff_drive_controller/cmd_vel_unstamped")")"

tmux select-pane -t "${teleop_pane}"

exec tmux attach -t "${SESSION_NAME}"
