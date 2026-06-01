#!/usr/bin/env bash
set -euo pipefail

ORIN_HOST="${ORIN_HOST:-orin}"
ORIN_IP="${ORIN_IP:-192.168.1.20}"
SESSION_NAME="${AMR_ORIN_DASHBOARD_SESSION:-amr_orin_dashboard}"
CONTAINER_NAME="${AMR_ORIN_NUC_CONTAINER:-amr_orin_nuc_monitor}"
ORIN_CONTAINER_NAME="${AMR_ORIN_HW_CONTAINER:-amr_orin_hw}"
IMAGE_NAME="${AMR_ORIN_NUC_IMAGE:-ros:humble-ros-base}"
PROCESSING_MODE="${AMR_ORIN_DASHBOARD_MODE:-remote}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_ONLY_VALUE="${ROS_LOCALHOST_ONLY:-0}"
LOCAL_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_TMUX_WIDTH="$(tput cols 2>/dev/null || printf '200')"
DEFAULT_TMUX_HEIGHT="$(tput lines 2>/dev/null || printf '60')"
SESSION_WIDTH="${AMR_ORIN_TMUX_WIDTH:-${DEFAULT_TMUX_WIDTH}}"
SESSION_HEIGHT="${AMR_ORIN_TMUX_HEIGHT:-${DEFAULT_TMUX_HEIGHT}}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

nuc_ip_for_orin() {
  ip -4 route get "${ORIN_IP}" 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}'
}

start_monitor_container() {
  local nuc_ip="$1"
  if [ -z "${nuc_ip}" ]; then
    echo "Could not determine the NUC IP used to reach ${ORIN_IP}." >&2
    exit 1
  fi

  if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    echo "Docker image ${IMAGE_NAME} is not present locally; pulling it now." >&2
    docker pull "${IMAGE_NAME}"
  fi

  if [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || true)" != "true" ]; then
    docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    docker run -d --name "${CONTAINER_NAME}" --net=host \
      -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
      -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY_VALUE}" \
      -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      -v "${LOCAL_REPO}:/workspaces/AMR-development:ro" \
      "${IMAGE_NAME}" \
      bash -lc "while sleep 3600; do :; done" >/dev/null
  fi
}

monitor_cmd() {
  local mode="$1"
  printf "docker exec -e TERM=xterm -it %q bash -lc %q" \
    "${CONTAINER_NAME}" \
    "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}; export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY_VALUE}; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; unset FASTRTPS_DEFAULT_PROFILES_FILE; source /opt/ros/humble/setup.bash; python3 -u /workspaces/AMR-development/scripts/amr_orin_dashboard_monitor.py --mode ${mode}"
}

cli_cmd() {
  local text="$1"
  printf "docker exec -e TERM=xterm -it %q bash -lc %q" \
    "${CONTAINER_NAME}" \
    "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}; export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY_VALUE}; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; unset FASTRTPS_DEFAULT_PROFILES_FILE; source /opt/ros/humble/setup.bash; ${text}"
}

remote_env='container_env() { tr '\''\0'\'' '\''\n'\'' < /proc/1/environ 2>/dev/null | awk -F= -v key="$1" '\''$1 == key {print substr($0, length(key) + 2); exit}'\''; }; export RMW_IMPLEMENTATION="$(container_env RMW_IMPLEMENTATION)"; [ -n "${RMW_IMPLEMENTATION}" ] || export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; if [ "${RMW_IMPLEMENTATION}" = "rmw_fastrtps_cpp" ]; then export FASTRTPS_DEFAULT_PROFILES_FILE="$(container_env FASTRTPS_DEFAULT_PROFILES_FILE)"; [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE}" ] || export FASTRTPS_DEFAULT_PROFILES_FILE=$(ls /tmp/fastdds_*.xml 2>/dev/null | head -1); elif [ "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp" ]; then export CYCLONEDDS_URI="$(container_env CYCLONEDDS_URI)"; [ -n "${CYCLONEDDS_URI}" ] || export CYCLONEDDS_URI=$(ls /tmp/cyclonedds_*.xml 2>/dev/null | head -1 | sed "s#^#file://#"); fi; export ROS_DOMAIN_ID='"${ROS_DOMAIN_ID_VALUE}"'; export ROS_LOCALHOST_ONLY='"${ROS_LOCALHOST_ONLY_VALUE}"'; source /opt/ros/humble/setup.bash; source /opt/ros/driver_ws/install/setup.bash; source /workspaces/AMR-development/ros_ws/install/setup.bash'

remote_monitor_cmd() {
  local mode="$1"
  printf "ssh -t %q %q" \
    "${ORIN_HOST}" \
    "docker exec -e TERM=xterm -it ${ORIN_CONTAINER_NAME} bash -lc $(printf '%q' "${remote_env}; python3 -u /workspaces/AMR-development/scripts/amr_orin_dashboard_monitor.py --mode ${mode}")"
}

remote_cli_cmd() {
  local text="$1"
  printf "ssh -t %q %q" \
    "${ORIN_HOST}" \
    "docker exec -e TERM=xterm -it ${ORIN_CONTAINER_NAME} bash -lc $(printf '%q' "${remote_env}; ${text}")"
}

remote_static_cmd() {
  local title="$1"
  local text="$2"
  printf "ssh -t %q %q" \
    "${ORIN_HOST}" \
    "docker exec -e TERM=xterm -it ${ORIN_CONTAINER_NAME} bash -lc $(printf '%q' "${remote_env}; printf '== ${title} ==\n\n'; ${text}; printf '\nScroll with mouse wheel, or Ctrl-b [ then arrows/PageUp/PageDown. Press q to leave copy-mode.\n'; exec bash")"
}

cleanup_remote_dashboard() {
  ssh "${ORIN_HOST}" \
    "docker exec '${ORIN_CONTAINER_NAME}' bash -lc \"pkill -f '[a]mr_orin_dashboard_monitor.py' 2>/dev/null || true\""
}

cleanup_nuc_dashboard() {
  if [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || true)" = "true" ]; then
    docker exec "${CONTAINER_NAME}" bash -lc "pkill -f '[a]mr_orin_dashboard_monitor.py' 2>/dev/null || true"
  fi
}

require_cmd docker
require_cmd ip
require_cmd ssh
require_cmd tmux

if ! ssh "${ORIN_HOST}" "docker inspect -f '{{.State.Running}}' '${ORIN_CONTAINER_NAME}' 2>/dev/null" | grep -qx true; then
  echo "Orin hardware container ${ORIN_CONTAINER_NAME} is not running on ${ORIN_HOST}." >&2
  echo "Start the supervised Orin hardware launch before opening the NUC dashboard." >&2
  exit 1
fi

if [ "${PROCESSING_MODE}" = "nuc" ]; then
  NUC_IP="$(nuc_ip_for_orin)"
  start_monitor_container "${NUC_IP}"
  cleanup_nuc_dashboard
  TOPICS_CMD="$(monitor_cmd topics)"
  NODES_CMD="$(monitor_cmd nodes)"
  WHEELS_CMD="$(monitor_cmd wheels)"
  STATE_CMD="$(monitor_cmd state)"
else
  cleanup_remote_dashboard
  TOPICS_CMD="$(remote_monitor_cmd topics)"
  NODES_CMD="$(remote_monitor_cmd nodes)"
  WHEELS_CMD="$(remote_monitor_cmd wheels)"
  STATE_CMD="$(remote_monitor_cmd state)"
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  tmux kill-session -t "${SESSION_NAME}"
fi

tmux new-session -d -x "${SESSION_WIDTH}" -y "${SESSION_HEIGHT}" -s "${SESSION_NAME}" -n dashboard "${TOPICS_CMD}"
tmux set-option -t "${SESSION_NAME}" -g mouse on >/dev/null
tmux set-option -t "${SESSION_NAME}" -g history-limit 50000 >/dev/null
tmux set-option -t "${SESSION_NAME}" -g mode-keys vi >/dev/null

topics_pane="$(tmux display-message -p -t "${SESSION_NAME}:dashboard.0" '#{pane_id}')"
nodes_pane="$(tmux split-window -h -l 66% -P -F '#{pane_id}' -t "${topics_pane}" "${NODES_CMD}")"
wheels_pane="$(tmux split-window -h -l 50% -P -F '#{pane_id}' -t "${nodes_pane}" "${WHEELS_CMD}")"
state_pane="$(tmux split-window -v -l 65% -P -F '#{pane_id}' -t "${wheels_pane}" "${STATE_CMD}")"

tmux select-pane -t "${topics_pane}" -T "topics"
tmux select-pane -t "${nodes_pane}" -T "nodes"
tmux select-pane -t "${wheels_pane}" -T "wheels"
tmux select-pane -t "${state_pane}" -T "state"

if [ "${PROCESSING_MODE}" = "remote" ]; then
  tmux new-window -d -t "${SESSION_NAME}" -n lists "$(remote_static_cmd "Full Topics" "ros2 topic list -t | sort")"
  lists_left="$(tmux display-message -p -t "${SESSION_NAME}:lists.0" '#{pane_id}')"
  tmux split-window -h -l 50% -t "${lists_left}" "$(remote_static_cmd "Full Nodes and Services" "printf '-- Nodes --\n'; ros2 node list | sort; printf '\n-- Services --\n'; ros2 service list | sort")" >/dev/null
else
  tmux new-window -d -t "${SESSION_NAME}" -n lists "$(cli_cmd "printf '== Full Topics ==\n\n'; ros2 topic list -t | sort; printf '\nScroll with mouse wheel, or Ctrl-b [ then arrows/PageUp/PageDown. Press q to leave copy-mode.\n'; exec bash")"
  lists_left="$(tmux display-message -p -t "${SESSION_NAME}:lists.0" '#{pane_id}')"
  tmux split-window -h -l 50% -t "${lists_left}" "$(cli_cmd "printf '== Full Nodes and Services ==\n\n-- Nodes --\n'; ros2 node list | sort; printf '\n-- Services --\n'; ros2 service list | sort; printf '\nScroll with mouse wheel, or Ctrl-b [ then arrows/PageUp/PageDown. Press q to leave copy-mode.\n'; exec bash")" >/dev/null
fi

tmux select-pane -t "${topics_pane}"
tmux select-window -t "${SESSION_NAME}:dashboard"

exec tmux attach -t "${SESSION_NAME}"
