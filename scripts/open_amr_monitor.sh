#!/usr/bin/env bash
set -euo pipefail

JETSON_HOST="${JETSON_HOST:-jetson}"
SESSION_NAME="${AMR_LOCAL_TMUX_SESSION:-amr_bench}"
DEFAULT_TMUX_WIDTH="$(tput cols 2>/dev/null || printf '200')"
DEFAULT_TMUX_HEIGHT="$(tput lines 2>/dev/null || printf '60')"
SESSION_WIDTH="${AMR_LOCAL_TMUX_WIDTH:-${DEFAULT_TMUX_WIDTH}}"
SESSION_HEIGHT="${AMR_LOCAL_TMUX_HEIGHT:-${DEFAULT_TMUX_HEIGHT}}"
RECREATE_SESSION="${AMR_RECREATE_SESSION:-1}"
REMOTE_HELPER_PATH="${AMR_REMOTE_HELPER_PATH:-/tmp/amr_remote_monitor_cmd.sh}"
LOCAL_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_REPO="${AMR_REMOTE_REPO:-\$HOME/AMR-development}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_cmd ssh
require_cmd scp

copy_remote_helper() {
  scp "${LOCAL_REPO}/scripts/amr_remote_monitor_cmd.sh" "${JETSON_HOST}:${REMOTE_HELPER_PATH}" >/dev/null
  ssh -t "${JETSON_HOST}" "chmod +x ${REMOTE_HELPER_PATH}" >/dev/null
}

remote_mode_cmd() {
  local mode="$1"
  printf "ssh %q %q" "${JETSON_HOST}" "AMR_REMOTE_ROS_WS=${REMOTE_REPO}/ros_ws ${REMOTE_HELPER_PATH} ${mode}"
}

remote_mode_cmd_tty() {
  local mode="$1"
  printf "ssh -t %q %q" "${JETSON_HOST}" "AMR_REMOTE_ROS_WS=${REMOTE_REPO}/ros_ws ${REMOTE_HELPER_PATH} ${mode}"
}

tab_payload() {
  local title="$1"
  local cmd="$2"
  printf "printf '== %s ==\\n'; printf 'Starting monitor...\\n\\n'; %s; exec bash" "${title}" "${cmd}"
}

open_terminator_tabs() {
  require_cmd terminator

  local first_title="topics"
  local first_cmd
  first_cmd="$(tab_payload "${first_title}" "$(remote_mode_cmd topics_status)")"
  terminator -T "${first_title}" -x bash -lc "${first_cmd}" >/dev/null 2>&1 &
  sleep 1

  add_terminator_tab() {
    local title="$1"
    local mode="$2"
    terminator --new-tab -T "${title}" -x bash -lc "$(tab_payload "${title}" "$(remote_mode_cmd "${mode}")")" >/dev/null 2>&1 &
    sleep 0.2
  }

  add_terminator_tab "nodes" "nodes_status"
  add_terminator_tab "agent_log" "agent_log"
  add_terminator_tab "safety_fault" "state_summary"
  add_terminator_tab "left_summary" "left_summary"
  add_terminator_tab "right_summary" "right_summary"
  terminator --new-tab -T "drive_shell" -x bash -lc "$(tab_payload "drive_shell" "$(remote_mode_cmd_tty drive_shell)")" >/dev/null 2>&1 &
}

open_gnome_tabs() {
  require_cmd gnome-terminal

  local tabs=()
  add_tab() {
    local title="$1"
    local mode="$2"
    tabs+=(--tab --title="${title}" -- bash -lc "$(tab_payload "${title}" "$(remote_mode_cmd "${mode}")")")
  }

  add_tab "topics" "topics_status"
  add_tab "nodes" "nodes_status"
  add_tab "agent_log" "agent_log"
  add_tab "safety_fault" "state_summary"
  add_tab "left_summary" "left_summary"
  add_tab "right_summary" "right_summary"
  tabs+=(--tab --title="drive_shell" -- bash -lc "$(tab_payload "drive_shell" "$(remote_mode_cmd_tty drive_shell)")")

  gnome-terminal "${tabs[@]}"
}

copy_remote_helper

if ! command -v tmux >/dev/null 2>&1; then
  if command -v terminator >/dev/null 2>&1; then
    open_terminator_tabs
  else
    open_gnome_tabs
  fi
  exit 0
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  if [[ "${RECREATE_SESSION}" == "1" ]]; then
    tmux kill-session -t "${SESSION_NAME}"
  else
  exec tmux attach -t "${SESSION_NAME}"
  fi
fi

tmux new-session -d -x "${SESSION_WIDTH}" -y "${SESSION_HEIGHT}" -s "${SESSION_NAME}" -n bench "$(remote_mode_cmd topics_status)"

left_pane="$(tmux display-message -p -t "${SESSION_NAME}:bench.0" '#{pane_id}')"
middle_pane="$(tmux split-window -h -p 67 -P -F '#{pane_id}' -t "${left_pane}" "$(remote_mode_cmd nodes_status)")"
right_pane="$(tmux split-window -h -p 50 -P -F '#{pane_id}' -t "${middle_pane}" "$(remote_mode_cmd left_summary)")"

middle_bottom="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${middle_pane}" "$(remote_mode_cmd agent_log)")"
tmux split-window -v -p 33 -P -F '#{pane_id}' -t "${middle_bottom}" "$(remote_mode_cmd state_summary)" >/dev/null

right_bottom="$(tmux split-window -v -p 67 -P -F '#{pane_id}' -t "${right_pane}" "$(remote_mode_cmd right_summary)")"
drive_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${right_bottom}" "$(remote_mode_cmd_tty drive_shell)")"

tmux select-pane -t "${drive_pane}"

exec tmux attach -t "${SESSION_NAME}"
