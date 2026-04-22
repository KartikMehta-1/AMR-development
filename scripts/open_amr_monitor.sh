#!/usr/bin/env bash
set -euo pipefail

JETSON_HOST="${JETSON_HOST:-jetson}"
SESSION_NAME="${AMR_LOCAL_TMUX_SESSION:-amr_bench}"
SESSION_WIDTH="${AMR_LOCAL_TMUX_WIDTH:-240}"
SESSION_HEIGHT="${AMR_LOCAL_TMUX_HEIGHT:-70}"
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
  printf "ssh -t %q %q" "${JETSON_HOST}" "AMR_REMOTE_ROS_WS=${REMOTE_REPO}/ros_ws ${REMOTE_HELPER_PATH} ${mode}"
}

tab_payload() {
  local title="$1"
  local cmd="$2"
  printf "printf '== %s ==\\n'; printf 'Starting monitor...\\n\\n'; %s; exec bash" "${title}" "${cmd}"
}

open_terminator_tabs() {
  require_cmd terminator

  local first_title="launch_status"
  local first_cmd
  first_cmd="$(tab_payload "${first_title}" "$(remote_mode_cmd launch_status)")"
  terminator -T "${first_title}" -x bash -lc "${first_cmd}" >/dev/null 2>&1 &
  sleep 1

  add_terminator_tab() {
    local title="$1"
    local mode="$2"
    terminator --new-tab -T "${title}" -x bash -lc "$(tab_payload "${title}" "$(remote_mode_cmd "${mode}")")" >/dev/null 2>&1 &
    sleep 0.2
  }

  add_terminator_tab "state_summary" "state_summary"
  add_terminator_tab "left_summary" "left_summary"
  add_terminator_tab "right_summary" "right_summary"
  add_terminator_tab "drive_shell" "drive_shell"
}

open_gnome_tabs() {
  require_cmd gnome-terminal

  local tabs=()
  add_tab() {
    local title="$1"
    local mode="$2"
    tabs+=(--tab --title="${title}" -- bash -lc "$(tab_payload "${title}" "$(remote_mode_cmd "${mode}")")")
  }

  add_tab "launch_status" "launch_status"
  add_tab "state_summary" "state_summary"
  add_tab "left_summary" "left_summary"
  add_tab "right_summary" "right_summary"
  add_tab "drive_shell" "drive_shell"

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

tmux new-session -d -x "${SESSION_WIDTH}" -y "${SESSION_HEIGHT}" -s "${SESSION_NAME}" -n bench "$(remote_mode_cmd launch_status)"

main_pane="$(tmux display-message -p -t "${SESSION_NAME}:bench.0" '#{pane_id}')"
left_pane="$(tmux split-window -h -p 67 -P -F '#{pane_id}' -t "${main_pane}" "$(remote_mode_cmd left_summary)")"
right_pane="$(tmux split-window -h -p 50 -P -F '#{pane_id}' -t "${left_pane}" "$(remote_mode_cmd right_summary)")"

state_pane="$(tmux split-window -v -p 67 -P -F '#{pane_id}' -t "${main_pane}" "$(remote_mode_cmd state_summary)")"
drive_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${state_pane}" "$(remote_mode_cmd drive_shell)")"

tmux select-pane -t "${drive_pane}"

exec tmux attach -t "${SESSION_NAME}"
