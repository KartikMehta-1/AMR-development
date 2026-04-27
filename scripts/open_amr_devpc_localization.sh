#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${AMR_DEVPC_CONTAINER:-amr_devpc}"
SESSION_NAME="${AMR_DEVPC_SESSION:-amr_devpc_loc}"
LOCAL_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISPLAY_VALUE="${DISPLAY:-:0}"
DEFAULT_TMUX_WIDTH="$(tput cols 2>/dev/null || printf '180')"
DEFAULT_TMUX_HEIGHT="$(tput lines 2>/dev/null || printf '48')"
SESSION_WIDTH="${AMR_DEVPC_WIDTH:-${DEFAULT_TMUX_WIDTH}}"
SESSION_HEIGHT="${AMR_DEVPC_HEIGHT:-${DEFAULT_TMUX_HEIGHT}}"
RECREATE_SESSION="${AMR_RECREATE_SESSION:-1}"
JETSON_HOST="${JETSON_HOST:-jetson}"
JETSON_CONTAINER="${AMR_JETSON_CONTAINER:-amr_foxy}"
REMOTE_REPO="${AMR_REMOTE_REPO:-$HOME/AMR-development}"
AGENT_DEV="${AMR_AGENT_DEV:-/dev/ttyACM0}"
AGENT_BAUD="${AMR_AGENT_BAUD:-460800}"
START_LIDAR="${AMR_START_LIDAR:-true}"
START_CAMERA="${AMR_START_CAMERA:-false}"
STM_RESET_DELAY_SEC="${AMR_STM_RESET_DELAY_SEC:-3}"

usage() {
  cat >&2 <<'EOF'
Usage:
  ./scripts/open_amr_devpc_localization.sh <map_name_or_yaml>

Examples:
  ./scripts/open_amr_devpc_localization.sh my_new_map
  ./scripts/open_amr_devpc_localization.sh my_new_map.yaml
  ./scripts/open_amr_devpc_localization.sh /workspaces/AMR-development/ros_ws/maps/my_new_map.yaml

This starts:
  - Jetson hardware stack (amr_foxy)
  - Jetson ST-LINK reset of the STM after startup
  - RViz
  - Nav2 AMCL localization on the given saved map
  - keyboard teleop for localization checks
EOF
  exit 1
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

resolve_map_paths() {
  local input="$1"
  local container_map
  local host_map

  if [[ "${input}" == /* ]]; then
    container_map="${input}"
    if [[ "${container_map}" == /workspaces/AMR-development/* ]]; then
      host_map="${LOCAL_REPO}${container_map#/workspaces/AMR-development}"
    else
      echo "Absolute map path must be under /workspaces/AMR-development" >&2
      exit 1
    fi
  else
    local map_file="${input}"
    [[ "${map_file}" == *.yaml ]] || map_file="${map_file}.yaml"
    container_map="/workspaces/AMR-development/ros_ws/maps/${map_file}"
    host_map="${LOCAL_REPO}/ros_ws/maps/${map_file}"
  fi

  if [[ ! -f "${host_map}" ]]; then
    echo "Map file not found: ${host_map}" >&2
    exit 1
  fi

  MAP_PATH_CONTAINER="${container_map}"
  MAP_PATH_HOST="${host_map}"
}

require_cmd docker
require_cmd xhost
require_cmd tmux
require_cmd ssh

[[ $# -ge 1 ]] || usage
resolve_map_paths "$1"

start_jetson_hardware() {
  ssh "${JETSON_HOST}" bash -s -- \
    "${JETSON_CONTAINER}" \
    "${REMOTE_REPO}" \
    "${ROS_DOMAIN_ID:-0}" \
    "${ROS_LOCALHOST_ONLY:-0}" \
    "${AGENT_DEV}" \
    "${AGENT_BAUD}" \
    "${START_LIDAR}" \
    "${START_CAMERA}" <<'EOF'
set -euo pipefail

container_name="$1"
remote_repo="$2"
ros_domain_id="$3"
ros_localhost_only="$4"
agent_dev="$5"
agent_baud="$6"
start_lidar="$7"
start_camera="$8"

docker rm -f "${container_name}" >/dev/null 2>&1 || true

docker run -d --name "${container_name}" --net=host --privileged --runtime nvidia \
  -e ROS_DOMAIN_ID="${ros_domain_id}" \
  -e ROS_LOCALHOST_ONLY="${ros_localhost_only}" \
  -v "${remote_repo}/ros_ws:/workspaces/ros_ws" \
  amr/ros2-foxy-jetson:arm64 \
  bash -lc "
cd /workspaces/ros_ws
[ -f /opt/ros/driver_ws/install/setup.bash ] && source /opt/ros/driver_ws/install/setup.bash
colcon build --merge-install --symlink-install --packages-select amr_hardware amr_description
source install/setup.bash
ros2 launch amr_description hardware.launch.py \
  use_sim_time:=false \
  agent_dev:=${agent_dev} \
  agent_baud:=${agent_baud} \
  start_lidar:=${start_lidar} \
  start_camera:=${start_camera}
" >/dev/null
EOF
}

reset_stm_via_stlink() {
  ssh "${JETSON_HOST}" bash -s -- "${STM_RESET_DELAY_SEC}" <<'EOF'
set -euo pipefail

delay_sec="$1"
sleep "${delay_sec}"
sudo -n openocd -s /usr/share/openocd/scripts \
  -f interface/stlink-v2-1.cfg \
  -f target/stm32f4x.cfg \
  -c "init; reset run; shutdown" >/dev/null
EOF
}

container_cmd() {
  local command_text="$1"
  printf 'docker exec -e TERM=xterm -e DISPLAY=%q -e QT_X11_NO_MITSHM=1 -it %q bash -lc %q' \
    "${DISPLAY_VALUE}" "${CONTAINER_NAME}" "${command_text}"
}

xhost +local:root >/dev/null 2>&1 || true

printf 'Starting Jetson hardware stack on %s...\n' "${JETSON_HOST}" >&2
start_jetson_hardware
printf 'Resetting STM over ST-LINK...\n' >&2
reset_stm_via_stlink
printf 'Jetson hardware stack started and STM reset completed.\n' >&2

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

tmux new-session -d -x "${SESSION_WIDTH}" -y "${SESSION_HEIGHT}" -s "${SESSION_NAME}" -n localization \
  "$(container_cmd "source /opt/ros/foxy/setup.bash; export LIBGL_ALWAYS_SOFTWARE=1; rviz2 -d /workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz")"

rviz_pane="$(tmux display-message -p -t "${SESSION_NAME}:localization.0" '#{pane_id}')"
loc_pane="$(tmux split-window -h -p 40 -P -F '#{pane_id}' -t "${rviz_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; ros2 launch nav2_bringup localization_launch.py use_sim_time:=false map:=${MAP_PATH_CONTAINER} params_file:=/workspaces/AMR-development/ros_ws/src/amr_description/config/nav2_params_amr.yaml")")"
teleop_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${loc_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; python3 /workspaces/AMR-development/scripts/amr_teleop_keyboard.py --speed 0.1 --turn 0.15 --topic /diff_drive_controller/cmd_vel_unstamped")")"

tmux select-pane -t "${teleop_pane}"

exec tmux attach -t "${SESSION_NAME}"
