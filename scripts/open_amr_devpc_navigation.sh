#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${AMR_DEVPC_CONTAINER:-amr_devpc}"
SESSION_NAME="${AMR_DEVPC_SESSION:-amr_devpc_nav}"
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
JETSON_READY_TIMEOUT_SEC="${AMR_JETSON_READY_TIMEOUT_SEC:-45}"
STM_POST_RESET_TIMEOUT_SEC="${AMR_STM_POST_RESET_TIMEOUT_SEC:-30}"
JETSON_CONTROLLERS_TIMEOUT_SEC="${AMR_JETSON_CONTROLLERS_TIMEOUT_SEC:-30}"

usage() {
  cat >&2 <<'EOF'
Usage:
  ./scripts/open_amr_devpc_navigation.sh <map_name_or_yaml>

Examples:
  ./scripts/open_amr_devpc_navigation.sh my_new_map
  ./scripts/open_amr_devpc_navigation.sh my_new_map.yaml
  ./scripts/open_amr_devpc_navigation.sh /workspaces/AMR-development/ros_ws/maps/my_new_map.yaml

This starts:
  - Jetson hardware stack (amr_foxy)
  - Jetson ST-LINK reset of the STM after startup
  - RViz
  - Nav2 localization + navigation on the given saved map
  - mission_server
  - live mission status pane
  - a mission command shell for mission_cli actions
  - keyboard teleop for fallback checks
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
plugdev_gid="$(getent group plugdev | cut -d: -f3 || true)"
docker_group_args=()
if [[ -n "${plugdev_gid}" ]]; then
  docker_group_args+=(--group-add "${plugdev_gid}")
fi

docker rm -f "${container_name}" >/dev/null 2>&1 || true

docker run -d --name "${container_name}" --net=host --privileged --runtime nvidia \
  "${docker_group_args[@]}" \
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
  ssh "${JETSON_HOST}" bash -s -- \
    "${JETSON_CONTAINER}" \
    "${AGENT_DEV}" \
    "${STM_RESET_DELAY_SEC}" \
    "${JETSON_READY_TIMEOUT_SEC}" \
    "${STM_POST_RESET_TIMEOUT_SEC}" \
    "${JETSON_CONTROLLERS_TIMEOUT_SEC}" <<'EOF'
set -euo pipefail

container_name="$1"
agent_dev="$2"
delay_sec="$3"
ready_timeout="$4"
post_reset_timeout="$5"
controllers_timeout="$6"

wait_for_agent() {
  local deadline=$((SECONDS + ready_timeout))
  while (( SECONDS < deadline )); do
    if docker exec "${container_name}" bash -lc "pgrep -af 'micro_ros_agent.*${agent_dev}' >/dev/null" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_controllers() {
  local deadline=$((SECONDS + controllers_timeout))
  while (( SECONDS < deadline )); do
    if docker exec "${container_name}" /entrypoint.sh bash -lc \
      "ros2 control list_controllers 2>/dev/null | grep -q '^joint_state_broadcaster\\[.* active' && \
       ros2 control list_controllers 2>/dev/null | grep -q '^diff_drive_controller\\[.* active'" \
      >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_wheel_state() {
  local deadline=$((SECONDS + post_reset_timeout))
  while (( SECONDS < deadline )); do
    if docker exec "${container_name}" /entrypoint.sh bash -lc \
      "ros2 topic info -v /amr/wheel_state 2>/dev/null | grep -q 'Publisher count: 1\\|Publisher count: 2\\|Publisher count: 3\\|Publisher count: 4\\|Publisher count: 5\\|Publisher count: 6\\|Publisher count: 7\\|Publisher count: 8\\|Publisher count: 9'" \
      >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

do_reset() {
  sudo -n openocd -s /usr/share/openocd/scripts \
    -f interface/stlink-v2-1.cfg \
    -f target/stm32f4x.cfg \
    -c "init; reset run; shutdown" >/dev/null
}

if ! wait_for_agent; then
  echo "Timed out waiting for micro_ros_agent in ${container_name}" >&2
  exit 1
fi

if ! wait_for_controllers; then
  echo "Timed out waiting for ROS2 controllers to become active in ${container_name}" >&2
  echo "Check on Jetson:" >&2
  echo "  docker exec -it ${container_name} /entrypoint.sh bash -lc \"ros2 control list_controllers\"" >&2
  exit 1
fi

sleep "${delay_sec}"
do_reset

if wait_for_wheel_state; then
  exit 0
fi

echo "First STM reset did not produce /amr/wheel_state, retrying once..." >&2
sleep 2
do_reset

if ! wait_for_wheel_state; then
  echo "STM reset completed but /amr/wheel_state still has no publisher." >&2
  echo "Check on Jetson:" >&2
  echo "  docker exec -it ${container_name} /entrypoint.sh bash -lc \"ros2 topic info -v /amr/wheel_state\"" >&2
  echo "  docker exec -it ${container_name} /entrypoint.sh bash -lc \"ros2 control list_controllers\"" >&2
  exit 1
fi
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

tmux new-session -d -x "${SESSION_WIDTH}" -y "${SESSION_HEIGHT}" -s "${SESSION_NAME}" -n navigation \
  "$(container_cmd "source /opt/ros/foxy/setup.bash; export LIBGL_ALWAYS_SOFTWARE=1; rviz2 -d /workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz")"

rviz_pane="$(tmux display-message -p -t "${SESSION_NAME}:navigation.0" '#{pane_id}')"
nav_pane="$(tmux split-window -h -p 40 -P -F '#{pane_id}' -t "${rviz_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; ros2 launch /workspaces/AMR-development/ros_ws/src/amr_description/launch/bringup_nav2.launch.py use_sim_time:=false use_rviz:=false map:=${MAP_PATH_CONTAINER}")")"
teleop_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${nav_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; python3 /workspaces/AMR-development/scripts/amr_teleop_keyboard.py --speed 0.1 --turn 0.15 --topic /diff_drive_controller/cmd_vel_unstamped")")"
mission_server_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${rviz_pane}" "$(container_cmd "cd /workspaces/AMR-development/ros_ws; source /opt/ros/foxy/setup.bash; COLCON_LOG_PATH=/tmp/amr_missions_colcon_logs colcon build --merge-install --packages-select amr_missions_msgs amr_missions; source install/setup.bash; ros2 run amr_missions mission_server")")"
mission_status_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${mission_server_pane}" "$(container_cmd "cd /workspaces/AMR-development/ros_ws; source /opt/ros/foxy/setup.bash; while [ ! -f install/setup.bash ]; do sleep 1; done; source install/setup.bash; ros2 topic echo /amr_missions/status")")"
mission_shell_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${mission_status_pane}" "$(container_cmd "cd /workspaces/AMR-development/ros_ws; source /opt/ros/foxy/setup.bash; while [ ! -f install/setup.bash ]; do sleep 1; done; source install/setup.bash; echo Mission shell ready.; echo Examples:; echo ros2 run amr_missions mission_cli status; echo ros2 run amr_missions mission_cli go_to kitchen; echo ros2 run amr_missions mission_cli patrol home hall door --return-home home; echo ros2 run amr_missions mission_cli cancel; exec bash -i")")"

tmux select-layout -t "${SESSION_NAME}:navigation" tiled
tmux select-pane -t "${mission_shell_pane}"

exec tmux attach -t "${SESSION_NAME}"
