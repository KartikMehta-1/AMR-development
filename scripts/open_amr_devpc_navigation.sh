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
PARAMS_PATH_CONTAINER="/workspaces/AMR-development/ros_ws/src/amr_description/config/nav2_params_amr.yaml"
RVIZ_CONFIG_PATH="${AMR_RVIZ_CONFIG_PATH:-/workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz}"
START_LIDAR="${AMR_START_LIDAR:-true}"
START_CAMERA="${AMR_START_CAMERA:-false}"
START_LINK_WATCHDOG="${AMR_START_LINK_WATCHDOG:-true}"
STM_RESET_DELAY_SEC="${AMR_STM_RESET_DELAY_SEC:-3}"
JETSON_READY_TIMEOUT_SEC="${AMR_JETSON_READY_TIMEOUT_SEC:-45}"
STM_POST_RESET_TIMEOUT_SEC="${AMR_STM_POST_RESET_TIMEOUT_SEC:-30}"
JETSON_CONTROLLERS_TIMEOUT_SEC="${AMR_JETSON_CONTROLLERS_TIMEOUT_SEC:-45}"
STM_AUTO_RESET="${AMR_STM_AUTO_RESET:-true}"
SAFETY_ENFORCE="${AMR_SAFETY_ENFORCE:-false}"
SAFETY_REQUIRE_AMCL="${AMR_SAFETY_REQUIRE_AMCL:-false}"
VOICE_MODE="${AMR_VOICE_MODE:-off}"
GRAPH_MONITOR_PERIOD="${AMR_GRAPH_MONITOR_PERIOD:-3.0}"
MAP_READY_TIMEOUT_SEC="${AMR_MAP_READY_TIMEOUT_SEC:-12}"
MAP_FALLBACK_PUBLISHER="${AMR_MAP_FALLBACK_PUBLISHER:-true}"
START_RVIZ="${AMR_START_RVIZ:-true}"
ATTACH_TMUX="${AMR_ATTACH_TMUX:-true}"

usage() {
  cat >&2 <<'EOF'
Usage:
  ./scripts/open_amr_devpc_navigation.sh <map_name_or_yaml>

Examples:
  ./scripts/open_amr_devpc_navigation.sh my_new_map
  ./scripts/open_amr_devpc_navigation.sh my_new_map.yaml
  ./scripts/open_amr_devpc_navigation.sh /workspaces/AMR-development/ros_ws/maps/my_new_map.yaml

Optional environment:
  AMR_RVIZ_CONFIG_PATH=/workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz
  AMR_VOICE_MODE=off
  AMR_GRAPH_MONITOR_PERIOD=3.0
  AMR_MAP_FALLBACK_PUBLISHER=true|false
  AMR_START_RVIZ=true|false
  AMR_ATTACH_TMUX=true|false

This starts:
  - Jetson hardware stack (amr_foxy)
  - Jetson ST-LINK reset of the STM after startup
  - RViz when AMR_START_RVIZ=true
  - Nav2 localization + navigation on the given saved map
  - mission_server
  - live mission status pane
  - safety_supervisor and live safety status pane
  - a mission command shell for mission_cli actions
  - keyboard teleop for fallback checks
  - no legacy voice command panes; voice work is MCP-based
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

case "${VOICE_MODE}" in
  off) ;;
  *)
    echo "Invalid AMR_VOICE_MODE='${VOICE_MODE}'. Legacy voice panes were removed; use AMR_VOICE_MODE=off." >&2
    exit 1
    ;;
esac

check_jetson_workspace_namespace() {
  ssh "${JETSON_HOST}" bash -s -- "${REMOTE_REPO}" <<'EOF'
set -euo pipefail

remote_repo="$1"
if [[ ! -d "${remote_repo}/ros_ws/src" ]]; then
  echo "Jetson repo is missing: ${remote_repo}/ros_ws/src" >&2
  exit 1
fi

if ! grep -R "/amr_stm/wheel_state" \
    "${remote_repo}/ros_ws/src/amr_hardware" \
    "${remote_repo}/ros_ws/src/amr_description" \
    >/dev/null 2>&1; then
  echo "Jetson workspace is stale: ${remote_repo}/ros_ws/src still does not use /amr_stm/wheel_state." >&2
  echo "Sync the current branch to the Jetson before launching navigation." >&2
  exit 1
fi
EOF
}

start_jetson_hardware() {
  ssh "${JETSON_HOST}" bash -s -- \
    "${JETSON_CONTAINER}" \
    "${REMOTE_REPO}" \
    "${ROS_DOMAIN_ID:-0}" \
    "${ROS_LOCALHOST_ONLY:-0}" \
    "${AGENT_DEV}" \
    "${AGENT_BAUD}" \
    "${START_LIDAR}" \
    "${START_CAMERA}" \
    "${START_LINK_WATCHDOG}" <<'EOF'
set -euo pipefail

container_name="$1"
remote_repo="$2"
ros_domain_id="$3"
ros_localhost_only="$4"
agent_dev="$5"
agent_baud="$6"
start_lidar="$7"
start_camera="$8"
start_link_watchdog="$9"
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
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "${remote_repo}/ros_ws:/workspaces/ros_ws" \
  amr/ros2-foxy-jetson:arm64 \
  bash -lc "
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /workspaces/ros_ws
[ -f /opt/ros/driver_ws/install/setup.bash ] && source /opt/ros/driver_ws/install/setup.bash
colcon build --merge-install --symlink-install --packages-select amr_hardware amr_description
source install/setup.bash
ros2 launch amr_description hardware.launch.py \
  use_sim_time:=false \
  agent_dev:=${agent_dev} \
  agent_baud:=${agent_baud} \
  start_lidar:=${start_lidar} \
  start_camera:=${start_camera} \
  start_link_watchdog:=${start_link_watchdog}
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
    "${JETSON_CONTROLLERS_TIMEOUT_SEC}" \
    "${STM_AUTO_RESET}" <<'EOF'
set -euo pipefail

container_name="$1"
agent_dev="$2"
delay_sec="$3"
ready_timeout="$4"
post_reset_timeout="$5"
controllers_timeout="$6"
auto_reset="$7"

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
    if docker exec -i "${container_name}" /entrypoint.sh python3 - >/dev/null 2>&1 <<'PY'
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("amr_wheel_state_probe")
qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT
seen = {"ok": False}

def on_wheel_state(_msg):
    seen["ok"] = True

node.create_subscription(JointState, "/amr_stm/wheel_state", on_wheel_state, qos)
deadline = time.monotonic() + 4.0
while rclpy.ok() and not seen["ok"] and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if seen["ok"] else 1)
PY
    then
      return 0
    fi
    sleep 1
  done
  return 1
}

restart_agent() {
  docker exec "${container_name}" bash -lc \
    "pkill -f '[m]icro_ros_agent.*serial --dev ${agent_dev}' || true" \
    >/dev/null 2>&1 || true
  sleep 3
  wait_for_agent
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

if wait_for_wheel_state; then
  exit 0
fi

echo "/amr_stm/wheel_state did not appear after startup; resetting STM..." >&2
if [[ "${auto_reset}" != "true" ]]; then
  echo "STM auto-reset is disabled. Set AMR_STM_AUTO_RESET=true to allow ST-LINK reset." >&2
  echo "Check/power-cycle the STM manually, then verify:" >&2
  echo "  docker exec -it ${container_name} /entrypoint.sh bash -lc \"ros2 topic info -v /amr_stm/wheel_state\"" >&2
  exit 1
fi
sleep "${delay_sec}"
do_reset

if wait_for_wheel_state; then
  exit 0
fi

echo "First STM reset did not produce /amr_stm/wheel_state, restarting micro-ROS agent and retrying..." >&2
if ! restart_agent; then
  echo "micro_ros_agent did not restart cleanly in ${container_name}" >&2
  exit 1
fi
do_reset

if wait_for_wheel_state; then
  exit 0
fi

echo "Agent restart did not produce /amr_stm/wheel_state, retrying STM reset once..." >&2
sleep 2
do_reset

if ! wait_for_wheel_state; then
  echo "STM reset completed but /amr_stm/wheel_state still has no messages." >&2
  echo "Check on Jetson:" >&2
  echo "  docker exec -it ${container_name} /entrypoint.sh bash -lc \"ros2 topic info -v /amr_stm/wheel_state\"" >&2
  echo "  docker exec -it ${container_name} /entrypoint.sh bash -lc \"ros2 control list_controllers\"" >&2
  exit 1
fi
EOF
}

container_cmd() {
  local command_text="$1"
  printf 'docker exec -e TERM=xterm -e DISPLAY=%q -e QT_X11_NO_MITSHM=1 -it %q /entrypoint.sh bash -lc %q' \
    "${DISPLAY_VALUE}" "${CONTAINER_NAME}" \
    "unset CYCLONEDDS_URI; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; ${command_text}"
}

xhost +local:root >/dev/null 2>&1 || true

printf 'Starting Jetson hardware stack on %s...\n' "${JETSON_HOST}" >&2
check_jetson_workspace_namespace
start_jetson_hardware
printf 'Resetting STM over ST-LINK...\n' >&2
reset_stm_via_stlink
printf 'Jetson hardware stack started and STM reset completed.\n' >&2

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
audio_gid="$(getent group audio | cut -d: -f3 || true)"
docker_audio_args=()
if [[ -d /dev/snd ]]; then
  docker_audio_args+=(--device /dev/snd)
  if [[ -n "${audio_gid}" ]]; then
    docker_audio_args+=(--group-add "${audio_gid}")
  fi
fi
docker run -d --name "${CONTAINER_NAME}" --net=host \
  "${docker_audio_args[@]}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
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

build_ready_file="/tmp/amr_nav_colcon_ready"
localization_ready_file="/tmp/amr_nav_localization_ready"
wait_for_build="while [ ! -f ${build_ready_file} ]; do sleep 1; done"
wait_for_localization="while [ ! -f ${localization_ready_file} ]; do echo 'Waiting for AMCL localization to become ready...'; sleep 2; done"
wait_for_mission_services="until ros2 service list | grep -qx /amr_missions/state && ros2 service list | grep -qx /amr_missions/go_to && ros2 service list | grep -qx /amr_missions/cancel; do sleep 1; done"

tmux new-session -d -x "${SESSION_WIDTH}" -y "${SESSION_HEIGHT}" -s "${SESSION_NAME}" -n navigation \
  "$(container_cmd "cd /workspaces/AMR-development/ros_ws; source /opt/ros/foxy/setup.bash; ${wait_for_build}; source install/setup.bash; ${wait_for_localization}; ${wait_for_mission_services}; echo Mission shell ready.; echo 'Click panes to switch focus, or use Ctrl-b then arrow keys.'; echo 'Switch windows with Ctrl-b n / Ctrl-b p, or Ctrl-b 0/1/2.'; echo Examples:; echo ros2 run amr_missions mission_cli status; echo ros2 run amr_missions mission_cli go_to kitchen; echo ros2 run amr_missions mission_cli patrol home hall door --return-home home; echo ros2 run amr_missions mission_cli cancel; exec bash -i")"
tmux set-option -t "${SESSION_NAME}" mouse on
tmux set-option -t "${SESSION_NAME}" history-limit 50000
tmux set-window-option -t "${SESSION_NAME}:navigation" pane-border-status top
tmux set-window-option -t "${SESSION_NAME}:navigation" pane-border-format ' #{pane_index}: #{pane_title} '

mission_shell_pane="$(tmux display-message -p -t "${SESSION_NAME}:navigation.0" '#{pane_id}')"
tmux select-pane -t "${mission_shell_pane}" -T "Mission Shell"
nav_runtime_cmd="source /opt/ros/foxy/setup.bash; \
rm -f ${localization_ready_file}; \
cleanup() { [ -n \"\${scan_filter_pid:-}\" ] && kill \"\${scan_filter_pid}\" 2>/dev/null || true; [ -n \"\${nav2_pid:-}\" ] && kill \"\${nav2_pid}\" 2>/dev/null || true; [ -n \"\${rviz_pid:-}\" ] && kill \"\${rviz_pid}\" 2>/dev/null || true; [ -n \"\${map_fallback_pid:-}\" ] && kill \"\${map_fallback_pid}\" 2>/dev/null || true; }; \
trap cleanup EXIT INT TERM; \
pkill -f '[a]mr_static_map_publisher.py' 2>/dev/null || true; \
python3 /workspaces/AMR-development/scripts/amr_scan_sanitizer.py & \
scan_filter_pid=\$!; \
echo 'Starting full Nav2 bringup.'; \
ros2 launch /workspaces/AMR-development/ros_ws/src/amr_description/launch/bringup_nav2.launch.py use_sim_time:=false use_rviz:=false autostart:=true map:=${MAP_PATH_CONTAINER} params_file:=${PARAMS_PATH_CONTAINER} cmd_vel_topic:=/diff_drive_controller/cmd_vel_unstamped odom_topic:=/odom & \
nav2_pid=\$!; \
echo 'Waiting for /map to be receivable by a late subscriber...'; \
if ! python3 /workspaces/AMR-development/scripts/amr_wait_for_map.py --timeout ${MAP_READY_TIMEOUT_SEC}; then \
  if [ '${MAP_FALLBACK_PUBLISHER}' = 'true' ]; then \
    echo 'map_server did not expose /map reliably; starting static map fallback publisher.'; \
    python3 /workspaces/AMR-development/scripts/amr_static_map_publisher.py ${MAP_PATH_CONTAINER} >/tmp/amr_static_map_publisher.log 2>&1 & \
    map_fallback_pid=\$!; \
    python3 /workspaces/AMR-development/scripts/amr_wait_for_map.py --timeout ${MAP_READY_TIMEOUT_SEC}; \
  else \
    echo 'Map was not receivable and AMR_MAP_FALLBACK_PUBLISHER=false.'; \
  fi; \
fi; \
if [ '${START_RVIZ}' = 'true' ]; then \
  echo 'Starting RViz in the background. RViz logs: /tmp/amr_rviz.log'; \
  export LIBGL_ALWAYS_SOFTWARE=1; \
  rviz2 -d ${RVIZ_CONFIG_PATH} >/tmp/amr_rviz.log 2>&1 & \
  rviz_pid=\$!; \
  echo 'Set the initial pose in RViz with 2D Pose Estimate.'; \
else \
  echo 'RViz disabled by AMR_START_RVIZ=false. Set /initialpose from an external RViz or helper before missions.'; \
fi; \
echo 'Waiting for fresh AMCL pose and map->odom before enabling mission commands...'; \
if ! python3 /workspaces/AMR-development/scripts/amr_wait_for_localization.py --timeout 180.0; then \
  echo 'Localization did not become ready. Leave Nav2 running and set RViz 2D Pose Estimate again.'; \
  wait \${nav2_pid}; \
  exit 1; \
fi; \
touch ${localization_ready_file}; \
echo 'AMCL localization is fresh; mission commands are enabled.'; \
wait \${nav2_pid}"
nav_pane="$(tmux split-window -h -p 45 -P -F '#{pane_id}' -t "${mission_shell_pane}" "$(container_cmd "${nav_runtime_cmd}")")"
tmux select-pane -t "${nav_pane}" -T "Nav2 + AMCL"
teleop_pane="$(tmux split-window -v -p 35 -P -F '#{pane_id}' -t "${nav_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; python3 /workspaces/AMR-development/scripts/amr_teleop_keyboard.py --speed 0.1 --turn 0.15 --topic /diff_drive_controller/cmd_vel_unstamped")")"
tmux select-pane -t "${teleop_pane}" -T "Teleop"
select_build_packages="build_packages='amr_missions_msgs amr_clients amr_missions'; [ -f src/amr_safety/package.xml ] && build_packages=\"\${build_packages} amr_safety\"; [ -f src/amr_voice/package.xml ] && build_packages=\"\${build_packages} amr_voice\""
tmux select-layout -t "${SESSION_NAME}:navigation" tiled

topics_pane="$(tmux new-window -d -P -F '#{pane_id}' -t "${SESSION_NAME}" -n monitor "$(container_cmd "source /opt/ros/foxy/setup.bash; python3 /workspaces/AMR-development/scripts/amr_graph_monitor.py --mode topics --period ${GRAPH_MONITOR_PERIOD}")")"
tmux set-window-option -t "${SESSION_NAME}:monitor" pane-border-status top
tmux set-window-option -t "${SESSION_NAME}:monitor" pane-border-format ' #{pane_index}: #{pane_title} '
tmux select-pane -t "${topics_pane}" -T "Topics"
nodes_pane="$(tmux split-window -h -p 50 -P -F '#{pane_id}' -t "${topics_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; python3 /workspaces/AMR-development/scripts/amr_graph_monitor.py --mode nodes --period ${GRAPH_MONITOR_PERIOD}")")"
tmux select-pane -t "${nodes_pane}" -T "Nodes"
mission_server_pane="$(tmux split-window -v -p 66 -P -F '#{pane_id}' -t "${topics_pane}" "$(container_cmd "cd /workspaces/AMR-development/ros_ws; rm -f ${build_ready_file}; source /opt/ros/foxy/setup.bash; ${select_build_packages}; COLCON_LOG_PATH=/tmp/amr_missions_colcon_logs colcon build --merge-install --packages-select \${build_packages}; touch ${build_ready_file}; source install/setup.bash; echo 'Mission server waiting for AMCL localization readiness...'; ${wait_for_localization}; echo 'Starting mission_server after localization readiness.'; ros2 run amr_missions mission_server")")"
tmux select-pane -t "${mission_server_pane}" -T "Mission Server"
mission_status_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${mission_server_pane}" "$(container_cmd "cd /workspaces/AMR-development/ros_ws; source /opt/ros/foxy/setup.bash; ${wait_for_build}; source install/setup.bash; python3 /workspaces/AMR-development/scripts/amr_mission_status_monitor.py")")"
tmux select-pane -t "${mission_status_pane}" -T "Mission Status"
safety_args="-p odom_topic:=/odom -p enforce:=${SAFETY_ENFORCE} -p require_amcl:=${SAFETY_REQUIRE_AMCL} -p auto_reenable_when_safe:=false"
safety_pane="$(tmux split-window -v -p 66 -P -F '#{pane_id}' -t "${nodes_pane}" "$(container_cmd "cd /workspaces/AMR-development/ros_ws; source /opt/ros/foxy/setup.bash; ${wait_for_build}; source install/setup.bash; if [ -f src/amr_safety/package.xml ]; then echo 'Starting safety_supervisor with enforce=${SAFETY_ENFORCE}, require_amcl=${SAFETY_REQUIRE_AMCL}'; ros2 run amr_safety safety_supervisor --ros-args ${safety_args}; else echo 'amr_safety package not present; safety pane disabled.'; exec bash -i; fi")")"
tmux select-pane -t "${safety_pane}" -T "Safety"
safety_status_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${safety_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; python3 /workspaces/AMR-development/scripts/amr_safety_status_monitor.py")")"
tmux select-pane -t "${safety_status_pane}" -T "Safety Status"

tmux select-window -t "${SESSION_NAME}:navigation"
tmux select-pane -t "${mission_shell_pane}"

if [[ "${ATTACH_TMUX}" == "true" ]]; then
  exec tmux attach -t "${SESSION_NAME}"
fi

printf 'AMR navigation launch created tmux session: %s\n' "${SESSION_NAME}" >&2
printf 'Attach with: tmux attach -t %s\n' "${SESSION_NAME}" >&2
