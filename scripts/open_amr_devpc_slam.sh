#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${AMR_DEVPC_CONTAINER:-amr_devpc}"
SESSION_NAME="${AMR_DEVPC_SESSION:-amr_devpc_slam}"
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
START_LINK_WATCHDOG="${AMR_START_LINK_WATCHDOG:-true}"
STM_RESET_DELAY_SEC="${AMR_STM_RESET_DELAY_SEC:-3}"
JETSON_READY_TIMEOUT_SEC="${AMR_JETSON_READY_TIMEOUT_SEC:-45}"
STM_POST_RESET_TIMEOUT_SEC="${AMR_STM_POST_RESET_TIMEOUT_SEC:-30}"
JETSON_CONTROLLERS_TIMEOUT_SEC="${AMR_JETSON_CONTROLLERS_TIMEOUT_SEC:-45}"
STM_AUTO_RESET="${AMR_STM_AUTO_RESET:-true}"
SLAM_PARAMS_PATH="${AMR_SLAM_PARAMS_PATH:-/workspaces/AMR-development/ros_ws/src/amr_description/config/slam_toolbox_online_async.yaml}"
RVIZ_CONFIG_PATH="${AMR_RVIZ_CONFIG_PATH:-/workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz}"
MAP_SAVE_PREFIX="${AMR_MAP_SAVE_PREFIX:-/workspaces/AMR-development/ros_ws/maps/full_house_map}"

usage() {
  cat >&2 <<'EOF'
Usage:
  ./scripts/open_amr_devpc_slam.sh

Optional environment:
  AMR_MAP_SAVE_PREFIX=/workspaces/AMR-development/ros_ws/maps/full_house_map
  AMR_SLAM_PARAMS_PATH=/workspaces/AMR-development/ros_ws/src/amr_description/config/slam_toolbox_online_async.yaml
  AMR_STM_AUTO_RESET=true|false

This starts:
  - Jetson hardware stack (amr_foxy)
  - Jetson ST-LINK reset/recovery of the STM if wheel state is missing
  - RViz
  - slam_toolbox in mapping mode
  - keyboard teleop for mapping
  - a map-save shell with the exact save command
EOF
  exit "${1:-1}"
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

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
  echo "Sync the current branch to the Jetson before launching SLAM." >&2
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

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage 0
fi

require_cmd docker
require_cmd xhost
require_cmd tmux
require_cmd ssh

xhost +local:root >/dev/null 2>&1 || true

printf 'Starting Jetson hardware stack on %s...\n' "${JETSON_HOST}" >&2
check_jetson_workspace_namespace
start_jetson_hardware
printf 'Checking STM link and resetting over ST-LINK if needed...\n' >&2
reset_stm_via_stlink
printf 'Jetson hardware stack is ready for SLAM.\n' >&2

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
docker run -d --name "${CONTAINER_NAME}" --net=host \
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

tmux new-session -d -x "${SESSION_WIDTH}" -y "${SESSION_HEIGHT}" -s "${SESSION_NAME}" -n slam \
  "$(container_cmd "source /opt/ros/foxy/setup.bash; export LIBGL_ALWAYS_SOFTWARE=1; rviz2 -d ${RVIZ_CONFIG_PATH}")"
tmux set-option -t "${SESSION_NAME}" mouse on
tmux set-option -t "${SESSION_NAME}" history-limit 50000

rviz_pane="$(tmux display-message -p -t "${SESSION_NAME}:slam.0" '#{pane_id}')"
slam_pane="$(tmux split-window -h -p 40 -P -F '#{pane_id}' -t "${rviz_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; echo 'Starting slam_toolbox mapping. Drive slowly and close loops when possible.'; ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false params_file:=${SLAM_PARAMS_PATH}")")"
teleop_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${slam_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; python3 /workspaces/AMR-development/scripts/amr_teleop_keyboard.py --speed 0.1 --turn 0.15 --topic /diff_drive_controller/cmd_vel_unstamped")")"
status_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${rviz_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; echo 'SLAM health topics:'; echo '  /map, /scan, /odom, /amr_stm/wheel_state, /amr_stm/comm_status'; ros2 topic hz /map")")"
save_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${status_pane}" "$(container_cmd "source /opt/ros/foxy/setup.bash; echo 'Map save shell ready.'; echo 'Recommended save command:'; echo 'ros2 run nav2_map_server map_saver_cli -t /map -f ${MAP_SAVE_PREFIX} --ros-args -p save_map_timeout:=10000'; echo; exec bash -i")")"

tmux select-layout -t "${SESSION_NAME}:slam" tiled
tmux select-pane -t "${teleop_pane}"

exec tmux attach -t "${SESSION_NAME}"
