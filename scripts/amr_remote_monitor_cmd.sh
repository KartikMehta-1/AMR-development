#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-}"
CONTAINER_NAME="${AMR_CONTAINER_NAME:-amr_foxy}"
IMAGE_NAME="${AMR_IMAGE_NAME:-amr/ros2-foxy-jetson:arm64}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_ONLY_VALUE="${ROS_LOCALHOST_ONLY:-0}"
REMOTE_ROS_WS="${AMR_REMOTE_ROS_WS:-$HOME/AMR-development/ros_ws}"
AGENT_BAUD="${AMR_AGENT_BAUD:-460800}"
AGENT_LOG="${AMR_AGENT_LOG:-/tmp/amr_monitor_agent.log}"

default_agent_dev() {
  printf '%s\n' "/dev/ttyACM0"
}

AGENT_DEV="${AMR_AGENT_DEV:-$(default_agent_dev)}"
STM_NS="${AMR_STM_NS:-/amr_stm}"

ensure_container() {
  local running
  running="$(docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}')"
  if [[ "${running}" != "${CONTAINER_NAME}" ]]; then
    if docker ps -a --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
      docker rm -f "${CONTAINER_NAME}" >/dev/null
    fi
    local plugdev_gid
    plugdev_gid="$(getent group plugdev | cut -d: -f3 || true)"
    local docker_group_args=()
    if [[ -n "${plugdev_gid}" ]]; then
      docker_group_args+=(--group-add "${plugdev_gid}")
    fi
    docker run -d --name "${CONTAINER_NAME}" --net=host --privileged --runtime nvidia \
      "${docker_group_args[@]}" \
      -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
      -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY_VALUE}" \
      -v "${REMOTE_ROS_WS}:/workspaces/ros_ws" \
      "${IMAGE_NAME}" \
      /entrypoint.sh bash -lc ": > ${AGENT_LOG}; ros2 run micro_ros_agent micro_ros_agent serial --dev ${AGENT_DEV} -b ${AGENT_BAUD} >>${AGENT_LOG} 2>&1 & while sleep 3600; do :; done" >/dev/null
  fi

  docker exec "${CONTAINER_NAME}" /entrypoint.sh bash -lc \
    "touch ${AGENT_LOG}; mkdir -p /tmp/amr_monitor; while ! mkdir /tmp/amr_monitor/agent_lock 2>/dev/null; do sleep 0.1; done; trap 'rmdir /tmp/amr_monitor/agent_lock' EXIT; ps -ef | grep -F 'micro_ros_agent serial --dev ${AGENT_DEV} -b ${AGENT_BAUD}' | grep -v grep >/dev/null || (: > ${AGENT_LOG}; ros2 run micro_ros_agent micro_ros_agent serial --dev ${AGENT_DEV} -b ${AGENT_BAUD} >>${AGENT_LOG} 2>&1 &) ; sleep 1" >/dev/null
}

run_in_container() {
  docker exec -e TERM=xterm -i "${CONTAINER_NAME}" /entrypoint.sh bash -lc "$1"
}

run_in_container_tty() {
  docker exec -e TERM=xterm -it "${CONTAINER_NAME}" /entrypoint.sh bash -lc "$1"
}

run_python_monitor() {
  local mode="$1"
  run_in_container "cat >/tmp/amr_live_monitor.py <<'PY'
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32, UInt32


class BenchMonitor(Node):
    def __init__(self, mode: str):
        super().__init__(f'amr_bench_{mode}')
        self.mode = mode
        self.last_render = 0.0
        self.safety_state = None
        self.fault_mask = None
        self.wheel_names = []
        self.wheel_positions = []
        self.wheel_velocities = []
        self.duty = None
        self.current_ma = None
        self.current_adc = None
        self.current_zero = None
        self.safety_age = None
        self.fault_age = None
        self.wheel_age = None
        self.duty_age = None
        self.current_ma_age = None
        self.current_adc_age = None
        self.current_zero_age = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        if mode == 'state':
            self.create_subscription(UInt32, '${STM_NS}/safety_state', self.safety_cb, qos)
            self.create_subscription(Int32, '${STM_NS}/fault_mask', self.fault_cb, qos)
        else:
            side = mode
            self.side_index = 0 if side == 'left' else 1
            self.create_subscription(JointState, '${STM_NS}/wheel_state', self.wheel_cb, qos)
            self.create_subscription(Float32, f'${STM_NS}/duty_cmd_{side}', self.duty_cb, qos)
            self.create_subscription(Int32, f'${STM_NS}/current_{side}_ma', self.current_ma_cb, qos)
            self.create_subscription(UInt32, f'${STM_NS}/current_{side}_adc', self.current_adc_cb, qos)
            self.create_subscription(UInt32, f'${STM_NS}/current_{side}_zero', self.current_zero_cb, qos)

    def safety_cb(self, msg: UInt32) -> None:
        self.safety_state = msg.data
        self.safety_age = time.monotonic()

    def fault_cb(self, msg: Int32) -> None:
        self.fault_mask = msg.data
        self.fault_age = time.monotonic()

    def wheel_cb(self, msg: JointState) -> None:
        self.wheel_names = list(msg.name)
        self.wheel_positions = list(msg.position)
        self.wheel_velocities = list(msg.velocity)
        self.wheel_age = time.monotonic()

    def duty_cb(self, msg: Float32) -> None:
        self.duty = msg.data
        self.duty_age = time.monotonic()

    def current_ma_cb(self, msg: Int32) -> None:
        self.current_ma = msg.data
        self.current_ma_age = time.monotonic()

    def current_adc_cb(self, msg: UInt32) -> None:
        self.current_adc = msg.data
        self.current_adc_age = time.monotonic()

    def current_zero_cb(self, msg: UInt32) -> None:
        self.current_zero = msg.data
        self.current_zero_age = time.monotonic()

    def age_text(self, stamp) -> str:
        if stamp is None:
            return 'never'
        age = time.monotonic() - stamp
        stale = ' STALE' if age > 1.0 else ''
        return f'{age:.2f}s ago{stale}'

    def render(self) -> None:
        sys.stdout.write('\033[2J\033[H')
        sys.stdout.flush()
        if self.mode == 'state':
            print('== safety_fault ==', flush=True)
            print('', flush=True)
            print(f'safety_state: {self.safety_state} ({self.age_text(self.safety_age)})', flush=True)
            print(f'fault_mask:   {self.fault_mask} ({self.age_text(self.fault_age)})', flush=True)
            return

        side = self.mode
        idx = self.side_index
        joint = self.wheel_names[idx] if idx < len(self.wheel_names) else f'{side}_wheel_joint'
        pos = self.wheel_positions[idx] if idx < len(self.wheel_positions) else None
        vel = self.wheel_velocities[idx] if idx < len(self.wheel_velocities) else None

        print(f'== {side}_summary ==', flush=True)
        print('', flush=True)
        print(f'joint:    {joint}', flush=True)
        print(f'position: {pos} ({self.age_text(self.wheel_age)})', flush=True)
        print(f'velocity: {vel} ({self.age_text(self.wheel_age)})', flush=True)
        print('', flush=True)
        print(f'duty:     {self.duty} ({self.age_text(self.duty_age)})', flush=True)
        print(f'current:  {self.current_ma} mA ({self.age_text(self.current_ma_age)})', flush=True)
        print(f'adc:      {self.current_adc} ({self.age_text(self.current_adc_age)})', flush=True)
        print(f'zero:     {self.current_zero} ({self.age_text(self.current_zero_age)})', flush=True)


def main() -> None:
    mode = sys.argv[1]
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(line_buffering=True, write_through=True)
    rclpy.init()
    node = BenchMonitor(mode)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - node.last_render >= 0.2:
                node.render()
                node.last_render = now
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PY
python3 -u /tmp/amr_live_monitor.py ${mode}"
}

run_topics_status() {
  run_in_container "while true; do \
    out=\"\$( \
      printf '== topics ==\n\n'; \
      printf 'container: ${CONTAINER_NAME} (connected)\n\n'; \
      printf 'STM topics:\n'; \
      ros2 topic list 2>/dev/null | grep '^${STM_NS}/' || true; \
      printf '\nCore topics:\n'; \
      ros2 topic list 2>/dev/null | grep -E '^/(scan|odom|tf|tf_static|joint_states|dynamic_joint_states|robot_description|parameter_events|rosout)$|^/diff_drive_controller/' || true; \
    )\"; \
    printf '\033[2J\033[H%s' \"\$out\"; \
    sleep 3; \
  done"
}

run_nodes_status() {
  run_in_container "while true; do \
    out=\"\$( \
      printf '== nodes ==\n\n'; \
      printf 'container: ${CONTAINER_NAME} (connected)\n\n'; \
      ros2 node list 2>/dev/null | sort -u || true; \
    )\"; \
    printf '\033[2J\033[H%s' \"\$out\"; \
    sleep 3; \
  done"
}

cleanup_monitor_processes() {
  run_in_container "current_pid=\$\$; parent_pid=\$PPID; \
    ps -eo pid=,args= | while read -r pid args; do \
      case \"\$args\" in \
        *'/tmp/amr_live_monitor.py'*|*'/tmp/amr_monitor_teleop.py'*|*'/opt/ros/foxy/lib/teleop_twist_keyboard/teleop_twist_keyboard'*) \
          if [ \"\$pid\" != \"\$current_pid\" ] && [ \"\$pid\" != \"\$parent_pid\" ]; then \
            kill \"\$pid\" 2>/dev/null || true; \
          fi; \
          ;; \
      esac; \
    done; \
    sleep 0.3"
}

ensure_container

case "${MODE}" in
  cleanup_monitor)
    cleanup_monitor_processes
    ;;
  agent_log)
    run_in_container "printf '== agent_log ==\n'; printf 'Waiting for agent log output...\n\n'; tail -f ${AGENT_LOG}"
    ;;
  safety)
    run_in_container "printf '== safety ==\n'; printf 'Waiting for ${STM_NS}/safety_state ...\n\n'; ros2 topic echo ${STM_NS}/safety_state std_msgs/msg/UInt32 --qos-reliability best_effort"
    ;;
  fault)
    run_in_container "printf '== fault ==\n'; printf 'Waiting for ${STM_NS}/fault_mask ...\n\n'; ros2 topic echo ${STM_NS}/fault_mask std_msgs/msg/Int32 --qos-reliability best_effort"
    ;;
  wheel_state)
    run_in_container "printf '== wheel_state ==\n'; printf 'Waiting for ${STM_NS}/wheel_state ...\n\n'; ros2 topic echo ${STM_NS}/wheel_state sensor_msgs/msg/JointState --qos-reliability best_effort"
    ;;
  current_left_ma)
    run_in_container "printf '== current_left_ma ==\n'; printf 'Waiting for ${STM_NS}/current_left_ma ...\n\n'; ros2 topic echo ${STM_NS}/current_left_ma std_msgs/msg/Int32 --qos-reliability best_effort"
    ;;
  current_right_ma)
    run_in_container "printf '== current_right_ma ==\n'; printf 'Waiting for ${STM_NS}/current_right_ma ...\n\n'; ros2 topic echo ${STM_NS}/current_right_ma std_msgs/msg/Int32 --qos-reliability best_effort"
    ;;
  current_left_adc)
    run_in_container "printf '== current_left_adc ==\n'; printf 'Waiting for ${STM_NS}/current_left_adc ...\n\n'; ros2 topic echo ${STM_NS}/current_left_adc std_msgs/msg/UInt32 --qos-reliability best_effort"
    ;;
  current_right_adc)
    run_in_container "printf '== current_right_adc ==\n'; printf 'Waiting for ${STM_NS}/current_right_adc ...\n\n'; ros2 topic echo ${STM_NS}/current_right_adc std_msgs/msg/UInt32 --qos-reliability best_effort"
    ;;
  current_left_zero)
    run_in_container "printf '== current_left_zero ==\n'; printf 'Waiting for ${STM_NS}/current_left_zero ...\n\n'; ros2 topic echo ${STM_NS}/current_left_zero std_msgs/msg/UInt32 --qos-reliability best_effort"
    ;;
  current_right_zero)
    run_in_container "printf '== current_right_zero ==\n'; printf 'Waiting for ${STM_NS}/current_right_zero ...\n\n'; ros2 topic echo ${STM_NS}/current_right_zero std_msgs/msg/UInt32 --qos-reliability best_effort"
    ;;
  duty_left)
    run_in_container "printf '== duty_left ==\n'; printf 'Waiting for ${STM_NS}/duty_cmd_left ...\n\n'; ros2 topic echo ${STM_NS}/duty_cmd_left std_msgs/msg/Float32 --qos-reliability best_effort"
    ;;
  duty_right)
    run_in_container "printf '== duty_right ==\n'; printf 'Waiting for ${STM_NS}/duty_cmd_right ...\n\n'; ros2 topic echo ${STM_NS}/duty_cmd_right std_msgs/msg/Float32 --qos-reliability best_effort"
    ;;
  drive_shell)
    run_in_container_tty "printf 'safe reset:\n'; printf 'ros2 topic pub --once ${STM_NS}/enable std_msgs/msg/Bool \"{data: false}\"\n'; printf 'ros2 topic pub --once ${STM_NS}/clear_fault std_msgs/msg/Empty \"{}\"\n'; printf '\n'; bash"
    ;;
  launch_status|topics_status)
    run_topics_status
    ;;
  nodes_status)
    run_nodes_status
    ;;
  state_summary)
    run_python_monitor state
    ;;
  left_summary)
    run_python_monitor left
    ;;
  right_summary)
    run_python_monitor right
    ;;
  *)
    echo "Unknown mode: ${MODE}" >&2
    exit 2
    ;;
esac
