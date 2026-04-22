#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-}"
CONTAINER_NAME="${AMR_CONTAINER_NAME:-amr_foxy}"
IMAGE_NAME="${AMR_IMAGE_NAME:-amr/ros2-foxy-jetson:arm64}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_ONLY_VALUE="${ROS_LOCALHOST_ONLY:-0}"
REMOTE_ROS_WS="${AMR_REMOTE_ROS_WS:-$HOME/AMR-development/ros_ws}"
AGENT_DEV="${AMR_AGENT_DEV:-/dev/ttyACM0}"
AGENT_BAUD="${AMR_AGENT_BAUD:-460800}"

ensure_container() {
  local running
  running="$(docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}')"
  if [[ "${running}" != "${CONTAINER_NAME}" ]]; then
    if docker ps -a --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
      docker rm -f "${CONTAINER_NAME}" >/dev/null
    fi
    docker run -d --name "${CONTAINER_NAME}" --net=host --privileged --runtime nvidia \
      -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
      -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY_VALUE}" \
      -v "${REMOTE_ROS_WS}:/workspaces/ros_ws" \
      "${IMAGE_NAME}" \
      bash -lc "touch /tmp/micro_ros_agent.log; ros2 run micro_ros_agent micro_ros_agent serial --dev ${AGENT_DEV} -b ${AGENT_BAUD} >>/tmp/micro_ros_agent.log 2>&1 & while sleep 3600; do :; done" >/dev/null
  fi

  docker exec "${CONTAINER_NAME}" bash -lc \
    "touch /tmp/micro_ros_agent.log; mkdir -p /tmp/amr_monitor; while ! mkdir /tmp/amr_monitor/agent_lock 2>/dev/null; do sleep 0.1; done; trap 'rmdir /tmp/amr_monitor/agent_lock' EXIT; ps -ef | grep -F 'micro_ros_agent serial --dev ${AGENT_DEV} -b ${AGENT_BAUD}' | grep -v grep >/dev/null || (ros2 run micro_ros_agent micro_ros_agent serial --dev ${AGENT_DEV} -b ${AGENT_BAUD} >>/tmp/micro_ros_agent.log 2>&1 &) ; sleep 1" >/dev/null
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

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        if mode == 'state':
            self.create_subscription(UInt32, '/amr/safety_state', self.safety_cb, qos)
            self.create_subscription(Int32, '/amr/fault_mask', self.fault_cb, qos)
        else:
            side = mode
            self.side_index = 0 if side == 'left' else 1
            self.create_subscription(JointState, '/amr/wheel_state', self.wheel_cb, qos)
            self.create_subscription(Float32, f'/amr/duty_cmd_{side}', self.duty_cb, qos)
            self.create_subscription(Int32, f'/amr/current_{side}_ma', self.current_ma_cb, qos)
            self.create_subscription(UInt32, f'/amr/current_{side}_adc', self.current_adc_cb, qos)
            self.create_subscription(UInt32, f'/amr/current_{side}_zero', self.current_zero_cb, qos)

    def safety_cb(self, msg: UInt32) -> None:
        self.safety_state = msg.data

    def fault_cb(self, msg: Int32) -> None:
        self.fault_mask = msg.data

    def wheel_cb(self, msg: JointState) -> None:
        self.wheel_names = list(msg.name)
        self.wheel_positions = list(msg.position)
        self.wheel_velocities = list(msg.velocity)

    def duty_cb(self, msg: Float32) -> None:
        self.duty = msg.data

    def current_ma_cb(self, msg: Int32) -> None:
        self.current_ma = msg.data

    def current_adc_cb(self, msg: UInt32) -> None:
        self.current_adc = msg.data

    def current_zero_cb(self, msg: UInt32) -> None:
        self.current_zero = msg.data

    def render(self) -> None:
        sys.stdout.write('\033[2J\033[H')
        sys.stdout.flush()
        if self.mode == 'state':
            print('== safety_fault ==', flush=True)
            print('', flush=True)
            print(f'safety_state: {self.safety_state}', flush=True)
            print(f'fault_mask:   {self.fault_mask}', flush=True)
            return

        side = self.mode
        idx = self.side_index
        joint = self.wheel_names[idx] if idx < len(self.wheel_names) else f'{side}_wheel_joint'
        pos = self.wheel_positions[idx] if idx < len(self.wheel_positions) else None
        vel = self.wheel_velocities[idx] if idx < len(self.wheel_velocities) else None

        print(f'== {side}_summary ==', flush=True)
        print('', flush=True)
        print(f'joint:    {joint}', flush=True)
        print(f'position: {pos}', flush=True)
        print(f'velocity: {vel}', flush=True)
        print('', flush=True)
        print(f'duty:     {self.duty}', flush=True)
        print(f'current:  {self.current_ma} mA', flush=True)
        print(f'adc:      {self.current_adc}', flush=True)
        print(f'zero:     {self.current_zero}', flush=True)


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

run_launch_status() {
  run_in_container "while true; do printf '\033[2J\033[H'; printf '== launch_status ==\n\n'; printf 'container: ${CONTAINER_NAME} (connected)\n\n'; printf 'nodes:\n'; ros2 node list 2>/dev/null || true; printf '\nAMR topics:\n'; ros2 topic list 2>/dev/null | grep '^/amr/' || true; printf '\nagent log:\n'; tail -n 10 /tmp/micro_ros_agent.log 2>/dev/null || true; sleep 3; done"
}

ensure_container

case "${MODE}" in
  agent_log)
    run_in_container "printf '== agent_log ==\n'; printf 'Waiting for agent log output...\n\n'; tail -f /tmp/micro_ros_agent.log"
    ;;
  safety)
    run_in_container "printf '== safety ==\n'; printf 'Waiting for /amr/safety_state ...\n\n'; ros2 topic echo /amr/safety_state std_msgs/msg/UInt32 --qos-reliability best_effort"
    ;;
  fault)
    run_in_container "printf '== fault ==\n'; printf 'Waiting for /amr/fault_mask ...\n\n'; ros2 topic echo /amr/fault_mask std_msgs/msg/Int32 --qos-reliability best_effort"
    ;;
  wheel_state)
    run_in_container "printf '== wheel_state ==\n'; printf 'Waiting for /amr/wheel_state ...\n\n'; ros2 topic echo /amr/wheel_state sensor_msgs/msg/JointState --qos-reliability best_effort"
    ;;
  current_left_ma)
    run_in_container "printf '== current_left_ma ==\n'; printf 'Waiting for /amr/current_left_ma ...\n\n'; ros2 topic echo /amr/current_left_ma std_msgs/msg/Int32 --qos-reliability best_effort"
    ;;
  current_right_ma)
    run_in_container "printf '== current_right_ma ==\n'; printf 'Waiting for /amr/current_right_ma ...\n\n'; ros2 topic echo /amr/current_right_ma std_msgs/msg/Int32 --qos-reliability best_effort"
    ;;
  current_left_adc)
    run_in_container "printf '== current_left_adc ==\n'; printf 'Waiting for /amr/current_left_adc ...\n\n'; ros2 topic echo /amr/current_left_adc std_msgs/msg/UInt32 --qos-reliability best_effort"
    ;;
  current_right_adc)
    run_in_container "printf '== current_right_adc ==\n'; printf 'Waiting for /amr/current_right_adc ...\n\n'; ros2 topic echo /amr/current_right_adc std_msgs/msg/UInt32 --qos-reliability best_effort"
    ;;
  current_left_zero)
    run_in_container "printf '== current_left_zero ==\n'; printf 'Waiting for /amr/current_left_zero ...\n\n'; ros2 topic echo /amr/current_left_zero std_msgs/msg/UInt32 --qos-reliability best_effort"
    ;;
  current_right_zero)
    run_in_container "printf '== current_right_zero ==\n'; printf 'Waiting for /amr/current_right_zero ...\n\n'; ros2 topic echo /amr/current_right_zero std_msgs/msg/UInt32 --qos-reliability best_effort"
    ;;
  duty_left)
    run_in_container "printf '== duty_left ==\n'; printf 'Waiting for /amr/duty_cmd_left ...\n\n'; ros2 topic echo /amr/duty_cmd_left std_msgs/msg/Float32 --qos-reliability best_effort"
    ;;
  duty_right)
    run_in_container "printf '== duty_right ==\n'; printf 'Waiting for /amr/duty_cmd_right ...\n\n'; ros2 topic echo /amr/duty_cmd_right std_msgs/msg/Float32 --qos-reliability best_effort"
    ;;
  drive_shell)
    run_in_container_tty "printf 'safe reset:\n'; printf 'ros2 topic pub --once /amr/enable std_msgs/msg/Bool \"{data: false}\"\n'; printf 'ros2 topic pub --once /amr/clear_fault std_msgs/msg/Empty \"{}\"\n'; printf '\n'; bash"
    ;;
  launch_status)
    run_launch_status
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
