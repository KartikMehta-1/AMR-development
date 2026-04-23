#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="${AMR_TMUX_SESSION:-amr_bench}"
CONTAINER_NAME="${AMR_CONTAINER_NAME:-amr_foxy}"
SESSION_WIDTH="${AMR_TMUX_WIDTH:-240}"
SESSION_HEIGHT="${AMR_TMUX_HEIGHT:-70}"
RECREATE_SESSION="${AMR_RECREATE_SESSION:-1}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_cmd docker
require_cmd tmux

fail_with_usage() {
  cat >&2 <<EOF
${1}

This script only attaches monitoring panes to an already-running container.
It does not create, restart, or replace Docker containers.

Expected running container:
  ${CONTAINER_NAME}

Typical flow:
  1. Start the hardware stack first:
     docker run --rm -it --net=host --privileged --runtime nvidia \\
       --name ${CONTAINER_NAME} \\
       amr/ros2-foxy-jetson:arm64 \\
       ros2 launch amr_description hardware.launch.py ...

  2. In another terminal on the same machine, run:
     ./scripts/amr_existing_container_tmux.sh
EOF
  exit 1
}

ensure_running_container() {
  local running
  running="$(docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}')"
  if [[ "${running}" != "${CONTAINER_NAME}" ]]; then
    fail_with_usage "Container '${CONTAINER_NAME}' is not currently running."
  fi
}

pane_cmd() {
  local command_text="$1"
  printf 'docker exec -e TERM=xterm -i %q /entrypoint.sh bash -lc %q' "${CONTAINER_NAME}" "${command_text}"
}

summary_cmd() {
  local mode="$1"
  local python_mode="$2"
  pane_cmd "cat >/tmp/amr_live_monitor.py <<'PY'
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
python3 -u /tmp/amr_live_monitor.py ${python_mode}"
}

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  if [[ "${RECREATE_SESSION}" == "1" ]]; then
    tmux kill-session -t "${SESSION_NAME}"
  else
    exec tmux attach -t "${SESSION_NAME}"
  fi
fi

ensure_running_container

tmux new-session -d -x "${SESSION_WIDTH}" -y "${SESSION_HEIGHT}" -s "${SESSION_NAME}" -n bench \
  "$(pane_cmd "while true; do printf '\033[2J\033[H'; printf '== launch_status ==\n\n'; printf 'container: ${CONTAINER_NAME} (existing)\n\n'; printf 'nodes:\n'; ros2 node list 2>/dev/null || true; printf '\nAMR topics:\n'; ros2 topic list 2>/dev/null | grep '^/amr/' || true; sleep 3; done")"

main_pane="$(tmux display-message -p -t "${SESSION_NAME}:bench.0" '#{pane_id}')"
middle_pane="$(tmux split-window -h -p 67 -P -F '#{pane_id}' -t "${main_pane}" "$(summary_cmd left_summary left)")"
right_pane="$(tmux split-window -h -p 50 -P -F '#{pane_id}' -t "${middle_pane}" "$(summary_cmd state_summary state)")"

tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${middle_pane}" "$(summary_cmd right_summary right)" >/dev/null
command_pane="$(tmux split-window -v -p 50 -P -F '#{pane_id}' -t "${right_pane}" "$(pane_cmd "printf '%s\n%s\n%s\n%s\n' \
  'safe reset:' \
  'ros2 topic pub --once /amr/enable std_msgs/msg/Bool \"{data: false}\"' \
  'ros2 topic pub --once /amr/clear_fault std_msgs/msg/Empty \"{}\"' \
  'teleop monitor shell ready' ; bash")")"
tmux select-layout -t "${SESSION_NAME}:bench" even-horizontal
tmux select-window -t "${SESSION_NAME}:bench"
tmux select-pane -t "${command_pane}"

exec tmux attach -t "${SESSION_NAME}"
