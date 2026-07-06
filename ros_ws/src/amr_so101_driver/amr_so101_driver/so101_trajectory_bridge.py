import math
import threading
import time
from dataclasses import dataclass

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


TICKS_PER_REV = 4096.0
RAD_PER_TICK = 2.0 * math.pi / TICKS_PER_REV


@dataclass(frozen=True)
class JointSpec:
    name: str
    motor_id: int
    center_raw: int
    direction: int


DEFAULT_JOINTS = [
    JointSpec("so101_shoulder_pan", 1, 2048, 1),
    JointSpec("so101_shoulder_lift", 2, 2048, 1),
    JointSpec("so101_elbow_flex", 3, 2048, 1),
    JointSpec("so101_wrist_flex", 4, 2048, 1),
    JointSpec("so101_wrist_roll", 5, 2048, 1),
    JointSpec("so101_gripper", 6, 2048, 1),
]


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def raw_to_rad(raw_value, spec):
    return (float(raw_value) - float(spec.center_raw)) * RAD_PER_TICK * spec.direction


def rad_to_raw(position, spec):
    raw = spec.center_raw + (float(position) / RAD_PER_TICK) * spec.direction
    return int(min(4095, max(0, round(raw))))


class FakeSo101Bus:
    def __init__(self, joints):
        self._raw = {spec.name: spec.center_raw for spec in joints}
        self._torque = False

    def connect(self):
        return None

    def read_positions(self):
        return dict(self._raw)

    def write_positions(self, raw_positions):
        self._torque = True
        self._raw.update(raw_positions)

    def disable_torque(self):
        self._torque = False

    def close(self):
        return None


class FeetechSo101Bus:
    def __init__(self, port, baudrate, joints):
        self._port = port
        self._baudrate = int(baudrate)
        self._joints = joints
        self._port_handler = None
        self._packet_handler = None

    def connect(self):
        try:
            import scservo_sdk as scs
        except ImportError as exc:
            raise RuntimeError(
                "scservo_sdk is required for real SO-101 hardware. "
                "Install it in the Orin container or run with use_fake_hardware:=true."
            ) from exc

        self._scs = scs
        self._port_handler = scs.PortHandler(self._port)
        self._packet_handler = scs.PacketHandler(0)
        if not self._port_handler.openPort():
            raise RuntimeError(f"Failed to open SO-101 serial port {self._port}")
        if not self._port_handler.setBaudRate(self._baudrate):
            raise RuntimeError(f"Failed to set SO-101 baudrate {self._baudrate}")

        missing = []
        for spec in self._joints:
            model, comm, error = self._packet_handler.ping(self._port_handler, spec.motor_id)
            if comm != self._scs.COMM_SUCCESS or error != 0:
                missing.append(spec.motor_id)
            elif model != 777:
                raise RuntimeError(
                    f"Motor {spec.motor_id} responded with model {model}, "
                    "expected STS3215 model 777"
                )
        if missing:
            raise RuntimeError(f"Missing SO-101 motor IDs: {missing}")

    def _read2(self, motor_id, address):
        value, comm, error = self._packet_handler.read2ByteTxRx(
            self._port_handler, motor_id, address
        )
        if comm != self._scs.COMM_SUCCESS:
            raise RuntimeError(self._packet_handler.getTxRxResult(comm))
        if error != 0:
            raise RuntimeError(self._packet_handler.getRxPacketError(error))
        return int(value)

    def _write1(self, motor_id, address, value):
        comm, error = self._packet_handler.write1ByteTxRx(
            self._port_handler, motor_id, address, int(value)
        )
        if comm != self._scs.COMM_SUCCESS:
            raise RuntimeError(self._packet_handler.getTxRxResult(comm))
        if error != 0:
            raise RuntimeError(self._packet_handler.getRxPacketError(error))

    def _write2(self, motor_id, address, value):
        comm, error = self._packet_handler.write2ByteTxRx(
            self._port_handler, motor_id, address, int(value)
        )
        if comm != self._scs.COMM_SUCCESS:
            raise RuntimeError(self._packet_handler.getTxRxResult(comm))
        if error != 0:
            raise RuntimeError(self._packet_handler.getRxPacketError(error))

    def read_positions(self):
        return {spec.name: self._read2(spec.motor_id, 56) for spec in self._joints}

    def write_positions(self, raw_positions):
        for spec in self._joints:
            if spec.name in raw_positions:
                self._write2(spec.motor_id, 42, raw_positions[spec.name])

    def disable_torque(self):
        for spec in self._joints:
            self._write1(spec.motor_id, 40, 0)
            self._write1(spec.motor_id, 55, 0)

    def close(self):
        if self._port_handler is not None:
            self._port_handler.closePort()


class So101TrajectoryBridge(Node):
    def __init__(self):
        super().__init__("so101_trajectory_bridge")

        self.declare_parameter("use_fake_hardware", True)
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("joint_states_topic", "/so101/joint_states")
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("allowed_joints_csv", "so101_wrist_roll")
        self.declare_parameter("start_tolerance_rad", 0.20)
        self.declare_parameter("goal_tolerance_rad", 0.10)
        self.declare_parameter("max_step_rad", 0.12)
        self.declare_parameter("settle_timeout_sec", 0.80)
        self.declare_parameter("settle_poll_sec", 0.05)
        self.declare_parameter("disable_torque_on_shutdown", True)

        self._joints = list(DEFAULT_JOINTS)
        self._joint_by_name = {spec.name: spec for spec in self._joints}
        allowed_csv = self.get_parameter("allowed_joints_csv").value
        self._allowed_joints = {
            item.strip() for item in str(allowed_csv).split(",") if item.strip()
        }
        self._lock = threading.RLock()
        self._last_raw = {spec.name: spec.center_raw for spec in self._joints}
        self._active_goal = None

        if as_bool(self.get_parameter("use_fake_hardware").value):
            self._bus = FakeSo101Bus(self._joints)
            self.get_logger().warn("SO-101 bridge is using fake hardware; no motors will move.")
        else:
            self._bus = FeetechSo101Bus(
                self.get_parameter("port").value,
                self.get_parameter("baudrate").value,
                self._joints,
            )

        self._bus.connect()
        self._last_raw = self._bus.read_positions()

        self._joint_state_pub = self.create_publisher(
            JointState, self.get_parameter("joint_states_topic").value, 10
        )
        publish_period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(publish_period, self._publish_joint_state)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/so101_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self._free_servos_service = self.create_service(
            Trigger,
            "/so101/free_servos",
            self._free_servos_callback,
        )
        self.get_logger().info(
            "SO-101 trajectory bridge ready. Allowed execution joints: "
            f"{sorted(self._allowed_joints)}"
        )

    def destroy_node(self):
        if as_bool(self.get_parameter("disable_torque_on_shutdown").value):
            try:
                self._bus.disable_torque()
            except Exception as exc:
                self.get_logger().error(f"Failed to disable SO-101 torque: {exc}")
        try:
            self._bus.close()
        finally:
            super().destroy_node()

    def _publish_joint_state(self):
        with self._lock:
            try:
                self._last_raw = self._bus.read_positions()
            except Exception as exc:
                self.get_logger().error(f"SO-101 position read failed: {exc}")
                return
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [spec.name for spec in self._joints]
            msg.position = [
                raw_to_rad(self._last_raw[spec.name], spec) for spec in self._joints
            ]
            self._joint_state_pub.publish(msg)

    def _goal_callback(self, goal_request):
        names = list(goal_request.trajectory.joint_names)
        if not names:
            self.get_logger().warn("Rejected empty SO-101 trajectory goal")
            return GoalResponse.REJECT
        unknown = [name for name in names if name not in self._joint_by_name]
        if unknown:
            self.get_logger().warn(f"Rejected trajectory with unknown joints: {unknown}")
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            self.get_logger().warn("Rejected trajectory with no points")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().warn("SO-101 trajectory cancel requested")
        return CancelResponse.ACCEPT

    def _free_servos_callback(self, request, response):
        del request
        with self._lock:
            try:
                self._bus.disable_torque()
            except Exception as exc:
                response.success = False
                response.message = f"failed to release SO-101 servos: {exc}"
                self.get_logger().error(response.message)
                return response
        response.success = True
        response.message = "SO-101 servos released"
        self.get_logger().warn(response.message)
        return response

    def _current_positions_rad(self):
        raw = self._bus.read_positions()
        self._last_raw = raw
        return {
            spec.name: raw_to_rad(raw[spec.name], spec)
            for spec in self._joints
        }

    def _trajectory_position_map(self, trajectory, point):
        return {
            name: float(point.positions[index])
            for index, name in enumerate(trajectory.joint_names)
        }

    def _validate_trajectory(self, trajectory, current):
        start_tolerance = float(self.get_parameter("start_tolerance_rad").value)
        max_step = float(self.get_parameter("max_step_rad").value)

        first = self._trajectory_position_map(trajectory, trajectory.points[0])
        for name, requested in first.items():
            if abs(requested - current[name]) > start_tolerance:
                return (
                    False,
                    f"trajectory start for {name} differs from current position "
                    f"by {abs(requested - current[name]):.3f} rad",
                )

        previous = dict(current)
        for point in trajectory.points:
            point_map = self._trajectory_position_map(trajectory, point)
            for name, requested in point_map.items():
                if (
                    name not in self._allowed_joints
                    and abs(requested - current[name]) > start_tolerance
                ):
                    return (
                        False,
                        f"joint {name} is not allowed for execution yet",
                    )
                if name in self._allowed_joints and abs(requested - previous[name]) > max_step:
                    return (
                        False,
                        f"joint {name} step {abs(requested - previous[name]):.3f} rad "
                        f"exceeds max_step_rad={max_step:.3f}",
                    )
                previous[name] = requested
        return True, ""

    def _wait_for_goal_tolerance(self, trajectory, final_point):
        goal_tolerance = float(self.get_parameter("goal_tolerance_rad").value)
        settle_timeout = float(self.get_parameter("settle_timeout_sec").value)
        settle_poll = float(self.get_parameter("settle_poll_sec").value)
        deadline = time.monotonic() + max(0.0, settle_timeout)
        errors = {}

        while True:
            actual = self._current_positions_rad()
            errors = {
                name: abs(final_point[name] - actual[name])
                for name in trajectory.joint_names
                if name in self._allowed_joints
            }
            if not errors or all(error <= goal_tolerance for error in errors.values()):
                return True, errors
            if time.monotonic() >= deadline:
                return False, errors
            time.sleep(max(0.01, settle_poll))

    def _execute_callback(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        command_rate = float(self.get_parameter("command_rate_hz").value)
        period = 1.0 / command_rate

        with self._lock:
            try:
                current = self._current_positions_rad()
                ok, reason = self._validate_trajectory(trajectory, current)
                if not ok:
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = reason
                    goal_handle.abort()
                    self.get_logger().warn(f"Aborted SO-101 trajectory: {reason}")
                    return result

                feedback = FollowJointTrajectory.Feedback()
                feedback.joint_names = list(trajectory.joint_names)

                for point in trajectory.points:
                    if goal_handle.is_cancel_requested:
                        self._bus.disable_torque()
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "canceled and torque disabled"
                        return result

                    point_map = self._trajectory_position_map(trajectory, point)
                    raw_command = {}
                    for name in self._allowed_joints:
                        if name in point_map:
                            raw_command[name] = rad_to_raw(
                                point_map[name], self._joint_by_name[name]
                            )
                    if raw_command:
                        self._bus.write_positions(raw_command)

                    actual = self._current_positions_rad()
                    feedback.desired = point
                    feedback.actual.positions = [
                        actual[name] for name in trajectory.joint_names
                    ]
                    feedback.error.positions = [
                        point_map[name] - actual[name] for name in trajectory.joint_names
                    ]
                    goal_handle.publish_feedback(feedback)
                    time.sleep(period)

                final_point = self._trajectory_position_map(trajectory, trajectory.points[-1])
                ok, errors = self._wait_for_goal_tolerance(trajectory, final_point)
                if not ok:
                    result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    result.error_string = f"goal tolerance violated: {errors}"
                    goal_handle.abort()
                    return result

                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "SO-101 trajectory completed"
                goal_handle.succeed()
                return result
            except Exception as exc:
                try:
                    self._bus.disable_torque()
                except Exception:
                    pass
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = str(exc)
                goal_handle.abort()
                self.get_logger().error(f"SO-101 trajectory failed: {exc}")
                return result


def main(args=None):
    rclpy.init(args=args)
    node = So101TrajectoryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
