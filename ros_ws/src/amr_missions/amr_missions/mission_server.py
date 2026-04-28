import threading
import time
from typing import Dict, Iterable, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from amr_missions.common import NamedPlace, default_places_path, load_places, yaw_to_quaternion
from amr_missions_msgs.srv import GoToNamedPose, ListPlaces, PatrolNamedPoses


class MissionServer(Node):
    def __init__(self, places_path: str):
        super().__init__("amr_mission_server")
        self._places_path = places_path
        self._places: Dict[str, NamedPlace] = load_places(places_path)
        self._callback_group = ReentrantCallbackGroup()
        self._client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self._callback_group,
        )
        self._status_pub = self.create_publisher(String, "/amr_missions/status", 10)
        self.create_subscription(
            String,
            "/amr_missions/command",
            self._handle_command,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(
            ListPlaces,
            "/amr_missions/list_places",
            self._handle_list_places,
            callback_group=self._callback_group,
        )
        self.create_service(
            GoToNamedPose,
            "/amr_missions/go_to",
            self._handle_go_to,
            callback_group=self._callback_group,
        )
        self.create_service(
            PatrolNamedPoses,
            "/amr_missions/patrol",
            self._handle_patrol,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/amr_missions/cancel",
            self._handle_cancel,
            callback_group=self._callback_group,
        )
        self._lock = threading.Lock()
        self._mission_thread: Optional[threading.Thread] = None
        self._cancel_requested = threading.Event()
        self._active_goal_handle = None
        self._publish_status("idle")

    @property
    def places(self) -> Dict[str, NamedPlace]:
        return self._places

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)

    def refresh_places(self) -> None:
        self._places = load_places(self._places_path)

    def _build_goal(self, place: NamedPlace) -> NavigateToPose.Goal:
        pose = PoseStamped()
        pose.header.frame_id = place.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = place.x
        pose.pose.position.y = place.y
        pose.pose.position.z = 0.0
        z, w = yaw_to_quaternion(place.yaw)
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        return event.wait(timeout_sec)

    def _wait_for_result(self, result_future, timeout_sec: float) -> bool:
        start = time.monotonic()
        while rclpy.ok():
            if result_future.done():
                return True
            if self._cancel_requested.is_set():
                return False
            if timeout_sec > 0.0 and (time.monotonic() - start) >= timeout_sec:
                return False
            time.sleep(0.1)
        return False

    def _clear_active_mission(self) -> None:
        with self._lock:
            self._mission_thread = None
            self._active_goal_handle = None
            self._cancel_requested.clear()

    def _ensure_server(self) -> bool:
        if self._client.wait_for_server(timeout_sec=20.0):
            return True
        self._publish_status("error: navigate_to_pose action server unavailable")
        return False

    def _execute_go_to(self, place_name: str, timeout_sec: float) -> Tuple[bool, str]:
        if place_name not in self._places:
            return False, f"unknown place '{place_name}'"

        if not self._ensure_server():
            return False, "navigate_to_pose action server unavailable"

        place = self._places[place_name]
        self._publish_status(
            f"navigating:{place.name}:x={place.x:.3f},y={place.y:.3f},yaw={place.yaw:.3f}"
        )
        send_future = self._client.send_goal_async(self._build_goal(place))
        if not self._wait_for_future(send_future, 10.0):
            return False, f"timed out sending goal to '{place_name}'"

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, f"goal to '{place_name}' was rejected"

        with self._lock:
            self._active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        finished = self._wait_for_result(result_future, timeout_sec)
        if not finished:
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_for_future(cancel_future, 5.0)
            if self._cancel_requested.is_set():
                return False, f"goal to '{place_name}' cancelled"
            return False, f"goal to '{place_name}' timed out after {timeout_sec:.1f}s"

        result = result_future.result()
        if result is None:
            return False, f"no result received for '{place_name}'"

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True, f"reached '{place_name}'"
        return False, f"navigation to '{place_name}' finished with status {status}"

    def _run_patrol(
        self,
        places: Iterable[str],
        loops: int,
        timeout: float,
        retries: int,
        return_home: Optional[str],
    ) -> Tuple[bool, str]:
        remaining_loops = loops
        while remaining_loops != 0 and not self._cancel_requested.is_set():
            for place in places:
                attempts = retries + 1
                while attempts > 0 and not self._cancel_requested.is_set():
                    success, message = self._execute_go_to(place, timeout_sec=timeout)
                    if success:
                        break
                    attempts -= 1
                    if attempts > 0:
                        self._publish_status(f"retrying:{place}:remaining={attempts}")
                else:
                    if return_home and not self._cancel_requested.is_set():
                        self._execute_go_to(return_home, timeout_sec=timeout)
                    return False, f"patrol failed at '{place}'"
            if remaining_loops > 0:
                remaining_loops -= 1

        if self._cancel_requested.is_set():
            return False, "patrol cancelled"

        if return_home:
            success, message = self._execute_go_to(return_home, timeout_sec=timeout)
            if not success:
                return False, message
        return True, "patrol complete"

    def _start_thread(self, target, *args) -> Tuple[bool, str]:
        with self._lock:
            if self._mission_thread is not None and self._mission_thread.is_alive():
                return False, "mission already running"
            self._cancel_requested.clear()
            self._mission_thread = threading.Thread(target=target, args=args, daemon=True)
            self._mission_thread.start()
        return True, "mission started"

    def _run_single_mission_thread(self, place: str, timeout: float) -> None:
        try:
            success, message = self._execute_go_to(place, timeout_sec=timeout)
            self._publish_status(("success:" if success else "error:") + message)
        finally:
            self._clear_active_mission()

    def _run_patrol_thread(
        self,
        places: Iterable[str],
        loops: int,
        timeout: float,
        retries: int,
        return_home: Optional[str],
    ) -> None:
        try:
            self._publish_status(f"patrol_started:{','.join(places)}")
            success, message = self._run_patrol(places, loops, timeout, retries, return_home)
            self._publish_status(("success:" if success else "error:") + message)
        finally:
            self._clear_active_mission()

    def _handle_list_places(self, request, response):
        del request
        self.refresh_places()
        response.places = sorted(self._places.keys())
        return response

    def _handle_go_to(self, request, response):
        self.refresh_places()
        ok, message = self._start_thread(
            self._run_single_mission_thread,
            request.place,
            float(request.timeout_sec),
        )
        response.success = ok
        response.message = message
        if ok:
            self._publish_status(f"accepted:go_to:{request.place}")
        return response

    def _handle_patrol(self, request, response):
        self.refresh_places()
        ok, message = self._start_thread(
            self._run_patrol_thread,
            list(request.places),
            int(request.loops),
            float(request.timeout_sec),
            int(request.retries),
            request.return_home or None,
        )
        response.success = ok
        response.message = message
        if ok:
            self._publish_status(f"accepted:patrol:{','.join(request.places)}")
        return response

    def _handle_cancel(self, request, response):
        del request
        with self._lock:
            running = self._mission_thread is not None and self._mission_thread.is_alive()
            goal_handle = self._active_goal_handle
            self._cancel_requested.set()

        if goal_handle is not None:
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_for_future(cancel_future, 5.0)

        response.success = running
        response.message = "cancel requested" if running else "no active mission"
        self._publish_status("cancel_requested" if running else "cancel_ignored:no_active_mission")
        return response

    def _handle_command(self, msg: String) -> None:
        data = msg.data.strip()
        if not data:
            return
        if data == "cancel":
            self._handle_cancel(None, Trigger.Response())
            return
        if data.startswith("go_to:"):
            place = data.split(":", 1)[1].strip()
            if place:
                self._start_thread(self._run_single_mission_thread, place, 180.0)
            return
        if data.startswith("patrol:"):
            raw_places = data.split(":", 1)[1].strip()
            places = [part.strip() for part in raw_places.split(",") if part.strip()]
            if places:
                self._start_thread(self._run_patrol_thread, places, 1, 180.0, 1, None)


def main() -> None:
    rclpy.init()
    node = MissionServer(default_places_path())
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        if node._client is not None:
            node._client.destroy()
        node.destroy_node()
        rclpy.shutdown()
