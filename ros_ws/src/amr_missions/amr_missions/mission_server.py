import json
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

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
from amr_missions_msgs.msg import MissionStatus
from amr_missions_msgs.srv import GetMissionState, GoToNamedPose, ListPlaces, PatrolNamedPoses


@dataclass
class MissionRuntimeState:
    state: str = "idle"
    mission_type: str = "none"
    target_places: List[str] = field(default_factory=list)
    current_place: str = ""
    current_loop: int = 0
    total_loops: int = 0
    retries_remaining: int = 0
    detail: str = "idle"


class MissionServer(Node):
    def __init__(self, places_path: str):
        super().__init__("amr_mission_server")
        self._places_path = places_path
        self._last_place_path = self._default_last_place_path()
        self._places: Dict[str, NamedPlace] = load_places(places_path)
        self._callback_group = ReentrantCallbackGroup()
        self._client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self._callback_group,
        )
        self._status_pub = self.create_publisher(MissionStatus, "/amr_missions/status", 10)
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
            GetMissionState,
            "/amr_missions/state",
            self._handle_state,
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
        self._mission_state = MissionRuntimeState()
        self._publish_status()
        self.create_timer(1.0, self._publish_status, callback_group=self._callback_group)

    def _default_last_place_path(self) -> Path:
        configured = os.environ.get("AMR_MISSION_LAST_PLACE_PATH")
        if configured:
            return Path(configured)
        workspace_path = Path("/workspaces/AMR-development/ros_ws/log/amr_last_place.json")
        if workspace_path.parent.exists() or Path("/workspaces/AMR-development").exists():
            return workspace_path
        return Path.home() / ".ros" / "amr_last_place.json"

    @property
    def places(self) -> Dict[str, NamedPlace]:
        return self._places

    def _status_message(self) -> MissionStatus:
        with self._lock:
            snapshot = MissionRuntimeState(
                state=self._mission_state.state,
                mission_type=self._mission_state.mission_type,
                target_places=list(self._mission_state.target_places),
                current_place=self._mission_state.current_place,
                current_loop=self._mission_state.current_loop,
                total_loops=self._mission_state.total_loops,
                retries_remaining=self._mission_state.retries_remaining,
                detail=self._mission_state.detail,
            )
        msg = MissionStatus()
        msg.state = snapshot.state
        msg.mission_type = snapshot.mission_type
        msg.target_places = snapshot.target_places
        msg.current_place = snapshot.current_place
        msg.current_loop = snapshot.current_loop
        msg.total_loops = snapshot.total_loops
        msg.retries_remaining = snapshot.retries_remaining
        msg.detail = snapshot.detail
        return msg

    def _set_state(self, **kwargs) -> None:
        with self._lock:
            for key, value in kwargs.items():
                setattr(self._mission_state, key, value)

    def _publish_status(self) -> None:
        msg = self._status_message()
        self._status_pub.publish(msg)
        self.get_logger().info(
            f"state={msg.state} mission={msg.mission_type} place={msg.current_place} detail={msg.detail}"
        )

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

    def _record_last_place(self, place: NamedPlace) -> None:
        payload = {
            "place": place.name,
            "frame_id": place.frame_id,
            "x": place.x,
            "y": place.y,
            "yaw": place.yaw,
            "stamp_sec": time.time(),
            "source": "mission_server",
        }
        try:
            self._last_place_path.parent.mkdir(parents=True, exist_ok=True)
            tmp_path = self._last_place_path.with_suffix(f"{self._last_place_path.suffix}.tmp")
            tmp_path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
            tmp_path.replace(self._last_place_path)
        except OSError as exc:
            self.get_logger().warning(f"failed to record last AMR place: {exc}")

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
        self._publish_status()

    def _ensure_server(self) -> bool:
        if self._client.wait_for_server(timeout_sec=20.0):
            return True
        self._set_state(state="error", detail="navigate_to_pose action server unavailable")
        self._publish_status()
        return False

    def _execute_go_to(self, place_name: str, timeout_sec: float) -> Tuple[bool, str]:
        if place_name not in self._places:
            return False, f"unknown place '{place_name}'"

        if not self._ensure_server():
            return False, "navigate_to_pose action server unavailable"

        place = self._places[place_name]
        self._set_state(
            state="navigating",
            current_place=place.name,
            detail=f"x={place.x:.3f},y={place.y:.3f},yaw={place.yaw:.3f}",
        )
        self._publish_status()
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
            self._record_last_place(place)
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
        completed_loops = 0
        while remaining_loops != 0 and not self._cancel_requested.is_set():
            completed_loops += 1
            self._set_state(current_loop=completed_loops)
            self._publish_status()
            for place in places:
                attempts = retries + 1
                while attempts > 0 and not self._cancel_requested.is_set():
                    self._set_state(retries_remaining=attempts - 1)
                    success, message = self._execute_go_to(place, timeout_sec=timeout)
                    if success:
                        break
                    attempts -= 1
                    if attempts > 0:
                        self._set_state(
                            state="retrying",
                            current_place=place,
                            retries_remaining=attempts,
                            detail=f"retrying mission target '{place}'",
                        )
                        self._publish_status()
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

    def _begin_mission(
        self,
        mission_type: str,
        target_places: List[str],
        current_place: str,
        current_loop: int,
        total_loops: int,
        retries_remaining: int,
        detail: str,
        target,
        *args,
    ) -> Tuple[bool, str]:
        with self._lock:
            if self._mission_thread is not None and self._mission_thread.is_alive():
                return False, "mission already running"
            self._cancel_requested.clear()
            self._mission_state = MissionRuntimeState(
                state="accepted",
                mission_type=mission_type,
                target_places=list(target_places),
                current_place=current_place,
                current_loop=current_loop,
                total_loops=total_loops,
                retries_remaining=retries_remaining,
                detail=detail,
            )
            self._mission_thread = threading.Thread(target=target, args=args, daemon=True)
            self._mission_thread.start()
        self._publish_status()
        return True, "mission started"

    def _validate_place(self, place_name: str) -> Tuple[bool, str]:
        if place_name not in self._places:
            return False, f"unknown place '{place_name}'"
        return True, ""

    def _validate_patrol_request(
        self,
        places: Iterable[str],
        loops: int,
        timeout_sec: float,
        retries: int,
        return_home: Optional[str],
    ) -> Tuple[bool, str]:
        places_list = list(places)
        if not places_list:
            return False, "patrol requires at least one place"
        for place in places_list:
            ok, message = self._validate_place(place)
            if not ok:
                return False, message
        if return_home:
            ok, message = self._validate_place(return_home)
            if not ok:
                return False, message
        if loops < 0:
            return False, "loops must be >= 0"
        if timeout_sec <= 0.0:
            return False, "timeout_sec must be > 0"
        if retries < 0:
            return False, "retries must be >= 0"
        return True, ""

    def _run_single_mission_thread(self, place: str, timeout: float) -> None:
        try:
            success, message = self._execute_go_to(place, timeout_sec=timeout)
            self._set_state(
                state="succeeded" if success else "error",
                detail=message,
            )
            self._publish_status()
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
            self._set_state(
                state="running",
                mission_type="patrol",
                target_places=list(places),
                current_loop=0,
                total_loops=int(loops),
                retries_remaining=int(retries),
                detail="patrol started",
            )
            self._publish_status()
            success, message = self._run_patrol(places, loops, timeout, retries, return_home)
            self._set_state(
                state="succeeded" if success else "error",
                detail=message,
            )
            self._publish_status()
        finally:
            self._clear_active_mission()

    def _handle_list_places(self, request, response):
        del request
        self.refresh_places()
        response.places = sorted(self._places.keys())
        return response

    def _handle_state(self, request, response):
        del request
        response.status = self._status_message()
        return response

    def _handle_go_to(self, request, response):
        self.refresh_places()
        ok, message = self._validate_place(request.place)
        if not ok:
            response.success = False
            response.message = message
            return response
        if request.timeout_sec <= 0.0:
            response.success = False
            response.message = "timeout_sec must be > 0"
            return response
        ok, message = self._begin_mission(
            "go_to",
            [request.place],
            request.place,
            1,
            1,
            0,
            "go_to accepted",
            self._run_single_mission_thread,
            request.place,
            float(request.timeout_sec),
        )
        response.success = ok
        response.message = message
        return response

    def _handle_patrol(self, request, response):
        self.refresh_places()
        ok, message = self._validate_patrol_request(
            request.places,
            int(request.loops),
            float(request.timeout_sec),
            int(request.retries),
            request.return_home or None,
        )
        if not ok:
            response.success = False
            response.message = message
            return response
        ok, message = self._begin_mission(
            "patrol",
            list(request.places),
            "",
            0,
            int(request.loops),
            int(request.retries),
            "patrol accepted",
            self._run_patrol_thread,
            list(request.places),
            int(request.loops),
            float(request.timeout_sec),
            int(request.retries),
            request.return_home or None,
        )
        response.success = ok
        response.message = message
        return response

    def _handle_cancel(self, request, response):
        del request
        with self._lock:
            running = self._mission_thread is not None and self._mission_thread.is_alive()
            self._cancel_requested.set()
            if running:
                goal_handle = self._active_goal_handle
            else:
                goal_handle = None
                self._active_goal_handle = None
                self._cancel_requested.clear()

        if goal_handle is not None:
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_for_future(cancel_future, 5.0)

        response.success = running
        response.message = "cancel requested" if running else "no active mission"
        self._set_state(
            state="cancel_requested" if running else "idle",
            detail=response.message,
        )
        self._publish_status()
        return response

    def _handle_command(self, msg) -> None:
        self.refresh_places()
        data = msg.data.strip()
        if not data:
            return
        if data == "cancel":
            self._handle_cancel(None, Trigger.Response())
            return
        if data.startswith("go_to:"):
            place = data.split(":", 1)[1].strip()
            if place:
                ok, message = self._validate_place(place)
                if ok:
                    started, start_message = self._begin_mission(
                        "go_to",
                        [place],
                        place,
                        1,
                        1,
                        0,
                        "go_to accepted from topic command",
                        self._run_single_mission_thread,
                        place,
                        180.0,
                    )
                    if started:
                        pass
                    else:
                        self._set_state(state="error", detail=start_message)
                        self._publish_status()
                else:
                    self._set_state(state="error", detail=message)
                    self._publish_status()
            return
        if data.startswith("patrol:"):
            raw_places = data.split(":", 1)[1].strip()
            places = [part.strip() for part in raw_places.split(",") if part.strip()]
            if places:
                ok, message = self._validate_patrol_request(places, 1, 180.0, 1, None)
                if ok:
                    started, start_message = self._begin_mission(
                        "patrol",
                        list(places),
                        "",
                        0,
                        1,
                        1,
                        "patrol accepted from topic command",
                        self._run_patrol_thread, places, 1, 180.0, 1, None
                    )
                    if started:
                        pass
                    else:
                        self._set_state(state="error", detail=start_message)
                        self._publish_status()
                else:
                    self._set_state(state="error", detail=message)
                    self._publish_status()


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
