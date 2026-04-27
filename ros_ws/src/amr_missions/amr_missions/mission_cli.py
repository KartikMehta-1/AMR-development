import argparse
import math
import os
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


@dataclass
class NamedPlace:
    name: str
    x: float
    y: float
    yaw: float
    frame_id: str = "map"


def default_places_path() -> str:
    share_dir = get_package_share_directory("amr_missions")
    return os.path.join(share_dir, "config", "places.yaml")


def load_places(path: str) -> Dict[str, NamedPlace]:
    with open(path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    places: Dict[str, NamedPlace] = {}
    for name, value in raw.items():
        if not isinstance(value, dict):
            raise ValueError(f"Place '{name}' must be a mapping")
        try:
            places[name] = NamedPlace(
                name=name,
                x=float(value["x"]),
                y=float(value["y"]),
                yaw=float(value.get("yaw", 0.0)),
                frame_id=str(value.get("frame_id", "map")),
            )
        except KeyError as exc:
            raise ValueError(f"Place '{name}' missing required key: {exc}") from exc
    return places


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


class MissionClient(Node):
    def __init__(self, places_path: str):
        super().__init__("amr_mission_cli")
        self._places_path = places_path
        self._places = load_places(places_path)
        self._client = None

    @property
    def places(self) -> Dict[str, NamedPlace]:
        return self._places

    def refresh_places(self) -> None:
        self._places = load_places(self._places_path)

    def wait_for_server(self, timeout_sec: float) -> bool:
        if self._client is None:
            self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        return self._client.wait_for_server(timeout_sec=timeout_sec)

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

    def go_to(self, place_name: str, timeout_sec: float) -> bool:
        if place_name not in self._places:
            raise KeyError(f"Unknown place '{place_name}'")

        place = self._places[place_name]
        self.get_logger().info(
            f"Navigating to '{place.name}' at x={place.x:.3f}, y={place.y:.3f}, yaw={place.yaw:.3f}"
        )

        if self._client is None:
            raise RuntimeError("Nav2 action client is not initialized")

        send_future = self._client.send_goal_async(self._build_goal(place))
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Goal to '{place_name}' was rejected")
            return False

        result_future = goal_handle.get_result_async()
        if timeout_sec > 0.0:
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
            if not result_future.done():
                self.get_logger().error(f"Goal to '{place_name}' timed out after {timeout_sec:.1f}s")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return False
        else:
            rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result is None:
            self.get_logger().error(f"No result received for '{place_name}'")
            return False

        status = result.status
        succeeded = status == 4
        if succeeded:
            self.get_logger().info(f"Reached '{place_name}'")
        else:
            self.get_logger().error(f"Navigation to '{place_name}' finished with status {status}")
        return succeeded


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Named-place Nav2 mission CLI for the AMR")
    parser.add_argument(
        "--places-file",
        default=default_places_path(),
        help="YAML file containing named places (default: package config/places.yaml)",
    )
    parser.add_argument(
        "--server-timeout",
        type=float,
        default=20.0,
        help="Seconds to wait for the Nav2 navigate_to_pose action server",
    )

    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("list", help="List named places")

    go_to = subparsers.add_parser("go_to", help="Navigate to a named place")
    go_to.add_argument("place", help="Named place to navigate to")
    go_to.add_argument("--timeout", type=float, default=180.0, help="Goal timeout in seconds")

    patrol = subparsers.add_parser("patrol", help="Navigate through a sequence of named places")
    patrol.add_argument("places", nargs="+", help="Ordered named places to visit")
    patrol.add_argument("--loops", type=int, default=1, help="How many loops to run (0 means forever)")
    patrol.add_argument("--timeout", type=float, default=180.0, help="Per-goal timeout in seconds")
    patrol.add_argument("--retries", type=int, default=1, help="Retries per failed waypoint")
    patrol.add_argument(
        "--return-home",
        default=None,
        help="Optional place name to return to if patrol fails or when patrol completes",
    )

    args = parser.parse_args()
    if args.command is None:
        parser.error("a command is required")
    return args


def run_patrol(
    node: MissionClient,
    places: Iterable[str],
    loops: int,
    timeout: float,
    retries: int,
    return_home: Optional[str],
) -> int:
    remaining_loops = loops
    while remaining_loops != 0:
        for place in places:
            attempts = retries + 1
            while attempts > 0:
                if node.go_to(place, timeout_sec=timeout):
                    break
                attempts -= 1
                if attempts > 0:
                    node.get_logger().warn(f"Retrying '{place}' ({attempts} attempts remaining)")
            else:
                node.get_logger().error(f"Patrol failed at '{place}'")
                if return_home:
                    node.go_to(return_home, timeout_sec=timeout)
                return 1
        if remaining_loops > 0:
            remaining_loops -= 1

    if return_home:
        if not node.go_to(return_home, timeout_sec=timeout):
            return 1
    return 0


def main() -> None:
    args = parse_args()
    rclpy.init()
    exit_code = 0
    node: Optional[MissionClient] = None
    try:
        node = MissionClient(args.places_file)
        if args.command == "list":
            for place in sorted(node.places.values(), key=lambda p: p.name):
                print(
                    f"{place.name}: frame={place.frame_id} x={place.x:.3f} y={place.y:.3f} yaw={place.yaw:.3f}"
                )
            return

        if not node.wait_for_server(timeout_sec=args.server_timeout):
            node.get_logger().error("Nav2 action server 'navigate_to_pose' is not available")
            exit_code = 1
            return

        if args.command == "go_to":
            exit_code = 0 if node.go_to(args.place, timeout_sec=args.timeout) else 1
            return

        if args.command == "patrol":
            exit_code = run_patrol(
                node,
                places=args.places,
                loops=args.loops,
                timeout=args.timeout,
                retries=args.retries,
                return_home=args.return_home,
            )
            return
    except Exception as exc:  # pragma: no cover - CLI error path
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"mission_cli error: {exc}")
        exit_code = 1
    finally:
        if node is not None and node._client is not None:
            node._client.destroy()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(exit_code)
