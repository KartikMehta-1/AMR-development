import argparse
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from amr_missions.common import default_places_path, load_places
from amr_missions_msgs.srv import GetMissionState, GoToNamedPose, ListPlaces, PatrolNamedPoses


class MissionClient(Node):
    def __init__(self, places_path: str):
        super().__init__(f"amr_mission_cli_{os.getpid()}")
        self._places_path = places_path
        self._places = load_places(places_path)
        self._list_client = self.create_client(ListPlaces, "/amr_missions/list_places")
        self._go_to_client = self.create_client(GoToNamedPose, "/amr_missions/go_to")
        self._patrol_client = self.create_client(PatrolNamedPoses, "/amr_missions/patrol")
        self._state_client = self.create_client(GetMissionState, "/amr_missions/state")
        self._cancel_client = self.create_client(Trigger, "/amr_missions/cancel")

    @property
    def places(self):
        return self._places

    def refresh_places(self) -> None:
        self._places = load_places(self._places_path)

    def wait_for_service(self, client, service_name: str, timeout_sec: float) -> bool:
        if client.wait_for_service(timeout_sec=timeout_sec):
            return True
        self.get_logger().error(f"Mission service is not available: {service_name}")
        return False

    def _wait_for_response(self, future, service_name: str, timeout_sec: float):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().error(f"Timed out waiting for response from {service_name}")
            return None
        response = future.result()
        if response is None:
            self.get_logger().error(f"No response from {service_name}")
        return response

    def list_places_remote(self, response_timeout_sec: float = 2.0) -> Optional[list]:
        future = self._list_client.call_async(ListPlaces.Request())
        response = self._wait_for_response(
            future,
            "/amr_missions/list_places",
            timeout_sec=response_timeout_sec,
        )
        return None if response is None else list(response.places)

    def go_to(self, place_name: str, timeout_sec: float, response_timeout_sec: float) -> bool:
        request = GoToNamedPose.Request()
        request.place = place_name
        request.timeout_sec = float(timeout_sec)
        future = self._go_to_client.call_async(request)
        response = self._wait_for_response(
            future,
            "/amr_missions/go_to",
            timeout_sec=response_timeout_sec,
        )
        if response is None:
            return False
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response.success

    def mission_state(self, response_timeout_sec: float = 2.0):
        future = self._state_client.call_async(GetMissionState.Request())
        response = self._wait_for_response(
            future,
            "/amr_missions/state",
            timeout_sec=response_timeout_sec,
        )
        return None if response is None else response.status

    def patrol(
        self,
        places,
        loops: int,
        timeout: float,
        retries: int,
        return_home: Optional[str],
        response_timeout_sec: float,
    ) -> bool:
        request = PatrolNamedPoses.Request()
        request.places = list(places)
        request.loops = int(loops)
        request.timeout_sec = float(timeout)
        request.retries = int(retries)
        request.return_home = return_home or ""
        future = self._patrol_client.call_async(request)
        response = self._wait_for_response(
            future,
            "/amr_missions/patrol",
            timeout_sec=response_timeout_sec,
        )
        if response is None:
            return False
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response.success

    def cancel(self, response_timeout_sec: float) -> bool:
        future = self._cancel_client.call_async(Trigger.Request())
        response = self._wait_for_response(
            future,
            "/amr_missions/cancel",
            timeout_sec=response_timeout_sec,
        )
        if response is None:
            return False
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)
        return response.success


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
        help="Seconds to wait for the required mission server service",
    )

    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("list", help="List named places")
    subparsers.add_parser("status", help="Show current mission runtime state")

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
    subparsers.add_parser("cancel", help="Cancel the currently running mission")

    args = parser.parse_args()
    if args.command is None:
        parser.error("a command is required")
    return args

def main() -> None:
    args = parse_args()
    rclpy.init()
    exit_code = 0
    node: Optional[MissionClient] = None
    try:
        node = MissionClient(args.places_file)
        if args.command == "list":
            places = None
            if node._list_client.wait_for_service(timeout_sec=2.0):
                places = node.list_places_remote(response_timeout_sec=2.0)
            if places is None:
                node.refresh_places()
                places = sorted(node.places.keys())
            for place_name in places:
                place = node.places.get(place_name)
                if place is None:
                    print(place_name)
                    continue
                print(
                    f"{place.name}: frame={place.frame_id} x={place.x:.3f} y={place.y:.3f} yaw={place.yaw:.3f}"
                )
            return

        if args.command == "status":
            if not node._state_client.wait_for_service(timeout_sec=2.0):
                node.get_logger().error("Mission state service is not available")
                exit_code = 1
                return
            status = node.mission_state(response_timeout_sec=args.server_timeout)
            if status is None:
                node.get_logger().error("No response from mission server")
                exit_code = 1
                return
            print(f"state: {status.state}")
            print(f"mission_type: {status.mission_type}")
            print(f"target_places: {', '.join(status.target_places) if status.target_places else '-'}")
            print(f"current_place: {status.current_place or '-'}")
            print(f"current_loop: {status.current_loop}")
            print(f"total_loops: {status.total_loops}")
            print(f"retries_remaining: {status.retries_remaining}")
            print(f"detail: {status.detail}")
            return

        if args.command == "go_to":
            if not node.wait_for_service(
                node._go_to_client,
                "/amr_missions/go_to",
                timeout_sec=args.server_timeout,
            ):
                exit_code = 1
                return
            exit_code = 0 if node.go_to(
                args.place,
                timeout_sec=args.timeout,
                response_timeout_sec=args.server_timeout,
            ) else 1
            return

        if args.command == "patrol":
            if not node.wait_for_service(
                node._patrol_client,
                "/amr_missions/patrol",
                timeout_sec=args.server_timeout,
            ):
                exit_code = 1
                return
            exit_code = 0 if node.patrol(
                args.places,
                loops=args.loops,
                timeout=args.timeout,
                retries=args.retries,
                return_home=args.return_home,
                response_timeout_sec=args.server_timeout,
            ) else 1
            return
        if args.command == "cancel":
            if not node.wait_for_service(
                node._cancel_client,
                "/amr_missions/cancel",
                timeout_sec=args.server_timeout,
            ):
                exit_code = 1
                return
            exit_code = 0 if node.cancel(response_timeout_sec=args.server_timeout) else 1
            return
    except Exception as exc:  # pragma: no cover - CLI error path
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"mission_cli error: {exc}")
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(exit_code)
