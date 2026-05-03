import argparse
import os
import sys
import time
from typing import Iterable, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from amr_missions.common import default_places_path, load_places
from amr_missions_msgs.srv import GetMissionState, GoToNamedPose, ListPlaces
from amr_voice.command_parser import (
    CANCEL,
    GO_TO,
    LIST_PLACES,
    STATUS,
    UNKNOWN,
    WAKE,
    ParsedCommand,
    parse_text_command,
)


class VoiceTextCommandNode(Node):
    def __init__(self, places_file: str, wake_word: str):
        super().__init__(f"amr_voice_text_{os.getpid()}")
        self._places_file = places_file
        self._wake_word = wake_word
        self._places = load_places(places_file)
        self._list_client = self.create_client(ListPlaces, "/amr_missions/list_places")
        self._go_to_client = self.create_client(GoToNamedPose, "/amr_missions/go_to")
        self._state_client = self.create_client(GetMissionState, "/amr_missions/state")
        self._cancel_client = self.create_client(Trigger, "/amr_missions/cancel")

    @property
    def known_places(self) -> Iterable[str]:
        return sorted(self._places.keys())

    def parse(self, text: str) -> ParsedCommand:
        return parse_text_command(
            text,
            known_places=self.known_places,
            wake_word=self._wake_word,
            require_wake_word=False,
        )

    def execute(self, command: ParsedCommand, server_timeout: float, goal_timeout: float) -> bool:
        if command.action == GO_TO:
            assert command.place is not None
            return self._go_to(command.place, server_timeout=server_timeout, goal_timeout=goal_timeout)
        if command.action == CANCEL:
            return self._cancel(server_timeout=server_timeout)
        if command.action == STATUS:
            return self._status(server_timeout=server_timeout)
        if command.action == LIST_PLACES:
            return self._list_places(server_timeout=server_timeout)
        self.get_logger().warn(command.detail or "Unknown command")
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

    def _go_to(self, place: str, server_timeout: float, goal_timeout: float) -> bool:
        if not self._go_to_client.wait_for_service(timeout_sec=server_timeout):
            self.get_logger().error("Mission go_to service is not available")
            return False
        request = GoToNamedPose.Request()
        request.place = place
        request.timeout_sec = float(goal_timeout)
        response = self._wait_for_response(
            self._go_to_client.call_async(request),
            "/amr_missions/go_to",
            timeout_sec=server_timeout,
        )
        if response is None:
            return False
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return bool(response.success)

    def _cancel(self, server_timeout: float) -> bool:
        if not self._cancel_client.wait_for_service(timeout_sec=server_timeout):
            self.get_logger().error("Mission cancel service is not available")
            return False
        response = self._wait_for_response(
            self._cancel_client.call_async(Trigger.Request()),
            "/amr_missions/cancel",
            timeout_sec=server_timeout,
        )
        if response is None:
            return False
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)
        return bool(response.success)

    def _status(self, server_timeout: float) -> bool:
        if not self._state_client.wait_for_service(timeout_sec=server_timeout):
            self.get_logger().error("Mission state service is not available")
            return False
        response = self._wait_for_response(
            self._state_client.call_async(GetMissionState.Request()),
            "/amr_missions/state",
            timeout_sec=server_timeout,
        )
        if response is None:
            return False
        status = response.status
        print(f"state: {status.state}")
        print(f"mission_type: {status.mission_type}")
        print(f"target_places: {', '.join(status.target_places) if status.target_places else '-'}")
        print(f"current_place: {status.current_place or '-'}")
        print(f"current_loop: {status.current_loop}")
        print(f"total_loops: {status.total_loops}")
        print(f"retries_remaining: {status.retries_remaining}")
        print(f"detail: {status.detail}")
        return True

    def _list_places(self, server_timeout: float) -> bool:
        places = None
        if self._list_client.wait_for_service(timeout_sec=2.0):
            response = self._wait_for_response(
                self._list_client.call_async(ListPlaces.Request()),
                "/amr_missions/list_places",
                timeout_sec=server_timeout,
            )
            if response is not None:
                places = list(response.places)
        if places is None:
            self._places = load_places(self._places_file)
            places = sorted(self._places.keys())
        print("places: " + ", ".join(places))
        return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Typed natural-language command interface for AMR missions")
    parser.add_argument("--places-file", default=default_places_path())
    parser.add_argument("--wake-word", default="lovely")
    parser.add_argument(
        "--wake-gated",
        action="store_true",
        help="Require the wake word or an active wake window for non-stop commands.",
    )
    parser.add_argument(
        "--require-wake-word",
        action="store_true",
        help="Compatibility alias for --wake-gated.",
    )
    parser.add_argument("--wake-window-sec", type=float, default=12.0)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--goal-timeout", type=float, default=180.0)
    parser.add_argument("--command", help="Run one command and exit instead of starting the interactive prompt")
    parser.add_argument(
        "--input-mode",
        default="text",
        choices=["text"],
        help="Compatibility option for the navigation tmux launcher; only text is supported in Phase 1.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Parse commands without calling mission services")
    return parser.parse_args()


def _wake_gate_enabled(args: argparse.Namespace) -> bool:
    return bool(args.wake_gated or args.require_wake_word)


def _handle_line(
    node: VoiceTextCommandNode,
    args: argparse.Namespace,
    line: str,
    wake_until: float,
) -> Tuple[bool, float]:
    command = node.parse(line)
    now = time.monotonic()
    gate_enabled = _wake_gate_enabled(args)

    if command.action == WAKE:
        wake_until = now + max(0.0, args.wake_window_sec)
        print(f"wake: listening for {args.wake_window_sec:.0f}s")
        return True, wake_until

    if command.action == UNKNOWN:
        print(f"unrecognized: {command.detail}")
        return False, wake_until

    if gate_enabled and command.action != CANCEL and not command.wake_word_detected and now > wake_until:
        print(f"ignored: say '{args.wake_word}' first")
        return False, wake_until

    place_text = f" place={command.place}" if command.place else ""
    wake_text = " wake=yes" if command.wake_word_detected else " wake=no"
    print(f"intent: {command.action}{place_text} confidence={command.confidence:.2f}{wake_text}")
    if args.dry_run:
        return True, wake_until
    return node.execute(command, server_timeout=args.server_timeout, goal_timeout=args.goal_timeout), wake_until


def main() -> None:
    args = parse_args()
    rclpy.init()
    exit_code = 0
    node: Optional[VoiceTextCommandNode] = None
    try:
        node = VoiceTextCommandNode(args.places_file, wake_word=args.wake_word)
        wake_until = 0.0
        if args.command:
            handled, wake_until = _handle_line(node, args, args.command, wake_until)
            exit_code = 0 if handled else 1
            return

        print("AMR text command mode. Examples: lovely go kitchen, lovely, go hall, stop, status, list places.")
        if _wake_gate_enabled(args):
            print(f"Wake gate enabled. Say '{args.wake_word}' before non-stop commands.")
        print("Type q or quit to exit.")
        for line in sys.stdin:
            text = line.strip()
            if not text:
                continue
            if text in {"q", "quit", "exit"}:
                break
            _, wake_until = _handle_line(node, args, text, wake_until)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(exit_code)
