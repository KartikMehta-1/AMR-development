from __future__ import annotations

import argparse
import json
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from amr_voice.command_parser import UNKNOWN
from amr_voice.conversation import DEFAULT_KNOWN_PLACES, plan_turn, transcript_text
from amr_voice.local_llm import qwen_responder_from_env


class ConversationRuntimeNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__(f"amr_conversation_runtime_{os.getpid()}")
        self._args = args
        self._llm = qwen_responder_from_env() if args.enable_llm else None
        self._say_pub = self.create_publisher(String, "/amr_voice/say", 10)
        self._plan_pub = self.create_publisher(String, "/amr_voice/conversation_plan", 10)
        self.create_subscription(String, args.transcript_topic, self._transcript_cb, 10)
        self.get_logger().info(
            f"conversation runtime ready transcript_topic={args.transcript_topic} dry_run={args.dry_run}"
        )

    def _transcript_cb(self, msg: String) -> None:
        text = transcript_text(msg.data)
        if not text:
            return
        turn = plan_turn(
            text,
            known_places=self._args.known_places,
            wake_word=self._args.wake_word,
            require_wake_word=False,
            dry_run=self._args.dry_run,
        )
        assistant_response = turn.assistant_response
        allowed = turn.allowed
        blockers = list(turn.blockers)
        llm_used = False
        if self._llm is not None and turn.command.get("action") == UNKNOWN:
            result = self._llm.respond(text)
            if result.ok:
                assistant_response = result.text
                allowed = True
                blockers = []
                llm_used = True
            else:
                self.get_logger().warn(f"local LLM unavailable: {result.message}")
        plan_payload = {
            "event": "conversation_turn",
            "text": turn.text,
            "assistant_response": assistant_response,
            "command": turn.command,
            "next_tool": turn.next_tool,
            "requires_confirmation": turn.requires_confirmation,
            "allowed": allowed,
            "blockers": blockers,
            "llm_used": llm_used,
        }
        self._publish_json(self._plan_pub, plan_payload)
        if allowed:
            self._publish_json(
                self._say_pub,
                {
                    "text": assistant_response,
                    "source": "conversation_runtime",
                    "priority": "normal",
                    "interrupt": False,
                },
            )
        self.get_logger().info(
            f"transcript='{turn.text}' response='{assistant_response}' blockers={blockers} llm_used={llm_used}"
        )

    @staticmethod
    def _publish_json(publisher, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        publisher.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Persistent AMR voice conversation runtime")
    parser.add_argument("--transcript-topic", default=os.environ.get("AMR_CONVERSATION_TRANSCRIPT_TOPIC", "/amr_voice/transcript"))
    parser.add_argument("--wake-word", default=os.environ.get("AMR_VOICE_WAKE_WORD", "hey jarvis"))
    parser.add_argument(
        "--known-place",
        action="append",
        dest="known_places",
        default=list(DEFAULT_KNOWN_PLACES),
    )
    parser.add_argument("--dry-run", action="store_true", default=os.environ.get("AMR_CONVERSATION_DRY_RUN", "true").lower() in {"1", "true", "yes"})
    parser.add_argument("--enable-llm", action="store_true", default=os.environ.get("AMR_CONVERSATION_ENABLE_LLM", "false").lower() in {"1", "true", "yes"})
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node: Optional[ConversationRuntimeNode] = None
    try:
        node = ConversationRuntimeNode(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
