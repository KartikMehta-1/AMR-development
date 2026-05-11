from __future__ import annotations

import argparse
import json
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from amr_voice.tts import PiperSpeaker, SpeechRequest, sanitize_speech_text, speech_request_from_json


class TtsNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__(f"amr_tts_{os.getpid()}")
        self._args = args
        self._speaker = PiperSpeaker(
            piper_bin=args.piper_bin,
            model_path=args.model,
            speaker=args.speaker,
            output_device=args.output_device,
            length_scale=args.length_scale,
            noise_scale=args.noise_scale,
            noise_w_scale=args.noise_w_scale,
            sentence_silence=args.sentence_silence,
            volume=args.volume,
            dry_run=args.dry_run,
        )
        self._last_spoken: dict[str, float] = {}
        self.create_subscription(String, "/amr_voice/say", self.say_cb, 10)
        if args.speak_feedback:
            self.create_subscription(String, "/amr_voice/feedback", self.feedback_cb, 10)
        self.get_logger().info(
            "tts node ready "
            f"dry_run={args.dry_run} speak_feedback={args.speak_feedback} "
            f"status={json.dumps(self._speaker.status(), sort_keys=True)}"
        )

    def say_cb(self, msg: String) -> None:
        self._speak(speech_request_from_json(msg.data))

    def feedback_cb(self, msg: String) -> None:
        self._speak(SpeechRequest(text=sanitize_speech_text(msg.data), source="feedback"))

    def _speak(self, request: SpeechRequest) -> None:
        text = sanitize_speech_text(request.text)
        if not text:
            return
        now = time.monotonic()
        if now - self._last_spoken.get(text, 0.0) < self._args.dedupe_sec:
            self.get_logger().debug(f"deduped speech: {text}")
            return
        self._last_spoken[text] = now
        result = self._speaker.speak(
            SpeechRequest(
                text=text,
                source=request.source,
                priority=request.priority,
                interrupt=request.interrupt,
            )
        )
        if result.ok:
            action = "spoke" if result.spoken else "accepted"
            self.get_logger().info(f"{action}: {result.text}")
        else:
            self.get_logger().warn(f"speech unavailable: {result.message}; text={result.text}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AMR Piper TTS speaker node")
    parser.add_argument("--piper-bin", default=os.environ.get("AMR_PIPER_BIN", "piper"))
    parser.add_argument("--model", default=os.environ.get("AMR_PIPER_MODEL", ""))
    parser.add_argument("--speaker", type=int, default=None)
    parser.add_argument("--output-device", default=os.environ.get("AMR_TTS_OUTPUT_DEVICE", ""))
    parser.add_argument(
        "--length-scale",
        type=_optional_float,
        default=_env_float("AMR_TTS_LENGTH_SCALE"),
    )
    parser.add_argument(
        "--noise-scale",
        type=_optional_float,
        default=_env_float("AMR_TTS_NOISE_SCALE"),
    )
    parser.add_argument(
        "--noise-w-scale",
        type=_optional_float,
        default=_env_float("AMR_TTS_NOISE_W_SCALE"),
    )
    parser.add_argument(
        "--sentence-silence",
        type=_optional_float,
        default=_env_float("AMR_TTS_SENTENCE_SILENCE"),
    )
    parser.add_argument("--volume", type=_optional_float, default=_env_float("AMR_TTS_VOLUME"))
    parser.add_argument("--dedupe-sec", type=float, default=2.0)
    parser.add_argument("--speak-feedback", action="store_true", default=os.environ.get("AMR_TTS_SPEAK_FEEDBACK", "true").lower() in {"1", "true", "yes"})
    parser.add_argument("--dry-run", action="store_true", default=os.environ.get("AMR_TTS_DRY_RUN", "false").lower() in {"1", "true", "yes"})
    return parser.parse_args()


def _env_float(name: str) -> Optional[float]:
    value = os.environ.get(name, "").strip()
    if not value:
        return None
    return float(value)


def _optional_float(value: str) -> Optional[float]:
    if str(value).strip() == "":
        return None
    return float(value)


def main() -> None:
    args = parse_args()
    rclpy.init()
    node: Optional[TtsNode] = None
    try:
        node = TtsNode(args)
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
