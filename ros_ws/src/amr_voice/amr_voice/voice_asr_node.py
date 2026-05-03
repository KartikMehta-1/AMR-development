import argparse
import json
import os
import queue
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from std_msgs.msg import String

from amr_missions.common import default_places_path
from amr_voice.command_parser import CANCEL, CONFIRM, REJECT, UNKNOWN, WAKE
from amr_voice.voice_text_cli import PendingCommand, VoiceTextCommandNode


DEFAULT_MODEL_PATH = "/workspaces/AMR-development/models/vosk-model-small-en-us-0.15"


class VoiceAsrNode(VoiceTextCommandNode):
    def __init__(self, args: argparse.Namespace):
        super().__init__(args.places_file, wake_word=args.wake_word)
        self._args = args
        self._wake_until = 0.0
        self._pending: Optional[PendingCommand] = None
        self._transcript_pub = self.create_publisher(String, "/amr_voice/transcript", 10)
        self._partial_pub = self.create_publisher(String, "/amr_voice/partial_transcript", 10)

    def handle_transcript(self, text: str) -> bool:
        text = text.strip()
        if not text:
            return False
        self._publish(self._transcript_pub, text)
        self.get_logger().info(f"transcript: {text}")

        command = self.parse(text)
        now = time.monotonic()
        if self._pending is not None and now > self._pending.expires_at:
            self.get_logger().info("confirmation expired")
            self._pending = None

        if command.action == WAKE:
            self._wake_until = now + max(0.0, self._args.wake_window_sec)
            self.get_logger().info(f"wake: listening for {self._args.wake_window_sec:.0f}s")
            return True

        if command.action == CONFIRM:
            if self._pending is None:
                self.get_logger().info("nothing to confirm")
                return False
            pending = self._pending
            self._pending = None
            place_text = f" place={pending.command.place}" if pending.command.place else ""
            self.get_logger().info(f"confirmed: {pending.command.action}{place_text}")
            if self._args.dry_run:
                return True
            return self.execute(
                pending.command,
                server_timeout=self._args.server_timeout,
                goal_timeout=self._args.goal_timeout,
            )

        if command.action == REJECT:
            if self._pending is not None:
                self._pending = None
                self.get_logger().info("rejected")
                return True
            self.get_logger().info("nothing to reject")
            return False

        if command.action == UNKNOWN:
            if command.wake_word_detected or now <= self._wake_until:
                self.get_logger().warn(command.detail)
            else:
                self.get_logger().debug(command.detail)
            return False

        if (
            self._args.wake_gated
            and command.action != CANCEL
            and not command.wake_word_detected
            and now > self._wake_until
        ):
            self.get_logger().info(f"ignored: say '{self._args.wake_word}' first")
            return False

        place_text = f" place={command.place}" if command.place else ""
        self.get_logger().info(
            f"intent: {command.action}{place_text} confidence={command.confidence:.2f}"
        )
        if self._args.confirm_motion and self.requires_confirmation(command):
            self._pending = PendingCommand(
                command=command,
                expires_at=now + max(0.0, self._args.confirm_window_sec),
            )
            self.get_logger().info(self.confirmation_prompt(command))
            return True
        if self._args.dry_run:
            return True
        if command.action == CANCEL:
            self._pending = None
        return self.execute(
            command,
            server_timeout=self._args.server_timeout,
            goal_timeout=self._args.goal_timeout,
        )

    def publish_partial(self, text: str) -> None:
        text = text.strip()
        if text:
            self._publish(self._partial_pub, text)

    @staticmethod
    def _publish(publisher, text: str) -> None:
        msg = String()
        msg.data = text
        publisher.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Laptop microphone ASR interface for AMR voice commands")
    parser.add_argument("--places-file", default=default_places_path())
    parser.add_argument("--wake-word", default="lovely")
    parser.add_argument("--wake-gated", action="store_true", default=True)
    parser.add_argument("--no-wake-gated", dest="wake_gated", action="store_false")
    parser.add_argument("--wake-window-sec", type=float, default=12.0)
    parser.add_argument("--confirm-motion", action="store_true", default=True)
    parser.add_argument("--no-confirm-motion", dest="confirm_motion", action="store_false")
    parser.add_argument("--confirm-window-sec", type=float, default=8.0)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--goal-timeout", type=float, default=180.0)
    parser.add_argument("--model", default=os.environ.get("VOSK_MODEL_PATH", DEFAULT_MODEL_PATH))
    parser.add_argument("--device", default=os.environ.get("AMR_VOICE_DEVICE", None))
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--blocksize", type=int, default=4000)
    parser.add_argument("--duration-sec", type=float, default=0.0, help="Stop after N seconds; 0 means run forever.")
    parser.add_argument("--dry-run", action="store_true", help="Recognize and parse, but do not call mission services.")
    parser.add_argument("--list-devices", action="store_true", help="List sounddevice input devices and exit.")
    return parser.parse_args()


def _parse_device(device: Optional[str]):
    if device is None or device == "":
        return None
    try:
        return int(device)
    except ValueError:
        return device


def _load_audio_modules():
    try:
        import sounddevice as sd
    except Exception as exc:
        raise RuntimeError("Python package 'sounddevice' is not available in this container") from exc
    try:
        import vosk
    except Exception as exc:
        raise RuntimeError("Python package 'vosk' is not available in this container") from exc
    return sd, vosk


def _extract_text(result_json: str, key: str = "text") -> str:
    try:
        payload = json.loads(result_json)
    except json.JSONDecodeError:
        return ""
    value = payload.get(key, "")
    return value if isinstance(value, str) else ""


def _validate_model(model_path: str) -> Path:
    path = Path(model_path).expanduser()
    if not path.exists():
        raise FileNotFoundError(
            f"Vosk model not found: {path}. Download one with: "
            "mkdir -p /workspaces/AMR-development/models && cd /workspaces/AMR-development/models && "
            "wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip && "
            "unzip vosk-model-small-en-us-0.15.zip"
        )
    return path


def main() -> None:
    args = parse_args()
    sd, vosk = _load_audio_modules()

    if args.list_devices:
        print(sd.query_devices())
        return

    try:
        model_path = _validate_model(args.model)
    except FileNotFoundError as exc:
        print(f"voice_asr_node error: {exc}", file=sys.stderr)
        raise SystemExit(2)
    audio_queue: "queue.Queue[bytes]" = queue.Queue()

    def callback(indata, frames, time_info, status):  # pragma: no cover - callback path
        if status:
            print(status, file=sys.stderr)
        audio_queue.put(bytes(indata))

    rclpy.init()
    node: Optional[VoiceAsrNode] = None
    try:
        node = VoiceAsrNode(args)
        node.get_logger().info(f"loading Vosk model: {model_path}")
        model = vosk.Model(str(model_path))
        recognizer = vosk.KaldiRecognizer(model, args.sample_rate)
        device = _parse_device(args.device)
        start = time.monotonic()
        node.get_logger().info(
            f"listening on device={device if device is not None else 'default'} "
            f"sample_rate={args.sample_rate} wake_gated={args.wake_gated} dry_run={args.dry_run}"
        )

        with sd.RawInputStream(
            samplerate=args.sample_rate,
            blocksize=args.blocksize,
            device=device,
            dtype="int16",
            channels=1,
            callback=callback,
        ):
            while rclpy.ok():
                if args.duration_sec > 0.0 and time.monotonic() - start >= args.duration_sec:
                    break
                try:
                    data = audio_queue.get(timeout=0.1)
                except queue.Empty:
                    rclpy.spin_once(node, timeout_sec=0.0)
                    continue
                if recognizer.AcceptWaveform(data):
                    text = _extract_text(recognizer.Result(), key="text")
                    if text:
                        node.handle_transcript(text)
                else:
                    node.publish_partial(_extract_text(recognizer.PartialResult(), key="partial"))
                rclpy.spin_once(node, timeout_sec=0.0)

        final_text = _extract_text(recognizer.FinalResult(), key="text")
        if node is not None and final_text:
            node.handle_transcript(final_text)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
