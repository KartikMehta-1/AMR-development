import argparse
import audioop
import json
import os
import queue
import struct
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from std_msgs.msg import String

from amr_missions.common import default_places_path
from amr_voice.command_parser import CANCEL, CONFIRM, GO_TO, REJECT, UNKNOWN, WAKE
from amr_voice.voice_text_cli import PendingCommand, VoiceTextCommandNode


DEFAULT_MODEL_PATH = "/workspaces/AMR-development/models/vosk-model-small-en-us-0.15"


class VoiceAsrNode(VoiceTextCommandNode):
    def __init__(self, args: argparse.Namespace):
        super().__init__(
            args.places_file,
            wake_word=args.wake_word,
            require_localization=args.require_localization,
        )
        self._args = args
        self._wake_until = 0.0
        self._pending: Optional[PendingCommand] = None
        self._last_audio_log = 0.0
        self._silent_audio_logs = 0
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
            self.emit_feedback(f"wake: listening for {self._args.wake_window_sec:.0f}s")
            return True

        if command.action == CONFIRM:
            if self._pending is None:
                self.emit_feedback("nothing to confirm")
                return False
            pending = self._pending
            self._pending = None
            place_text = f" place={pending.command.place}" if pending.command.place else ""
            self.emit_feedback(f"confirmed: {pending.command.action}{place_text}")
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
                self.emit_feedback("rejected")
                return True
            self.emit_feedback("nothing to reject")
            return False

        if command.action == UNKNOWN:
            if command.wake_word_detected or now <= self._wake_until:
                self.emit_feedback(command.detail, level="warn")
            else:
                self.get_logger().debug(command.detail)
            return False

        if (
            self._args.wake_gated
            and command.action != CANCEL
            and not command.wake_word_detected
            and now > self._wake_until
        ):
            self.emit_feedback(f"ignored: say '{self._args.wake_word}' first")
            return False

        place_text = f" place={command.place}" if command.place else ""
        self.get_logger().info(
            f"intent: {command.action}{place_text} confidence={command.confidence:.2f}"
        )
        if self._args.confirm_motion and self.requires_confirmation(command):
            if (
                not self._args.dry_run
                and self._args.require_localization
                and command.action == GO_TO
                and not self.localization_ready()
            ):
                self.emit_feedback(
                    "Localization is not ready. Set the 2D pose estimate before starting navigation.",
                    level="warn",
                )
                return False
            self._pending = PendingCommand(
                command=command,
                expires_at=now + max(0.0, self._args.confirm_window_sec),
            )
            self.emit_feedback(self.confirmation_prompt(command))
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
            if self._args.log_partials:
                self.get_logger().info(f"partial: {text}")

    def log_audio_level(self, data: bytes) -> None:
        now = time.monotonic()
        if not self._args.log_audio_level or now - self._last_audio_log < 1.0:
            return
        self._last_audio_log = now
        if not data:
            self.get_logger().info("audio: rms=0 peak=0")
            return
        samples = struct.unpack(f"<{len(data) // 2}h", data)
        if not samples:
            self.get_logger().info("audio: rms=0 peak=0")
            return
        rms = int((sum(sample * sample for sample in samples) / len(samples)) ** 0.5)
        peak = max(abs(sample) for sample in samples)
        self.get_logger().info(f"audio: rms={rms} peak={peak}")
        if peak <= 1:
            self._silent_audio_logs += 1
            if self._silent_audio_logs == 3:
                self.get_logger().warn(
                    "microphone signal is near zero; check the selected --device. "
                    "On this laptop the 16 kHz digital mic is usually device 9, while device 4 may be a silent headset input."
                )
        else:
            self._silent_audio_logs = 0

    def grammar(self) -> str:
        phrases = {
            self._args.wake_word,
            "status",
            "mission status",
            "stop",
            "cancel",
            "yes",
            "yeah",
            "no",
            "list places",
            "where are you",
        }
        for place in self.known_places:
            phrases.update(
                {
                    place,
                    f"go {place}",
                    f"go to {place}",
                    f"go to the {place}",
                    f"navigate to {place}",
                    f"move to {place}",
                }
            )
        phrases.update({"return home", "come home", "[unk]"})
        return json.dumps(sorted(phrases))

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
    parser.add_argument("--require-localization", action="store_true", default=True)
    parser.add_argument("--no-require-localization", dest="require_localization", action="store_false")
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--goal-timeout", type=float, default=180.0)
    parser.add_argument("--model", default=os.environ.get("VOSK_MODEL_PATH", DEFAULT_MODEL_PATH))
    parser.add_argument(
        "--device",
        default=os.environ.get("AMR_VOICE_DEVICE", "auto"),
        help="Input device index/name, or 'auto' to choose the laptop digital mic.",
    )
    parser.add_argument(
        "--sample-rate",
        type=int,
        default=0,
        help="Microphone capture sample rate. Use 0 to use the selected device default.",
    )
    parser.add_argument("--recognition-rate", type=int, default=16000, help="Sample rate passed to Vosk.")
    parser.add_argument("--channels", type=int, default=2, help="Microphone capture channel count.")
    parser.add_argument("--blocksize", type=int, default=12000)
    parser.add_argument("--grammar", action="store_true", default=True)
    parser.add_argument("--no-grammar", dest="grammar", action="store_false")
    parser.add_argument("--log-partials", action="store_true", help="Log Vosk partial transcripts while speaking.")
    parser.add_argument("--log-audio-level", action="store_true", help="Log captured audio RMS/peak once per second.")
    parser.add_argument("--duration-sec", type=float, default=0.0, help="Stop after N seconds; 0 means run forever.")
    parser.add_argument("--dry-run", action="store_true", help="Recognize and parse, but do not call mission services.")
    parser.add_argument("--list-devices", action="store_true", help="List sounddevice input devices and exit.")
    return parser.parse_args()


def _parse_device(device: Optional[str]):
    if device is None or device == "" or device == "auto":
        return None
    try:
        return int(device)
    except ValueError:
        return device


def _select_input_device(sd, requested: Optional[str]):
    if requested not in {None, "", "auto"}:
        device = _parse_device(requested)
        info = sd.query_devices(device, "input")
        return device, info

    devices = sd.query_devices()
    input_devices = [
        (index, info)
        for index, info in enumerate(devices)
        if int(info.get("max_input_channels", 0)) > 0
    ]
    if not input_devices:
        raise RuntimeError("No input audio devices are visible in this container")

    def score(item):
        _index, info = item
        name = str(info.get("name", "")).lower()
        max_output = int(info.get("max_output_channels", 0))
        default_rate = int(float(info.get("default_samplerate", 0)))
        score_value = 0
        if max_output == 0:
            score_value += 100
        if default_rate == 16000:
            score_value += 50
        if "hdmi" in name:
            score_value -= 100
        if "dmic" in name or "sof-hda-dsp" in name:
            score_value += 10
        return score_value

    device, info = max(input_devices, key=score)
    return device, info


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
        audio_queue.put(bytes(indata))

    rclpy.init()
    node: Optional[VoiceAsrNode] = None
    try:
        node = VoiceAsrNode(args)
        node.get_logger().info(f"loading Vosk model: {model_path}")
        model = vosk.Model(str(model_path))
        if args.grammar:
            recognizer = vosk.KaldiRecognizer(model, args.recognition_rate, node.grammar())
        else:
            recognizer = vosk.KaldiRecognizer(model, args.recognition_rate)
        device, device_info = _select_input_device(sd, args.device)
        if args.sample_rate <= 0:
            args.sample_rate = int(device_info["default_samplerate"])
        rate_state = None
        start = time.monotonic()
        node.get_logger().info(
            f"listening on device={device if device is not None else 'default'} "
            f"sample_rate={args.sample_rate} recognition_rate={args.recognition_rate} "
            f"channels={args.channels} wake_gated={args.wake_gated} dry_run={args.dry_run}"
        )

        with sd.RawInputStream(
            samplerate=args.sample_rate,
            blocksize=args.blocksize,
            device=device,
            dtype="int16",
            channels=args.channels,
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
                if args.channels == 2:
                    data = audioop.tomono(data, 2, 0.5, 0.5)
                elif args.channels != 1:
                    raise RuntimeError(f"Unsupported channel count: {args.channels}")
                if args.sample_rate != args.recognition_rate:
                    data, rate_state = audioop.ratecv(
                        data,
                        2,
                        1,
                        args.sample_rate,
                        args.recognition_rate,
                        rate_state,
                    )
                node.log_audio_level(data)
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
