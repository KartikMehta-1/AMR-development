from __future__ import annotations

import argparse
import audioop
import json
import os
import queue
import struct
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from amr_voice.audio_devices import parse_device, select_input_device
from amr_voice.vad import (
    DEFAULT_VAD_END_FRAMES,
    DEFAULT_VAD_RELEASE_THRESHOLD,
    DEFAULT_VAD_START_FRAMES,
    DEFAULT_VAD_THRESHOLD,
    SileroVadDetector,
    VadGate,
)


class VadNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__(f"amr_vad_{os.getpid()}")
        self._args = args
        self._detector = SileroVadDetector(n_threads=args.threads)
        self._gate = VadGate(
            threshold=args.threshold,
            release_threshold=args.release_threshold,
            start_frames=args.start_frames,
            end_frames=args.end_frames,
        )
        self._event_pub = self.create_publisher(String, "/amr_voice/vad", 10)
        self._feedback_pub = self.create_publisher(String, "/amr_voice/feedback", 10)
        self._last_audio_log = 0.0

    def process_audio(self, data: bytes) -> None:
        score = self._detector.score(data, frame_size=self._args.vad_frame_samples)
        decision = self._gate.update(score)
        if self._args.log_scores:
            self.get_logger().info(
                f"vad score={decision.score:.3f} active={decision.speech_active}"
            )
        if not decision.event:
            return
        payload = {
            "event": decision.event,
            "score": decision.score,
            "threshold": decision.threshold,
            "release_threshold": decision.release_threshold,
            "source": self._args.source,
            "timestamp_monotonic_sec": time.monotonic(),
        }
        self._publish(self._event_pub, json.dumps(payload, sort_keys=True))
        self._publish(self._feedback_pub, f"vad: {decision.event}")
        self.get_logger().info(f"vad event={decision.event} score={decision.score:.3f}")

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

    @staticmethod
    def _publish(publisher, text: str) -> None:
        msg = String()
        msg.data = text
        publisher.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Silero VAD stage for AMR voice input")
    parser.add_argument("--source", default=os.environ.get("AMR_VOICE_SOURCE", "laptop"))
    parser.add_argument(
        "--device",
        default=os.environ.get("AMR_VOICE_DEVICE", "auto"),
        help="Input device index/name, or 'auto' to choose the laptop digital mic.",
    )
    parser.add_argument("--sample-rate", type=int, default=0)
    parser.add_argument("--recognition-rate", type=int, default=16000)
    parser.add_argument("--channels", type=int, default=2)
    parser.add_argument("--frame-ms", type=int, default=30)
    parser.add_argument("--vad-frame-samples", type=int, default=480)
    parser.add_argument("--threshold", type=float, default=DEFAULT_VAD_THRESHOLD)
    parser.add_argument("--release-threshold", type=float, default=DEFAULT_VAD_RELEASE_THRESHOLD)
    parser.add_argument("--start-frames", type=int, default=DEFAULT_VAD_START_FRAMES)
    parser.add_argument("--end-frames", type=int, default=DEFAULT_VAD_END_FRAMES)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--log-scores", action="store_true")
    parser.add_argument("--log-audio-level", action="store_true")
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Initialize audio/model and log VAD events without changing robot state. This node never commands motion.",
    )
    return parser.parse_args()


def _load_sounddevice():
    try:
        import sounddevice as sd
    except Exception as exc:
        raise RuntimeError("Python package 'sounddevice' is not available in this container") from exc
    return sd


def main() -> None:
    args = parse_args()
    sd = _load_sounddevice()
    if args.list_devices:
        print(sd.query_devices())
        return

    audio_queue: "queue.Queue[bytes]" = queue.Queue()

    def callback(indata, frames, time_info, status):  # pragma: no cover - callback path
        audio_queue.put(bytes(indata))

    rclpy.init()
    node: Optional[VadNode] = None
    try:
        node = VadNode(args)
        device, device_info = select_input_device(sd, args.device)
        if args.sample_rate <= 0:
            args.sample_rate = int(device_info["default_samplerate"])
        frame_samples = max(1, int(args.sample_rate * args.frame_ms / 1000))
        rate_state = None
        start = time.monotonic()
        node.get_logger().info(
            f"vad listening source={args.source} threshold={args.threshold:.2f} "
            f"release_threshold={args.release_threshold:.2f} "
            f"device={device if device is not None else 'default'} sample_rate={args.sample_rate} "
            f"recognition_rate={args.recognition_rate} channels={args.channels}"
        )
        with sd.RawInputStream(
            samplerate=args.sample_rate,
            blocksize=frame_samples,
            device=parse_device(args.device) if args.device not in {None, "", "auto"} else device,
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
                node.process_audio(data)
                rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
