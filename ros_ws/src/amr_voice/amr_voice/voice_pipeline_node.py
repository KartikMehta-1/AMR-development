from __future__ import annotations

import argparse
import audioop
import json
import os
import queue
import struct
import time
import wave
from collections import deque
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from amr_voice.asr import (
    WhisperCppConfig,
    WhisperCppTranscriber,
    VoskConfig,
    VoskGrammarTranscriber,
    build_mcp_transcript_payload,
    default_command_grammar,
    default_whisper_cpp_config,
)
from amr_voice.audio_devices import parse_device, select_input_device
from amr_voice.vad import (
    DEFAULT_VAD_END_FRAMES,
    DEFAULT_VAD_RELEASE_THRESHOLD,
    DEFAULT_VAD_START_FRAMES,
    DEFAULT_VAD_THRESHOLD,
    SileroVadDetector,
    VadGate,
)
from amr_voice.wake_word import DEFAULT_WAKE_MODEL, DEFAULT_WAKE_THRESHOLD, OpenWakeWordDetector


class VoicePipelineNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__(f"amr_voice_pipeline_{os.getpid()}")
        self._args = args
        self._wake = OpenWakeWordDetector(
            model_name=args.wake_model,
            threshold=args.wake_threshold,
            inference_framework=args.inference_framework,
            enable_speex_noise_suppression=args.speex_noise_suppression,
        )
        self._vad = SileroVadDetector(n_threads=args.vad_threads)
        self._gate = VadGate(
            threshold=args.vad_threshold,
            release_threshold=args.vad_release_threshold,
            start_frames=args.vad_start_frames,
            end_frames=args.vad_end_frames,
        )
        if args.asr_backend == "vosk":
            self._asr = VoskGrammarTranscriber(
                VoskConfig(
                    model_path=args.vosk_model,
                    sample_rate=args.recognition_rate,
                    grammar=default_command_grammar(args.known_places, args.wake_phrase),
                )
            )
        else:
            self._asr = WhisperCppTranscriber(
                WhisperCppConfig(
                    executable=args.whisper_bin,
                    model_path=args.whisper_model,
                    language=args.whisper_language,
                    threads=args.whisper_threads,
                )
            )
        self._wake_pub = self.create_publisher(String, "/amr_voice/wake_word", 10)
        self._vad_pub = self.create_publisher(String, "/amr_voice/vad", 10)
        self._transcript_pub = self.create_publisher(String, "/amr_voice/transcript", 10)
        self._mcp_pub = self.create_publisher(String, "/amr_voice/mcp_arguments", 10)
        self._feedback_pub = self.create_publisher(String, "/amr_voice/feedback", 10)
        self._pre_roll: deque[bytes] = deque(maxlen=max(1, args.pre_roll_frames))
        self._utterance_frames: list[bytes] = []
        self._listening = False
        self._speech_started = False
        self._listen_started_at = 0.0
        self._last_wake_at = 0.0
        self._last_audio_log = 0.0
        if args.start_listening:
            self._listening = True
            self._listen_started_at = time.monotonic()
            self.get_logger().info("start-listening mode: wake detection bypassed for this dry run")

    def process_audio(self, data: bytes, sample_rate: int) -> None:
        self._pre_roll.append(data)
        now = time.monotonic()

        wake = self._wake.process(data)
        if self._args.log_scores:
            self.get_logger().info(
                f"wake score model={wake.model} score={wake.score:.3f} threshold={wake.threshold:.3f}"
            )
        if wake.detected and now - self._last_wake_at >= self._args.wake_cooldown_sec:
            self._last_wake_at = now
            self._listening = True
            self._speech_started = False
            self._gate = VadGate(
                threshold=self._args.vad_threshold,
                release_threshold=self._args.vad_release_threshold,
                start_frames=self._args.vad_start_frames,
                end_frames=self._args.vad_end_frames,
            )
            self._utterance_frames = list(self._pre_roll)
            payload = {
                "event": "wake_word_detected",
                "model": wake.model,
                "score": wake.score,
                "threshold": wake.threshold,
                "source": self._args.source,
                "timestamp_monotonic_sec": now,
            }
            self._publish(self._wake_pub, payload)
            self._publish_text(self._feedback_pub, f"wake word detected: {wake.model}")
            self.get_logger().info(f"wake word detected score={wake.score:.3f}")
            self._listen_started_at = now

        if not self._listening:
            return

        if not self._utterance_frames or self._utterance_frames[-1] != data:
            self._utterance_frames.append(data)

        vad_score = self._vad.score(data, frame_size=self._args.vad_frame_samples)
        vad = self._gate.update(vad_score)
        if self._args.log_scores:
            self.get_logger().info(f"vad score={vad.score:.3f} active={vad.speech_active}")
        if vad.event:
            payload = {
                "event": vad.event,
                "score": vad.score,
                "threshold": vad.threshold,
                "release_threshold": vad.release_threshold,
                "source": self._args.source,
                "timestamp_monotonic_sec": now,
            }
            self._publish(self._vad_pub, payload)
            self.get_logger().info(f"vad event={vad.event} score={vad.score:.3f}")
            if vad.event == "speech_started":
                self._speech_started = True
            elif vad.event == "speech_ended" and self._speech_started:
                self._finalize_utterance(sample_rate, reason="vad_speech_ended")
                return

        elapsed = now - self._listen_started_at
        if elapsed >= self._args.max_utterance_sec:
            self._finalize_utterance(sample_rate, reason="max_utterance_sec")
        elif not self._speech_started and elapsed >= self._args.no_speech_timeout_sec:
            self._finalize_utterance(sample_rate, reason="no_speech_timeout_sec")

    def log_audio_level(self, data: bytes) -> None:
        now = time.monotonic()
        if not self._args.log_audio_level or now - self._last_audio_log < 1.0:
            return
        self._last_audio_log = now
        if not data:
            self.get_logger().info("audio: rms=0 peak=0")
            return
        samples = struct.unpack(f"<{len(data) // 2}h", data)
        rms = int((sum(sample * sample for sample in samples) / len(samples)) ** 0.5) if samples else 0
        peak = max((abs(sample) for sample in samples), default=0)
        self.get_logger().info(f"audio: rms={rms} peak={peak}")

    def _finalize_utterance(self, sample_rate: int, *, reason: str) -> None:
        frames = self._utterance_frames
        self._utterance_frames = []
        self._listening = False
        self._speech_started = False
        if not frames:
            return
        self._args.output_dir.mkdir(parents=True, exist_ok=True)
        wav_path = self._args.output_dir / f"utterance_{int(time.time() * 1000)}.wav"
        with wave.open(str(wav_path), "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(sample_rate)
            wav.writeframes(b"".join(frames))
        self.get_logger().info(f"transcribing {wav_path} reason={reason}")
        try:
            transcript = self._asr.transcribe_wav(wav_path)
        except Exception as exc:
            self.get_logger().error(f"ASR failed: {exc}")
            self._publish_text(self._feedback_pub, f"asr failed: {exc}")
            return
        payload = build_mcp_transcript_payload(
            transcript,
            source=self._args.source,
            wake_word=self._args.wake_phrase,
            require_wake_word=False,
            known_places=self._args.known_places,
        )
        transcript_event = {
            "event": "asr_transcript",
            "text": transcript.text,
            "wav_path": str(wav_path),
            "source": self._args.source,
            "reason": reason,
        }
        self._publish(self._transcript_pub, transcript_event)
        self._publish(self._mcp_pub, payload)
        self._publish_text(self._feedback_pub, f"transcript: {transcript.text}")
        print(json.dumps({"transcript": transcript.text, "mcp_arguments": payload}, sort_keys=True), flush=True)

    @staticmethod
    def _publish(publisher, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        publisher.publish(msg)

    @staticmethod
    def _publish_text(publisher, text: str) -> None:
        msg = String()
        msg.data = text
        publisher.publish(msg)


def parse_args() -> argparse.Namespace:
    defaults = default_whisper_cpp_config()
    parser = argparse.ArgumentParser(description="Wake -> VAD -> whisper.cpp transcript pipeline")
    parser.add_argument("--source", default=os.environ.get("AMR_VOICE_SOURCE", "laptop_transcript"))
    parser.add_argument("--device", default=os.environ.get("AMR_VOICE_DEVICE", "auto"))
    parser.add_argument("--sample-rate", type=int, default=0)
    parser.add_argument("--recognition-rate", type=int, default=16000)
    parser.add_argument("--channels", type=int, default=2)
    parser.add_argument("--frame-ms", type=int, default=80)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--wake-model", default=os.environ.get("AMR_WAKE_MODEL", DEFAULT_WAKE_MODEL))
    parser.add_argument("--wake-phrase", default=os.environ.get("AMR_VOICE_WAKE_WORD", "hey jarvis"))
    parser.add_argument("--wake-threshold", type=float, default=DEFAULT_WAKE_THRESHOLD)
    parser.add_argument("--wake-cooldown-sec", type=float, default=2.0)
    parser.add_argument("--inference-framework", choices=["onnx", "tflite"], default="onnx")
    parser.add_argument("--speex-noise-suppression", action="store_true")
    parser.add_argument("--vad-threshold", type=float, default=DEFAULT_VAD_THRESHOLD)
    parser.add_argument("--vad-release-threshold", type=float, default=DEFAULT_VAD_RELEASE_THRESHOLD)
    parser.add_argument("--vad-start-frames", type=int, default=DEFAULT_VAD_START_FRAMES)
    parser.add_argument("--vad-end-frames", type=int, default=DEFAULT_VAD_END_FRAMES)
    parser.add_argument("--vad-frame-samples", type=int, default=480)
    parser.add_argument("--vad-threads", type=int, default=1)
    parser.add_argument("--pre-roll-sec", type=float, default=0.4)
    parser.add_argument("--no-speech-timeout-sec", type=float, default=3.0)
    parser.add_argument("--max-utterance-sec", type=float, default=8.0)
    parser.add_argument(
        "--start-listening",
        action="store_true",
        help="Bypass wake detection and immediately capture one utterance for VAD/ASR dry-run testing.",
    )
    parser.add_argument("--whisper-bin", default=defaults.executable)
    parser.add_argument("--whisper-model", default=defaults.model_path)
    parser.add_argument("--whisper-language", default=defaults.language)
    parser.add_argument("--whisper-threads", type=int, default=defaults.threads)
    parser.add_argument("--asr-backend", choices=["vosk", "whisper"], default=os.environ.get("AMR_ASR_BACKEND", "vosk"))
    parser.add_argument("--vosk-model", default=os.environ.get("AMR_VOSK_MODEL", "/workspaces/AMR-development/models/vosk-model-small-en-us-0.15"))
    parser.add_argument("--known-place", action="append", dest="known_places", default=["home", "hall", "kitchen", "door"])
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/amr_voice_pipeline"))
    parser.add_argument("--log-audio-level", action="store_true")
    parser.add_argument("--log-scores", action="store_true")
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="This node always runs dry-run and never commands motion.")
    args = parser.parse_args()
    frame_sec = max(0.001, args.frame_ms / 1000.0)
    args.pre_roll_frames = max(1, int(args.pre_roll_sec / frame_sec))
    return args


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
    node: Optional[VoicePipelineNode] = None
    try:
        node = VoicePipelineNode(args)
        device, device_info = select_input_device(sd, args.device)
        if args.sample_rate <= 0:
            args.sample_rate = int(device_info["default_samplerate"])
        frame_samples = max(1, int(args.sample_rate * args.frame_ms / 1000))
        rate_state = None
        start = time.monotonic()
        node.get_logger().info(
            f"voice pipeline listening device={device if device is not None else 'default'} "
            f"sample_rate={args.sample_rate} recognition_rate={args.recognition_rate} "
            f"channels={args.channels} wake_model={args.wake_model}"
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
                node.process_audio(data, sample_rate=args.recognition_rate)
                rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
