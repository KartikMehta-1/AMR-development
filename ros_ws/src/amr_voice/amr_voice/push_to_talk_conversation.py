from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
import wave
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from amr_voice.asr import FasterWhisperConfig, FasterWhisperTranscriber
from amr_voice.command_parser import UNKNOWN
from amr_voice.conversation import DEFAULT_KNOWN_PLACES, next_tool_for, plan_turn
from amr_voice.intent_router import (
    CANCEL as ROUTED_CANCEL,
    CONFIRM as ROUTED_CONFIRM,
    DIAGNOSTICS as ROUTED_DIAGNOSTICS,
    GENERAL_QUESTION as ROUTED_GENERAL_QUESTION,
    GO_TO_PLACE as ROUTED_GO_TO_PLACE,
    LIST_PLACES as ROUTED_LIST_PLACES,
    READ_STATUS as ROUTED_READ_STATUS,
    REJECT as ROUTED_REJECT,
    LocalIntentRouter,
    RoutedIntent,
)
from amr_voice.local_llm import LocalQwenResponder, qwen_responder_from_env
from amr_voice.tts import PiperSpeaker, SpeechRequest
from amr_voice.vad import (
    DEFAULT_VAD_END_FRAMES,
    DEFAULT_VAD_RELEASE_THRESHOLD,
    DEFAULT_VAD_START_FRAMES,
    DEFAULT_VAD_THRESHOLD,
    SileroVadDetector,
    VadGate,
)


@dataclass(frozen=True)
class AssistantReply:
    text: str
    llm_used: bool
    fallback_used: bool
    blockers: list[str]
    tool_result: dict | None = None
    intent: str | None = None
    pending_request: dict | None = None


def build_assistant_reply(
    text: str,
    *,
    llm: Optional[LocalQwenResponder] = None,
    known_places: Optional[list[str]] = None,
    tool_executor=None,
    intent_router=None,
    pending_request: dict | None = None,
) -> AssistantReply:
    places = known_places or list(DEFAULT_KNOWN_PLACES)
    turn = plan_turn(
        text,
        known_places=places,
        require_wake_word=False,
        dry_run=True,
    )
    if turn.command.get("action") != UNKNOWN:
        tool_result = None
        if tool_executor is not None and _read_only_tool_allowed(turn.next_tool):
            tool_result = tool_executor(turn.next_tool)
            return AssistantReply(
                text=_tool_reply(turn.assistant_response, tool_result),
                llm_used=False,
                fallback_used=False,
                blockers=list(turn.blockers) + list(tool_result.get("blockers", [])),
                tool_result=tool_result,
                intent=str(turn.command.get("action")),
            )
        return AssistantReply(
            text=turn.assistant_response,
            llm_used=False,
            fallback_used=False,
            blockers=list(turn.blockers),
            tool_result=tool_result,
            intent=str(turn.command.get("action")),
            pending_request=turn.next_tool if turn.requires_confirmation else None,
        )
    routed = _route_intent(intent_router, text, known_places=places, pending_request=pending_request)
    if routed is not None and routed.intent != ROUTED_GENERAL_QUESTION:
        routed_reply = _reply_for_routed_intent(
            routed,
            known_places=places,
            tool_executor=tool_executor,
            pending_request=pending_request,
        )
        if routed_reply is not None:
            return routed_reply
    if llm is not None:
        result = llm.respond(text)
        if result.ok:
            return AssistantReply(
                text=result.text,
                llm_used=True,
                fallback_used=False,
                blockers=[],
                tool_result=None,
                intent=routed.intent if routed is not None else ROUTED_GENERAL_QUESTION,
            )
    return AssistantReply(
        text=turn.assistant_response,
        llm_used=False,
        fallback_used=True,
        blockers=list(turn.blockers),
        tool_result=None,
        intent=routed.intent if routed is not None else None,
    )


def _route_intent(intent_router, text: str, *, known_places: list[str], pending_request: dict | None) -> RoutedIntent | None:
    if intent_router is None:
        return None
    route = intent_router.route(text, known_places=known_places, pending_request=pending_request)
    if route.confidence < 0.45:
        return None
    return route


def _reply_for_routed_intent(
    routed: RoutedIntent,
    *,
    known_places: list[str],
    tool_executor,
    pending_request: dict | None,
) -> AssistantReply | None:
    if routed.intent == ROUTED_CONFIRM:
        if pending_request is None:
            return AssistantReply(
                text="I heard the confirmation, but I need an active pending request before taking action.",
                llm_used=False,
                fallback_used=False,
                blockers=[],
                intent=routed.intent,
            )
        if _read_only_tool_allowed(pending_request) and tool_executor is not None:
            tool_result = tool_executor(pending_request)
            return AssistantReply(
                text=_tool_reply("I will proceed with the pending read-only request.", tool_result),
                llm_used=False,
                fallback_used=False,
                blockers=list(tool_result.get("blockers", [])),
                tool_result=tool_result,
                intent=routed.intent,
            )
        return AssistantReply(
            text="I heard the confirmation, but voice control will not start robot motion without supervised operator approval in the mission interface.",
            llm_used=False,
            fallback_used=False,
            blockers=["supervised_confirmation_required"],
            intent=routed.intent,
            pending_request=pending_request,
        )
    if routed.intent == ROUTED_REJECT:
        return AssistantReply(
            text="Understood. I will not continue with the pending request.",
            llm_used=False,
            fallback_used=False,
            blockers=[],
            intent=routed.intent,
        )
    next_tool = _next_tool_for_routed_intent(routed, known_places=known_places)
    if next_tool is not None and _read_only_tool_allowed(next_tool) and tool_executor is not None:
        tool_result = tool_executor(next_tool)
        return AssistantReply(
            text=_tool_reply(_prefix_for_routed_intent(routed), tool_result),
            llm_used=False,
            fallback_used=False,
            blockers=list(tool_result.get("blockers", [])),
            tool_result=tool_result,
            intent=routed.intent,
        )
    if routed.intent == ROUTED_GO_TO_PLACE:
        place = str(routed.arguments.get("place", "")).strip().lower()
        if place in set(known_places):
            return AssistantReply(
                text=f"I can help send the robot to {place}, but I need supervised confirmation before motion starts.",
                llm_used=False,
                fallback_used=False,
                blockers=[],
                intent=routed.intent,
                pending_request=next_tool,
            )
        return AssistantReply(
            text="I heard a motion request, but I could not map it to a known place.",
            llm_used=False,
            fallback_used=False,
            blockers=["unknown_place"],
            intent=routed.intent,
        )
    if routed.intent == ROUTED_CANCEL:
        return AssistantReply(
            text="I can request mission cancellation after confirmation.",
            llm_used=False,
            fallback_used=False,
            blockers=[],
            intent=routed.intent,
            pending_request=next_tool,
        )
    if routed.intent in {ROUTED_READ_STATUS, ROUTED_DIAGNOSTICS, ROUTED_LIST_PLACES}:
        return AssistantReply(
            text=_prefix_for_routed_intent(routed),
            llm_used=False,
            fallback_used=False,
            blockers=["read_only_tool_executor_unavailable"] if next_tool else ["no_safe_tool_route"],
            intent=routed.intent,
        )
    return None


def _next_tool_for_routed_intent(routed: RoutedIntent, *, known_places: list[str]) -> dict | None:
    if routed.intent == ROUTED_READ_STATUS:
        return {
            "server": "amr_state_inspection",
            "tool_plan": [
                {"tool": "get_robot_health", "arguments": {"require_localization": True}},
                {"tool": "get_mission_state", "arguments": {}},
                {"tool": "get_localization_state", "arguments": {}},
            ],
            "requires_operator_confirmation": False,
        }
    if routed.intent == ROUTED_DIAGNOSTICS:
        return next_tool_for("diagnose", None, dry_run=True)
    if routed.intent == ROUTED_LIST_PLACES:
        return next_tool_for("list_places", None, dry_run=True)
    if routed.intent == ROUTED_GO_TO_PLACE:
        place = str(routed.arguments.get("place", "")).strip().lower()
        if place in set(known_places):
            return next_tool_for("go_to", place, dry_run=True)
    if routed.intent == ROUTED_CANCEL:
        return next_tool_for("cancel", None, dry_run=True)
    return None


def _prefix_for_routed_intent(routed: RoutedIntent) -> str:
    if routed.intent == ROUTED_READ_STATUS:
        return "I checked the robot status."
    if routed.intent == ROUTED_DIAGNOSTICS:
        return "I checked the robot diagnostics."
    if routed.intent == ROUTED_LIST_PLACES:
        return "I checked the known places."
    return "I checked the request."


def _read_only_tool_allowed(next_tool: dict | None) -> bool:
    if not isinstance(next_tool, dict):
        return False
    if next_tool.get("requires_operator_confirmation"):
        return False
    if next_tool.get("server") == "amr_mission_control" and next_tool.get("tool") in {
        "get_mission_state",
        "list_named_places",
    }:
        return True
    if next_tool.get("server") == "amr_state_inspection" and isinstance(next_tool.get("tool_plan"), list):
        return True
    return False


def _tool_reply(prefix: str, tool_result: dict) -> str:
    if not tool_result.get("ok", False):
        message = str(tool_result.get("message", "tool call failed"))
        blockers = tool_result.get("blockers") or []
        if blockers:
            return f"{prefix} The read-only MCP call failed: {message}. Blockers: {', '.join(map(str, blockers))}."
        return f"{prefix} The read-only MCP call failed: {message}."
    summary = _summarize_tool_data(tool_result.get("data"))
    if summary:
        return f"{prefix} {summary}"
    return f"{prefix} The read-only MCP call completed successfully."


def _summarize_tool_data(data) -> str:
    if isinstance(data, dict):
        parts = []
        for key in ("state", "active_request", "goal_name", "current_place", "last_place"):
            value = data.get(key)
            if value not in (None, "", []):
                parts.append(f"{key.replace('_', ' ')}: {value}")
        if parts:
            return "Current status: " + "; ".join(parts) + "."
        if "places" in data:
            places = data.get("places")
            if isinstance(places, list):
                return "Known places: " + ", ".join(map(str, places)) + "."
    if isinstance(data, list):
        if data and all(isinstance(item, dict) and "tool" in item and "result" in item for item in data):
            parts = []
            for item in data:
                result = item.get("result") if isinstance(item.get("result"), dict) else {}
                if not result.get("ok", False):
                    parts.append(f"{item.get('tool')}: unavailable")
                    continue
                payload = result.get("data")
                if isinstance(payload, dict):
                    state = payload.get("state") or payload.get("current_place") or payload.get("last_place")
                    if state not in (None, "", []):
                        parts.append(f"{str(item.get('tool')).replace('_', ' ')}: {state}")
            if parts:
                return "Current status: " + "; ".join(parts) + "."
        return "Result: " + ", ".join(map(str, data[:8])) + "."
    if data not in (None, ""):
        return f"Result: {data}."
    return ""


class PushToTalkConversation:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.asr = FasterWhisperTranscriber(
            FasterWhisperConfig(
                model_path=args.faster_whisper_model,
                language=args.language,
                device=args.faster_whisper_device,
                compute_type=args.faster_whisper_compute_type,
                beam_size=args.faster_whisper_beam_size,
            )
        )
        self.llm = None if args.no_llm else qwen_responder_from_env()
        self.intent_router = None if args.no_intent_router else LocalIntentRouter.from_env()
        self.pending_request: dict | None = None
        self.speaker = PiperSpeaker(
            piper_bin=args.piper_bin,
            model_path=args.piper_model,
            output_device=args.tts_output_device,
            length_scale=args.tts_length_scale,
            sentence_silence=args.tts_sentence_silence,
            dry_run=args.no_tts,
        )

    def run(self) -> int:
        print("AMR faster-whisper conversation is ready.")
        print("Press Enter, speak one phrase, then pause. Type q then Enter to quit.")
        while True:
            answer = input("> ").strip().lower()
            if answer == "q":
                return 0
            wav_path = self.capture_utterance()
            if wav_path is None:
                print("no speech detected")
                continue
            transcript = self.asr.transcribe_wav(wav_path)
            if not transcript.text:
                print("transcript: <empty>")
                continue
            print(f"transcript: {transcript.text}")
            reply = build_assistant_reply(
                transcript.text,
                llm=self.llm,
                known_places=list(self.args.known_places),
                tool_executor=self._call_read_only_tool,
                intent_router=self.intent_router,
                pending_request=self.pending_request,
            )
            if reply.intent == ROUTED_REJECT:
                self.pending_request = None
            elif reply.pending_request is not None:
                self.pending_request = reply.pending_request
            elif reply.intent == ROUTED_CONFIRM and reply.tool_result is not None:
                self.pending_request = None
            print(f"assistant: {reply.text}")
            print(
                f"intent={reply.intent} llm_used={reply.llm_used} "
                f"fallback_used={reply.fallback_used} blockers={reply.blockers}"
            )
            speech = self.speaker.speak(
                SpeechRequest(
                    text=reply.text,
                    source="push_to_talk_conversation",
                    priority="normal",
                    interrupt=False,
                )
            )
            if not speech.ok:
                print(f"tts unavailable: {speech.message}")
            if self.args.post_tts_pause_sec > 0:
                time.sleep(self.args.post_tts_pause_sec)

    def _call_read_only_tool(self, next_tool: dict) -> dict:
        caller = McpToolCaller(repo_root=self.args.repo_root)
        if next_tool.get("server") == "amr_state_inspection":
            results = []
            ok = True
            blockers: list[str] = []
            for item in next_tool.get("tool_plan", []):
                result = caller.call("amr_state_inspection", str(item["tool"]), dict(item.get("arguments") or {}))
                results.append({"tool": item["tool"], "result": result})
                ok = ok and bool(result.get("ok", False))
                blockers.extend(result.get("blockers") or [])
            return {
                "ok": ok,
                "message": "diagnostics completed" if ok else "diagnostics had blockers",
                "data": results,
                "blockers": sorted(set(map(str, blockers))),
            }
        return caller.call(
            str(next_tool["server"]),
            str(next_tool["tool"]),
            dict(next_tool.get("arguments") or {}),
        )

    def capture_utterance(self) -> Optional[Path]:
        recorder = AlsaVadRecorder(
            alsa_device=self.args.alsa_device,
            sample_rate=self.args.sample_rate,
            frame_ms=self.args.frame_ms,
            pre_roll_sec=self.args.pre_roll_sec,
            no_speech_timeout_sec=self.args.no_speech_timeout_sec,
            max_utterance_sec=self.args.max_utterance_sec,
            vad_threshold=self.args.vad_threshold,
            vad_release_threshold=self.args.vad_release_threshold,
            vad_start_frames=self.args.vad_start_frames,
            vad_end_frames=self.args.vad_end_frames,
            vad_frame_samples=self.args.vad_frame_samples,
            vad_threads=self.args.vad_threads,
            output_dir=self.args.output_dir,
            log_audio_level=self.args.log_audio_level,
        )
        return recorder.record()


class AlsaVadRecorder:
    def __init__(
        self,
        *,
        alsa_device: str,
        sample_rate: int,
        frame_ms: int,
        pre_roll_sec: float,
        no_speech_timeout_sec: float,
        max_utterance_sec: float,
        vad_threshold: float,
        vad_release_threshold: float,
        vad_start_frames: int,
        vad_end_frames: int,
        vad_frame_samples: int,
        vad_threads: int,
        output_dir: Path,
        log_audio_level: bool,
    ):
        self.alsa_device = alsa_device
        self.sample_rate = sample_rate
        self.frame_ms = frame_ms
        self.pre_roll_sec = pre_roll_sec
        self.no_speech_timeout_sec = no_speech_timeout_sec
        self.max_utterance_sec = max_utterance_sec
        self.vad_threshold = vad_threshold
        self.vad_release_threshold = vad_release_threshold
        self.vad_start_frames = vad_start_frames
        self.vad_end_frames = vad_end_frames
        self.vad_frame_samples = vad_frame_samples
        self.vad_threads = vad_threads
        self.output_dir = output_dir
        self.log_audio_level = log_audio_level

    def record(self) -> Optional[Path]:
        vad = SileroVadDetector(n_threads=self.vad_threads)
        gate = VadGate(
            threshold=self.vad_threshold,
            release_threshold=self.vad_release_threshold,
            start_frames=self.vad_start_frames,
            end_frames=self.vad_end_frames,
        )
        frame_samples = max(1, int(self.sample_rate * self.frame_ms / 1000))
        frame_bytes = frame_samples * 2
        pre_roll_frames = max(1, int(self.pre_roll_sec / max(0.001, self.frame_ms / 1000.0)))
        pre_roll: deque[bytes] = deque(maxlen=pre_roll_frames)
        utterance_frames: list[bytes] = []
        speech_started = False
        start = time.monotonic()
        last_audio_log = 0.0
        command = [
            "arecord",
            "-q",
            "-D",
            self.alsa_device,
            "-f",
            "S16_LE",
            "-r",
            str(self.sample_rate),
            "-c",
            "1",
            "-t",
            "raw",
        ]
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            if process.stdout is None:
                raise RuntimeError("arecord stdout unavailable")
            while True:
                now = time.monotonic()
                data = process.stdout.read(frame_bytes)
                if not data:
                    stderr = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
                    raise RuntimeError(f"arecord stopped unexpectedly: {stderr.strip()}")
                pre_roll.append(data)
                if speech_started:
                    utterance_frames.append(data)
                else:
                    utterance_frames = list(pre_roll)
                if self.log_audio_level and now - last_audio_log >= 1.0:
                    last_audio_log = now
                    print(_audio_level(data))
                decision = gate.update(vad.score(data, frame_size=self.vad_frame_samples))
                if decision.event == "speech_started":
                    speech_started = True
                    utterance_frames = list(pre_roll)
                    print(f"vad: speech_started score={decision.score:.3f}")
                elif decision.event == "speech_ended" and speech_started:
                    print(f"vad: speech_ended score={decision.score:.3f}")
                    break
                elapsed = now - start
                if speech_started and elapsed >= self.max_utterance_sec:
                    print("vad: max_utterance_sec")
                    break
                if not speech_started and elapsed >= self.no_speech_timeout_sec:
                    print("vad: no_speech_timeout_sec")
                    return None
        finally:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
        return self._write_wav(utterance_frames)

    def _write_wav(self, frames: list[bytes]) -> Path:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        fd, path = tempfile.mkstemp(prefix="amr_ptt_", suffix=".wav", dir=str(self.output_dir))
        os.close(fd)
        wav_path = Path(path)
        with wave.open(str(wav_path), "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(self.sample_rate)
            wav.writeframes(b"".join(frames))
        return wav_path


def _audio_level(data: bytes) -> str:
    if not data:
        return "audio: rms=0 peak=0"
    samples = [
        int.from_bytes(data[index : index + 2], byteorder="little", signed=True)
        for index in range(0, len(data) - 1, 2)
    ]
    if not samples:
        return "audio: rms=0 peak=0"
    rms = int((sum(sample * sample for sample in samples) / len(samples)) ** 0.5)
    peak = max(abs(sample) for sample in samples)
    return f"audio: rms={rms} peak={peak}"


class McpToolCaller:
    SERVER_PATHS = {
        "amr_mission_control": "mcp_servers/amr_mission_control/server.py",
        "amr_state_inspection": "mcp_servers/amr_state_inspection/server.py",
    }

    def __init__(self, repo_root: str):
        self.repo_root = Path(repo_root)

    def call(self, server: str, tool_name: str, arguments: dict) -> dict:
        if server not in self.SERVER_PATHS:
            return {
                "ok": False,
                "message": f"unsupported MCP server for voice runner: {server}",
                "data": None,
                "blockers": ["unsupported_mcp_server"],
            }
        server_path = self.repo_root / self.SERVER_PATHS[server]
        proc = subprocess.Popen(
            [sys.executable, str(server_path)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(self.repo_root),
        )
        assert proc.stdin is not None
        assert proc.stdout is not None
        try:
            proc.stdin.write(
                _encode_mcp(
                    {
                        "jsonrpc": "2.0",
                        "id": 1,
                        "method": "initialize",
                        "params": {
                            "protocolVersion": "2024-11-05",
                            "capabilities": {},
                            "clientInfo": {"name": "amr-voice-ptt", "version": "0.1.0"},
                        },
                    }
                )
            )
            proc.stdin.write(_encode_mcp({"jsonrpc": "2.0", "method": "notifications/initialized"}))
            proc.stdin.write(
                _encode_mcp(
                    {
                        "jsonrpc": "2.0",
                        "id": 2,
                        "method": "tools/call",
                        "params": {"name": tool_name, "arguments": arguments},
                    }
                )
            )
            proc.stdin.flush()
            _read_mcp_response(proc.stdout)
            response = _read_mcp_response(proc.stdout)
            result = response["result"]
            return json.loads(result["content"][0]["text"])
        except Exception as exc:
            return {
                "ok": False,
                "message": f"MCP call failed: {exc}",
                "data": None,
                "blockers": ["mcp_call_failed"],
            }
        finally:
            proc.kill()
            proc.wait(timeout=5)


def _encode_mcp(message: dict) -> bytes:
    body = json.dumps(message).encode("utf-8")
    return f"Content-Length: {len(body)}\r\n\r\n".encode("utf-8") + body


def _read_mcp_response(stream) -> dict:
    headers: dict[str, str] = {}
    while True:
        line = stream.readline()
        if line == b"":
            raise RuntimeError("MCP server closed stdout")
        text = line.decode("utf-8").strip()
        if not text:
            break
        key, value = text.split(":", 1)
        headers[key.lower()] = value.strip()
    return json.loads(stream.read(int(headers["content-length"])).decode("utf-8"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Half-duplex AMR push-to-talk conversation loop")
    parser.add_argument("--alsa-device", default=os.environ.get("AMR_VOICE_ALSA_DEVICE", "plughw:CARD=sofhdadsp,DEV=7"))
    parser.add_argument("--sample-rate", type=int, default=int(os.environ.get("AMR_VOICE_SAMPLE_RATE", "16000")))
    parser.add_argument("--frame-ms", type=int, default=int(os.environ.get("AMR_VOICE_FRAME_MS", "80")))
    parser.add_argument("--pre-roll-sec", type=float, default=float(os.environ.get("AMR_VOICE_PRE_ROLL_SEC", "0.4")))
    parser.add_argument("--no-speech-timeout-sec", type=float, default=float(os.environ.get("AMR_VOICE_NO_SPEECH_TIMEOUT_SEC", "8")))
    parser.add_argument("--max-utterance-sec", type=float, default=float(os.environ.get("AMR_VOICE_MAX_UTTERANCE_SEC", "14")))
    parser.add_argument("--vad-threshold", type=float, default=DEFAULT_VAD_THRESHOLD)
    parser.add_argument("--vad-release-threshold", type=float, default=DEFAULT_VAD_RELEASE_THRESHOLD)
    parser.add_argument("--vad-start-frames", type=int, default=DEFAULT_VAD_START_FRAMES)
    parser.add_argument("--vad-end-frames", type=int, default=DEFAULT_VAD_END_FRAMES)
    parser.add_argument("--vad-frame-samples", type=int, default=480)
    parser.add_argument("--vad-threads", type=int, default=1)
    parser.add_argument("--faster-whisper-model", default=os.environ.get("AMR_FASTER_WHISPER_MODEL_DIR", "/workspaces/AMR-development/models/faster-whisper/small.en"))
    parser.add_argument("--faster-whisper-device", default=os.environ.get("AMR_FASTER_WHISPER_DEVICE", "cpu"))
    parser.add_argument("--faster-whisper-compute-type", default=os.environ.get("AMR_FASTER_WHISPER_COMPUTE_TYPE", "int8"))
    parser.add_argument("--faster-whisper-beam-size", type=int, default=int(os.environ.get("AMR_FASTER_WHISPER_BEAM_SIZE", "5")))
    parser.add_argument("--language", default=os.environ.get("AMR_WHISPER_LANGUAGE", "en"))
    parser.add_argument("--known-place", action="append", dest="known_places", default=list(DEFAULT_KNOWN_PLACES))
    parser.add_argument("--piper-bin", default=os.environ.get("AMR_PIPER_BIN", "/workspaces/AMR-development/models/piper-runtime/piper"))
    parser.add_argument("--piper-model", default=os.environ.get("AMR_PIPER_MODEL", "/workspaces/AMR-development/models/piper/en_US-lessac-medium/en_US-lessac-medium.onnx"))
    parser.add_argument("--tts-output-device", default=os.environ.get("AMR_TTS_OUTPUT_DEVICE", "plughw:CARD=sofhdadsp,DEV=0"))
    parser.add_argument("--tts-length-scale", type=float, default=float(os.environ.get("AMR_TTS_LENGTH_SCALE", "1.05")))
    parser.add_argument("--tts-sentence-silence", type=float, default=float(os.environ.get("AMR_TTS_SENTENCE_SILENCE", "0.2")))
    parser.add_argument("--post-tts-pause-sec", type=float, default=float(os.environ.get("AMR_POST_TTS_PAUSE_SEC", "0.8")))
    parser.add_argument("--output-dir", type=Path, default=Path(os.environ.get("AMR_VOICE_OUTPUT_DIR", "/tmp/amr_voice_pipeline")))
    parser.add_argument("--repo-root", default=os.environ.get("AMR_REPO_ROOT", "/workspaces/AMR-development"))
    parser.add_argument("--no-llm", action="store_true")
    parser.add_argument("--no-intent-router", action="store_true")
    parser.add_argument("--no-tts", action="store_true")
    parser.add_argument("--log-audio-level", action="store_true")
    return parser.parse_args()


def main() -> int:
    return PushToTalkConversation(parse_args()).run()


if __name__ == "__main__":
    raise SystemExit(main())
