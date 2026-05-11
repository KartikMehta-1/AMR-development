from __future__ import annotations

import json
import re
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


MAX_SPEECH_CHARS = 280


@dataclass(frozen=True)
class SpeechRequest:
    text: str
    source: str = "amr"
    priority: str = "normal"
    interrupt: bool = False


@dataclass(frozen=True)
class TtsResult:
    ok: bool
    spoken: bool
    message: str
    text: str
    engine: str
    blockers: list[str]


def sanitize_speech_text(text: str, max_chars: int = MAX_SPEECH_CHARS) -> str:
    cleaned = re.sub(r"\s+", " ", str(text)).strip()
    cleaned = cleaned.replace("\x00", "")
    if len(cleaned) > max_chars:
        return cleaned[: max(0, max_chars - 1)].rstrip() + "."
    return cleaned


def speech_request_from_json(payload: str) -> SpeechRequest:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return SpeechRequest(text=sanitize_speech_text(payload), source="plain_text")
    if not isinstance(data, dict):
        return SpeechRequest(text=sanitize_speech_text(payload), source="plain_text")
    return SpeechRequest(
        text=sanitize_speech_text(str(data.get("text", ""))),
        source=str(data.get("source", "amr")),
        priority=str(data.get("priority", "normal")),
        interrupt=bool(data.get("interrupt", False)),
    )


class PiperSpeaker:
    def __init__(
        self,
        piper_bin: str = "piper",
        model_path: str = "",
        speaker: Optional[int] = None,
        output_device: str = "",
        length_scale: Optional[float] = None,
        noise_scale: Optional[float] = None,
        noise_w_scale: Optional[float] = None,
        sentence_silence: Optional[float] = None,
        volume: Optional[float] = None,
        dry_run: bool = False,
    ):
        self.piper_bin = piper_bin
        self.model_path = model_path
        self.speaker = speaker
        self.output_device = output_device
        self.length_scale = length_scale
        self.noise_scale = noise_scale
        self.noise_w_scale = noise_w_scale
        self.sentence_silence = sentence_silence
        self.volume = volume
        self.dry_run = dry_run

    def status(self) -> dict:
        return {
            "engine": "piper",
            "piper_bin": self.piper_bin,
            "piper_available": self._bin_available(self.piper_bin),
            "model_path": self.model_path,
            "model_available": bool(self.model_path) and Path(self.model_path).expanduser().is_file(),
            "aplay_available": self._bin_available("aplay"),
            "output_device": self.output_device or "default",
            "voice_tuning": {
                "length_scale": self.length_scale,
                "noise_scale": self.noise_scale,
                "noise_w_scale": self.noise_w_scale,
                "sentence_silence": self.sentence_silence,
                "volume": self.volume,
            },
            "dry_run": self.dry_run,
        }

    def speak(self, request: SpeechRequest) -> TtsResult:
        text = sanitize_speech_text(request.text)
        if not text:
            return TtsResult(False, False, "empty speech request", text, "piper", ["empty_text"])
        if self.dry_run:
            return TtsResult(True, False, "dry-run speech request accepted", text, "piper", [])
        blockers = self._blockers()
        if blockers:
            return TtsResult(False, False, "Piper TTS is not ready", text, "piper", blockers)
        with tempfile.TemporaryDirectory(prefix="amr_tts_") as tmp_dir:
            wav_path = Path(tmp_dir) / "speech.wav"
            command = [
                self.piper_bin,
                "--model",
                str(Path(self.model_path).expanduser()),
                "--output_file",
                str(wav_path),
            ]
            if self.speaker is not None:
                command.extend(["--speaker", str(self.speaker)])
            for option, value in [
                ("--length-scale", self.length_scale),
                ("--noise-scale", self.noise_scale),
                ("--noise-w-scale", self.noise_w_scale),
                ("--sentence-silence", self.sentence_silence),
                ("--volume", self.volume),
            ]:
                if value is not None:
                    command.extend([option, str(value)])
            synth = subprocess.run(
                command,
                input=text,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            if synth.returncode != 0:
                return TtsResult(
                    False,
                    False,
                    f"piper failed with code {synth.returncode}: {(synth.stderr or synth.stdout).strip()}",
                    text,
                    "piper",
                    ["piper_synthesis_failed"],
                )
            play_command = ["aplay"]
            if self.output_device:
                play_command.extend(["-D", self.output_device])
            play_command.append(str(wav_path))
            play = subprocess.run(play_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
            if play.returncode != 0:
                return TtsResult(
                    False,
                    False,
                    f"audio playback failed with code {play.returncode}: {(play.stderr or play.stdout).strip()}",
                    text,
                    "piper",
                    ["audio_playback_failed"],
                )
        return TtsResult(True, True, "speech played", text, "piper", [])

    def _blockers(self) -> list[str]:
        blockers = []
        if not self._bin_available(self.piper_bin):
            blockers.append("piper_binary_missing")
        if not self.model_path:
            blockers.append("piper_model_missing")
        elif not Path(self.model_path).expanduser().is_file():
            blockers.append("piper_model_not_found")
        if not self._bin_available("aplay"):
            blockers.append("aplay_missing")
        return blockers

    @staticmethod
    def _bin_available(binary: str) -> bool:
        return bool(shutil.which(binary) or Path(binary).expanduser().exists())
