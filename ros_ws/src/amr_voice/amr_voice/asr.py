from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence


DEFAULT_WHISPER_MODEL = "/workspaces/AMR-development/models/whisper/ggml-base.en.bin"
DEFAULT_WHISPER_LANGUAGE = "en"


@dataclass(frozen=True)
class AsrTranscript:
    text: str
    wav_path: str
    model_path: str
    executable: str
    language: str


@dataclass(frozen=True)
class WhisperCppConfig:
    executable: str
    model_path: str
    language: str = DEFAULT_WHISPER_LANGUAGE
    threads: int = 4
    extra_args: tuple[str, ...] = ()


def default_whisper_cpp_config() -> WhisperCppConfig:
    executable = os.environ.get("AMR_WHISPER_CPP_BIN") or find_whisper_cpp_executable()
    model_path = os.environ.get("AMR_WHISPER_MODEL", DEFAULT_WHISPER_MODEL)
    language = os.environ.get("AMR_WHISPER_LANGUAGE", DEFAULT_WHISPER_LANGUAGE)
    threads = int(os.environ.get("AMR_WHISPER_THREADS", "4"))
    extra_args = tuple(arg for arg in os.environ.get("AMR_WHISPER_EXTRA_ARGS", "").split() if arg)
    return WhisperCppConfig(
        executable=executable,
        model_path=model_path,
        language=language,
        threads=threads,
        extra_args=extra_args,
    )


def find_whisper_cpp_executable() -> str:
    for name in ("whisper-cli", "whisper-cpp", "main"):
        found = shutil.which(name)
        if found:
            return found
    return "whisper-cli"


class WhisperCppTranscriber:
    def __init__(
        self,
        config: Optional[WhisperCppConfig] = None,
        runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
    ):
        self.config = config or default_whisper_cpp_config()
        self._runner = runner

    def transcribe_wav(self, wav_path: str | Path) -> AsrTranscript:
        wav = Path(wav_path).expanduser()
        if not wav.exists():
            raise FileNotFoundError(f"ASR input WAV not found: {wav}")
        model = Path(self.config.model_path).expanduser()
        if not model.exists():
            raise FileNotFoundError(
                f"Whisper model not found: {model}. Set AMR_WHISPER_MODEL to a whisper.cpp ggml model."
            )
        executable = self.config.executable
        if shutil.which(executable) is None and not Path(executable).exists():
            raise FileNotFoundError(
                f"whisper.cpp executable not found: {executable}. Set AMR_WHISPER_CPP_BIN."
            )

        with tempfile.TemporaryDirectory(prefix="amr_asr_") as tmp_dir:
            output_base = Path(tmp_dir) / "transcript"
            command = self._build_command(wav, model, output_base)
            result = self._runner(
                command,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            if result.returncode != 0:
                stderr = (result.stderr or result.stdout or "").strip()
                raise RuntimeError(f"whisper.cpp failed with code {result.returncode}: {stderr}")
            transcript_path = output_base.with_suffix(".txt")
            text = transcript_path.read_text(encoding="utf-8").strip()
        return AsrTranscript(
            text=normalize_transcript(text),
            wav_path=str(wav),
            model_path=str(model),
            executable=executable,
            language=self.config.language,
        )

    def _build_command(self, wav: Path, model: Path, output_base: Path) -> list[str]:
        command = [
            self.config.executable,
            "-m",
            str(model),
            "-f",
            str(wav),
            "-l",
            self.config.language,
            "-t",
            str(self.config.threads),
            "-nt",
            "-otxt",
            "-of",
            str(output_base),
        ]
        command.extend(self.config.extra_args)
        return command


def normalize_transcript(text: str) -> str:
    return " ".join(text.strip().split())


def build_mcp_transcript_payload(
    transcript: AsrTranscript,
    *,
    source: str = "laptop_transcript",
    wake_word: str = "hey jarvis",
    require_wake_word: bool = True,
    known_places: Optional[Sequence[str]] = None,
) -> dict:
    payload = {
        "text": transcript.text,
        "source": source,
        "wake_word": wake_word,
        "require_wake_word": require_wake_word,
        "dry_run": True,
    }
    if known_places is not None:
        payload["known_places"] = list(known_places)
    return payload
