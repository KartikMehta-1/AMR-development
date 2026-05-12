from __future__ import annotations

import os
import json
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence


DEFAULT_WHISPER_MODEL = "/workspaces/AMR-development/models/whisper/ggml-base.en.bin"
DEFAULT_WHISPER_LANGUAGE = "en"
DEFAULT_FASTER_WHISPER_MODEL = "/workspaces/AMR-development/models/faster-whisper/small.en"
DEFAULT_VOSK_MODEL = "/workspaces/AMR-development/models/vosk-model-small-en-us-0.15"


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


@dataclass(frozen=True)
class VoskConfig:
    model_path: str = DEFAULT_VOSK_MODEL
    sample_rate: int = 16000
    grammar: tuple[str, ...] = ()


@dataclass(frozen=True)
class FasterWhisperConfig:
    model_path: str = DEFAULT_FASTER_WHISPER_MODEL
    language: str = DEFAULT_WHISPER_LANGUAGE
    device: str = "cpu"
    compute_type: str = "int8"
    beam_size: int = 5


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
                env=self._build_env(),
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

    def _build_env(self) -> dict[str, str]:
        env = dict(os.environ)
        executable = Path(self.config.executable).expanduser()
        build_dir = executable.parents[1] if len(executable.parents) > 1 else executable.parent
        library_paths = [
            build_dir / "src",
            build_dir / "ggml" / "src",
        ]
        existing = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = ":".join(
            [str(path) for path in library_paths if path.exists()] + ([existing] if existing else [])
        )
        return env


class FasterWhisperTranscriber:
    def __init__(self, config: Optional[FasterWhisperConfig] = None):
        self.config = config or FasterWhisperConfig()
        self._model = None

    def transcribe_wav(self, wav_path: str | Path) -> AsrTranscript:
        try:
            from faster_whisper import WhisperModel
        except Exception as exc:
            raise RuntimeError("Python package 'faster-whisper' is not available in this runtime") from exc

        wav = Path(wav_path).expanduser()
        if not wav.exists():
            raise FileNotFoundError(f"ASR input WAV not found: {wav}")
        model_path = Path(self.config.model_path).expanduser()
        if not model_path.exists():
            raise FileNotFoundError(f"faster-whisper model not found: {model_path}")

        if self._model is None:
            self._model = WhisperModel(
                str(model_path),
                device=self.config.device,
                compute_type=self.config.compute_type,
            )
        segments, _info = self._model.transcribe(
            str(wav),
            language=self.config.language,
            beam_size=self.config.beam_size,
            vad_filter=False,
        )
        text = normalize_transcript(" ".join(segment.text for segment in segments))
        return AsrTranscript(
            text=text,
            wav_path=str(wav),
            model_path=str(model_path),
            executable="faster-whisper",
            language=self.config.language,
        )


def normalize_transcript(text: str) -> str:
    return " ".join(text.strip().split())


class VoskGrammarTranscriber:
    def __init__(self, config: Optional[VoskConfig] = None):
        self.config = config or VoskConfig(grammar=default_command_grammar())

    def transcribe_wav(self, wav_path: str | Path) -> AsrTranscript:
        try:
            import vosk
        except Exception as exc:
            raise RuntimeError("Python package 'vosk' is not available in this runtime") from exc
        import wave
        if hasattr(vosk, "SetLogLevel"):
            vosk.SetLogLevel(-1)

        wav = Path(wav_path).expanduser()
        if not wav.exists():
            raise FileNotFoundError(f"ASR input WAV not found: {wav}")
        model_path = Path(self.config.model_path).expanduser()
        if not model_path.exists():
            raise FileNotFoundError(f"Vosk model not found: {model_path}")

        model = vosk.Model(str(model_path))
        with wave.open(str(wav), "rb") as audio:
            recognizer = vosk.KaldiRecognizer(
                model,
                audio.getframerate(),
                json.dumps(list(self.config.grammar or default_command_grammar())),
            )
            text_parts: list[str] = []
            while True:
                data = audio.readframes(4000)
                if not data:
                    break
                if recognizer.AcceptWaveform(data):
                    text = _extract_vosk_text(recognizer.Result())
                    if text:
                        text_parts.append(text)
            final_text = _extract_vosk_text(recognizer.FinalResult())
            if final_text:
                text_parts.append(final_text)

        return AsrTranscript(
            text=normalize_transcript(" ".join(text_parts)),
            wav_path=str(wav),
            model_path=str(model_path),
            executable="vosk",
            language="en",
        )


def default_command_grammar(known_places: Optional[Sequence[str]] = None, wake_word: str = "hey jarvis") -> tuple[str, ...]:
    places = tuple(known_places or ("home", "hall", "kitchen", "door"))
    phrases = {
        wake_word,
        "status",
        "mission status",
        "stop",
        "cancel",
        "yes",
        "yeah",
        "no",
        "list places",
        "where are you",
        "what are you doing",
        "what can you do",
        "what is your name",
        "who are you",
        "hello",
        "hello robot",
        "hi",
        "hey robot",
        "thank you",
        "thanks",
        "debug",
        "debug what failed",
        "diagnose",
        "what failed",
        "what is wrong",
        "run diagnostics",
        "return home",
        "come home",
        "[unk]",
    }
    for place in places:
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
    return tuple(sorted(phrases))


def _extract_vosk_text(result_json: str) -> str:
    try:
        payload = json.loads(result_json)
    except json.JSONDecodeError:
        return ""
    value = payload.get("text", "")
    return value if isinstance(value, str) else ""


def build_mcp_transcript_payload(
    transcript: AsrTranscript,
    *,
    source: str = "laptop_transcript",
    wake_word: str = "hey jarvis",
    require_wake_word: bool = False,
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
