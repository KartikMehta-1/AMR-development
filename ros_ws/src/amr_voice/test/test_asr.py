from pathlib import Path

from amr_voice.asr import (
    AsrTranscript,
    WhisperCppConfig,
    WhisperCppTranscriber,
    build_mcp_transcript_payload,
    normalize_transcript,
)


def test_normalize_transcript_collapses_whitespace():
    assert normalize_transcript("  hey   jarvis\n go to kitchen  ") == "hey jarvis go to kitchen"


def test_build_mcp_transcript_payload_keeps_asr_separate_from_execution():
    transcript = AsrTranscript(
        text="hey jarvis go to kitchen",
        wav_path="/tmp/input.wav",
        model_path="/models/ggml-base.en.bin",
        executable="whisper-cli",
        language="en",
    )

    payload = build_mcp_transcript_payload(
        transcript,
        known_places=["home", "hall", "kitchen"],
    )

    assert payload["text"] == "hey jarvis go to kitchen"
    assert payload["source"] == "laptop_transcript"
    assert payload["dry_run"] is True
    assert payload["require_wake_word"] is False
    assert payload["known_places"] == ["home", "hall", "kitchen"]


def test_whisper_cpp_transcriber_reads_text_output(tmp_path: Path):
    wav = tmp_path / "input.wav"
    wav.write_bytes(b"RIFF")
    model = tmp_path / "ggml-base.en.bin"
    model.write_bytes(b"model")
    whisper_bin = tmp_path / "whisper-cli"
    whisper_bin.write_text("#!/bin/sh\n", encoding="utf-8")
    whisper_bin.chmod(0o755)
    commands = []

    def runner(command, check, stdout, stderr, text):
        commands.append(command)
        output_base = Path(command[command.index("-of") + 1])
        output_base.with_suffix(".txt").write_text(" hey   jarvis go kitchen\n", encoding="utf-8")

        class Result:
            returncode = 0
            stdout = ""
            stderr = ""

        return Result()

    config = WhisperCppConfig(
        executable=str(whisper_bin),
        model_path=str(model),
        threads=2,
    )
    transcript = WhisperCppTranscriber(config=config, runner=runner).transcribe_wav(wav)

    assert transcript.text == "hey jarvis go kitchen"
    assert commands[0][0] == str(whisper_bin)
    assert "-otxt" in commands[0]


def test_whisper_cpp_transcriber_requires_model(tmp_path: Path):
    wav = tmp_path / "input.wav"
    wav.write_bytes(b"RIFF")
    config = WhisperCppConfig(executable="whisper-cli", model_path=str(tmp_path / "missing.bin"))

    try:
        WhisperCppTranscriber(config=config).transcribe_wav(wav)
    except FileNotFoundError:
        return
    raise AssertionError("expected missing model to raise FileNotFoundError")
