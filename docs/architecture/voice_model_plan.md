# Voice Model Plan

This plan keeps the AMR voice stack open source and feasible for laptop, Jetson, and
eventual Orin NX deployment.

## Progressive Stack

| Stage | Initial model/tool | Status | Notes |
| --- | --- | --- | --- |
| Wake word | `openWakeWord` built-in `hey_jarvis` | Started | Runs locally on streaming 16 kHz audio. Use `hey jarvis` until a custom AMR wake word is trained. |
| VAD | Silero VAD through `openwakeword.vad` | Started | Gate ASR after wake detection and detect end-of-utterance. |
| ASR | `whisper.cpp` with `base.en` first, `small.en` if latency allows | Started | Local transcript producer only; transcripts are submitted to the voice MCP. |
| TTS | Piper | Planned | Local speech feedback for accepted, denied, and status messages. |

## Current Wake-Word Runtime

ROS entry point:

```bash
ros2 run amr_voice wake_word_node --model hey_jarvis --threshold 0.5
```

The node publishes JSON strings on:

```text
/amr_voice/wake_word
```

## Current VAD Runtime

ROS entry point:

```bash
ros2 run amr_voice vad_node --threshold 0.5 --release-threshold 0.35
```

The node publishes JSON strings on:

```text
/amr_voice/vad
```

Example events:

```json
{"event": "speech_started", "score": 0.82, "source": "laptop"}
{"event": "speech_ended", "score": 0.12, "source": "laptop"}
```

## Current ASR Boundary

ASR uses `whisper.cpp` as a local open-source transcript producer. The first target
model is `ggml-base.en.bin`; `small.en` can be tested later if Orin NX latency is
acceptable.

The file-transcription entry point is:

```bash
./scripts/setup_whisper_cpp.sh

ros2 run amr_voice asr_file_cli input.wav \
  --whisper-bin models/whisper.cpp/build/bin/whisper-cli \
  --model /workspaces/AMR-development/models/whisper/ggml-base.en.bin
```

It prints a JSON payload shaped for `mcp_servers/amr_voice_interface` and does not
parse, confirm, or execute robot commands. Live microphone ASR should be built by
feeding wake/VAD speech segments into the same transcript boundary.

Example event:

```json
{
  "event": "wake_word_detected",
  "model": "hey_jarvis",
  "score": 0.74,
  "threshold": 0.5,
  "source": "laptop"
}
```

## Safety Boundary

Wake-word detection, VAD, and ASR only produce attention windows, speech windows, and
transcripts. They must not call mission services, clear faults, publish `/cmd_vel`, or
bypass the voice MCP and mission-control MCP confirmation/readiness gates.
