# Voice Model Plan

This plan keeps the AMR voice stack open source and feasible for laptop, Jetson, and
eventual Orin NX deployment.

## Progressive Stack

| Stage | Initial model/tool | Status | Notes |
| --- | --- | --- | --- |
| Wake word | `openWakeWord` built-in `hey_jarvis` | Started | Runs locally on streaming 16 kHz audio. Use `hey jarvis` until a custom AMR wake word is trained. |
| VAD | Silero VAD through `openwakeword.vad` | Started | Gate ASR after wake detection and detect end-of-utterance. |
| ASR | Vosk grammar for commands; `whisper.cpp` for future free-form | Started | Vosk is the default command ASR because it works well with constrained robot intents. |
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

ASR uses an open-source local transcript producer. The default command backend is
Vosk with a constrained grammar because this laptop microphone path already works well
with that model. `whisper.cpp` remains available for future free-form dictation or
larger command vocabularies.

The file-transcription entry point is:

```bash
./scripts/setup_whisper_cpp.sh

# For use from the Foxy container:
AMR_WHISPER_BUILD_DIR=/workspaces/AMR-development/models/whisper.cpp/build-foxy \
  ./scripts/setup_whisper_cpp.sh

ros2 run amr_voice asr_file_cli input.wav \
  --whisper-bin models/whisper.cpp/build-foxy/bin/whisper-cli \
  --model /workspaces/AMR-development/models/whisper/ggml-base.en.bin
```

It prints a JSON payload shaped for `mcp_servers/amr_voice_interface` and does not
parse, confirm, or execute robot commands. Because live ASR runs after wake-word
detection, the emitted MCP payload does not require the wake phrase to be present in
the transcript text by default. Live microphone ASR should be built by feeding
wake/VAD speech segments into the same transcript boundary.

The first live end-to-end dry-run node is:

```bash
ros2 run amr_voice voice_pipeline_node \
  --device 9 \
  --asr-backend vosk \
  --vosk-model /workspaces/AMR-development/models/vosk-model-small-en-us-0.15 \
  --log-audio-level
```

To test Whisper instead:

```bash
ros2 run amr_voice voice_pipeline_node \
  --device 9 \
  --asr-backend whisper \
  --whisper-bin /workspaces/AMR-development/models/whisper.cpp/build-foxy/bin/whisper-cli \
  --whisper-model /workspaces/AMR-development/models/whisper/ggml-base.en.bin \
  --log-audio-level
```

It publishes wake events, VAD events, transcripts, and MCP arguments, but it does not
call the voice MCP or mission-control MCP by itself.

For VAD/ASR tuning without wake detection:

```bash
ros2 run amr_voice voice_pipeline_node --start-listening --device 9 ...
```

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
