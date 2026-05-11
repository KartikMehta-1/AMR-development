# Voice Model Plan

This plan keeps the AMR voice stack open source and feasible for laptop, Jetson, and
eventual Orin NX deployment.

## Progressive Stack

| Stage | Initial model/tool | Status | Notes |
| --- | --- | --- | --- |
| Wake word | `openWakeWord` built-in `hey_jarvis` | Started | Runs locally on streaming 16 kHz audio. Use `hey jarvis` until a custom AMR wake word is trained. |
| VAD | Silero VAD through `openwakeword.vad` | Started | Gate ASR after wake detection and detect end-of-utterance. |
| ASR | Vosk grammar for commands; `whisper.cpp` for future free-form | Started | Vosk is the default command ASR because it works well with constrained robot intents. |
| TTS | Piper | Started | Local speech feedback through `/amr_voice/say` and the speaker MCP. Use `medium` voices for a less robotic laptop/Orin experience. |
| Conversation | `amr_conversation` MCP | Started | Stateless turn planner that returns conversational text plus safe MCP routing. |

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

## Current TTS Boundary

TTS uses Piper for local speech output. The ROS node listens for speech requests and
does not call MCP tools, mission services, recovery tools, or robot motion commands.

Dry-run startup:

```bash
AMR_TTS_DRY_RUN=true ros2 run amr_voice tts_node
```

Live playback requires a Piper executable, a Piper voice model, and an audio output:

```bash
./scripts/setup_piper_runtime.sh
./scripts/setup_piper_voice.sh en_US-lessac-medium

AMR_PIPER_BIN=/workspaces/AMR-development/models/piper-runtime/piper \
AMR_PIPER_MODEL=/workspaces/AMR-development/models/piper/en_US-lessac-medium/en_US-lessac-medium.onnx \
ros2 run amr_voice tts_node
```

Optional Piper tuning is available through:

```text
AMR_TTS_LENGTH_SCALE
AMR_TTS_NOISE_SCALE
AMR_TTS_NOISE_W_SCALE
AMR_TTS_SENTENCE_SILENCE
AMR_TTS_VOLUME
```

The speaker MCP publishes text to the node:

```bash
python3 mcp_servers/amr_speaker/server.py
python3 mcp_servers/amr_speaker/smoke_test.py
```

Debug/status speech follows this path:

```text
operator transcript -> voice MCP -> read-only state MCP tools -> LLM summary -> speaker MCP -> /amr_voice/say -> tts_node
```

The conversation MCP can prepare the response and tool route:

```bash
python3 mcp_servers/amr_conversation/server.py
python3 mcp_servers/amr_conversation/smoke_test.py
```

It remains a planner only. It does not inspect state, speak directly, call mission
services, clear faults, or start recovery.

For wake-word gated conversation testing:

```bash
AMR_VOICE_ALSA_DEVICE=plughw:CARD=sofhdadsp,DEV=7 \
AMR_TTS_OUTPUT_DEVICE=plughw:CARD=sofhdadsp,DEV=0 \
./scripts/open_amr_voice_conversation.sh
```

This starts three Foxy nodes inside `amr_devpc`: `voice_pipeline_node`,
`conversation_runtime_node`, and `tts_node`. The host only provides `/dev/snd`; host
ROS is not used. The TTS output device defaults to the laptop analog output above
and can be overridden with `AMR_TTS_OUTPUT_DEVICE`. The default listen mode is
`wake`, so the system listens for the wake word, captures one utterance, speaks the
assistant response if the turn is allowed, then returns to wake-word idle. Use
`AMR_VOICE_LISTEN_MODE=one-shot` for one immediate capture, or
`AMR_VOICE_LISTEN_MODE=continuous` only with headphones or echo suppression.

For controlled push-to-talk testing:

```bash
./scripts/open_amr_voice_push_to_talk.sh
```

The push-to-talk runner keeps TTS and conversation nodes alive, but captures ASR
only after an explicit Enter press. It is the preferred laptop-speaker test mode
until wake-word and echo behavior are tuned.

Both launchers apply the current laptop capture-gain profile before starting:
`Dmic0=60%`, `Dmic1 2nd=60%`, and `Capture=70%`. Override with
`AMR_VOICE_DMIC_GAIN`, `AMR_VOICE_CAPTURE_GAIN`, or set
`AMR_VOICE_CONFIGURE_AUDIO=false` to leave mixer gains untouched.

## Safety Boundary

Wake-word detection, VAD, ASR, and TTS only produce attention windows, speech
windows, transcripts, and spoken feedback. They must not call mission services,
clear faults, publish `/cmd_vel`, or bypass the voice MCP and mission-control MCP
confirmation/readiness gates.
