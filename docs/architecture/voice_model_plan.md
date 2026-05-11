# Voice Model Plan

This plan keeps the AMR voice stack open source and feasible for laptop, Jetson, and
eventual Orin NX deployment.

## Progressive Stack

| Stage | Initial model/tool | Status | Notes |
| --- | --- | --- | --- |
| Wake word | `openWakeWord` built-in `hey_jarvis` | Started | Runs locally on streaming 16 kHz audio. Use `hey jarvis` until a custom AMR wake word is trained. |
| VAD | Silero VAD through `openwakeword.vad` | Started | Gate ASR after wake detection and detect end-of-utterance. |
| ASR | `whisper.cpp` with `base.en` or `small.en` | Planned | Replace the old Vosk path for better quality while staying Orin-feasible. |
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

Wake-word detection and VAD only open and close attention/speech windows for
downstream ASR. They must not call mission services, clear faults, publish `/cmd_vel`,
or bypass the voice MCP and mission-control MCP confirmation/readiness gates.
