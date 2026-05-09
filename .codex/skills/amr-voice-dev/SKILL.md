---
name: amr-voice-dev
description: "Use when developing AMR voice, text intent, ASR, TTS, wake-word, confirmation, feedback, command parsing, or operator-interface behavior. Voice must call mission/safety interfaces and must not bypass confirmation or safety checks."
---

# AMR Voice Dev

Use this skill for `amr_voice`, voice launchers, text parser changes, ASR/TTS behavior, feedback, and operator command handling.

## Source Of Truth

Read first:

- `docs/agentic/roles/voice_operator_interface_agent.md`
- `docs/agentic/agent_tool_permissions.md`

Read when relevant:

- `ros_ws/src/amr_voice`
- `ros_ws/src/amr_missions`
- `ros_ws/src/amr_missions/config/places.yaml`
- `scripts/open_amr_voice_asr.sh`

## Workflow

1. Treat voice as an intent layer, not an execution layer.
2. Preserve wake-word, explicit confirmation, localization/safety prechecks, and rejection of ambiguous commands.
3. Map motion commands through mission runtime, not direct Nav2 goals or raw velocity.
4. Keep parser behavior deterministic and testable.
5. Coordinate with Navigation / Mission / Safety when adding executable intents.
6. Coordinate with Runtime Environment Agent when model files, audio devices, containers, or Jetson runtime are affected.

## Safe Checks

```bash
python3 -m compileall ros_ws/src/amr_voice
colcon build --packages-select amr_voice
colcon test --packages-select amr_voice
```

Do not run live ASR-to-motion loops or mission commands without explicit supervised confirmation.

## Output Format

```text
Voice Scope
- ...

Intent/Safety Behavior
- ...

Checks Run
- ...

Live Motion Not Run
- ...

Next Step
- ...
```
