# Voice Operator Interface Agent Contract

## Purpose

Develop voice, text, TTS, and future operator UI behavior as intent interfaces over the mission and safety layers. This agent should remain thin: it interprets operator intent but does not own robot execution.

## Owned Areas

- `ros_ws/src/amr_voice`
- `mcp_servers/amr_voice_interface`
- `mcp_servers/amr_conversation`
- `mcp_servers/amr_speaker`
- Voice parser tests.
- Wake-word, VAD, ASR, local LLM intent routing, confirmation, rejection,
  TTS/feedback, and transcript logging behavior.
- Future operator UI command/feedback layer.
- Voice/interface portions of docs and examples.

## Allowed Commands

- Read and edit voice/interface files when requested.
- Run parser and dry-run tests.
- Run software-only interface tests.
- Run MCP smoke tests for voice, conversation, and speaker servers.
- Read-only repo commands such as `rg`, `find`, `sed`, `git diff`.

## Blocked Commands

- Bypassing mission runtime for motion commands.
- Publishing direct Nav2 goals from the voice layer when mission services exist.
- Skipping confirmation for motion commands.
- Treating ambiguous voice commands as executable commands.
- Starting hardware motion without explicit confirmation and readiness checks.

## Required Checks

- Motion commands require confirmation unless they are stop/cancel.
- Stop/cancel must remain urgent and not blocked behind confirmation.
- Localization readiness is required before base motion.
- Safety readiness is required before motion.
- Ambiguous commands must ask for clarification or refuse.
- Parser changes need tests for accepted and rejected phrases.
- Local LLM routing must stay constrained to the fixed allowed intent set and fall
  back safely when the model server is unavailable.
- Read-only diagnostic/status requests may use state-inspection MCP plans; robot
  facts must not be invented by the LLM.

## Done Criteria

- Parser tests cover new phrases.
- Intent-router tests cover JSON parsing, confidence, and confirmation semantics.
- Push-to-talk tests cover pending requests and blocked motion execution.
- Confirmation behavior is tested.
- Dry-run behavior is reported.
- Voice commands still route through mission/safety layers.

## Common Failure Modes

- A new phrase bypasses confirmation.
- Wake-word state leaks across commands.
- Stop/cancel becomes delayed by confirmation.
- Ambiguous commands trigger motion.
- Text feedback diverges from actual mission state.
- Local Qwen or a general LLM answer is treated as authoritative robot state.
- Push-to-talk pending confirmation state leaks into an unrelated command.

## Escalation Rules

- If a requested voice command maps to motion, coordinate with Navigation Mission Safety Agent.
- If voice feedback depends on robot health, use read-only diagnostics or MCP health tools when available.
- If the operator phrase is ambiguous, refuse or request clarification.
