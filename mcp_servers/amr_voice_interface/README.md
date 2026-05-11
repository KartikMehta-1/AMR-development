# AMR Voice Interface MCP Server

Input-agnostic voice/text intent MCP server for AMR operator commands.

This server accepts typed text or ASR transcripts from a laptop, Jetson, or mobile
device and converts them into deterministic AMR intents. It does not capture audio,
start ASR, call mission services, publish `/cmd_vel`, clear faults, or start motion.

## Tools

- `get_voice_interface_status`
- `parse_text_intent`
- `describe_voice_source_contract`

## Boundary

The voice MCP is a parser and routing layer only:

```text
laptop/Jetson/mobile audio -> ASR transcript -> voice MCP -> recommended mission MCP tool
```

Motion-causing intents, such as `go_to kitchen`, return a recommended
`amr_mission_control.go_to_named_place` call with `operator_confirmed_supervised=false`.
The operator or supervising LLM must still run mission-control readiness checks and
set confirmation only after supervised approval.

Diagnostic intents, such as `debug what failed`, return a read-only
`amr_state_inspection` tool plan. The LLM should execute the inspection plan,
summarize the result, and may then call `amr_speaker.speak_text` for spoken feedback.
The voice MCP does not clear faults or start recovery.

## Runtime

Run from the repository root:

```bash
cd /home/kartik/AMR-development
python3 mcp_servers/amr_voice_interface/server.py
```

## Smoke Test

```bash
python3 mcp_servers/amr_voice_interface/smoke_test.py
```

## Source Types

The server accepts the same transcript contract from any source:

- `text`
- `laptop_transcript`
- `jetson_transcript`
- `mobile_transcript`

Audio capture and ASR remain separate producer concerns. This keeps the robot command
layer generic while allowing different microphone devices to be added later.
