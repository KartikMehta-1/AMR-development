# AMR Conversation MCP Server

Conversation planner for AMR voice/text interaction.

This server turns a user utterance into a short robot-facing response and, when
appropriate, returns the next MCP tool plan. It does not execute missions, inspect
robot state, synthesize audio, publish `/cmd_vel`, clear faults, or start recovery.

## Tools

- `get_conversation_status`
- `plan_conversation_turn`
- `describe_conversation_contract`

## Boundary

```text
ASR/text -> amr_conversation.plan_conversation_turn
         -> optional voice/mission/state MCP plan
         -> optional amr_speaker.speak_text request
```

Motion-causing requests still require the mission-control MCP readiness checks and
explicit supervised confirmation.

## Smoke Test

```bash
python3 mcp_servers/amr_conversation/smoke_test.py
```
