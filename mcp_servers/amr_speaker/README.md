# AMR Speaker MCP Server

Spoken-feedback MCP server for AMR status, diagnostics, and operator responses.

This server publishes speech requests to `/amr_voice/say`. It does not synthesize
audio itself, command motion, clear faults, start recovery, or inspect robot state.
The ROS `amr_voice tts_node` owns TTS playback.

## Tools

- `get_speaker_status`
- `speak_text`
- `describe_speaker_contract`

## Boundary

```text
LLM / MCP diagnosis summary -> amr_speaker.speak_text -> /amr_voice/say -> tts_node -> speaker
```

For debugging requests, the LLM should first call read-only inspection tools, summarize
the result, and then call `speak_text` with the final operator-facing message.

For conversation, `amr_conversation.plan_conversation_turn` can return a
`speaker_request` shaped for `speak_text`.

## Smoke Test

```bash
python3 mcp_servers/amr_speaker/smoke_test.py
```
