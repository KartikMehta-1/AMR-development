from amr_voice.local_llm import LocalLlmResult
from amr_voice.push_to_talk_conversation import build_assistant_reply


class _FakeLlm:
    def respond(self, text):
        return LocalLlmResult(
            ok=True,
            text=f"local answer for {text}",
            message="ok",
            command=[],
            raw_output="",
        )


class _FailingLlm:
    def respond(self, text):
        return LocalLlmResult(
            ok=False,
            text="",
            message="failed",
            command=[],
            raw_output="",
        )


def test_known_robot_command_uses_deterministic_reply_without_llm():
    reply = build_assistant_reply("status", llm=_FakeLlm())

    assert reply.llm_used is False
    assert reply.fallback_used is False
    assert "mission status" in reply.text


def test_status_can_use_read_only_tool_executor():
    def executor(next_tool):
        assert next_tool["server"] == "amr_mission_control"
        assert next_tool["tool"] == "get_mission_state"
        return {
            "ok": True,
            "message": "mission status",
            "data": {"state": "idle", "active_request": "none"},
            "blockers": [],
        }

    reply = build_assistant_reply("what is the robot's status", tool_executor=executor)

    assert reply.llm_used is False
    assert reply.tool_result["ok"] is True
    assert "state: idle" in reply.text


def test_unknown_question_uses_llm_reply():
    reply = build_assistant_reply("what do you know about the robot", llm=_FakeLlm())

    assert reply.llm_used is True
    assert reply.fallback_used is False
    assert reply.text == "local answer for what do you know about the robot"


def test_unknown_question_falls_back_when_llm_fails():
    reply = build_assistant_reply("what do you know about the robot", llm=_FailingLlm())

    assert reply.llm_used is False
    assert reply.fallback_used is True
    assert "safe robot command" in reply.text
