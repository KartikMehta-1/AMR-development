from amr_voice.local_llm import LocalLlmResult
from amr_voice.push_to_talk_conversation import build_assistant_reply
from amr_voice.intent_router import RoutedIntent


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


class _FakeRouter:
    def __init__(self, route):
        self.route_result = route

    def route(self, text, *, known_places, pending_request=None):
        return self.route_result


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


def test_generic_status_uses_intent_router_and_read_only_tool_executor():
    calls = []

    def executor(next_tool):
        calls.append(next_tool)
        assert next_tool["server"] == "amr_state_inspection"
        assert next_tool["tool_plan"][0]["tool"] == "get_robot_health"
        return {
            "ok": True,
            "message": "robot status",
            "data": [
                {"tool": "get_robot_health", "result": {"ok": True, "data": {"state": "ready"}, "blockers": []}},
                {"tool": "get_mission_state", "result": {"ok": True, "data": {"state": "idle"}, "blockers": []}},
            ],
            "blockers": [],
        }

    reply = build_assistant_reply(
        "how is the robot doing right now",
        intent_router=_FakeRouter(RoutedIntent("read_status", 0.91, {}, False, "test")),
        tool_executor=executor,
    )

    assert calls
    assert reply.llm_used is False
    assert reply.intent == "read_status"
    assert "robot health: ready" in reply.text


def test_router_motion_intent_does_not_execute_without_supervised_confirmation():
    def executor(next_tool):
        raise AssertionError(f"motion tool should not be called: {next_tool}")

    reply = build_assistant_reply(
        "could you take yourself over to the kitchen",
        known_places=["hall", "kitchen"],
        intent_router=_FakeRouter(
            RoutedIntent("go_to_place", 0.88, {"place": "kitchen"}, True, "test")
        ),
        tool_executor=executor,
    )

    assert reply.llm_used is False
    assert reply.intent == "go_to_place"
    assert reply.pending_request["tool"] == "go_to_named_place"
    assert "supervised confirmation" in reply.text
