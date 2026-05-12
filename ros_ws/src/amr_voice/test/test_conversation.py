from amr_voice.conversation import plan_turn, transcript_text


def test_transcript_text_reads_json_payload():
    assert transcript_text('{"text": "hello robot", "source": "test"}') == "hello robot"


def test_general_conversation_turn_is_allowed():
    turn = plan_turn("hello robot")

    assert turn.allowed is True
    assert turn.blockers == []
    assert "ready to help" in turn.assistant_response
    assert turn.next_tool is None


def test_motion_conversation_turn_requires_confirmation():
    turn = plan_turn("go to kitchen", known_places=["hall", "kitchen"])

    assert turn.allowed is True
    assert turn.requires_confirmation is True
    assert turn.next_tool["server"] == "amr_mission_control"
    assert turn.next_tool["arguments"]["operator_confirmed_supervised"] is False


def test_robot_status_phrases_route_to_status_tool():
    for text in [
        "robot status",
        "check robot status",
        "hey what is the robot's status",
        "what is the robot's status",
        "what is the robot status",
        "what is the status of the robot",
    ]:
        turn = plan_turn(text)

        assert turn.command["action"] == "status"
        assert turn.next_tool["server"] == "amr_mission_control"
        assert turn.next_tool["tool"] == "get_mission_state"


def test_unknown_conversation_turn_has_safe_fallback():
    turn = plan_turn("can you")

    assert turn.allowed is False
    assert "no_safe_tool_route" in turn.blockers
    assert "safe robot command" in turn.assistant_response
