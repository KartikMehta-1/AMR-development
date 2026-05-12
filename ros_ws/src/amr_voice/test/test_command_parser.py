from amr_voice.command_parser import DIAGNOSE, GO_TO, STATUS, parse_text_command


def test_parse_debug_command_maps_to_diagnose():
    command = parse_text_command("hey jarvis debug what failed", require_wake_word=True)

    assert command.action == DIAGNOSE
    assert command.wake_word_detected is True
    assert command.confidence >= 0.9


def test_parse_diagnose_does_not_require_place():
    command = parse_text_command("run diagnostics", known_places=["hall", "kitchen"])

    assert command.action == DIAGNOSE
    assert command.place is None


def test_parse_status_command_still_maps_to_status():
    command = parse_text_command("what are you doing")

    assert command.action == STATUS


def test_parse_motion_command_still_maps_to_go_to():
    command = parse_text_command("go to kitchen", known_places=["hall", "kitchen"])

    assert command.action == GO_TO
    assert command.place == "kitchen"
