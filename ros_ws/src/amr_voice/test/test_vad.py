from amr_voice.vad import VadGate


def test_vad_gate_requires_consecutive_speech_frames():
    gate = VadGate(threshold=0.5, release_threshold=0.35, start_frames=2, end_frames=2)

    first = gate.update(0.7)
    assert first.event is None
    assert first.speech_active is False

    second = gate.update(0.8)
    assert second.event == "speech_started"
    assert second.speech_active is True


def test_vad_gate_requires_consecutive_silence_frames_to_end():
    gate = VadGate(threshold=0.5, release_threshold=0.35, start_frames=1, end_frames=2)

    assert gate.update(0.8).event == "speech_started"
    assert gate.update(0.2).event is None
    ended = gate.update(0.1)
    assert ended.event == "speech_ended"
    assert ended.speech_active is False
