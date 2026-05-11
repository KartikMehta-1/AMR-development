from amr_voice.tts import PiperSpeaker, SpeechRequest, sanitize_speech_text, speech_request_from_json


def test_sanitize_speech_text_collapses_and_limits_text():
    text = sanitize_speech_text("  hello\n\nrobot  ", max_chars=20)
    assert text == "hello robot"
    assert sanitize_speech_text("x" * 30, max_chars=10) == "xxxxxxxxx."


def test_speech_request_from_json():
    request = speech_request_from_json('{"text": "Going to kitchen", "source": "test", "interrupt": true}')
    assert request.text == "Going to kitchen"
    assert request.source == "test"
    assert request.interrupt is True


def test_piper_speaker_dry_run_accepts_text():
    result = PiperSpeaker(dry_run=True).speak(SpeechRequest(text="I am checking robot health."))
    assert result.ok is True
    assert result.spoken is False
    assert result.blockers == []
