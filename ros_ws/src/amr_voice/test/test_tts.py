from amr_voice.tts import PiperSpeaker, SpeechRequest, sanitize_speech_text, speech_request_from_json


def test_sanitize_speech_text_collapses_and_limits_text():
    text = sanitize_speech_text("  hello\n\nrobot  ", max_chars=20)
    assert text == "hello robot"
    assert sanitize_speech_text("x" * 30, max_chars=10) == "xxxxxxxxx."


def test_sanitize_speech_text_truncates_at_sentence_boundary():
    text = "First complete sentence. Second sentence is much too long to keep in full."

    assert sanitize_speech_text(text, max_chars=45) == "First complete sentence."


def test_sanitize_speech_text_truncates_at_word_boundary():
    text = "This response contains several words and should not cut one in half."

    assert sanitize_speech_text(text, max_chars=40) == "This response contains several words."


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
