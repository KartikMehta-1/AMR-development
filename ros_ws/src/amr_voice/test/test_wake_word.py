from amr_voice.wake_word import detect_from_scores


def test_detect_from_scores_above_threshold():
    detection = detect_from_scores({"hey_jarvis": 0.72}, model_name="hey_jarvis", threshold=0.5)

    assert detection.detected is True
    assert detection.model == "hey_jarvis"
    assert detection.score == 0.72


def test_detect_from_scores_below_threshold():
    detection = detect_from_scores({"hey_jarvis": 0.2}, model_name="hey_jarvis", threshold=0.5)

    assert detection.detected is False
    assert detection.score == 0.2


def test_detect_from_scores_falls_back_to_max_score():
    detection = detect_from_scores({"other": 0.9}, model_name="hey_jarvis", threshold=0.5)

    assert detection.detected is True
    assert detection.score == 0.9
