from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from amr_voice.wake_word import pcm16_to_numpy


DEFAULT_VAD_THRESHOLD = 0.5
DEFAULT_VAD_RELEASE_THRESHOLD = 0.35
DEFAULT_VAD_START_FRAMES = 2
DEFAULT_VAD_END_FRAMES = 8


@dataclass(frozen=True)
class VadDecision:
    score: float
    speech_active: bool
    event: Optional[str]
    threshold: float
    release_threshold: float


class VadGate:
    def __init__(
        self,
        threshold: float = DEFAULT_VAD_THRESHOLD,
        release_threshold: float = DEFAULT_VAD_RELEASE_THRESHOLD,
        start_frames: int = DEFAULT_VAD_START_FRAMES,
        end_frames: int = DEFAULT_VAD_END_FRAMES,
    ):
        self.threshold = float(threshold)
        self.release_threshold = float(release_threshold)
        self.start_frames = max(1, int(start_frames))
        self.end_frames = max(1, int(end_frames))
        self.speech_active = False
        self._speech_frames = 0
        self._silence_frames = 0

    def update(self, score: float) -> VadDecision:
        score = float(score)
        event = None
        if self.speech_active:
            if score < self.release_threshold:
                self._silence_frames += 1
            else:
                self._silence_frames = 0
            if self._silence_frames >= self.end_frames:
                self.speech_active = False
                self._speech_frames = 0
                event = "speech_ended"
        else:
            if score >= self.threshold:
                self._speech_frames += 1
            else:
                self._speech_frames = 0
            if self._speech_frames >= self.start_frames:
                self.speech_active = True
                self._silence_frames = 0
                event = "speech_started"

        return VadDecision(
            score=score,
            speech_active=self.speech_active,
            event=event,
            threshold=self.threshold,
            release_threshold=self.release_threshold,
        )


class SileroVadDetector:
    def __init__(self, n_threads: int = 1):
        try:
            from openwakeword.vad import VAD
        except Exception as exc:
            raise RuntimeError(
                "Python package 'openwakeword' with Silero VAD resources is not available."
            ) from exc
        self.model = VAD(n_threads=n_threads)

    def score(self, audio_frame, frame_size: int = 480) -> float:
        samples = pcm16_to_numpy(audio_frame)
        usable = (len(samples) // frame_size) * frame_size
        if usable <= 0:
            return 0.0
        samples = np.asarray(samples[:usable], dtype=np.int16)
        return float(self.model.predict(samples, frame_size=frame_size))
