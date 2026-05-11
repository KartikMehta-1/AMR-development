from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Protocol

import numpy as np


DEFAULT_WAKE_MODEL = "hey_jarvis"
DEFAULT_WAKE_THRESHOLD = 0.5


@dataclass(frozen=True)
class WakeWordDetection:
    detected: bool
    model: str
    score: float
    threshold: float
    scores: dict[str, float]


class WakeWordModel(Protocol):
    def predict(self, audio_frame) -> Mapping[str, float]:
        ...


class OpenWakeWordDetector:
    """Small adapter around openWakeWord's streaming prediction API."""

    def __init__(
        self,
        model_name: str = DEFAULT_WAKE_MODEL,
        threshold: float = DEFAULT_WAKE_THRESHOLD,
        inference_framework: str = "onnx",
        enable_speex_noise_suppression: bool = False,
    ):
        try:
            from openwakeword.model import Model
        except Exception as exc:
            raise RuntimeError(
                "Python package 'openwakeword' is not available. Install it in the voice runtime "
                "before running live wake-word detection."
            ) from exc
        self.model_name = model_name
        self.threshold = float(threshold)
        self.model = Model(
            wakeword_models=[model_name],
            inference_framework=inference_framework,
            enable_speex_noise_suppression=enable_speex_noise_suppression,
        )

    def process(self, audio_frame) -> WakeWordDetection:
        audio_frame = pcm16_to_numpy(audio_frame)
        return detect_from_scores(
            self.model.predict(audio_frame),
            model_name=self.model_name,
            threshold=self.threshold,
        )


def pcm16_to_numpy(audio_frame) -> np.ndarray:
    if isinstance(audio_frame, np.ndarray):
        return audio_frame.astype(np.int16, copy=False)
    if isinstance(audio_frame, (bytes, bytearray, memoryview)):
        return np.frombuffer(audio_frame, dtype=np.int16)
    return np.asarray(audio_frame, dtype=np.int16)


def detect_from_scores(
    scores: Mapping[str, float],
    model_name: str = DEFAULT_WAKE_MODEL,
    threshold: float = DEFAULT_WAKE_THRESHOLD,
) -> WakeWordDetection:
    normalized_scores = {str(key): float(value) for key, value in scores.items()}
    score = normalized_scores.get(model_name)
    if score is None and normalized_scores:
        score = max(normalized_scores.values())
    score = 0.0 if score is None else float(score)
    threshold = float(threshold)
    return WakeWordDetection(
        detected=score >= threshold,
        model=model_name,
        score=score,
        threshold=threshold,
        scores=normalized_scores,
    )
