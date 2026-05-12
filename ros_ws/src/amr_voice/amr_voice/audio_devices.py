from __future__ import annotations

from typing import Optional


def parse_device(device: Optional[str]):
    if device is None or device == "" or device == "auto":
        return None
    try:
        return int(device)
    except ValueError:
        return device


def select_input_device(sounddevice, requested: Optional[str]):
    if requested not in {None, "", "auto"}:
        device = parse_device(requested)
        info = sounddevice.query_devices(device, "input")
        return device, info

    devices = sounddevice.query_devices()
    input_devices = [
        (index, info)
        for index, info in enumerate(devices)
        if int(info.get("max_input_channels", 0)) > 0
    ]
    if not input_devices:
        raise RuntimeError("No input audio devices are visible in this container")

    def score(item):
        _index, info = item
        name = str(info.get("name", "")).lower()
        max_output = int(info.get("max_output_channels", 0))
        default_rate = int(float(info.get("default_samplerate", 0)))
        score_value = 0
        if max_output == 0:
            score_value += 100
        if default_rate == 16000:
            score_value += 50
        if "hdmi" in name:
            score_value -= 100
        if "dmic" in name or "sof-hda-dsp" in name:
            score_value += 10
        return score_value

    device, info = max(input_devices, key=score)
    return device, info
