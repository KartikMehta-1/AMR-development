import math
import os
from dataclasses import dataclass
from typing import Dict

import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass
class NamedPlace:
    name: str
    x: float
    y: float
    yaw: float
    frame_id: str = "map"


def default_places_path() -> str:
    share_dir = get_package_share_directory("amr_missions")
    return os.path.join(share_dir, "config", "places.yaml")


def load_places(path: str) -> Dict[str, NamedPlace]:
    with open(path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    places: Dict[str, NamedPlace] = {}
    for name, value in raw.items():
        if not isinstance(value, dict):
            raise ValueError(f"Place '{name}' must be a mapping")
        try:
            places[name] = NamedPlace(
                name=name,
                x=float(value["x"]),
                y=float(value["y"]),
                yaw=float(value.get("yaw", 0.0)),
                frame_id=str(value.get("frame_id", "map")),
            )
        except KeyError as exc:
            raise ValueError(f"Place '{name}' missing required key: {exc}") from exc
    return places


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)
