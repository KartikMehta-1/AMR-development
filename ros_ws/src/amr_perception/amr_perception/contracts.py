from __future__ import annotations

from dataclasses import asdict, dataclass, field, is_dataclass
from typing import Any


@dataclass(frozen=True)
class ObservationHeader:
    frame_id: str
    stamp_sec: float | None = None
    age_sec: float | None = None
    source: str = "unknown"

    @property
    def fresh(self) -> bool:
        return self.age_sec is None or self.age_sec <= 1.0


@dataclass(frozen=True)
class CameraHealth:
    ok: bool
    color_active: bool = False
    depth_active: bool = False
    pointcloud_active: bool = False
    frame_id: str = ""
    blockers: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class ObjectProposal:
    label: str
    confidence: float
    header: ObservationHeader
    pose_frame: str | None = None
    position_xyz_m: tuple[float, float, float] | None = None
    size_xyz_m: tuple[float, float, float] | None = None
    proposal_only: bool = True

    @property
    def usable(self) -> bool:
        return self.proposal_only and self.confidence >= 0.5 and self.header.fresh


@dataclass(frozen=True)
class GraspProposal:
    object_label: str
    confidence: float
    header: ObservationHeader
    grasp_frame: str
    approach_vector_xyz: tuple[float, float, float]
    width_m: float | None = None
    proposal_only: bool = True

    @property
    def executable(self) -> bool:
        return False


@dataclass(frozen=True)
class SceneSummary:
    header: ObservationHeader
    objects: list[ObjectProposal] = field(default_factory=list)
    blockers: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.blockers


def to_jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return {key: to_jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [to_jsonable(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)
