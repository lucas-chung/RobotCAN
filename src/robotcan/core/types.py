from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(slots=True)
class Pose:
    xyz: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float] | None = None


@dataclass(slots=True)
class DetectedObject:
    label: str
    confidence: float
    pixel_xy: tuple[float, float]
    bbox_xyxy: tuple[int, int, int, int]
    area_px: int
    depth_m: float
    camera_xyz_m: tuple[float, float, float]
    world_xyz_m: tuple[float, float, float]


@dataclass(slots=True)
class Observation:
    rgb: np.ndarray | None = None
    depth: np.ndarray | None = None
    camera_name: str | None = None
    robot_state: object | None = None
    objects: list[DetectedObject] = field(default_factory=list)


@dataclass(slots=True)
class RobotGoal:
    kind: str
    pose: Pose | None = None
    object_label: str | None = None
    metadata: dict[str, object] = field(default_factory=dict)


@dataclass(slots=True)
class RobotAction:
    joint_targets_deg: list[float] | None = None
    gripper_position: float | None = None
    end_effector_pose: Pose | None = None
    done: bool = False

