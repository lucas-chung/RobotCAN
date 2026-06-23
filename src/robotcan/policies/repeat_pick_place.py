from __future__ import annotations

from dataclasses import dataclass


HOME_JOINT_DEG = (-90.0, -90.0, 90.0, -90.0, -90.0, 0.0)
ROTATED_JOINT_DEG = (0.0, -90.0, 90.0, -90.0, -90.0, 0.0)
HOME_OBJECT_XYZ = (-0.134, 0.499, 0.035)
PLACE_OBJECT_XYZ = (-0.499, -0.134, 0.035)


@dataclass(slots=True)
class PickPlaceTarget:
    name: str
    arm_joint_deg: tuple[float, float, float, float, float, float]
    object_xyz: tuple[float, float, float]


class RepeatPickPlacePolicy:
    def __init__(self):
        self.home = PickPlaceTarget("home", HOME_JOINT_DEG, HOME_OBJECT_XYZ)
        self.place = PickPlaceTarget("place", ROTATED_JOINT_DEG, PLACE_OBJECT_XYZ)

    def targets_for_cycle(self, cycle_index: int) -> tuple[PickPlaceTarget, PickPlaceTarget]:
        if cycle_index % 2 == 0:
            return self.home, self.place
        return self.place, self.home
