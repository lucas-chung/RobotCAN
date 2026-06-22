from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class DemoStep:
    name: str
    duration_s: float
    hand_position_deg: float
    arm_joint_targets_deg: tuple[float, float, float, float, float, float]
    hand_enabled: bool = True
    arm_enabled: bool = True
    mode: int = 1


def build_scripted_demo_steps() -> list[DemoStep]:
    home = (-90.0, -90.0, 90.0, -90.0, -90.0, 0.0)
    pre_grasp = (-90.0, -105.0, 110.0, -95.0, -90.0, 0.0)
    lift_pose = (-90.0, -85.0, 80.0, -85.0, -90.0, 0.0)
    return [
        DemoStep(
            name="home_open",
            duration_s=2.0,
            hand_position_deg=80.0,
            arm_joint_targets_deg=home,
        ),
        DemoStep(
            name="approach_open",
            duration_s=2.5,
            hand_position_deg=80.0,
            arm_joint_targets_deg=pre_grasp,
        ),
        DemoStep(
            name="close_gripper",
            duration_s=2.0,
            hand_position_deg=0.0,
            arm_joint_targets_deg=pre_grasp,
        ),
        DemoStep(
            name="lift_closed",
            duration_s=2.0,
            hand_position_deg=0.0,
            arm_joint_targets_deg=lift_pose,
        ),
        DemoStep(
            name="return_home",
            duration_s=2.0,
            hand_position_deg=0.0,
            arm_joint_targets_deg=home,
        ),
    ]
