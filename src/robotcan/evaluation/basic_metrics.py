from __future__ import annotations

from dataclasses import dataclass

from robotcan.protocol import ArmStatusFeedback, HandStatusFeedback


@dataclass(slots=True)
class DemoMetrics:
    hand_samples: int = 0
    arm_samples: int = 0
    final_hand_position_deg: float = 0.0
    final_arm_joint_deg: tuple[float, float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    max_hand_temperature_c: int = 0
    hand_error_code: int = 0
    arm_error_code: int = 0


def summarize_demo_metrics(
    hand_feedback: HandStatusFeedback | None,
    arm_feedback: ArmStatusFeedback | None,
    hand_samples: int,
    arm_samples: int,
    max_hand_temperature_c: int,
) -> DemoMetrics:
    final_arm_joint_deg = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    arm_error_code = 0
    if arm_feedback is not None:
        final_arm_joint_deg = (
            arm_feedback.j1_actual_deg,
            arm_feedback.j2_actual_deg,
            arm_feedback.j3_actual_deg,
            arm_feedback.j4_actual_deg,
            arm_feedback.j5_actual_deg,
            arm_feedback.j6_actual_deg,
        )
        arm_error_code = arm_feedback.error_code

    return DemoMetrics(
        hand_samples=hand_samples,
        arm_samples=arm_samples,
        final_hand_position_deg=0.0 if hand_feedback is None else hand_feedback.actual_position_deg,
        final_arm_joint_deg=final_arm_joint_deg,
        max_hand_temperature_c=max_hand_temperature_c,
        hand_error_code=0 if hand_feedback is None else hand_feedback.error_code,
        arm_error_code=arm_error_code,
    )
