from __future__ import annotations

from dataclasses import dataclass
import time

from robotcan.evaluation import DemoMetrics, summarize_demo_metrics
from robotcan.policies import DemoStep, build_scripted_demo_steps
from robotcan.protocol import (
    ARM_STATUS_FEEDBACK_ID,
    HAND_STATUS_FEEDBACK_ID,
    ArmControlCommand,
    ArmStatusFeedback,
    HandControlCommand,
    HandStatusFeedback,
)
from robotcan.transport.tsmaster_attached import AttachedTSMaster


@dataclass(slots=True)
class AlgorithmDemoConfig:
    command_period_s: float = 0.05
    feedback_poll_period_s: float = 0.05
    steps: list[DemoStep] | None = None


class AlgorithmDemoRunner:
    def __init__(self, app: AttachedTSMaster, config: AlgorithmDemoConfig | None = None):
        self.app = app
        self.config = config or AlgorithmDemoConfig()
        self.steps = self.config.steps or build_scripted_demo_steps()

    def run(self) -> DemoMetrics:
        self.app.enable_fifo()
        latest_hand_feedback: HandStatusFeedback | None = None
        latest_arm_feedback: ArmStatusFeedback | None = None
        hand_samples = 0
        arm_samples = 0
        max_hand_temperature_c = 0

        for step in self.steps:
            print(f"demo step: {step.name} duration={step.duration_s:.1f}s")
            deadline = time.perf_counter() + step.duration_s
            next_feedback_poll = 0.0
            while True:
                now = time.perf_counter()
                if now >= deadline:
                    break

                self.app.send_canfd(
                    0x101,
                    HandControlCommand(
                        enable=step.hand_enabled,
                        mode=step.mode,
                        target_position_deg=step.hand_position_deg,
                        target_velocity_deg_s=0.0,
                        target_torque_nm=0.0,
                    ).encode(),
                )
                self.app.send_canfd(
                    0x110,
                    ArmControlCommand(
                        enable=step.arm_enabled,
                        mode=step.mode,
                        j1_target_deg=step.arm_joint_targets_deg[0],
                        j2_target_deg=step.arm_joint_targets_deg[1],
                        j3_target_deg=step.arm_joint_targets_deg[2],
                        j4_target_deg=step.arm_joint_targets_deg[3],
                        j5_target_deg=step.arm_joint_targets_deg[4],
                        j6_target_deg=step.arm_joint_targets_deg[5],
                    ).encode(),
                )

                if now >= next_feedback_poll:
                    frames = self.app.receive_canfd(max_count=100, only_rx=True)
                    for frame in frames:
                        if frame.identifier == HAND_STATUS_FEEDBACK_ID:
                            latest_hand_feedback = HandStatusFeedback.decode(frame.payload)
                            hand_samples += 1
                            max_hand_temperature_c = max(max_hand_temperature_c, latest_hand_feedback.motor_temperature_c)
                        elif frame.identifier == ARM_STATUS_FEEDBACK_ID:
                            latest_arm_feedback = ArmStatusFeedback.decode(frame.payload)
                            arm_samples += 1
                    next_feedback_poll = now + self.config.feedback_poll_period_s

                time.sleep(self.config.command_period_s)

        metrics = summarize_demo_metrics(
            hand_feedback=latest_hand_feedback,
            arm_feedback=latest_arm_feedback,
            hand_samples=hand_samples,
            arm_samples=arm_samples,
            max_hand_temperature_c=max_hand_temperature_c,
        )
        print(
            "demo finished:"
            f" hand_samples={metrics.hand_samples}"
            f" arm_samples={metrics.arm_samples}"
            f" hand_pos={metrics.final_hand_position_deg:.1f}"
            f" arm_joints={[round(value, 1) for value in metrics.final_arm_joint_deg]}"
            f" hand_err={metrics.hand_error_code}"
            f" arm_err={metrics.arm_error_code}"
        )
        return metrics
