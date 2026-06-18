"""Thin wrappers around protocol dataclasses for encode/decode flows."""

from __future__ import annotations

from robotcan.protocol.messages import (
    ArmControlCommand,
    ArmDiagResponse,
    ArmFaultStatus,
    ArmStatusFeedback,
    HandControlCommand,
    HandDiagResponse,
    HandFaultStatus,
    HandStatusFeedback,
    JointControlCommand,
    JointDiagResponse,
    JointFaultStatus,
    JointStatusFeedback,
)


def decode_hand_control_command(payload: bytes) -> HandControlCommand:
    return HandControlCommand.decode(payload)


def encode_hand_control_command(command: HandControlCommand) -> bytes:
    return command.encode()


def decode_hand_status_feedback(payload: bytes) -> HandStatusFeedback:
    return HandStatusFeedback.decode(payload)


def encode_hand_status_feedback(feedback: HandStatusFeedback) -> bytes:
    return feedback.encode()


def decode_hand_fault_status(payload: bytes) -> HandFaultStatus:
    return HandFaultStatus.decode(payload)


def encode_hand_fault_status(fault: HandFaultStatus) -> bytes:
    return fault.encode()


def decode_hand_diag_response(payload: bytes) -> HandDiagResponse:
    return HandDiagResponse.decode(payload)


def encode_hand_diag_response(diag: HandDiagResponse) -> bytes:
    return diag.encode()


def decode_arm_control_command(payload: bytes) -> ArmControlCommand:
    return ArmControlCommand.decode(payload)


def encode_arm_control_command(command: ArmControlCommand) -> bytes:
    return command.encode()


def decode_arm_status_feedback(payload: bytes) -> ArmStatusFeedback:
    return ArmStatusFeedback.decode(payload)


def encode_arm_status_feedback(feedback: ArmStatusFeedback) -> bytes:
    return feedback.encode()


def decode_arm_fault_status(payload: bytes) -> ArmFaultStatus:
    return ArmFaultStatus.decode(payload)


def encode_arm_fault_status(fault: ArmFaultStatus) -> bytes:
    return fault.encode()


def decode_arm_diag_response(payload: bytes) -> ArmDiagResponse:
    return ArmDiagResponse.decode(payload)


def encode_arm_diag_response(diag: ArmDiagResponse) -> bytes:
    return diag.encode()


# Compatibility aliases
def decode_control_command(payload: bytes) -> JointControlCommand:
    return JointControlCommand.decode(payload)


def encode_control_command(command: JointControlCommand) -> bytes:
    return command.encode()


def decode_status_feedback(payload: bytes) -> JointStatusFeedback:
    return JointStatusFeedback.decode(payload)


def encode_status_feedback(feedback: JointStatusFeedback) -> bytes:
    return feedback.encode()


def decode_fault_status(payload: bytes) -> JointFaultStatus:
    return JointFaultStatus.decode(payload)


def encode_fault_status(fault: JointFaultStatus) -> bytes:
    return fault.encode()


def decode_diag_response(payload: bytes) -> JointDiagResponse:
    return JointDiagResponse.decode(payload)


def encode_diag_response(diag: JointDiagResponse) -> bytes:
    return diag.encode()
