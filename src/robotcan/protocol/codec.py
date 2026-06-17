"""Thin wrappers around protocol dataclasses for encode/decode flows."""

from __future__ import annotations

from robotcan.protocol.messages import (
    JointControlCommand,
    JointDiagResponse,
    JointFaultStatus,
    JointStatusFeedback,
)


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
