from __future__ import annotations

from dataclasses import dataclass

from robotcan.protocol import (
    ArmControlCommand,
    ArmDiagResponse,
    ArmFaultStatus,
    ArmStatusFeedback,
    HandControlCommand,
    HandDiagResponse,
    HandFaultStatus,
    HandStatusFeedback,
)
from robotcan.simulation.mujoco.backend import ArmCommand, ArmState, HandCommand, HandState


@dataclass(slots=True)
class HandControllerOutputs:
    status: HandStatusFeedback
    fault: HandFaultStatus
    diag: HandDiagResponse


@dataclass(slots=True)
class ArmControllerOutputs:
    status: ArmStatusFeedback
    fault: ArmFaultStatus
    diag: ArmDiagResponse


def protocol_to_hand_command(command: HandControlCommand) -> HandCommand:
    return HandCommand(
        enable=command.enable,
        mode=command.mode,
        target_position_deg=command.target_position_deg,
        target_velocity_deg_s=command.target_velocity_deg_s,
        target_torque_nm=command.target_torque_nm,
    )


def protocol_to_arm_command(command: ArmControlCommand) -> ArmCommand:
    return ArmCommand(
        enable=command.enable,
        mode=command.mode,
        joint_targets_deg=[
            command.j1_target_deg,
            command.j2_target_deg,
            command.j3_target_deg,
            command.j4_target_deg,
            command.j5_target_deg,
            command.j6_target_deg,
        ],
    )


def hand_state_to_outputs(state: HandState) -> HandControllerOutputs:
    status = HandStatusFeedback(
        actual_position_deg=state.position_deg,
        actual_velocity_deg_s=state.velocity_deg_s,
        actual_torque_nm=state.torque_nm,
        motor_temperature_c=int(round(state.temperature_c)),
        error_code=state.error_code,
        enable_status=1 if state.enabled else 0,
        feedback_mode=state.mode,
    )
    fault = HandFaultStatus(
        fault_code=state.error_code,
        fault_severity=state.fault_severity,
        fault_source=state.fault_source,
        fault_detail=state.fault_detail,
    )
    diag = HandDiagResponse(
        service=0x22,
        sub_function=0x00,
        data0=int(state.position_deg) & 0xFF,
        data1=int(state.velocity_deg_s) & 0xFF,
        data2=int(state.torque_nm) & 0xFF,
        data3=int(state.temperature_c) & 0xFF,
        data4=state.error_code & 0xFF,
        data5=state.mode & 0xFF,
    )
    return HandControllerOutputs(status=status, fault=fault, diag=diag)


def arm_state_to_outputs(state: ArmState) -> ArmControllerOutputs:
    actual = list(state.joint_actual_deg) + [0.0] * (6 - len(state.joint_actual_deg))
    status = ArmStatusFeedback(
        j1_actual_deg=actual[0],
        j2_actual_deg=actual[1],
        j3_actual_deg=actual[2],
        j4_actual_deg=actual[3],
        j5_actual_deg=actual[4],
        j6_actual_deg=actual[5],
        enable_status=state.enable_status,
        feedback_mode=state.feedback_mode,
        error_code=state.error_code,
    )
    fault = ArmFaultStatus(
        fault_code=state.error_code,
        fault_severity=state.fault_severity,
        fault_source=state.fault_source,
        fault_detail=state.fault_detail,
    )
    diag = ArmDiagResponse(
        service=0x22,
        sub_function=0x00,
        data0=int(actual[0]) & 0xFF,
        data1=int(actual[1]) & 0xFF,
        data2=int(actual[2]) & 0xFF,
        data3=int(actual[3]) & 0xFF,
        data4=int(actual[4]) & 0xFF,
        data5=int(actual[5]) & 0xFF,
    )
    return ArmControllerOutputs(status=status, fault=fault, diag=diag)
