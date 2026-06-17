from __future__ import annotations

from dataclasses import dataclass

from robotcan.protocol import JointControlCommand, JointDiagResponse, JointFaultStatus, JointStatusFeedback
from robotcan.simulation.mujoco.backend import JointCommand, JointState


@dataclass(slots=True)
class ControllerOutputs:
    status: JointStatusFeedback
    fault: JointFaultStatus
    diag: JointDiagResponse


def protocol_to_joint_command(command: JointControlCommand) -> JointCommand:
    return JointCommand(
        enable=command.enable,
        mode=command.mode,
        target_position_deg=command.target_position_deg,
        target_velocity_deg_s=command.target_velocity_deg_s,
        target_torque_nm=command.target_torque_nm,
    )


def joint_state_to_outputs(state: JointState) -> ControllerOutputs:
    status = JointStatusFeedback(
        actual_position_deg=state.position_deg,
        actual_velocity_deg_s=state.velocity_deg_s,
        actual_torque_nm=state.torque_nm,
        motor_temperature_c=int(round(state.temperature_c)),
        error_code=state.error_code,
        enable_status=1 if state.enabled else 0,
        control_mode=state.mode,
    )
    fault = JointFaultStatus(
        error_code=state.error_code,
        severity=state.fault_severity,
        detail0=state.fault_source,
        detail1=state.fault_detail,
    )
    diag = JointDiagResponse(
        service=0x22,
        sub_function=0x00,
        data0=int(state.position_deg) & 0xFF,
        data1=int(state.velocity_deg_s) & 0xFF,
        data2=int(state.torque_nm) & 0xFF,
        data3=int(state.temperature_c) & 0xFF,
        data4=state.error_code & 0xFF,
        data5=state.mode & 0xFF,
    )
    return ControllerOutputs(status=status, fault=fault, diag=diag)
