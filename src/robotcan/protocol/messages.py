from __future__ import annotations

from dataclasses import dataclass
import struct


HAND_CONTROL_COMMAND_ID = 0x101
HAND_STATUS_FEEDBACK_ID = 0x201
HAND_FAULT_STATUS_ID = 0x301
HAND_DIAG_RESPONSE_ID = 0x401

ARM_CONTROL_COMMAND_ID = 0x110
ARM_STATUS_FEEDBACK_ID = 0x210
ARM_FAULT_STATUS_ID = 0x310
ARM_DIAG_RESPONSE_ID = 0x410

HAND_CONTROL_DLC = 16
HAND_STATUS_DLC = 16
HAND_FAULT_DLC = 16
HAND_DIAG_DLC = 16

ARM_CONTROL_DLC = 16
ARM_STATUS_DLC = 16
ARM_FAULT_DLC = 16
ARM_DIAG_DLC = 16


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(slots=True)
class HandControlCommand:
    enable: bool = True
    mode: int = 1
    target_position_deg: float = 0.0
    target_velocity_deg_s: float = 0.0
    target_torque_nm: float = 0.0
    reserved_control: int = 0

    def encode(self) -> bytes:
        position_raw = int(_clamp(self.target_position_deg, -360.0, 360.0) * 1000)
        velocity_raw = int(_clamp(self.target_velocity_deg_s, -5000.0, 5000.0) * 100)
        torque_raw = int(_clamp(self.target_torque_nm, -200.0, 200.0) * 100)
        enable_raw = 1 if self.enable else 0
        return struct.pack(
            "<BBiihH2x",
            enable_raw,
            self.mode & 0xFF,
            position_raw,
            velocity_raw,
            torque_raw,
            self.reserved_control & 0xFFFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "HandControlCommand":
        if len(data) < HAND_CONTROL_DLC:
            raise ValueError(f"hand control payload must be at least {HAND_CONTROL_DLC} bytes")
        enable_raw, mode_raw, position_raw, velocity_raw, torque_raw, reserved_control = struct.unpack(
            "<BBiihH2x",
            data[:HAND_CONTROL_DLC],
        )
        return cls(
            enable=bool(enable_raw),
            mode=mode_raw,
            target_position_deg=position_raw / 1000.0,
            target_velocity_deg_s=velocity_raw / 100.0,
            target_torque_nm=torque_raw / 100.0,
            reserved_control=reserved_control,
        )


@dataclass(slots=True)
class HandStatusFeedback:
    actual_position_deg: float
    actual_velocity_deg_s: float
    actual_torque_nm: float
    motor_temperature_c: int
    error_code: int
    enable_status: int
    feedback_mode: int
    reserved_status: int = 0

    def encode(self) -> bytes:
        position_raw = int(_clamp(self.actual_position_deg, -360.0, 360.0) * 1000)
        velocity_raw = int(_clamp(self.actual_velocity_deg_s, -5000.0, 5000.0) * 100)
        torque_raw = int(_clamp(self.actual_torque_nm, -200.0, 200.0) * 100)
        return struct.pack(
            "<iihBBBBH",
            position_raw,
            velocity_raw,
            torque_raw,
            int(_clamp(self.motor_temperature_c, 0, 200)),
            self.error_code & 0xFF,
            self.enable_status & 0xFF,
            self.feedback_mode & 0xFF,
            self.reserved_status & 0xFFFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "HandStatusFeedback":
        if len(data) < HAND_STATUS_DLC:
            raise ValueError(f"hand status payload must be at least {HAND_STATUS_DLC} bytes")
        position_raw, velocity_raw, torque_raw, temp_raw, err_raw, enable_raw, mode_raw, reserved_status = struct.unpack(
            "<iihBBBBH",
            data[:HAND_STATUS_DLC],
        )
        return cls(
            actual_position_deg=position_raw / 1000.0,
            actual_velocity_deg_s=velocity_raw / 100.0,
            actual_torque_nm=torque_raw / 100.0,
            motor_temperature_c=temp_raw,
            error_code=err_raw,
            enable_status=enable_raw,
            feedback_mode=mode_raw,
            reserved_status=reserved_status,
        )


@dataclass(slots=True)
class HandFaultStatus:
    fault_code: int
    fault_severity: int
    fault_source: int = 0
    fault_detail: int = 0

    def encode(self) -> bytes:
        return struct.pack(
            "<HBBI8x",
            self.fault_code & 0xFFFF,
            self.fault_severity & 0xFF,
            self.fault_source & 0xFF,
            self.fault_detail & 0xFFFFFFFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "HandFaultStatus":
        if len(data) < HAND_FAULT_DLC:
            raise ValueError(f"hand fault payload must be at least {HAND_FAULT_DLC} bytes")
        fault_code, fault_severity, fault_source, fault_detail = struct.unpack("<HBBI", data[:8])
        return cls(
            fault_code=fault_code,
            fault_severity=fault_severity,
            fault_source=fault_source,
            fault_detail=fault_detail,
        )


@dataclass(slots=True)
class HandDiagResponse:
    service: int = 0
    sub_function: int = 0
    data0: int = 0
    data1: int = 0
    data2: int = 0
    data3: int = 0
    data4: int = 0
    data5: int = 0

    def encode(self) -> bytes:
        return struct.pack(
            "<BBBBBBBB8x",
            self.service & 0xFF,
            self.sub_function & 0xFF,
            self.data0 & 0xFF,
            self.data1 & 0xFF,
            self.data2 & 0xFF,
            self.data3 & 0xFF,
            self.data4 & 0xFF,
            self.data5 & 0xFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "HandDiagResponse":
        if len(data) < HAND_DIAG_DLC:
            raise ValueError(f"hand diag payload must be at least {HAND_DIAG_DLC} bytes")
        return cls(*struct.unpack("<BBBBBBBB", data[:8]))


@dataclass(slots=True)
class ArmControlCommand:
    enable: bool = True
    mode: int = 1
    j1_target_deg: float = 0.0
    j2_target_deg: float = 0.0
    j3_target_deg: float = 0.0
    j4_target_deg: float = 0.0
    j5_target_deg: float = 0.0
    j6_target_deg: float = 0.0
    reserved_control: int = 0

    def encode(self) -> bytes:
        return struct.pack(
            "<BB6hH",
            1 if self.enable else 0,
            self.mode & 0xFF,
            int(_clamp(self.j1_target_deg, -360.0, 360.0) * 10),
            int(_clamp(self.j2_target_deg, -360.0, 360.0) * 10),
            int(_clamp(self.j3_target_deg, -360.0, 360.0) * 10),
            int(_clamp(self.j4_target_deg, -360.0, 360.0) * 10),
            int(_clamp(self.j5_target_deg, -360.0, 360.0) * 10),
            int(_clamp(self.j6_target_deg, -360.0, 360.0) * 10),
            self.reserved_control & 0xFFFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "ArmControlCommand":
        if len(data) < ARM_CONTROL_DLC:
            raise ValueError(f"arm control payload must be at least {ARM_CONTROL_DLC} bytes")
        unpacked = struct.unpack("<BB6hH", data[:ARM_CONTROL_DLC])
        return cls(
            enable=bool(unpacked[0]),
            mode=unpacked[1],
            j1_target_deg=unpacked[2] / 10.0,
            j2_target_deg=unpacked[3] / 10.0,
            j3_target_deg=unpacked[4] / 10.0,
            j4_target_deg=unpacked[5] / 10.0,
            j5_target_deg=unpacked[6] / 10.0,
            j6_target_deg=unpacked[7] / 10.0,
            reserved_control=unpacked[8],
        )


@dataclass(slots=True)
class ArmStatusFeedback:
    j1_actual_deg: float = 0.0
    j2_actual_deg: float = 0.0
    j3_actual_deg: float = 0.0
    j4_actual_deg: float = 0.0
    j5_actual_deg: float = 0.0
    j6_actual_deg: float = 0.0
    enable_status: int = 0
    feedback_mode: int = 0
    error_code: int = 0

    def encode(self) -> bytes:
        return struct.pack(
            "<6hBBH",
            int(_clamp(self.j1_actual_deg, -360.0, 360.0) * 10),
            int(_clamp(self.j2_actual_deg, -360.0, 360.0) * 10),
            int(_clamp(self.j3_actual_deg, -360.0, 360.0) * 10),
            int(_clamp(self.j4_actual_deg, -360.0, 360.0) * 10),
            int(_clamp(self.j5_actual_deg, -360.0, 360.0) * 10),
            int(_clamp(self.j6_actual_deg, -360.0, 360.0) * 10),
            self.enable_status & 0xFF,
            self.feedback_mode & 0xFF,
            self.error_code & 0xFFFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "ArmStatusFeedback":
        if len(data) < ARM_STATUS_DLC:
            raise ValueError(f"arm status payload must be at least {ARM_STATUS_DLC} bytes")
        unpacked = struct.unpack("<6hBBH", data[:ARM_STATUS_DLC])
        return cls(
            j1_actual_deg=unpacked[0] / 10.0,
            j2_actual_deg=unpacked[1] / 10.0,
            j3_actual_deg=unpacked[2] / 10.0,
            j4_actual_deg=unpacked[3] / 10.0,
            j5_actual_deg=unpacked[4] / 10.0,
            j6_actual_deg=unpacked[5] / 10.0,
            enable_status=unpacked[6],
            feedback_mode=unpacked[7],
            error_code=unpacked[8],
        )


@dataclass(slots=True)
class ArmFaultStatus:
    fault_code: int
    fault_severity: int
    fault_source: int = 0
    fault_detail: int = 0

    def encode(self) -> bytes:
        return struct.pack(
            "<HBBI8x",
            self.fault_code & 0xFFFF,
            self.fault_severity & 0xFF,
            self.fault_source & 0xFF,
            self.fault_detail & 0xFFFFFFFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "ArmFaultStatus":
        if len(data) < ARM_FAULT_DLC:
            raise ValueError(f"arm fault payload must be at least {ARM_FAULT_DLC} bytes")
        fault_code, fault_severity, fault_source, fault_detail = struct.unpack("<HBBI", data[:8])
        return cls(
            fault_code=fault_code,
            fault_severity=fault_severity,
            fault_source=fault_source,
            fault_detail=fault_detail,
        )


@dataclass(slots=True)
class ArmDiagResponse:
    service: int = 0
    sub_function: int = 0
    data0: int = 0
    data1: int = 0
    data2: int = 0
    data3: int = 0
    data4: int = 0
    data5: int = 0

    def encode(self) -> bytes:
        return struct.pack(
            "<BBBBBBBB8x",
            self.service & 0xFF,
            self.sub_function & 0xFF,
            self.data0 & 0xFF,
            self.data1 & 0xFF,
            self.data2 & 0xFF,
            self.data3 & 0xFF,
            self.data4 & 0xFF,
            self.data5 & 0xFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "ArmDiagResponse":
        if len(data) < ARM_DIAG_DLC:
            raise ValueError(f"arm diag payload must be at least {ARM_DIAG_DLC} bytes")
        return cls(*struct.unpack("<BBBBBBBB", data[:8]))


# Compatibility aliases for older imports.
CONTROL_COMMAND_ID = HAND_CONTROL_COMMAND_ID
STATUS_FEEDBACK_ID = HAND_STATUS_FEEDBACK_ID
FAULT_STATUS_ID = HAND_FAULT_STATUS_ID
DIAG_RESPONSE_ID = HAND_DIAG_RESPONSE_ID
CONTROL_DLC = HAND_CONTROL_DLC
STATUS_DLC = HAND_STATUS_DLC
FAULT_DLC = HAND_FAULT_DLC
DIAG_DLC = HAND_DIAG_DLC
JointControlCommand = HandControlCommand
JointStatusFeedback = HandStatusFeedback
JointFaultStatus = HandFaultStatus
JointDiagResponse = HandDiagResponse
