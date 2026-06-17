from __future__ import annotations

from dataclasses import dataclass
import struct


CONTROL_COMMAND_ID = 0x101
STATUS_FEEDBACK_ID = 0x201
FAULT_STATUS_ID = 0x301
DIAG_RESPONSE_ID = 0x401

CONTROL_DLC = 16
STATUS_DLC = 16
FAULT_DLC = 8
DIAG_DLC = 8


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(slots=True)
class JointControlCommand:
    enable: bool = True
    mode: int = 1
    target_position_deg: float = 0.0
    target_velocity_deg_s: float = 0.0
    target_torque_nm: float = 0.0

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
            0,
        )

    @classmethod
    def decode(cls, data: bytes) -> "JointControlCommand":
        if len(data) < CONTROL_DLC:
            raise ValueError(f"control payload must be at least {CONTROL_DLC} bytes")
        enable_raw, mode_raw, position_raw, velocity_raw, torque_raw, _ = struct.unpack(
            "<BBiihH2x",
            data[:CONTROL_DLC],
        )
        return cls(
            enable=bool(enable_raw),
            mode=mode_raw,
            target_position_deg=position_raw / 1000.0,
            target_velocity_deg_s=velocity_raw / 100.0,
            target_torque_nm=torque_raw / 100.0,
        )


@dataclass(slots=True)
class JointStatusFeedback:
    actual_position_deg: float
    actual_velocity_deg_s: float
    actual_torque_nm: float
    motor_temperature_c: int
    error_code: int
    enable_status: int
    control_mode: int

    def encode(self) -> bytes:
        position_raw = int(_clamp(self.actual_position_deg, -360.0, 360.0) * 1000)
        velocity_raw = int(_clamp(self.actual_velocity_deg_s, -5000.0, 5000.0) * 100)
        torque_raw = int(_clamp(self.actual_torque_nm, -200.0, 200.0) * 100)
        return struct.pack(
            "<iihBBBBH",
            position_raw,
            velocity_raw,
            torque_raw,
            int(_clamp(self.motor_temperature_c, 0, 255)),
            self.error_code & 0xFF,
            self.enable_status & 0xFF,
            self.control_mode & 0xFF,
            0,
        )

    @classmethod
    def decode(cls, data: bytes) -> "JointStatusFeedback":
        if len(data) < STATUS_DLC:
            raise ValueError(f"status payload must be at least {STATUS_DLC} bytes")
        position_raw, velocity_raw, torque_raw, temp_raw, err_raw, enable_raw, mode_raw, _ = struct.unpack(
            "<iihBBBBH",
            data[:STATUS_DLC],
        )
        return cls(
            actual_position_deg=position_raw / 1000.0,
            actual_velocity_deg_s=velocity_raw / 100.0,
            actual_torque_nm=torque_raw / 100.0,
            motor_temperature_c=temp_raw,
            error_code=err_raw,
            enable_status=enable_raw,
            control_mode=mode_raw,
        )


@dataclass(slots=True)
class JointFaultStatus:
    error_code: int
    severity: int
    detail0: int = 0
    detail1: int = 0

    def encode(self) -> bytes:
        return struct.pack(
            "<HBBI",
            self.error_code & 0xFFFF,
            self.severity & 0xFF,
            self.detail0 & 0xFF,
            self.detail1 & 0xFFFFFFFF,
        )

    @classmethod
    def decode(cls, data: bytes) -> "JointFaultStatus":
        if len(data) < FAULT_DLC:
            raise ValueError(f"fault payload must be at least {FAULT_DLC} bytes")
        error_code, severity, detail0, detail1 = struct.unpack("<HBBI", data[:FAULT_DLC])
        return cls(error_code=error_code, severity=severity, detail0=detail0, detail1=detail1)


@dataclass(slots=True)
class JointDiagResponse:
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
            "<BBBBBBBB",
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
    def decode(cls, data: bytes) -> "JointDiagResponse":
        if len(data) < DIAG_DLC:
            raise ValueError(f"diag payload must be at least {DIAG_DLC} bytes")
        return cls(*struct.unpack("<BBBBBBBB", data[:DIAG_DLC]))
