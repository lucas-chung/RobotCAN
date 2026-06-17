from __future__ import annotations

from enum import IntEnum


class ControlMode(IntEnum):
    POSITION = 1
    VELOCITY = 2
    TORQUE = 3


class FaultSeverity(IntEnum):
    INFO = 0
    WARNING = 1
    ERROR = 2
    FATAL = 3
