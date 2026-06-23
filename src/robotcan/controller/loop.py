from __future__ import annotations

import time
from dataclasses import dataclass, field

from robotcan.controller.bridge import (
    arm_state_to_outputs,
    hand_state_to_outputs,
    protocol_to_arm_command,
    protocol_to_hand_command,
)
from robotcan.protocol import (
    ARM_CONTROL_COMMAND_ID,
    ARM_DIAG_RESPONSE_ID,
    ARM_FAULT_STATUS_ID,
    ARM_STATUS_FEEDBACK_ID,
    HAND_CONTROL_COMMAND_ID,
    HAND_DIAG_RESPONSE_ID,
    HAND_FAULT_STATUS_ID,
    HAND_STATUS_FEEDBACK_ID,
)
from robotcan.protocol.codec import decode_arm_control_command, decode_hand_control_command
from robotcan.simulation.mujoco.backend import ArmCommand, HandCommand, MujocoJointBackend
from robotcan.simulation.mujoco.camera_preview import MujocoCameraPreview
from robotcan.transport.tsmaster_attached import AttachedTSMaster, ReceivedFrame


@dataclass(slots=True)
class BridgeLoopConfig:
    dt_s: float = 0.01
    status_period_s: float = 0.01
    fault_period_s: float = 0.1
    diag_period_s: float = 0.1
    only_rx: bool = True
    fifo_batch_size: int = 100


@dataclass(slots=True)
class TSMasterMujocoBridgeLoop:
    app: AttachedTSMaster
    backend: MujocoJointBackend
    config: BridgeLoopConfig = field(default_factory=BridgeLoopConfig)
    camera_preview: MujocoCameraPreview | None = None
    _last_hand_command: HandCommand = field(default_factory=HandCommand)
    _last_arm_command: ArmCommand = field(default_factory=ArmCommand)

    def _consume_frames(self, frames: list[ReceivedFrame]) -> None:
        for frame in frames:
            if frame.identifier == HAND_CONTROL_COMMAND_ID:
                decoded = decode_hand_control_command(frame.payload)
                self._last_hand_command = protocol_to_hand_command(decoded)
            elif frame.identifier == ARM_CONTROL_COMMAND_ID:
                decoded = decode_arm_control_command(frame.payload)
                self._last_arm_command = protocol_to_arm_command(decoded)

    def run(self, duration_s: float | None = None) -> None:
        self.app.enable_fifo()
        print(
            "bridge started:"
            f" mujoco_loaded={self.backend.has_real_mujoco}"
            f" viewer={self.backend.viewer_enabled}"
            f" duration={duration_s if duration_s is not None else 'infinite'}"
        )
        if self.camera_preview is not None:
            print("camera preview opened")
            self.camera_preview.update(min_period_s=0.0)
        start_time = time.perf_counter()
        last_status = start_time
        last_fault = start_time
        last_diag = start_time

        while True:
            now = time.perf_counter()
            if duration_s is not None and now - start_time >= duration_s:
                break

            frames = self.app.receive_canfd(max_count=self.config.fifo_batch_size, only_rx=self.config.only_rx)
            self._consume_frames(frames)

            self.backend.apply_hand_command(self._last_hand_command)
            self.backend.apply_arm_command(self._last_arm_command)
            self.backend.step(self.config.dt_s)
            if self.camera_preview is not None:
                self.camera_preview.update()

            hand_outputs = hand_state_to_outputs(self.backend.read_hand_state())
            arm_outputs = arm_state_to_outputs(self.backend.read_arm_state())

            if now - last_status >= self.config.status_period_s:
                self.app.send_canfd(HAND_STATUS_FEEDBACK_ID, hand_outputs.status.encode())
                self.app.send_canfd(ARM_STATUS_FEEDBACK_ID, arm_outputs.status.encode())
                last_status = now
            if now - last_fault >= self.config.fault_period_s:
                self.app.send_canfd(HAND_FAULT_STATUS_ID, hand_outputs.fault.encode())
                self.app.send_canfd(ARM_FAULT_STATUS_ID, arm_outputs.fault.encode())
                last_fault = now
            if now - last_diag >= self.config.diag_period_s:
                self.app.send_canfd(HAND_DIAG_RESPONSE_ID, hand_outputs.diag.encode())
                self.app.send_canfd(ARM_DIAG_RESPONSE_ID, arm_outputs.diag.encode())
                last_diag = now

            time.sleep(self.config.dt_s)
