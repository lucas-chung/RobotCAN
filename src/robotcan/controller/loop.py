from __future__ import annotations

import time
from dataclasses import dataclass, field

from robotcan.controller.bridge import joint_state_to_outputs, protocol_to_joint_command
from robotcan.protocol import CONTROL_COMMAND_ID, DIAG_RESPONSE_ID, FAULT_STATUS_ID, STATUS_FEEDBACK_ID
from robotcan.protocol.codec import decode_control_command
from robotcan.simulation.mujoco.backend import JointCommand, MujocoJointBackend
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
    _last_command: JointCommand = field(default_factory=JointCommand)

    def _consume_frames(self, frames: list[ReceivedFrame]) -> None:
        for frame in frames:
            if frame.identifier != CONTROL_COMMAND_ID:
                continue
            decoded = decode_control_command(frame.payload)
            self._last_command = protocol_to_joint_command(decoded)

    def run(self, duration_s: float | None = None) -> None:
        self.app.enable_fifo()
        print(
            "bridge started:"
            f" mujoco_loaded={self.backend.has_real_mujoco}"
            f" viewer={self.backend.viewer_enabled}"
            f" duration={duration_s if duration_s is not None else 'infinite'}"
        )
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

            self.backend.apply_command(self._last_command)
            self.backend.step(self.config.dt_s)
            state = self.backend.read_state()
            outputs = joint_state_to_outputs(state)

            if now - last_status >= self.config.status_period_s:
                self.app.send_canfd(STATUS_FEEDBACK_ID, outputs.status.encode())
                last_status = now
            if now - last_fault >= self.config.fault_period_s:
                self.app.send_canfd(FAULT_STATUS_ID, outputs.fault.encode())
                last_fault = now
            if now - last_diag >= self.config.diag_period_s:
                self.app.send_canfd(DIAG_RESPONSE_ID, outputs.diag.encode())
                last_diag = now

            time.sleep(self.config.dt_s)
