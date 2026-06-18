from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import math


UR5E_HOME_CTRL = (-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0)
ARM_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
ARM_ACTUATOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
)


@dataclass(slots=True)
class HandCommand:
    enable: bool = False
    mode: int = 1
    target_position_deg: float = 0.0
    target_velocity_deg_s: float = 0.0
    target_torque_nm: float = 0.0


@dataclass(slots=True)
class HandState:
    position_deg: float = 0.0
    velocity_deg_s: float = 0.0
    torque_nm: float = 0.0
    temperature_c: float = 25.0
    error_code: int = 0
    enabled: bool = False
    mode: int = 1
    fault_severity: int = 0
    fault_source: int = 0
    fault_detail: int = 0


@dataclass(slots=True)
class ArmCommand:
    enable: bool = False
    mode: int = 1
    joint_targets_deg: list[float] = field(default_factory=lambda: [0.0] * 6)


@dataclass(slots=True)
class ArmState:
    joint_actual_deg: list[float] = field(default_factory=lambda: [0.0] * 6)
    enable_status: int = 0
    feedback_mode: int = 0
    error_code: int = 0
    fault_severity: int = 0
    fault_source: int = 0
    fault_detail: int = 0


class MujocoJointBackend:
    def __init__(self, model_path: str | None = None, enable_viewer: bool = True):
        self.model_path = Path(model_path).resolve() if model_path else None
        self.enable_viewer = enable_viewer
        self._mujoco = None
        self._viewer_module = None
        self._model = None
        self._data = None
        self._viewer = None

        self._hand_command = HandCommand()
        self._hand_state = HandState()
        self._arm_command = ArmCommand(joint_targets_deg=list(math.degrees(value) for value in UR5E_HOME_CTRL))
        self._arm_state = ArmState(joint_actual_deg=list(math.degrees(value) for value in UR5E_HOME_CTRL))

        self._hand_joint_ids: list[int] = []
        self._hand_actuator_ids: list[int] = []
        self._hand_joint_qpos_adrs: list[int] = []
        self._hand_joint_dof_adrs: list[int] = []
        self._hand_uses_percent_ctrl = False
        self._hand_target_native = 0.0

        self._arm_joint_ids: list[int] = []
        self._arm_actuator_ids: list[int] = []
        self._arm_joint_qpos_adrs: list[int] = []
        self._arm_home_ctrl: list[float] = list(UR5E_HOME_CTRL)
        self._arm_target_ctrl: list[float] = list(UR5E_HOME_CTRL)
        if self.model_path:
            self._try_load_mujoco_model(self.model_path)

    def _try_load_mujoco_model(self, model_path: Path) -> None:
        try:
            import mujoco  # type: ignore
        except ImportError:
            if self.enable_viewer:
                raise RuntimeError(
                    "MuJoCo is not installed in the active environment. "
                    "Install it first with: "
                    r"D:\data\RobotCAN\RobotCAN\Scripts\python.exe -m pip install mujoco"
                )
            return

        self._mujoco = mujoco
        try:
            import mujoco.viewer as viewer  # type: ignore
        except ImportError:
            viewer = None
        self._viewer_module = viewer
        self._model = mujoco.MjModel.from_xml_path(str(model_path))
        self._data = mujoco.MjData(self._model)

        self._hand_joint_ids = self._resolve_hand_joint_ids()
        self._hand_actuator_ids = self._resolve_hand_actuator_ids()
        self._hand_joint_qpos_adrs = [int(self._model.jnt_qposadr[joint_id]) for joint_id in self._hand_joint_ids]
        self._hand_joint_dof_adrs = [int(self._model.jnt_dofadr[joint_id]) for joint_id in self._hand_joint_ids]
        self._hand_uses_percent_ctrl = bool(self._hand_actuator_ids) and self._actuator_name(self._hand_actuator_ids[0]) == "fingers_actuator"

        self._arm_joint_ids = [self._name_to_id(self._mujoco.mjtObj.mjOBJ_JOINT, name) for name in ARM_JOINT_NAMES]
        self._arm_actuator_ids = [self._name_to_id(self._mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ARM_ACTUATOR_NAMES]
        self._arm_joint_qpos_adrs = [int(self._model.jnt_qposadr[joint_id]) for joint_id in self._arm_joint_ids]

        self._arm_target_ctrl = list(self._arm_home_ctrl)
        for actuator_id, ctrl_value in zip(self._arm_actuator_ids, self._arm_home_ctrl):
            self._data.ctrl[actuator_id] = ctrl_value

        closed_native = self._hand_command_position_to_native(0.0)
        for qpos_adr in self._hand_joint_qpos_adrs:
            self._data.qpos[qpos_adr] = closed_native
        for dof_adr in self._hand_joint_dof_adrs:
            if self._model.nv > 0:
                self._data.qvel[dof_adr] = 0.0
        self._hand_target_native = self._hand_command_position_to_actuator_ctrl(0.0)
        if self._hand_actuator_ids:
            self._data.ctrl[self._hand_actuator_ids[0]] = self._hand_target_native
        mujoco.mj_forward(self._model, self._data)
        self._launch_viewer_if_needed()

    def _name_to_id(self, obj_type, name: str) -> int:
        assert self._mujoco is not None
        assert self._model is not None
        return int(self._mujoco.mj_name2id(self._model, obj_type, name))

    def _resolve_hand_joint_ids(self) -> list[int]:
        assert self._mujoco is not None
        assert self._model is not None
        candidates = (
            ("left_gripper_hinge", "right_gripper_hinge"),
            ("left_gripper_slide", "right_gripper_slide"),
            ("left_driver_joint", "right_driver_joint"),
        )
        for left_name, right_name in candidates:
            left_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_JOINT, left_name)
            right_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_JOINT, right_name)
            if left_id >= 0 and right_id >= 0:
                return [left_id, right_id]
        raise RuntimeError("Unable to resolve hand joints in MuJoCo model.")

    def _resolve_hand_actuator_ids(self) -> list[int]:
        assert self._mujoco is not None
        fingers_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_ACTUATOR, "fingers_actuator")
        if fingers_id >= 0:
            return [fingers_id]
        left_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_ACTUATOR, "left_gripper_motor")
        right_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_ACTUATOR, "right_gripper_motor")
        if left_id >= 0 and right_id >= 0:
            return [left_id, right_id]
        raise RuntimeError("Unable to resolve hand actuators in MuJoCo model.")

    def _actuator_name(self, actuator_id: int) -> str:
        assert self._mujoco is not None
        assert self._model is not None
        name = self._mujoco.mj_id2name(self._model, self._mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        return "" if name is None else str(name)

    def _launch_viewer_if_needed(self) -> None:
        if not self.enable_viewer or self._viewer is not None:
            return
        if self._mujoco is None or self._model is None or self._data is None:
            return
        if self._viewer_module is None or not hasattr(self._viewer_module, "launch_passive"):
            raise RuntimeError("The installed MuJoCo package does not provide mujoco.viewer.launch_passive().")
        self._viewer = self._viewer_module.launch_passive(self._model, self._data)

    def close(self) -> None:
        if self._viewer is not None:
            close = getattr(self._viewer, "close", None)
            if callable(close):
                close()
            self._viewer = None

    @property
    def has_real_mujoco(self) -> bool:
        return self._model is not None and self._data is not None and self._mujoco is not None

    @property
    def viewer_enabled(self) -> bool:
        return self._viewer is not None

    def apply_hand_command(self, command: HandCommand) -> None:
        self._hand_command = command
        self._hand_state.enabled = command.enable
        self._hand_state.mode = command.mode

    def apply_arm_command(self, command: ArmCommand) -> None:
        self._arm_command = command
        self._arm_state.enable_status = 1 if command.enable else 0
        self._arm_state.feedback_mode = command.mode

    def step(self, dt_s: float) -> None:
        if self._model is not None and self._data is not None and self._mujoco is not None:
            self._step_mujoco(dt_s)
            return
        self._step_fallback(dt_s)

    def _hand_command_position_to_native(self, position_command: float) -> float:
        assert self._model is not None
        joint_range = self._model.jnt_range[self._hand_joint_ids[0]]
        minimum = float(joint_range[0])
        maximum = float(joint_range[1])
        normalized = max(0.0, min(100.0, position_command)) / 100.0
        return maximum - normalized * (maximum - minimum)

    def _hand_command_position_to_actuator_ctrl(self, position_command: float) -> float:
        normalized = max(0.0, min(100.0, position_command)) / 100.0
        if not self._hand_uses_percent_ctrl:
            return self._hand_command_position_to_native(position_command)
        assert self._model is not None
        ctrl_range = self._model.actuator_ctrlrange[self._hand_actuator_ids[0]]
        minimum = float(ctrl_range[0])
        maximum = float(ctrl_range[1])
        return maximum - normalized * (maximum - minimum)

    def _hand_native_position_to_status(self, native_value: float) -> float:
        assert self._model is not None
        joint_range = self._model.jnt_range[self._hand_joint_ids[0]]
        minimum = float(joint_range[0])
        maximum = float(joint_range[1])
        if maximum <= minimum:
            return 0.0
        return max(0.0, min(100.0, (maximum - native_value) / (maximum - minimum) * 100.0))

    def _hand_native_velocity_to_status(self, native_value: float) -> float:
        assert self._model is not None
        joint_range = self._model.jnt_range[self._hand_joint_ids[0]]
        span = float(joint_range[1] - joint_range[0])
        if span <= 1e-9:
            return 0.0
        return -(native_value / span) * 100.0

    def _step_mujoco(self, dt_s: float) -> None:
        assert self._mujoco is not None
        assert self._model is not None
        assert self._data is not None

        current_hand_native_position = sum(float(self._data.qpos[qpos_adr]) for qpos_adr in self._hand_joint_qpos_adrs) / len(self._hand_joint_qpos_adrs)
        current_hand_native_velocity = sum(float(self._data.qvel[dof_adr]) for dof_adr in self._hand_joint_dof_adrs) / len(self._hand_joint_dof_adrs)
        current_hand_position = self._hand_native_position_to_status(current_hand_native_position)

        if not self._hand_command.enable:
            self._hand_target_native = self._hand_command_position_to_actuator_ctrl(current_hand_position)
        elif self._hand_command.mode == 1:
            self._hand_target_native = self._hand_command_position_to_actuator_ctrl(self._hand_command.target_position_deg)
        elif self._hand_command.mode == 2:
            next_position = current_hand_position + self._hand_command.target_velocity_deg_s * dt_s
            self._hand_target_native = self._hand_command_position_to_actuator_ctrl(next_position)
        else:
            next_position = current_hand_position + self._hand_command.target_torque_nm * 0.5 * dt_s
            self._hand_target_native = self._hand_command_position_to_actuator_ctrl(next_position)
        self._data.ctrl[self._hand_actuator_ids[0]] = self._hand_target_native

        if not self._arm_command.enable:
            self._arm_target_ctrl = list(self._arm_home_ctrl)
        elif self._arm_command.mode == 1:
            self._arm_target_ctrl = [math.radians(value) for value in self._arm_command.joint_targets_deg]
        elif self._arm_command.mode == 2:
            for index, target_deg_s in enumerate(self._arm_command.joint_targets_deg):
                self._arm_target_ctrl[index] += math.radians(target_deg_s) * dt_s
        else:
            for index, torque_hint in enumerate(self._arm_command.joint_targets_deg):
                self._arm_target_ctrl[index] += torque_hint * 0.0005

        for index, actuator_id in enumerate(self._arm_actuator_ids):
            ctrl_range = self._model.actuator_ctrlrange[actuator_id]
            self._arm_target_ctrl[index] = max(float(ctrl_range[0]), min(float(ctrl_range[1]), self._arm_target_ctrl[index]))
            self._data.ctrl[actuator_id] = self._arm_target_ctrl[index]

        timestep = float(self._model.opt.timestep)
        steps = max(1, round(dt_s / max(timestep, 1e-6)))
        for _ in range(steps):
            self._mujoco.mj_step(self._model, self._data)

        current_hand_native_position = sum(float(self._data.qpos[qpos_adr]) for qpos_adr in self._hand_joint_qpos_adrs) / len(self._hand_joint_qpos_adrs)
        current_hand_native_velocity = sum(float(self._data.qvel[dof_adr]) for dof_adr in self._hand_joint_dof_adrs) / len(self._hand_joint_dof_adrs)
        self._hand_state.position_deg = self._hand_native_position_to_status(current_hand_native_position)
        self._hand_state.velocity_deg_s = self._hand_native_velocity_to_status(current_hand_native_velocity)
        self._hand_state.torque_nm = 0.0
        if hasattr(self._data, "actuator_force") and len(self._data.actuator_force) > self._hand_actuator_ids[0]:
            self._hand_state.torque_nm = float(self._data.actuator_force[self._hand_actuator_ids[0]])
        self._hand_state.temperature_c = min(120.0, self._hand_state.temperature_c + abs(self._hand_state.torque_nm) * 0.005)
        self._hand_state.error_code = 0 if self._hand_state.temperature_c < 100 else 2
        self._hand_state.fault_severity = 0 if self._hand_state.error_code == 0 else 2
        self._hand_state.fault_source = 3 if self._hand_state.error_code else 0
        self._hand_state.fault_detail = int(self._hand_state.temperature_c * 10) if self._hand_state.error_code else 0

        self._arm_state.joint_actual_deg = [math.degrees(float(self._data.qpos[qpos_adr])) for qpos_adr in self._arm_joint_qpos_adrs]
        arm_has_out_of_range = any(abs(value) > 360.0 for value in self._arm_state.joint_actual_deg)
        self._arm_state.error_code = 1 if arm_has_out_of_range else 0
        self._arm_state.fault_severity = 2 if self._arm_state.error_code else 0
        self._arm_state.fault_source = 1 if self._arm_state.error_code else 0
        self._arm_state.fault_detail = int(max(abs(value) for value in self._arm_state.joint_actual_deg)) if self._arm_state.error_code else 0
        self._sync_viewer()

    def _sync_viewer(self) -> None:
        if self._viewer is None:
            return
        sync = getattr(self._viewer, "sync", None)
        if callable(sync):
            sync()

    def _step_fallback(self, dt_s: float) -> None:
        if self._hand_command.enable:
            position_error = self._hand_command.target_position_deg - self._hand_state.position_deg
            self._hand_state.velocity_deg_s = position_error * 5.0
            self._hand_state.position_deg += self._hand_state.velocity_deg_s * dt_s
            self._hand_state.torque_nm = max(-50.0, min(50.0, position_error * 0.3))
        else:
            self._hand_state.velocity_deg_s *= 0.8
            self._hand_state.torque_nm = 0.0
        self._hand_state.temperature_c = min(120.0, self._hand_state.temperature_c + abs(self._hand_state.torque_nm) * 0.005)

        if self._arm_command.enable:
            self._arm_state.joint_actual_deg = [
                current + (target - current) * min(1.0, dt_s * 5.0)
                for current, target in zip(self._arm_state.joint_actual_deg, self._arm_command.joint_targets_deg)
            ]

    def read_hand_state(self) -> HandState:
        return HandState(
            position_deg=self._hand_state.position_deg,
            velocity_deg_s=self._hand_state.velocity_deg_s,
            torque_nm=self._hand_state.torque_nm,
            temperature_c=self._hand_state.temperature_c,
            error_code=self._hand_state.error_code,
            enabled=self._hand_state.enabled,
            mode=self._hand_state.mode,
            fault_severity=self._hand_state.fault_severity,
            fault_source=self._hand_state.fault_source,
            fault_detail=self._hand_state.fault_detail,
        )

    def read_arm_state(self) -> ArmState:
        return ArmState(
            joint_actual_deg=list(self._arm_state.joint_actual_deg),
            enable_status=self._arm_state.enable_status,
            feedback_mode=self._arm_state.feedback_mode,
            error_code=self._arm_state.error_code,
            fault_severity=self._arm_state.fault_severity,
            fault_source=self._arm_state.fault_source,
            fault_detail=self._arm_state.fault_detail,
        )
