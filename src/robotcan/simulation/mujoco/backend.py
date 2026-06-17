from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math


UR5E_HOME_CTRL = (-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0)


@dataclass(slots=True)
class JointCommand:
    enable: bool = False
    mode: int = 1
    target_position_deg: float = 0.0
    target_velocity_deg_s: float = 0.0
    target_torque_nm: float = 0.0


@dataclass(slots=True)
class JointState:
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


class MujocoJointBackend:
    def __init__(self, model_path: str | None = None, enable_viewer: bool = True):
        self.model_path = Path(model_path).resolve() if model_path else None
        self.enable_viewer = enable_viewer
        self._command = JointCommand()
        self._state = JointState()
        self._mujoco = None
        self._viewer_module = None
        self._model = None
        self._data = None
        self._viewer = None
        self._joint_ids: list[int] = []
        self._actuator_ids: list[int] = []
        self._joint_qpos_adrs: list[int] = []
        self._joint_dof_adrs: list[int] = []
        self._arm_actuator_ids: list[int] = []
        self._arm_home_ctrl: list[float] = list(UR5E_HOME_CTRL)
        self._is_gripper = False
        self._uses_gripper_percent_ctrl = False
        self._position_min = -180.0
        self._position_max = 180.0
        self._gripper_target_native = 0.0
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
        self._joint_ids = self._resolve_joint_ids()
        self._actuator_ids = self._resolve_actuator_ids()
        self._joint_qpos_adrs = [int(self._model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids]
        self._joint_dof_adrs = [int(self._model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids]
        self._arm_actuator_ids = self._resolve_arm_actuator_ids()
        self._is_gripper = len(self._joint_ids) > 1
        self._uses_gripper_percent_ctrl = self._actuator_name(self._actuator_ids[0]) == "fingers_actuator"
        self._position_min, self._position_max = self._detect_command_range()

        if self._arm_actuator_ids:
            for actuator_id, ctrl_value in zip(self._arm_actuator_ids, self._arm_home_ctrl):
                self._data.ctrl[actuator_id] = ctrl_value

        if self._is_gripper:
            closed_native = self._command_position_to_native(0.0)
            for qpos_adr in self._joint_qpos_adrs:
                self._data.qpos[qpos_adr] = closed_native
            for dof_adr in self._joint_dof_adrs:
                if self._model.nv > 0:
                    self._data.qvel[dof_adr] = 0.0
            self._gripper_target_native = self._command_position_to_actuator_ctrl(0.0)
            self._data.ctrl[self._actuator_ids[0]] = self._gripper_target_native
            mujoco.mj_forward(self._model, self._data)

        self._launch_viewer_if_needed()

    def _name_to_id(self, obj_type, name: str) -> int:
        assert self._mujoco is not None
        assert self._model is not None
        return int(self._mujoco.mj_name2id(self._model, obj_type, name))

    def _resolve_joint_ids(self) -> list[int]:
        assert self._mujoco is not None
        assert self._model is not None

        joint_pairs = (
            ("left_gripper_hinge", "right_gripper_hinge"),
            ("left_gripper_slide", "right_gripper_slide"),
            ("left_driver_joint", "right_driver_joint"),
        )
        for left_name, right_name in joint_pairs:
            left_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_JOINT, left_name)
            right_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_JOINT, right_name)
            if left_id >= 0 and right_id >= 0:
                return [left_id, right_id]

        return [
            self._resolve_named_or_first_id(
                self._mujoco.mjtObj.mjOBJ_JOINT,
                ("joint1", "gripper_slide", "right_driver_joint"),
                self._model.njnt,
            )
        ]

    def _resolve_actuator_ids(self) -> list[int]:
        assert self._mujoco is not None
        assert self._model is not None

        fingers_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_ACTUATOR, "fingers_actuator")
        if fingers_id >= 0:
            return [fingers_id]

        left_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_ACTUATOR, "left_gripper_motor")
        right_id = self._name_to_id(self._mujoco.mjtObj.mjOBJ_ACTUATOR, "right_gripper_motor")
        if left_id >= 0 and right_id >= 0:
            return [left_id, right_id]

        return [
            self._resolve_named_or_first_id(
                self._mujoco.mjtObj.mjOBJ_ACTUATOR,
                ("joint1_motor", "gripper_motor"),
                self._model.nu,
            )
        ]

    def _resolve_arm_actuator_ids(self) -> list[int]:
        assert self._mujoco is not None
        names = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")
        ids = [self._name_to_id(self._mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in names]
        if all(actuator_id >= 0 for actuator_id in ids):
            return ids
        return []

    def _resolve_named_or_first_id(self, obj_type, names: tuple[str, ...], count: int) -> int:
        assert self._mujoco is not None
        assert self._model is not None
        for name in names:
            obj_id = self._name_to_id(obj_type, name)
            if obj_id >= 0:
                return obj_id
        if count <= 0:
            raise RuntimeError(f"MuJoCo model does not contain required object type {obj_type}.")
        return 0

    def _actuator_name(self, actuator_id: int) -> str:
        assert self._mujoco is not None
        assert self._model is not None
        name = self._mujoco.mj_id2name(self._model, self._mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        return "" if name is None else str(name)

    def _detect_command_range(self) -> tuple[float, float]:
        assert self._model is not None
        if self._is_gripper:
            return 0.0, 100.0
        joint_range = self._model.jnt_range[self._joint_ids[0]]
        if len(joint_range) == 2:
            return float(joint_range[0]), float(joint_range[1])
        return -180.0, 180.0

    def _native_position_to_status(self, native_value: float) -> float:
        if self._is_gripper:
            assert self._model is not None
            joint_range = self._model.jnt_range[self._joint_ids[0]]
            minimum = float(joint_range[0])
            maximum = float(joint_range[1])
            if maximum <= minimum:
                return 0.0
            return max(0.0, min(100.0, (maximum - native_value) / (maximum - minimum) * 100.0))
        return math.degrees(native_value)

    def _native_velocity_to_status(self, native_value: float) -> float:
        if self._is_gripper:
            assert self._model is not None
            joint_range = self._model.jnt_range[self._joint_ids[0]]
            span = float(joint_range[1] - joint_range[0])
            if span <= 1e-9:
                return 0.0
            return -(native_value / span) * 100.0
        return math.degrees(native_value)

    def _command_position_to_native(self, position_command: float) -> float:
        if self._is_gripper:
            assert self._model is not None
            joint_range = self._model.jnt_range[self._joint_ids[0]]
            minimum = float(joint_range[0])
            maximum = float(joint_range[1])
            normalized = max(0.0, min(100.0, position_command)) / 100.0
            return maximum - normalized * (maximum - minimum)
        return math.radians(position_command)

    def _command_position_to_actuator_ctrl(self, position_command: float) -> float:
        normalized = max(0.0, min(100.0, position_command)) / 100.0
        if not self._uses_gripper_percent_ctrl:
            return self._command_position_to_native(position_command)
        assert self._model is not None
        ctrl_range = self._model.actuator_ctrlrange[self._actuator_ids[0]]
        minimum = float(ctrl_range[0])
        maximum = float(ctrl_range[1])
        return maximum - normalized * (maximum - minimum)

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

    def apply_command(self, command: JointCommand) -> None:
        self._command = command
        self._state.enabled = command.enable
        self._state.mode = command.mode

    def step(self, dt_s: float) -> None:
        if self._model is not None and self._data is not None and self._mujoco is not None:
            self._step_mujoco(dt_s)
            return
        self._step_fallback(dt_s)

    def _step_mujoco(self, dt_s: float) -> None:
        assert self._mujoco is not None
        assert self._model is not None
        assert self._data is not None

        if self._arm_actuator_ids:
            for actuator_id, ctrl_value in zip(self._arm_actuator_ids, self._arm_home_ctrl):
                self._data.ctrl[actuator_id] = ctrl_value

        current_native_position = (
            sum(float(self._data.qpos[qpos_adr]) for qpos_adr in self._joint_qpos_adrs) / max(len(self._joint_qpos_adrs), 1)
            if self._model.nq > 0
            else 0.0
        )
        current_native_velocity = (
            sum(float(self._data.qvel[dof_adr]) for dof_adr in self._joint_dof_adrs) / max(len(self._joint_dof_adrs), 1)
            if self._model.nv > 0
            else 0.0
        )
        current_position_status = self._native_position_to_status(current_native_position)
        current_velocity_status = self._native_velocity_to_status(current_native_velocity)

        if self._is_gripper:
            if not self._command.enable:
                self._gripper_target_native = self._command_position_to_actuator_ctrl(current_position_status)
            elif self._command.mode == 1:
                self._gripper_target_native = self._command_position_to_actuator_ctrl(self._command.target_position_deg)
            elif self._command.mode == 2:
                next_position = current_position_status + self._command.target_velocity_deg_s * dt_s
                self._gripper_target_native = self._command_position_to_actuator_ctrl(next_position)
            else:
                next_position = current_position_status + self._command.target_torque_nm * 0.5 * dt_s
                self._gripper_target_native = self._command_position_to_actuator_ctrl(next_position)
            self._data.ctrl[self._actuator_ids[0]] = self._gripper_target_native

            commanded_torque = 0.0
            if hasattr(self._data, "actuator_force") and len(self._data.actuator_force) > self._actuator_ids[0]:
                commanded_torque = float(self._data.actuator_force[self._actuator_ids[0]])
        else:
            actuator_id = self._actuator_ids[0]
            if not self._command.enable:
                commanded_torque = 0.0
            elif self._command.mode == 1:
                commanded_torque = (self._command.target_position_deg - current_position_status) * 0.8 - current_velocity_status * 0.03
            elif self._command.mode == 2:
                velocity_error = self._command.target_velocity_deg_s - current_velocity_status
                commanded_torque = velocity_error * 0.1
            else:
                commanded_torque = self._command.target_torque_nm
            commanded_torque = max(-50.0, min(50.0, commanded_torque))
            self._data.ctrl[actuator_id] = commanded_torque

        timestep = float(self._model.opt.timestep)
        steps = max(1, round(dt_s / max(timestep, 1e-6)))
        for _ in range(steps):
            self._mujoco.mj_step(self._model, self._data)

        if self._model.nq > 0:
            current_native_position = sum(float(self._data.qpos[qpos_adr]) for qpos_adr in self._joint_qpos_adrs) / max(len(self._joint_qpos_adrs), 1)
            self._state.position_deg = self._native_position_to_status(current_native_position)
        if self._model.nv > 0:
            current_native_velocity = sum(float(self._data.qvel[dof_adr]) for dof_adr in self._joint_dof_adrs) / max(len(self._joint_dof_adrs), 1)
            self._state.velocity_deg_s = self._native_velocity_to_status(current_native_velocity)
        self._state.torque_nm = commanded_torque
        self._state.temperature_c = min(120.0, self._state.temperature_c + abs(self._state.torque_nm) * 0.005)
        self._state.error_code = 0 if self._state.temperature_c < 100 else 2
        self._state.fault_severity = 0 if self._state.error_code == 0 else 2
        self._state.fault_source = 3 if self._state.error_code else 0
        self._state.fault_detail = int(self._state.temperature_c * 10) if self._state.error_code else 0
        self._sync_viewer()

    def _sync_viewer(self) -> None:
        if self._viewer is None:
            return
        sync = getattr(self._viewer, "sync", None)
        if callable(sync):
            sync()

    def _step_fallback(self, dt_s: float) -> None:
        if not self._command.enable:
            self._state.velocity_deg_s *= 0.8
            self._state.torque_nm = 0.0
            return

        if self._command.mode == 1:
            position_error = self._command.target_position_deg - self._state.position_deg
            self._state.velocity_deg_s = position_error * 5.0
            self._state.position_deg += self._state.velocity_deg_s * dt_s
            self._state.torque_nm = max(-50.0, min(50.0, position_error * 0.3))
        elif self._command.mode == 2:
            self._state.velocity_deg_s = self._command.target_velocity_deg_s
            self._state.position_deg += self._state.velocity_deg_s * dt_s
            self._state.torque_nm = self._state.velocity_deg_s * 0.02
        elif self._command.mode == 3:
            self._state.torque_nm = self._command.target_torque_nm
            self._state.velocity_deg_s += self._state.torque_nm * dt_s
            self._state.position_deg += self._state.velocity_deg_s * dt_s

        self._state.temperature_c = min(120.0, self._state.temperature_c + abs(self._state.torque_nm) * 0.005)
        self._state.error_code = 0 if self._state.temperature_c < 100 else 2
        self._state.fault_severity = 0 if self._state.error_code == 0 else 2
        self._state.fault_source = 3 if self._state.error_code else 0
        self._state.fault_detail = int(self._state.temperature_c * 10) if self._state.error_code else 0

    def read_state(self) -> JointState:
        return JointState(
            position_deg=self._state.position_deg,
            velocity_deg_s=self._state.velocity_deg_s,
            torque_nm=self._state.torque_nm,
            temperature_c=self._state.temperature_c,
            error_code=self._state.error_code,
            enabled=self._state.enabled,
            mode=self._state.mode,
            fault_severity=self._state.fault_severity,
            fault_source=self._state.fault_source,
            fault_detail=self._state.fault_detail,
        )
