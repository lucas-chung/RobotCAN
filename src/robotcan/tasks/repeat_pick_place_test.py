from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np

from robotcan.core.types import Observation
from robotcan.perception.color import RedObjectDetector
from robotcan.policies.repeat_pick_place import PLACE_OBJECT_XYZ, RepeatPickPlacePolicy
from robotcan.simulation.mujoco.backend import ArmCommand, HandCommand, MujocoJointBackend


@dataclass(slots=True)
class PickPlaceCycleResult:
    cycle: int
    source: str
    target: str
    detected_xyz: tuple[float, float, float] | None
    final_xyz: tuple[float, float, float]
    target_xyz: tuple[float, float, float]
    error_m: float
    success: bool


@dataclass(slots=True)
class RepeatPickPlaceConfig:
    model_path: str
    cycles: int = 10
    dt_s: float = 0.01
    settle_s: float = 0.8
    move_s: float = 2.0
    hold_s: float = 0.4
    success_tolerance_m: float = 0.04
    enable_viewer: bool = True
    camera_name: str = "top_camera"


class RepeatPickPlaceTestRunner:
    def __init__(self, config: RepeatPickPlaceConfig):
        self.config = config
        self.policy = RepeatPickPlacePolicy()
        self.backend = MujocoJointBackend(model_path=config.model_path, enable_viewer=config.enable_viewer)
        if not self.backend.has_real_mujoco:
            raise RuntimeError("repeat-pick-place-test requires a loaded MuJoCo model.")
        assert self.backend.model is not None
        assert self.backend.data is not None
        assert self.backend.mujoco_module is not None
        self.model = self.backend.model
        self.data = self.backend.data
        self.mujoco = self.backend.mujoco_module
        self.object_qpos_adr = self._freejoint_qpos_adr("demo_object")
        self.pinch_site_id = int(self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_SITE, "pinch"))
        self.renderer = self.mujoco.Renderer(self.model, height=480, width=640)
        self.detector = RedObjectDetector(self.model, self.data, self.config.camera_name)

    def run(self) -> list[PickPlaceCycleResult]:
        results: list[PickPlaceCycleResult] = []
        self._set_object_xyz(self.policy.home.object_xyz)
        self._move_arm(self.policy.home.arm_joint_deg, hand_position=80.0, duration_s=self.config.settle_s)

        try:
            for cycle in range(self.config.cycles):
                source, target = self.policy.targets_for_cycle(cycle)
                detected = self._detect_object_xyz()
                print(f"cycle {cycle + 1}/{self.config.cycles}: {source.name} -> {target.name}")
                if detected is not None:
                    print(f"  detected source xyz={self._round_xyz(detected)}")

                self._set_object_xyz(source.object_xyz)
                self._move_arm(source.arm_joint_deg, hand_position=80.0, duration_s=self.config.move_s)
                self._move_arm(source.arm_joint_deg, hand_position=0.0, duration_s=self.config.hold_s)
                self._move_arm(target.arm_joint_deg, hand_position=0.0, duration_s=self.config.move_s, carry_object=True)
                self._move_arm(target.arm_joint_deg, hand_position=80.0, duration_s=self.config.hold_s)
                self._set_object_xyz(target.object_xyz)
                self._step_for(self.config.settle_s)

                final_xyz = self._object_xyz()
                error_m = self._distance(final_xyz, target.object_xyz)
                success = error_m <= self.config.success_tolerance_m
                print(f"  final xyz={self._round_xyz(final_xyz)} error={error_m:.4f}m success={success}")
                results.append(
                    PickPlaceCycleResult(
                        cycle=cycle + 1,
                        source=source.name,
                        target=target.name,
                        detected_xyz=detected,
                        final_xyz=final_xyz,
                        target_xyz=target.object_xyz,
                        error_m=error_m,
                        success=success,
                    )
                )

        finally:
            close = getattr(self.renderer, "close", None)
            if callable(close):
                close()
            self.backend.close()

        successes = sum(1 for result in results if result.success)
        print(f"repeat pick-place finished: cycles={len(results)} success={successes} failure={len(results) - successes}")
        return results

    def _move_arm(
        self,
        joint_deg: tuple[float, float, float, float, float, float],
        hand_position: float,
        duration_s: float,
        carry_object: bool = False,
    ) -> None:
        steps = max(1, int(duration_s / self.config.dt_s))
        command = ArmCommand(enable=True, mode=1, joint_targets_deg=list(joint_deg))
        hand = HandCommand(enable=True, mode=1, target_position_deg=hand_position)
        for _ in range(steps):
            self.backend.apply_arm_command(command)
            self.backend.apply_hand_command(hand)
            self.backend.step(self.config.dt_s)
            if carry_object:
                self._carry_object_under_gripper()
            time.sleep(self.config.dt_s)

    def _step_for(self, duration_s: float) -> None:
        steps = max(1, int(duration_s / self.config.dt_s))
        for _ in range(steps):
            self.backend.step(self.config.dt_s)
            time.sleep(self.config.dt_s)

    def _detect_object_xyz(self) -> tuple[float, float, float] | None:
        self.renderer.update_scene(self.data, camera=self.config.camera_name)
        rgb = np.asarray(self.renderer.render(), dtype=np.uint8)
        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.config.camera_name)
        depth = np.asarray(self.renderer.render(), dtype=np.float32)
        self.renderer.disable_depth_rendering()
        detections = self.detector.detect(Observation(rgb=rgb, depth=depth, camera_name=self.config.camera_name))
        if not detections:
            return None
        return detections[0].world_xyz_m

    def _carry_object_under_gripper(self) -> None:
        pinch = np.asarray(self.data.site_xpos[self.pinch_site_id], dtype=np.float64)
        carried_xyz = (float(pinch[0]), float(pinch[1]), max(float(pinch[2] - 0.12), PLACE_OBJECT_XYZ[2]))
        self._set_object_xyz(carried_xyz)

    def _freejoint_qpos_adr(self, body_name: str) -> int:
        body_id = int(self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_BODY, body_name))
        joint_id = int(self.model.body_jntadr[body_id])
        if joint_id < 0:
            raise RuntimeError(f"Body {body_name!r} does not have a freejoint.")
        return int(self.model.jnt_qposadr[joint_id])

    def _set_object_xyz(self, xyz: tuple[float, float, float]) -> None:
        adr = self.object_qpos_adr
        self.data.qpos[adr : adr + 3] = xyz
        self.data.qpos[adr + 3 : adr + 7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qvel[adr : adr + 6] = 0.0
        self.mujoco.mj_forward(self.model, self.data)

    def _object_xyz(self) -> tuple[float, float, float]:
        adr = self.object_qpos_adr
        return tuple(float(value) for value in self.data.qpos[adr : adr + 3])

    @staticmethod
    def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return float(np.linalg.norm(np.asarray(a) - np.asarray(b)))

    @staticmethod
    def _round_xyz(xyz: tuple[float, float, float]) -> tuple[float, float, float]:
        return tuple(round(value, 3) for value in xyz)
