from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from robotcan.core.types import Observation
from robotcan.perception.color import RedObjectDetector
from robotcan.policies.repeat_pick_place import RepeatPickPlacePolicy
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
    move_s: float = 1.0
    hold_s: float = 1.0
    approach_z_m: float = 0.28
    grasp_z_m: float = 0.03
    grip_position: float = 60.0
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
        self.object_geom_id = self._geom_id("demo_object_geom")
        self.left_pad_geom_ids = tuple(self._geom_id(name) for name in ("left_pad1", "left_pad2"))
        self.right_pad_geom_ids = tuple(self._geom_id(name) for name in ("right_pad1", "right_pad2"))
        self.pinch_site_id = int(self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_SITE, "pinch"))
        self.renderer = self.mujoco.Renderer(self.model, height=480, width=640)
        self.detector = RedObjectDetector(self.model, self.data, self.config.camera_name)
        self.arm_joint_qpos_adrs = self.backend.arm_joint_qpos_adrs

    def run(self) -> list[PickPlaceCycleResult]:
        results: list[PickPlaceCycleResult] = []
        self._move_arm(self.policy.home.arm_joint_deg, hand_position=80.0, duration_s=self.config.settle_s)
        self._set_object_xyz(self.policy.home.object_xyz)
        self._step_for(self.config.settle_s)

        try:
            for cycle in range(self.config.cycles):
                source, target = self.policy.targets_for_cycle(cycle)
                detected = self._detect_object_xyz()
                print(f"cycle {cycle + 1}/{self.config.cycles}: {source.name} -> {target.name}")
                if detected is not None:
                    print(f"  detected source xyz={self._round_xyz(detected)}")

                source_approach = self._solve_site_ik(
                    (source.object_xyz[0], source.object_xyz[1], self.config.approach_z_m),
                    source.arm_joint_deg,
                )
                source_grasp = self._solve_site_ik(
                    (source.object_xyz[0], source.object_xyz[1], self.config.grasp_z_m),
                    source_approach,
                )
                target_approach = self._solve_site_ik(
                    (target.object_xyz[0], target.object_xyz[1], self.config.approach_z_m),
                    target.arm_joint_deg,
                )
                target_place = self._solve_site_ik(
                    (target.object_xyz[0], target.object_xyz[1], self.config.grasp_z_m),
                    target_approach,
                )

                self._move_arm(source_approach, hand_position=80.0, duration_s=self.config.move_s)
                self._move_arm(source_grasp, hand_position=80.0, duration_s=self.config.move_s)
                self._move_arm(source_grasp, hand_position=self.config.grip_position, duration_s=self.config.hold_s)
                left_contact, right_contact = self._object_pad_contacts()
                print(f"  grasp contacts: left={left_contact} right={right_contact}")
                self._move_arm(source_approach, hand_position=self.config.grip_position, duration_s=self.config.move_s)
                lifted_xyz = self._object_xyz()
                lifted = lifted_xyz[2] > source.object_xyz[2] + 0.04
                print(f"  lifted xyz={self._round_xyz(lifted_xyz)} lifted={lifted}")
                self._move_arm(target_approach, hand_position=self.config.grip_position, duration_s=self.config.move_s)
                self._move_arm(target_place, hand_position=self.config.grip_position, duration_s=self.config.move_s)
                self._move_arm(target_place, hand_position=80.0, duration_s=self.config.hold_s)
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
    ) -> None:
        steps = max(1, int(duration_s / self.config.dt_s))
        command = ArmCommand(enable=True, mode=1, joint_targets_deg=list(joint_deg))
        hand = HandCommand(enable=True, mode=1, target_position_deg=hand_position)
        for _ in range(steps):
            self.backend.apply_arm_command(command)
            self.backend.apply_hand_command(hand)
            self.backend.step(self.config.dt_s)
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

    def _object_pad_contacts(self) -> tuple[bool, bool]:
        left_contact = False
        right_contact = False
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom_pair = {int(contact.geom1), int(contact.geom2)}
            if self.object_geom_id not in geom_pair:
                continue
            left_contact = left_contact or any(geom_id in geom_pair for geom_id in self.left_pad_geom_ids)
            right_contact = right_contact or any(geom_id in geom_pair for geom_id in self.right_pad_geom_ids)
        return left_contact, right_contact

    def _solve_site_ik(
        self,
        target_xyz: tuple[float, float, float],
        seed_joint_deg: tuple[float, float, float, float, float, float],
    ) -> tuple[float, float, float, float, float, float]:
        scratch = self.mujoco.MjData(self.model)
        scratch.qpos[:] = self.data.qpos[:]
        scratch.qvel[:] = 0.0
        for qpos_adr, joint_deg in zip(self.arm_joint_qpos_adrs, seed_joint_deg):
            scratch.qpos[qpos_adr] = np.radians(joint_deg)
        self.mujoco.mj_forward(self.model, scratch)

        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        jacr = np.zeros((3, self.model.nv), dtype=np.float64)
        target = np.asarray(target_xyz, dtype=np.float64)
        for _ in range(300):
            error = target - np.asarray(scratch.site_xpos[self.pinch_site_id], dtype=np.float64)
            if float(np.linalg.norm(error)) < 1e-3:
                break
            self.mujoco.mj_jacSite(self.model, scratch, jacp, jacr, self.pinch_site_id)
            arm_jac = jacp[:, :6]
            delta = arm_jac.T @ np.linalg.solve(arm_jac @ arm_jac.T + 1e-4 * np.eye(3), error)
            delta = np.clip(delta, -0.08, 0.08)
            for index, qpos_adr in enumerate(self.arm_joint_qpos_adrs):
                scratch.qpos[qpos_adr] += delta[index]
            self.mujoco.mj_forward(self.model, scratch)

        final_error = float(np.linalg.norm(target - np.asarray(scratch.site_xpos[self.pinch_site_id], dtype=np.float64)))
        if final_error > 0.02:
            raise RuntimeError(f"IK failed for target {target_xyz}: error={final_error:.4f}m")
        return tuple(float(np.degrees(scratch.qpos[qpos_adr])) for qpos_adr in self.arm_joint_qpos_adrs)

    def _freejoint_qpos_adr(self, body_name: str) -> int:
        body_id = int(self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_BODY, body_name))
        joint_id = int(self.model.body_jntadr[body_id])
        if joint_id < 0:
            raise RuntimeError(f"Body {body_name!r} does not have a freejoint.")
        return int(self.model.jnt_qposadr[joint_id])

    def _geom_id(self, geom_name: str) -> int:
        geom_id = int(self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_GEOM, geom_name))
        if geom_id < 0:
            raise RuntimeError(f"Unable to resolve MuJoCo geom {geom_name!r}.")
        return geom_id

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
