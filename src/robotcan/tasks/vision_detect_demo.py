from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np

from robotcan.core.types import Observation
from robotcan.perception.color import RedObjectDetector, draw_detection_overlay
from robotcan.simulation.mujoco.camera_preview import CameraPreviewConfig, MujocoCameraPreview


@dataclass(slots=True)
class VisionDetectConfig:
    model_path: str
    camera_name: str = "top_camera"
    width: int = 640
    height: int = 480
    duration_s: float | None = None
    print_period_s: float = 0.5


class VisionDetectDemoRunner:
    def __init__(self, config: VisionDetectConfig):
        self.config = config

    def run(self) -> int:
        import mujoco  # type: ignore

        model = mujoco.MjModel.from_xml_path(str(Path(self.config.model_path).resolve()))
        data = mujoco.MjData(model)
        renderer = mujoco.Renderer(model, height=self.config.height, width=self.config.width)
        detector = RedObjectDetector(model=model, data=data, camera_name=self.config.camera_name)
        preview = MujocoCameraPreview(
            model=model,
            data=data,
            mujoco_module=mujoco,
            config=CameraPreviewConfig(
                camera_name=self.config.camera_name,
                width=self.config.width,
                height=self.config.height,
                title="RobotCAN vision detect",
            ),
        )

        start = time.perf_counter()
        last_print = 0.0
        try:
            while not preview.closed:
                now = time.perf_counter()
                if self.config.duration_s is not None and now - start >= self.config.duration_s:
                    break

                mujoco.mj_step(model, data)
                rgb = self._render_rgb(renderer, data)
                depth = self._render_depth(renderer, data)
                observation = Observation(rgb=rgb, depth=depth, camera_name=self.config.camera_name)
                detections = detector.detect(observation)
                detection = detections[0] if detections else None
                overlay = draw_detection_overlay(rgb, detection)
                preview.show_image(overlay)

                if now - last_print >= self.config.print_period_s:
                    self._print_detection(detection)
                    last_print = now
                time.sleep(0.01)
        finally:
            preview.close()
            close = getattr(renderer, "close", None)
            if callable(close):
                close()
        return 0

    def _render_rgb(self, renderer, data) -> np.ndarray:
        renderer.update_scene(data, camera=self.config.camera_name)
        return np.asarray(renderer.render(), dtype=np.uint8)

    def _render_depth(self, renderer, data) -> np.ndarray:
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=self.config.camera_name)
        depth = np.asarray(renderer.render(), dtype=np.float32)
        renderer.disable_depth_rendering()
        return depth

    @staticmethod
    def _print_detection(detection) -> None:
        if detection is None:
            print("vision: target not found")
            return
        wx, wy, wz = detection.world_xyz_m
        print(
            "vision:"
            f" pixel=({detection.pixel_xy[0]:.1f}, {detection.pixel_xy[1]:.1f})"
            f" depth={detection.depth_m:.3f}m"
            f" world=({wx:.3f}, {wy:.3f}, {wz:.3f})"
            f" area={detection.area_px}"
        )
