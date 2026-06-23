from __future__ import annotations

from dataclasses import dataclass
import time
import tkinter as tk

import numpy as np


@dataclass(slots=True)
class CameraPreviewConfig:
    camera_name: str = "gripper_depth_camera"
    width: int = 640
    height: int = 480
    depth: bool = False
    rgbd: bool = False
    title: str = "MuJoCo camera"


class MujocoCameraPreview:
    def __init__(self, model, data, mujoco_module, config: CameraPreviewConfig | None = None):
        self.model = model
        self.data = data
        self.mujoco = mujoco_module
        self.config = config or CameraPreviewConfig()
        self._renderer = self.mujoco.Renderer(self.model, height=self.config.height, width=self.config.width)
        self._root: tk.Tk | None = None
        self._label: tk.Label | None = None
        self._photo: tk.PhotoImage | None = None
        self._last_update_s = 0.0
        self._closed = False

    def open(self) -> None:
        if self._root is not None:
            return
        self._root = tk.Tk()
        self._root.title(f"{self.config.title}: {self.config.camera_name}")
        self._label = tk.Label(self._root)
        self._label.pack()
        self._root.protocol("WM_DELETE_WINDOW", self.close)

    def update(self, min_period_s: float = 0.03) -> None:
        if self._closed:
            return
        self.open()
        now = time.perf_counter()
        if now - self._last_update_s < min_period_s:
            self._pump_events()
            return
        self._last_update_s = now

        image = self.render()
        self.show_image(image)

    def render(self) -> np.ndarray:
        if self.config.rgbd:
            rgb = self._render_rgb()
            depth_rgb = self._render_depth_rgb()
            return np.concatenate((rgb, depth_rgb), axis=1)

        if self.config.depth:
            return self._render_depth_rgb()

        return self._render_rgb()

    def _render_rgb(self) -> np.ndarray:
        self._renderer.update_scene(self.data, camera=self.config.camera_name)
        return np.asarray(self._renderer.render(), dtype=np.uint8)

    def _render_depth_rgb(self) -> np.ndarray:
        self._renderer.enable_depth_rendering()
        self._renderer.update_scene(self.data, camera=self.config.camera_name)
        depth = self._renderer.render()
        self._renderer.disable_depth_rendering()
        return self._depth_to_rgb(depth)

    def close(self) -> None:
        self._closed = True
        if self._root is not None:
            self._root.destroy()
            self._root = None
        close = getattr(self._renderer, "close", None)
        if callable(close):
            close()

    @property
    def closed(self) -> bool:
        return self._closed

    def show_image(self, image: np.ndarray) -> None:
        if self._closed:
            return
        self.open()
        ppm = self._rgb_to_ppm(image)
        self._photo = tk.PhotoImage(data=ppm, format="PPM")
        assert self._label is not None
        self._label.configure(image=self._photo)
        self._pump_events()

    def _pump_events(self) -> None:
        if self._root is None:
            return
        try:
            self._root.update_idletasks()
            self._root.update()
        except tk.TclError:
            self._closed = True

    @staticmethod
    def _rgb_to_ppm(image: np.ndarray) -> bytes:
        image = np.ascontiguousarray(image[:, :, :3], dtype=np.uint8)
        header = f"P6 {image.shape[1]} {image.shape[0]} 255\n".encode("ascii")
        return header + image.tobytes()

    @staticmethod
    def _depth_to_rgb(depth: np.ndarray) -> np.ndarray:
        depth = np.asarray(depth, dtype=np.float32)
        finite = np.isfinite(depth)
        if not finite.any():
            return np.zeros((*depth.shape, 3), dtype=np.uint8)
        near = float(np.percentile(depth[finite], 2))
        far = float(np.percentile(depth[finite], 98))
        if far <= near:
            far = near + 1e-6
        normalized = np.clip((depth - near) / (far - near), 0.0, 1.0)
        gray = ((1.0 - normalized) * 255.0).astype(np.uint8)
        return np.stack((gray, gray, gray), axis=-1)
