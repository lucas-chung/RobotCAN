from __future__ import annotations

import numpy as np

from robotcan.core.types import DetectedObject, Observation
from robotcan.perception.rgbd import camera_to_world_xyz, pixel_to_camera_xyz


class RedObjectDetector:
    def __init__(self, model, data, camera_name: str, min_area_px: int = 40):
        self.model = model
        self.data = data
        self.camera_name = camera_name
        self.min_area_px = min_area_px

    def detect(self, observation: Observation) -> list[DetectedObject]:
        if observation.rgb is None or observation.depth is None:
            return []

        rgb = observation.rgb
        depth = observation.depth
        mask = self._red_mask(rgb)
        ys, xs = np.nonzero(mask)
        if len(xs) < self.min_area_px:
            return []

        x1 = int(xs.min())
        y1 = int(ys.min())
        x2 = int(xs.max())
        y2 = int(ys.max())
        pixel_xy = (float(xs.mean()), float(ys.mean()))

        valid_depth = depth[ys, xs]
        valid_depth = valid_depth[np.isfinite(valid_depth)]
        valid_depth = valid_depth[valid_depth > 0.0]
        if valid_depth.size == 0:
            return []

        depth_m = float(np.median(valid_depth))
        camera_xyz = pixel_to_camera_xyz(
            pixel_xy=pixel_xy,
            depth_m=depth_m,
            image_size=(rgb.shape[1], rgb.shape[0]),
            model=self.model,
            camera_name=self.camera_name,
        )
        world_xyz = camera_to_world_xyz(camera_xyz, self.model, self.data, self.camera_name)

        return [
            DetectedObject(
                label="red_block",
                confidence=1.0,
                pixel_xy=pixel_xy,
                bbox_xyxy=(x1, y1, x2, y2),
                area_px=int(len(xs)),
                depth_m=depth_m,
                camera_xyz_m=camera_xyz,
                world_xyz_m=world_xyz,
            )
        ]

    @staticmethod
    def _red_mask(rgb: np.ndarray) -> np.ndarray:
        image = rgb.astype(np.int16)
        r = image[:, :, 0]
        g = image[:, :, 1]
        b = image[:, :, 2]
        return (r > 120) & (r > g * 2) & (r > b * 2)


def draw_detection_overlay(rgb: np.ndarray, detection: DetectedObject | None) -> np.ndarray:
    output = np.array(rgb, copy=True)
    if detection is None:
        return output

    x1, y1, x2, y2 = detection.bbox_xyxy
    x1 = max(0, min(output.shape[1] - 1, x1))
    x2 = max(0, min(output.shape[1] - 1, x2))
    y1 = max(0, min(output.shape[0] - 1, y1))
    y2 = max(0, min(output.shape[0] - 1, y2))
    output[y1 : y1 + 2, x1 : x2 + 1] = (0, 255, 0)
    output[y2 - 1 : y2 + 1, x1 : x2 + 1] = (0, 255, 0)
    output[y1 : y2 + 1, x1 : x1 + 2] = (0, 255, 0)
    output[y1 : y2 + 1, x2 - 1 : x2 + 1] = (0, 255, 0)
    cx = int(round(detection.pixel_xy[0]))
    cy = int(round(detection.pixel_xy[1]))
    output[max(0, cy - 6) : min(output.shape[0], cy + 7), max(0, cx - 1) : min(output.shape[1], cx + 2)] = (0, 255, 0)
    output[max(0, cy - 1) : min(output.shape[0], cy + 2), max(0, cx - 6) : min(output.shape[1], cx + 7)] = (0, 255, 0)
    return output

