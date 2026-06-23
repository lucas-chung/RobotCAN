from __future__ import annotations

import math

import numpy as np


def pixel_to_camera_xyz(
    pixel_xy: tuple[float, float],
    depth_m: float,
    image_size: tuple[int, int],
    model,
    camera_name: str,
) -> tuple[float, float, float]:
    width, height = image_size
    camera_id = int(model.camera(camera_name).id)
    fovy_rad = math.radians(float(model.cam_fovy[camera_id]))
    fy = 0.5 * height / math.tan(0.5 * fovy_rad)
    fx = fy
    cx = (width - 1) * 0.5
    cy = (height - 1) * 0.5
    x, y = pixel_xy
    camera_x = (x - cx) * depth_m / fx
    camera_y = -(y - cy) * depth_m / fy
    camera_z = -depth_m
    return (float(camera_x), float(camera_y), float(camera_z))


def camera_to_world_xyz(camera_xyz: tuple[float, float, float], model, data, camera_name: str) -> tuple[float, float, float]:
    camera_id = int(model.camera(camera_name).id)
    camera_pos = np.asarray(data.cam_xpos[camera_id], dtype=np.float64)
    camera_mat = np.asarray(data.cam_xmat[camera_id], dtype=np.float64).reshape(3, 3)
    world_xyz = camera_pos + camera_mat @ np.asarray(camera_xyz, dtype=np.float64)
    return tuple(float(value) for value in world_xyz)

