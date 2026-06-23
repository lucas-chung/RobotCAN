from __future__ import annotations

import argparse
from pathlib import Path
import time

from robotcan.simulation.mujoco.camera_preview import CameraPreviewConfig, MujocoCameraPreview


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Preview a MuJoCo camera without connecting to TSMaster.")
    parser.add_argument("--model", required=True, help="MuJoCo XML model path.")
    parser.add_argument("--camera-name", default="gripper_depth_camera")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--depth", action="store_true", help="Preview depth instead of RGB.")
    parser.add_argument("--rgbd", action="store_true", help="Preview RGB and depth side by side.")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--duration", type=float, help="Optional runtime limit in seconds.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    import mujoco  # type: ignore

    model = mujoco.MjModel.from_xml_path(str(Path(args.model).resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    preview = MujocoCameraPreview(
        model=model,
        data=data,
        mujoco_module=mujoco,
        config=CameraPreviewConfig(
            camera_name=args.camera_name,
            width=args.width,
            height=args.height,
            depth=args.depth,
            rgbd=args.rgbd,
            title="RobotCAN standalone camera",
        ),
    )
    start = time.perf_counter()
    try:
        while not preview.closed:
            if args.duration is not None and time.perf_counter() - start >= args.duration:
                break
            mujoco.mj_step(model, data)
            preview.update()
            time.sleep(args.dt)
    finally:
        preview.close()
    return 0
