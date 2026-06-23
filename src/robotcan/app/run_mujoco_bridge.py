from __future__ import annotations

import argparse

from robotcan.controller.loop import BridgeLoopConfig, TSMasterMujocoBridgeLoop
from robotcan.simulation.mujoco.backend import MujocoJointBackend
from robotcan.simulation.mujoco.camera_preview import CameraPreviewConfig, MujocoCameraPreview
from robotcan.transport.tsmaster_attached import AttachedTSMaster


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the RobotCAN TSMaster <-> MuJoCo bridge loop.")
    parser.add_argument("--app-name", default="TSMaster", help="Opened TSMaster project title, for example robot.")
    parser.add_argument("--server-name", default="TSMaster", help="RPC server name used by rpc_tsmaster_create_client.")
    parser.add_argument("--api-context-name", default="TSMaster", help="Local tsapp API context name used by initialize_lib_tsmaster and send/receive calls.")
    parser.add_argument("--model", help="Optional MuJoCo XML model path.")
    parser.add_argument("--no-viewer", action="store_true", help="Disable the MuJoCo viewer window and run headless.")
    parser.add_argument("--duration", type=float, help="Optional runtime limit in seconds.")
    parser.add_argument("--dt", type=float, default=0.01, help="Controller step in seconds.")
    parser.add_argument("--status-period", type=float, default=0.01, help="Status frame period in seconds.")
    parser.add_argument("--fault-period", type=float, default=0.1, help="Fault frame period in seconds.")
    parser.add_argument("--diag-period", type=float, default=0.1, help="Diagnostic frame period in seconds.")
    parser.add_argument("--camera-preview", action="store_true", help="Open a realtime MuJoCo camera preview window.")
    parser.add_argument("--camera-name", default="gripper_depth_camera", help="MuJoCo camera name used by --camera-preview.")
    parser.add_argument("--camera-width", type=int, default=640, help="Camera preview width in pixels.")
    parser.add_argument("--camera-height", type=int, default=480, help="Camera preview height in pixels.")
    parser.add_argument("--camera-depth", action="store_true", help="Preview depth instead of RGB.")
    parser.add_argument("--camera-rgbd", action="store_true", help="Preview RGB and depth side by side.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = BridgeLoopConfig(
        dt_s=args.dt,
        status_period_s=args.status_period,
        fault_period_s=args.fault_period,
        diag_period_s=args.diag_period,
    )
    backend = MujocoJointBackend(model_path=args.model, enable_viewer=not args.no_viewer)
    camera_preview = None
    try:
        if args.camera_preview:
            if not backend.has_real_mujoco or backend.model is None or backend.data is None or backend.mujoco_module is None:
                raise RuntimeError("Camera preview requires a loaded MuJoCo model.")
            camera_preview = MujocoCameraPreview(
                model=backend.model,
                data=backend.data,
                mujoco_module=backend.mujoco_module,
                config=CameraPreviewConfig(
                    camera_name=args.camera_name,
                    width=args.camera_width,
                    height=args.camera_height,
                    depth=args.camera_depth,
                    rgbd=args.camera_rgbd,
                    title="RobotCAN gripper camera",
                ),
            )
        with AttachedTSMaster(
            app_name=args.app_name,
            server_name=args.server_name,
            api_context_name=args.api_context_name,
        ) as app:
            loop = TSMasterMujocoBridgeLoop(app=app, backend=backend, config=config, camera_preview=camera_preview)
            loop.run(duration_s=args.duration)
    finally:
        if camera_preview is not None:
            camera_preview.close()
        backend.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
