from __future__ import annotations

import argparse

from robotcan.controller.loop import BridgeLoopConfig, TSMasterMujocoBridgeLoop
from robotcan.simulation.mujoco.backend import MujocoJointBackend
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
    try:
        with AttachedTSMaster(
            app_name=args.app_name,
            server_name=args.server_name,
            api_context_name=args.api_context_name,
        ) as app:
            loop = TSMasterMujocoBridgeLoop(app=app, backend=backend, config=config)
            loop.run(duration_s=args.duration)
    finally:
        backend.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
