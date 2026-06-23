from __future__ import annotations

import argparse
from pathlib import Path
import time

from robotcan.protocol import (
    ARM_STATUS_FEEDBACK_ID,
    ARM_FAULT_STATUS_ID,
    ArmControlCommand,
    ArmFaultStatus,
    ArmStatusFeedback,
    HandControlCommand,
    HandFaultStatus,
    HandStatusFeedback,
)
from robotcan.app.camera_preview import main as camera_preview_main
from robotcan.app.run_mujoco_bridge import main as bridge_main
from robotcan.tasks import (
    AlgorithmDemoConfig,
    AlgorithmDemoRunner,
    RepeatPickPlaceConfig,
    RepeatPickPlaceTestRunner,
    VisionDetectConfig,
    VisionDetectDemoRunner,
)
from robotcan.transport.tsmaster_attached import AttachedTSMaster, TSMasterError


def _add_connection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--app-name", default="TSMaster", help="Opened TSMaster project title, for example robot. This is informational for the bridge.")
    parser.add_argument("--server-name", default="TSMaster", help="RPC server name. Default follows the official RPC demo.")
    parser.add_argument("--api-context-name", default="TSMaster", help="Local tsapp API context name. Keep this as TSMaster for send/receive control.")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Attach to an existing TSMaster instance and drive RobotCAN demos.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    status_parser = subparsers.add_parser("status", help="Check whether the currently opened TSMaster simulation is running.")
    _add_connection_args(status_parser)

    start_parser = subparsers.add_parser("start", help="Start simulation through RPC on the opened TSMaster window.")
    _add_connection_args(start_parser)
    start_parser.add_argument("--log", default="RobotCAN attached client started simulation.", help="Log line written into TSMaster.")

    stop_parser = subparsers.add_parser("stop", help="Stop simulation through RPC on the opened TSMaster window.")
    _add_connection_args(stop_parser)
    stop_parser.add_argument("--log", default="RobotCAN attached client stopped simulation.", help="Log line written into TSMaster.")

    dbc_parser = subparsers.add_parser("load-dbc", help="Load a DBC into the current TSMaster application.")
    _add_connection_args(dbc_parser)
    dbc_parser.add_argument("path", help="Absolute or relative path to the DBC file.")
    dbc_parser.add_argument("--channel-mask", default="0", help='TSMaster channel mask, for example "0" or "0,1".')

    send_parser = subparsers.add_parser("send-control", help="Create a cyclic hand control command frame 0x101.")
    _add_connection_args(send_parser)
    send_parser.add_argument("--position", type=float, default=0.0, help="Target position in degrees.")
    send_parser.add_argument("--velocity", type=float, default=0.0, help="Target velocity in deg/s.")
    send_parser.add_argument("--torque", type=float, default=0.0, help="Target torque in Nm.")
    send_parser.add_argument("--mode", type=int, default=1, help="1=position, 2=velocity, 3=torque.")
    send_parser.add_argument("--disable", action="store_true", help="Clear the enable bit.")
    send_parser.add_argument("--period-ms", type=int, default=10, help="Cyclic transmission period in milliseconds.")

    arm_send_parser = subparsers.add_parser("send-arm-control", help="Create a cyclic arm control command frame 0x110.")
    _add_connection_args(arm_send_parser)
    arm_send_parser.add_argument("--j1", type=float, default=0.0)
    arm_send_parser.add_argument("--j2", type=float, default=0.0)
    arm_send_parser.add_argument("--j3", type=float, default=0.0)
    arm_send_parser.add_argument("--j4", type=float, default=0.0)
    arm_send_parser.add_argument("--j5", type=float, default=0.0)
    arm_send_parser.add_argument("--j6", type=float, default=0.0)
    arm_send_parser.add_argument("--mode", type=int, default=1, help="1=position, 2=velocity-like, 3=increment-like.")
    arm_send_parser.add_argument("--disable", action="store_true", help="Clear the enable bit.")
    arm_send_parser.add_argument("--period-ms", type=int, default=10, help="Cyclic transmission period in milliseconds.")

    clear_parser = subparsers.add_parser("clear-cyclic", help="Delete all cyclic messages in the current TSMaster application.")
    _add_connection_args(clear_parser)

    recv_parser = subparsers.add_parser("read-feedback", help="Read back CAN FD frames from TSMaster FIFO.")
    _add_connection_args(recv_parser)
    recv_parser.add_argument("--wait-ms", type=int, default=500, help="Wait time before reading the FIFO.")
    recv_parser.add_argument("--max-count", type=int, default=200, help="Maximum number of FIFO frames to read.")
    recv_parser.add_argument("--only-rx", action="store_true", help="Only fetch RX frames.")

    diag_parser = subparsers.add_parser("diag-demo", help="Create and delete a diagnostic module against the current application.")
    _add_connection_args(diag_parser)
    diag_parser.add_argument("--request-id", type=lambda value: int(value, 0), default=0x700)
    diag_parser.add_argument("--response-id", type=lambda value: int(value, 0), default=0x708)
    diag_parser.add_argument("--functional-id", type=lambda value: int(value, 0), default=0x7DF)

    bridge_parser = subparsers.add_parser("bridge", help="Run the TSMaster to MuJoCo bridge loop.")
    _add_connection_args(bridge_parser)
    bridge_parser.add_argument("--model", help="Optional MuJoCo XML model path.")
    bridge_parser.add_argument("--duration", type=float, help="Optional runtime limit in seconds.")
    bridge_parser.add_argument("--dt", type=float, default=0.01)
    bridge_parser.add_argument("--status-period", type=float, default=0.01)
    bridge_parser.add_argument("--fault-period", type=float, default=0.1)
    bridge_parser.add_argument("--diag-period", type=float, default=0.1)
    bridge_parser.add_argument("--camera-preview", action="store_true")
    bridge_parser.add_argument("--camera-name", default="gripper_depth_camera")
    bridge_parser.add_argument("--camera-width", type=int, default=640)
    bridge_parser.add_argument("--camera-height", type=int, default=480)
    bridge_parser.add_argument("--camera-depth", action="store_true")
    bridge_parser.add_argument("--camera-rgbd", action="store_true")

    demo_parser = subparsers.add_parser("algorithm-demo", help="Run a simple Python-side arm+hand demo while TSMaster stays open for observation.")
    _add_connection_args(demo_parser)
    demo_parser.add_argument("--command-period", type=float, default=0.05, help="Seconds between control commands sent from Python.")
    demo_parser.add_argument("--feedback-period", type=float, default=0.05, help="Seconds between feedback FIFO polls.")

    camera_parser = subparsers.add_parser("camera-preview", help="Preview a MuJoCo camera without connecting to TSMaster.")
    camera_parser.add_argument("--model", required=True)
    camera_parser.add_argument("--camera-name", default="gripper_depth_camera")
    camera_parser.add_argument("--width", type=int, default=640)
    camera_parser.add_argument("--height", type=int, default=480)
    camera_parser.add_argument("--depth", action="store_true")
    camera_parser.add_argument("--rgbd", action="store_true")
    camera_parser.add_argument("--dt", type=float, default=0.01)
    camera_parser.add_argument("--duration", type=float)

    vision_parser = subparsers.add_parser("vision-detect-demo", help="Detect the red MuJoCo target block from an RGB-D camera.")
    vision_parser.add_argument("--model", required=True)
    vision_parser.add_argument("--camera-name", default="top_camera")
    vision_parser.add_argument("--width", type=int, default=640)
    vision_parser.add_argument("--height", type=int, default=480)
    vision_parser.add_argument("--duration", type=float)
    vision_parser.add_argument("--print-period", type=float, default=0.5)

    repeat_parser = subparsers.add_parser("repeat-pick-place-test", help="Run repeated MuJoCo pick/place cycles between home and a 90-degree rotated target.")
    repeat_parser.add_argument("--model", required=True)
    repeat_parser.add_argument("--cycles", type=int, default=10)
    repeat_parser.add_argument("--dt", type=float, default=0.01)
    repeat_parser.add_argument("--no-viewer", action="store_true")
    repeat_parser.add_argument("--success-tolerance", type=float, default=0.04)
    repeat_parser.add_argument("--grasp-z", type=float, default=0.078, help="Pinch-site height used for the grasp pose, in meters.")
    repeat_parser.add_argument("--carry-offset", type=float, default=0.035, help="Object center offset below the pinch site while carried, in meters.")
    repeat_parser.add_argument("--attach-tolerance", type=float, default=0.018, help="Maximum pinch-site distance before the fixture attaches the block, in meters.")

    return parser


def _print_feedback(frames) -> None:
    for frame in frames:
        if frame.identifier == 0x201:
            try:
                decoded = HandStatusFeedback.decode(frame.payload)
                print(
                    "hand status:",
                    f"pos={decoded.actual_position_deg:.3f}deg",
                    f"vel={decoded.actual_velocity_deg_s:.2f}deg/s",
                    f"torque={decoded.actual_torque_nm:.2f}Nm",
                    f"temp={decoded.motor_temperature_c}C",
                    f"error={decoded.error_code}",
                    f"enable={decoded.enable_status}",
                    f"mode={decoded.feedback_mode}",
                )
            except Exception as exc:
                print(f"hand status decode failed: {exc}")
        elif frame.identifier == 0x301:
            try:
                fault = HandFaultStatus.decode(frame.payload)
                print(
                    "hand fault:",
                    f"error={fault.fault_code}",
                    f"severity={fault.fault_severity}",
                    f"source={fault.fault_source}",
                    f"detail={fault.fault_detail}",
                )
            except Exception as exc:
                print(f"hand fault decode failed: {exc}")
        elif frame.identifier == ARM_STATUS_FEEDBACK_ID:
            try:
                decoded = ArmStatusFeedback.decode(frame.payload)
                print(
                    "arm status:",
                    f"j1={decoded.j1_actual_deg:.1f}",
                    f"j2={decoded.j2_actual_deg:.1f}",
                    f"j3={decoded.j3_actual_deg:.1f}",
                    f"j4={decoded.j4_actual_deg:.1f}",
                    f"j5={decoded.j5_actual_deg:.1f}",
                    f"j6={decoded.j6_actual_deg:.1f}",
                    f"enable={decoded.enable_status}",
                    f"mode={decoded.feedback_mode}",
                    f"error={decoded.error_code}",
                )
            except Exception as exc:
                print(f"arm status decode failed: {exc}")
        elif frame.identifier == ARM_FAULT_STATUS_ID:
            try:
                fault = ArmFaultStatus.decode(frame.payload)
                print(
                    "arm fault:",
                    f"error={fault.fault_code}",
                    f"severity={fault.fault_severity}",
                    f"source={fault.fault_source}",
                    f"detail={fault.fault_detail}",
                )
            except Exception as exc:
                print(f"arm fault decode failed: {exc}")
        else:
            print(
                "frame:",
                hex(frame.identifier),
                frame.payload.hex(" "),
                f"ts={frame.timestamp_us}",
                f"chn={frame.channel}",
            )


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)

    if args.command == "camera-preview":
        camera_args = [
            "--model",
            args.model,
            "--camera-name",
            args.camera_name,
            "--width",
            str(args.width),
            "--height",
            str(args.height),
            "--dt",
            str(args.dt),
        ]
        if args.depth:
            camera_args.append("--depth")
        if args.rgbd:
            camera_args.append("--rgbd")
        if args.duration is not None:
            camera_args.extend(["--duration", str(args.duration)])
        return camera_preview_main(camera_args)

    if args.command == "vision-detect-demo":
        return VisionDetectDemoRunner(
            VisionDetectConfig(
                model_path=args.model,
                camera_name=args.camera_name,
                width=args.width,
                height=args.height,
                duration_s=args.duration,
                print_period_s=args.print_period,
            )
        ).run()

    if args.command == "repeat-pick-place-test":
        RepeatPickPlaceTestRunner(
            RepeatPickPlaceConfig(
                model_path=args.model,
                cycles=args.cycles,
                dt_s=args.dt,
                success_tolerance_m=args.success_tolerance,
                grasp_z_m=args.grasp_z,
                carry_z_offset_m=args.carry_offset,
                attach_tolerance_m=args.attach_tolerance,
                enable_viewer=not args.no_viewer,
            )
        ).run()
        return 0

    if args.command == "bridge":
        bridge_args = []
        bridge_args.extend(
            [
                "--app-name",
                args.app_name,
                "--server-name",
                args.server_name,
                "--api-context-name",
                args.api_context_name,
            ]
        )
        if args.model:
            bridge_args.extend(["--model", args.model])
        if args.duration is not None:
            bridge_args.extend(["--duration", str(args.duration)])
        bridge_args.extend(
            [
                "--dt",
                str(args.dt),
                "--status-period",
                str(args.status_period),
                "--fault-period",
                str(args.fault_period),
                "--diag-period",
                str(args.diag_period),
            ]
        )
        if args.camera_preview:
            bridge_args.append("--camera-preview")
        bridge_args.extend(["--camera-name", args.camera_name])
        bridge_args.extend(["--camera-width", str(args.camera_width)])
        bridge_args.extend(["--camera-height", str(args.camera_height)])
        if args.camera_depth:
            bridge_args.append("--camera-depth")
        if args.camera_rgbd:
            bridge_args.append("--camera-rgbd")
        try:
            return bridge_main(bridge_args)
        except RuntimeError as exc:
            print(f"RuntimeError: {exc}")
            return 1

    try:
        app = AttachedTSMaster(
            app_name=args.app_name,
            server_name=args.server_name,
            api_context_name=args.api_context_name,
        )
        rpc_only = args.command in {"status", "start", "stop"}
        app.open(rpc_only=rpc_only)
        try:
            current = app.get_current_application()
            if current:
                print(f"attached application: {current}")
            print(
                "rpc binding:"
                f" project_name={app.app_name}"
                f" local_application={app.local_application_name}"
                f" server_name={app.server_name}"
                f" api_context_name={app.api_context_name}"
            )

            if args.command == "status":
                print(f"simulation_running={app.is_simulation_running()}")
                return 0

            if args.command == "start":
                app.rpc_log(args.log)
                try:
                    app.start_simulation()
                except TSMasterError:
                    if not app.is_simulation_running():
                        raise
                print(f"simulation_running={app.is_simulation_running()}")
                return 0

            if args.command == "stop":
                try:
                    app.stop_simulation()
                except TSMasterError:
                    if app.is_simulation_running():
                        raise
                app.rpc_log(args.log)
                print(f"simulation_running={app.is_simulation_running()}")
                return 0

            if args.command == "load-dbc":
                dbc_path = Path(args.path).resolve()
                database_id = app.load_dbc(str(dbc_path), args.channel_mask.encode("utf-8"))
                print(f"dbc_loaded={dbc_path} db_id={database_id}")
                return 0

            if args.command == "send-control":
                payload = HandControlCommand(
                    enable=not args.disable,
                    mode=args.mode,
                    target_position_deg=args.position,
                    target_velocity_deg_s=args.velocity,
                    target_torque_nm=args.torque,
                ).encode()
                app.send_default_control(payload, args.period_ms)
                print(f"cyclic_control_added id=0x101 period_ms={args.period_ms} payload={payload.hex(' ')}")
                return 0

            if args.command == "send-arm-control":
                payload = ArmControlCommand(
                    enable=not args.disable,
                    mode=args.mode,
                    j1_target_deg=args.j1,
                    j2_target_deg=args.j2,
                    j3_target_deg=args.j3,
                    j4_target_deg=args.j4,
                    j5_target_deg=args.j5,
                    j6_target_deg=args.j6,
                ).encode()
                app.add_cyclic_canfd(0x110, payload, args.period_ms)
                print(f"cyclic_arm_control_added id=0x110 period_ms={args.period_ms} payload={payload.hex(' ')}")
                return 0

            if args.command == "clear-cyclic":
                app.clear_cyclic()
                print("cyclic_messages_deleted")
                return 0

            if args.command == "read-feedback":
                app.enable_fifo()
                time.sleep(args.wait_ms / 1000.0)
                frames = app.receive_canfd(max_count=args.max_count, only_rx=args.only_rx)
                print(f"frames={len(frames)}")
                _print_feedback(frames)
                return 0

            if args.command == "diag-demo":
                uds_handle = app.create_diag_module(
                    request_id=args.request_id,
                    response_id=args.response_id,
                    functional_id=args.functional_id,
                )
                print(f"diag_handle={uds_handle}")
                app.delete_diag_module(uds_handle)
                print(f"diag_deleted={uds_handle}")
                return 0

            if args.command == "algorithm-demo":
                runner = AlgorithmDemoRunner(
                    app=app,
                    config=AlgorithmDemoConfig(
                        command_period_s=args.command_period,
                        feedback_poll_period_s=args.feedback_period,
                    ),
                )
                runner.run()
                return 0
        finally:
            app.close()

    except TSMasterError as exc:
        print(f"TSMasterError: {exc}")
        return 1

    return 0
