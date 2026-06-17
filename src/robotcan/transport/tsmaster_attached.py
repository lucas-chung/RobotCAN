from __future__ import annotations

from ctypes import byref, c_bool, c_char_p, c_int32, c_size_t
from dataclasses import dataclass
import time
from typing import Iterable

from TSMasterAPI import (
    TLIBCANFD,
    finalize_lib_tsmaster,
    initialize_lib_tsmaster,
    tsapp_add_cyclic_msg_canfd,
    tsapp_connect,
    tsapp_delete_cyclic_msgs,
    tsapp_disconnect,
    tsapp_get_error_description,
    tsapp_set_current_application,
    tsapp_transmit_canfd_async,
    tsdb_load_can_db,
    tsdiag_can_create,
    tsfifo_enable_receive_fifo,
    tsfifo_receive_canfd_msgs,
)
from TSMasterAPI import TSAPI as dll

from robotcan.protocol import (
    CONTROL_COMMAND_ID,
    DIAG_RESPONSE_ID,
    FAULT_STATUS_ID,
    STATUS_FEEDBACK_ID,
)


API_CONTEXT_NAME = b"TSMaster"
TARGET_APPLICATION_NAME = b"TSMaster"
SERVER_NAME = b"TSMaster"
APP_CHANNEL = 0
READ_MODE_TX_RX_MESSAGES = 0
READ_MODE_ONLY_RX_MESSAGES = 1
FIFO_EMPTY_RET_CODES = {1}

CANFD_LENGTH_TO_DLC = {
    0: 0,
    1: 1,
    2: 2,
    3: 3,
    4: 4,
    5: 5,
    6: 6,
    7: 7,
    8: 8,
    12: 9,
    16: 10,
    20: 11,
    24: 12,
    32: 13,
    48: 14,
    64: 15,
}
CANFD_DLC_TO_LENGTH = {value: key for key, value in CANFD_LENGTH_TO_DLC.items()}


class TSMasterError(RuntimeError):
    pass


def _ensure_bytes(value: str | bytes) -> bytes:
    if isinstance(value, bytes):
        return value
    return value.encode("utf-8")


def _check(ret: int, action: str) -> int:
    if ret != 0:
        try:
            detail = tsapp_get_error_description(ret)
        except Exception:
            detail = ""
        if isinstance(detail, bytes):
            detail = detail.decode("utf-8", errors="ignore")
        suffix = f": {detail}" if detail else ""
        raise TSMasterError(f"{action} failed with code {ret}{suffix}")
    return ret


def _try_initialize(*candidates: bytes) -> tuple[int, bytes]:
    last_ret = -1
    tried: set[bytes] = set()
    for candidate in candidates:
        if not candidate or candidate in tried:
            continue
        tried.add(candidate)
        ret = initialize_lib_tsmaster(candidate)
        if ret == 0:
            return ret, candidate
        last_ret = ret
    return last_ret, candidates[0] if candidates else API_CONTEXT_NAME


def _pick_local_application_name(
    preferred_application: bytes,
    api_context_name: bytes,
    server_name: bytes,
) -> bytes:
    for candidate in (preferred_application, api_context_name, server_name):
        if candidate:
            return candidate
    return API_CONTEXT_NAME


def _to_bytes(frame: TLIBCANFD) -> bytes:
    payload_length = CANFD_DLC_TO_LENGTH.get(int(frame.FDLC), int(frame.FDLC))
    return bytes(int(frame.FData[index]) for index in range(payload_length))


def _build_canfd_frame(identifier: int, payload: bytes, channel: int = 0, brs: bool = True) -> TLIBCANFD:
    if len(payload) not in CANFD_LENGTH_TO_DLC:
        raise TSMasterError(
            "unsupported CAN FD payload length "
            f"{len(payload)}; expected one of {sorted(CANFD_LENGTH_TO_DLC)}"
        )
    frame = TLIBCANFD()
    frame.FIdxChn = channel
    frame.FIdentifier = identifier
    frame.FProperties = 1
    frame.FFDProperties = 0x1 | (0x2 if brs else 0x0)
    frame.FDLC = CANFD_LENGTH_TO_DLC[len(payload)]
    for index, value in enumerate(payload):
        frame.FData[index] = value
    return frame


@dataclass(slots=True)
class ReceivedFrame:
    identifier: int
    channel: int
    timestamp_us: int
    payload: bytes


class AttachedTSMaster:
    def __init__(
        self,
        app_name: str | bytes = TARGET_APPLICATION_NAME,
        server_name: str | bytes = SERVER_NAME,
        api_context_name: str | bytes = API_CONTEXT_NAME,
    ):
        self._target_application = _ensure_bytes(app_name)
        self._server_name = _ensure_bytes(server_name)
        self._api_context_name = _ensure_bytes(api_context_name)
        self._local_application = _pick_local_application_name(
            preferred_application=self._api_context_name,
            api_context_name=self._api_context_name,
            server_name=self._server_name,
        )
        self._client_handle = c_size_t(0)
        self._initialized = False
        self._rpc_active = False
        self._bus_connected = False
        self._rpc_only = False

    def open(self, rpc_only: bool = False) -> "AttachedTSMaster":
        self._rpc_only = rpc_only
        # Follow the RPC demo pattern: keep the local tsapp_* context attached to
        # the stable TSMaster application name, and use rpc_* to bind to the
        # running TSMaster process. The project title (for example "robot")
        # should not be used as the tsapp current application name.
        init_ret, chosen_context = _try_initialize(
            self._api_context_name,
            self._target_application,
            self._server_name,
        )
        _check(init_ret, "initialize_lib_tsmaster")
        self._initialized = True
        self._api_context_name = chosen_context
        if not rpc_only:
            self._local_application = _pick_local_application_name(
                preferred_application=chosen_context,
                api_context_name=chosen_context,
                server_name=self._server_name,
            )
            _check(tsapp_set_current_application(self._local_application), "tsapp_set_current_application")
        _check(dll.rpc_tsmaster_create_client(self._server_name, self._client_handle), "rpc_tsmaster_create_client")
        _check(dll.rpc_tsmaster_activate_client(self._client_handle, True), "rpc_tsmaster_activate_client")
        self._rpc_active = True
        return self

    def close(self) -> None:
        if self._rpc_active:
            dll.rpc_tsmaster_activate_client(self._client_handle, False)
            dll.rpc_tsmaster_delete_client(self._client_handle)
            self._rpc_active = False
        if self._bus_connected:
            try:
                tsapp_disconnect()
            finally:
                self._bus_connected = False
        if self._initialized:
            finalize_lib_tsmaster()
            self._initialized = False

    def __enter__(self) -> "AttachedTSMaster":
        return self.open()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def client_handle(self) -> c_size_t:
        return self._client_handle

    @property
    def app_name(self) -> str:
        return self._target_application.decode("utf-8", errors="ignore")

    @property
    def local_application_name(self) -> str:
        return self._local_application.decode("utf-8", errors="ignore")

    @property
    def server_name(self) -> str:
        return self._server_name.decode("utf-8", errors="ignore")

    @property
    def api_context_name(self) -> str:
        return self._api_context_name.decode("utf-8", errors="ignore")

    def get_current_application(self) -> str | None:
        if self._rpc_only:
            return None
        current_app = c_char_p()
        ret = dll.tsapp_get_current_application(byref(current_app))
        _check(ret, "tsapp_get_current_application")
        if current_app.value:
            return current_app.value.decode("utf-8", errors="ignore")
        return None

    def rpc_log(self, message: str, level: int = 0) -> None:
        _check(dll.rpc_tsmaster_cmd_log(self._client_handle, message.encode("utf-8"), level), "rpc_tsmaster_cmd_log")

    def is_simulation_running(self) -> bool:
        running = c_bool(False)
        _check(dll.rpc_tsmaster_is_simulation_running(self._client_handle, running), "rpc_tsmaster_is_simulation_running")
        return bool(running.value)

    def start_simulation(self) -> None:
        ret = dll.rpc_tsmaster_cmd_start_simulation(self._client_handle)
        if ret == 0:
            return
        time.sleep(0.3)
        try:
            if self.is_simulation_running():
                return
        except TSMasterError:
            pass
        _check(ret, "rpc_tsmaster_cmd_start_simulation")

    def stop_simulation(self) -> None:
        ret = dll.rpc_tsmaster_cmd_stop_simulation(self._client_handle)
        if ret == 0:
            return
        time.sleep(0.3)
        try:
            if not self.is_simulation_running():
                return
        except TSMasterError:
            pass
        _check(ret, "rpc_tsmaster_cmd_stop_simulation")

    def enable_fifo(self) -> None:
        if self._rpc_only:
            raise TSMasterError("tsapp bus access is unavailable in rpc-only mode")
        if not self._bus_connected:
            _check(tsapp_set_current_application(self._local_application), "tsapp_set_current_application")
            _check(tsapp_connect(), "tsapp_connect")
            self._bus_connected = True
        tsfifo_enable_receive_fifo()

    def load_dbc(self, dbc_path: str, channel_mask: bytes = b"0") -> int:
        database_id = c_int32(0)
        _check(tsdb_load_can_db(dbc_path.encode("utf-8"), channel_mask, database_id), "tsdb_load_can_db")
        return int(database_id.value)

    def create_diag_module(
        self,
        request_id: int = 0x700,
        response_id: int = 0x708,
        functional_id: int = 0x7DF,
    ) -> int:
        uds_handle = c_int32(0)
        _check(
            tsdiag_can_create(
                uds_handle,
                APP_CHANNEL,
                0,
                8,
                request_id,
                True,
                response_id,
                True,
                functional_id,
                True,
            ),
            "tsdiag_can_create",
        )
        return int(uds_handle.value)

    def delete_diag_module(self, uds_handle: int) -> None:
        _check(dll.tsdiag_can_delete(c_int32(uds_handle)), "tsdiag_can_delete")

    def send_canfd(self, identifier: int, payload: bytes, channel: int = 0) -> None:
        if not self._bus_connected:
            self.enable_fifo()
        frame = _build_canfd_frame(identifier, payload, channel, brs=True)
        _check(tsapp_transmit_canfd_async(frame), "tsapp_transmit_canfd_async")

    def add_cyclic_canfd(self, identifier: int, payload: bytes, period_ms: int, channel: int = 0) -> None:
        if not self._bus_connected:
            self.enable_fifo()
        frame = _build_canfd_frame(identifier, payload, channel, brs=True)
        _check(tsapp_add_cyclic_msg_canfd(frame, period_ms), "tsapp_add_cyclic_msg_canfd")

    def clear_cyclic(self) -> None:
        _check(tsapp_delete_cyclic_msgs(), "tsapp_delete_cyclic_msgs")

    def receive_canfd(self, max_count: int = 100, only_rx: bool = False) -> list[ReceivedFrame]:
        buffer = (TLIBCANFD * max_count)()
        count = c_int32(max_count)
        read_mode = READ_MODE_ONLY_RX_MESSAGES if only_rx else READ_MODE_TX_RX_MESSAGES
        ret = tsfifo_receive_canfd_msgs(buffer, count, 0, read_mode)
        if ret in FIFO_EMPTY_RET_CODES:
            return []
        _check(ret, "tsfifo_receive_canfd_msgs")
        frames: list[ReceivedFrame] = []
        for index in range(count.value):
            item = buffer[index]
            if (int(item.FFDProperties) & 0x1) != 0x1:
                continue
            frames.append(
                ReceivedFrame(
                    identifier=int(item.FIdentifier),
                    channel=int(item.FIdxChn),
                    timestamp_us=int(item.FTimeUs),
                    payload=_to_bytes(item),
                )
            )
        return frames

    def find_latest_feedback(self, frames: Iterable[ReceivedFrame]) -> tuple[ReceivedFrame | None, ReceivedFrame | None, ReceivedFrame | None]:
        status = None
        fault = None
        diag = None
        for frame in frames:
            if frame.identifier == STATUS_FEEDBACK_ID:
                status = frame
            elif frame.identifier == FAULT_STATUS_ID:
                fault = frame
            elif frame.identifier == DIAG_RESPONSE_ID:
                diag = frame
        return status, fault, diag

    def send_default_control(self, payload: bytes, period_ms: int = 10) -> None:
        self.add_cyclic_canfd(CONTROL_COMMAND_ID, payload, period_ms)
