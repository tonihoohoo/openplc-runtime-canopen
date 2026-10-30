# SPDX-License-Identifier: Apache-2.0
"""Live CANopen status and online requests, through the plugin's local socket.

The CANopen plugin answers one JSON request per line on a Unix socket: the
status, and the online tools' SDO, NMT and service mode requests. This module
forwards them and hands the answers on unchanged, so the plugin works the same
with or without these web routes.
"""

import json
import socket
from pathlib import Path
from typing import Dict, Optional, Tuple, Union

from webserver.logger import get_logger

logger, _ = get_logger(__name__)

STATUS_SOCKET = Path("/run/runtime/canopen.socket")
TIMEOUT_S = 0.5
MAX_ANSWER = 4 * 1024 * 1024
# The plugin refuses longer request lines.
MAX_REQUEST = 160 * 1024

# Requests a client may send through POST /api/canopen/request.
REQUEST_OPS = frozenset({"sdo_read", "sdo_write", "nmt", "job", "service_open", "service_close"})

JsonObject = Dict[str, object]


class CanopenStatusError(Exception):
    """The plugin's socket did not give a usable answer."""


def _ask(path: Path, timeout: float, request: bytes = b'{"op":"status"}') -> bytes:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(str(path))
        sock.sendall(request + b"\n")
        answer = b""
        while not answer.endswith(b"\n"):
            chunk = sock.recv(65536)
            if not chunk:
                break
            answer += chunk
            if len(answer) > MAX_ANSWER:
                raise CanopenStatusError("the answer is too long")
        return answer


def _exchange(path: Path, timeout: float, request: bytes) -> Optional[JsonObject]:
    """The plugin's answer to one request, or None when the plugin's socket is not there."""
    if not path.exists():
        return None
    try:
        raw = _ask(path, timeout, request)
    except (ConnectionRefusedError, FileNotFoundError):
        # The plugin stopped between the check and the connect.
        return None
    except socket.timeout as exc:
        raise CanopenStatusError("the CANopen plugin did not answer in time") from exc
    except OSError as exc:
        raise CanopenStatusError(f"cannot reach the CANopen plugin: {exc}") from exc
    try:
        answer: Union[JsonObject, object] = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CanopenStatusError("the CANopen answer is not valid JSON") from exc
    if not isinstance(answer, dict):
        raise CanopenStatusError("the CANopen answer is not a JSON object")
    return answer


def read_canopen_status(path: Path = STATUS_SOCKET, timeout: float = TIMEOUT_S) -> JsonObject:
    """The plugin's status answer, or ``{"active": False}`` when CANopen is not running.

    Raises CanopenStatusError when the socket exists but gives no valid answer.
    """
    answer = _exchange(path, timeout, b'{"op":"status"}')
    if answer is None:
        return {"active": False}
    if "error" in answer:
        raise CanopenStatusError(f"the CANopen plugin refused the request: {answer['error']}")
    return answer


def send_canopen_request(
    request: object, path: Path = STATUS_SOCKET, timeout: float = TIMEOUT_S
) -> Tuple[JsonObject, int]:
    """Forwards one online-tools request to the plugin; the answer body and HTTP status.

    The plugin's own answers, its refusals included, come back unchanged with 200.
    ``{"active": False}`` means the plugin is not loaded.
    """
    if not isinstance(request, dict):
        return {"error": "bad_request"}, 400
    op = request.get("op")
    if not isinstance(op, str) or op not in REQUEST_OPS:
        return {"error": "unknown_op"}, 400
    line = json.dumps(request, separators=(",", ":")).encode("utf-8")
    if len(line) > MAX_REQUEST:
        return {"error": "too_large"}, 413
    # SDO answers come back as jobs, so the plugin answers at once; opening service mode
    # sets up a CAN interface and may take longer.
    timeout = timeout * 10 if op == "service_open" else timeout
    try:
        answer = _exchange(path, timeout, line)
    except CanopenStatusError as exc:
        logger.warning("CANopen request %s: %s", op, exc)
        return {"error": str(exc)}, 502
    if answer is None:
        return {"active": False}, 200
    return answer, 200


def canopen_status_response() -> Tuple[JsonObject, int]:
    """Body and HTTP status for GET /api/canopen/status."""
    try:
        return read_canopen_status(), 200
    except CanopenStatusError as exc:
        logger.warning("CANopen status: %s", exc)
        return {"active": False, "error": str(exc)}, 502
