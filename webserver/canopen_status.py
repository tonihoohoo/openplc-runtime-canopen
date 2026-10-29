# SPDX-License-Identifier: Apache-2.0
"""Live CANopen status, read from the plugin's local socket.

The CANopen plugin answers ``{"op": "status"}`` on a Unix socket while it
runs. This module asks it and hands the answer on unchanged, so the plugin
works the same with or without this web route.
"""

import json
import socket
from pathlib import Path
from typing import Dict, Tuple, Union

from webserver.logger import get_logger

logger, _ = get_logger(__name__)

STATUS_SOCKET = Path("/run/runtime/canopen.socket")
TIMEOUT_S = 0.5
MAX_ANSWER = 4 * 1024 * 1024

JsonObject = Dict[str, object]


class CanopenStatusError(Exception):
    """The plugin's socket did not give a usable answer."""


def _ask(path: Path, timeout: float) -> bytes:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(str(path))
        sock.sendall(b'{"op":"status"}\n')
        answer = b""
        while not answer.endswith(b"\n"):
            chunk = sock.recv(65536)
            if not chunk:
                break
            answer += chunk
            if len(answer) > MAX_ANSWER:
                raise CanopenStatusError("the status answer is too long")
        return answer


def read_canopen_status(path: Path = STATUS_SOCKET, timeout: float = TIMEOUT_S) -> JsonObject:
    """The plugin's status answer, or ``{"active": False}`` when CANopen is not running.

    Raises CanopenStatusError when the socket exists but gives no valid answer.
    """
    if not path.exists():
        return {"active": False}
    try:
        raw = _ask(path, timeout)
    except (ConnectionRefusedError, FileNotFoundError):
        # The plugin stopped between the check and the connect.
        return {"active": False}
    except socket.timeout as exc:
        raise CanopenStatusError("the CANopen plugin did not answer in time") from exc
    except OSError as exc:
        raise CanopenStatusError(f"cannot read the CANopen status: {exc}") from exc
    try:
        answer: Union[JsonObject, object] = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CanopenStatusError("the CANopen status answer is not valid JSON") from exc
    if not isinstance(answer, dict):
        raise CanopenStatusError("the CANopen status answer is not a JSON object")
    if "error" in answer:
        raise CanopenStatusError(f"the CANopen plugin refused the request: {answer['error']}")
    return answer


def canopen_status_response() -> Tuple[JsonObject, int]:
    """Body and HTTP status for GET /api/canopen/status."""
    try:
        return read_canopen_status(), 200
    except CanopenStatusError as exc:
        logger.warning("CANopen status: %s", exc)
        return {"active": False, "error": str(exc)}, 502
