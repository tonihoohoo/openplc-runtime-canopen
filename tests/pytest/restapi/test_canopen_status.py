# SPDX-License-Identifier: Apache-2.0
"""GET /api/canopen/status and POST /api/canopen/request, through the CANopen plugin's socket.

A fake plugin socket stands in for the plugin.
"""

import socket
import threading
from pathlib import Path
from typing import List

import pytest
from conftest import auth, create_user, token_for

from webserver import canopen_status, restapi
from webserver.canopen_status import (
    MAX_REQUEST,
    CanopenStatusError,
    read_canopen_status,
    send_canopen_request,
)


class FakePlugin:
    """Answers each request line on a Unix socket with a fixed answer."""

    def __init__(self, path: Path, answer: bytes) -> None:
        self.path = path
        self.answer = answer
        self._requests: List[bytes] = []
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.bind(str(path))
        self._sock.listen(1)
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self) -> None:
        while True:
            try:
                conn, _ = self._sock.accept()
            except OSError:
                return
            with conn:
                request = conn.recv(4096)
                self._requests.append(request)
                if self.answer:
                    conn.sendall(self.answer)

    def received(self) -> List[bytes]:
        """The requests received so far."""
        return list(self._requests)

    def close(self) -> None:
        """Stops listening."""
        self._sock.close()


def test_no_socket_means_not_active(tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    assert read_canopen_status(sock_path) == {"active": False}


def test_answer_is_returned_unchanged(tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    plugin = FakePlugin(sock_path, b'{"version":1,"active":true,"networks":[]}\n')
    try:
        assert read_canopen_status(sock_path) == {"version": 1, "active": True, "networks": []}
        assert plugin.received() == [b'{"op":"status"}\n']
    finally:
        plugin.close()


def test_malformed_answer_is_an_error(tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    plugin = FakePlugin(sock_path, b"not json\n")
    try:
        with pytest.raises(CanopenStatusError, match="not valid JSON"):
            read_canopen_status(sock_path)
    finally:
        plugin.close()


def test_non_object_answer_is_an_error(tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    plugin = FakePlugin(sock_path, b"[1,2]\n")
    try:
        with pytest.raises(CanopenStatusError, match="not a JSON object"):
            read_canopen_status(sock_path)
    finally:
        plugin.close()


def test_silent_plugin_times_out(tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    plugin = FakePlugin(sock_path, b"")
    try:
        with pytest.raises(CanopenStatusError):
            read_canopen_status(sock_path, timeout=0.2)
    finally:
        plugin.close()


def test_stale_socket_file_means_not_active(tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    # Left behind by a plugin that is gone: the file exists, nobody listens.
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stale:
        stale.bind(str(sock_path))
    assert sock_path.exists()
    assert read_canopen_status(sock_path) == {"active": False}


def test_route_needs_a_login(client) -> None:
    assert client.get("/api/canopen/status").status_code == 401


def test_route_returns_the_status(client, monkeypatch, tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    create_user(client, "admin", "admin-pass")
    token = token_for(client, "admin", "admin-pass")
    plugin = FakePlugin(sock_path, b'{"version":1,"active":true,"networks":[]}\n')
    monkeypatch.setattr(
        canopen_status, "read_canopen_status", lambda: read_canopen_status(sock_path)
    )
    try:
        resp = client.get("/api/canopen/status", headers=auth(token))
        assert resp.status_code == 200
        assert resp.get_json()["active"] is True
    finally:
        plugin.close()


def test_route_reports_a_bad_answer(client, monkeypatch, tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    create_user(client, "admin", "admin-pass")
    token = token_for(client, "admin", "admin-pass")
    plugin = FakePlugin(sock_path, b"garbage\n")
    monkeypatch.setattr(
        canopen_status, "read_canopen_status", lambda: read_canopen_status(sock_path)
    )
    try:
        resp = client.get("/api/canopen/status", headers=auth(token))
        assert resp.status_code == 502
        assert resp.get_json()["active"] is False
    finally:
        plugin.close()


def test_route_without_canopen(client, monkeypatch, tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    create_user(client, "admin", "admin-pass")
    token = token_for(client, "admin", "admin-pass")
    monkeypatch.setattr(
        canopen_status, "read_canopen_status", lambda: read_canopen_status(sock_path)
    )
    resp = client.get("/api/canopen/status", headers=auth(token))
    assert resp.status_code == 200
    assert resp.get_json() == {"active": False}


def test_capabilities_list_the_status_feature(client, monkeypatch) -> None:
    monkeypatch.setattr(
        restapi, "canopen_capability", lambda: {"configVersions": [2], "features": ["status"]}
    )
    body = client.get("/api/capabilities").get_json()
    assert body["canopen"]["features"] == ["status"]


def test_request_is_forwarded_unchanged(tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    plugin = FakePlugin(sock_path, b'{"job":7}\n')
    request = {"op": "sdo_read", "network": 0, "node": 2, "index": 4120, "subindex": 1}
    try:
        assert send_canopen_request(request, sock_path) == ({"job": 7}, 200)
        assert plugin.received() == [
            b'{"op":"sdo_read","network":0,"node":2,"index":4120,"subindex":1}\n'
        ]
    finally:
        plugin.close()


def test_plugin_refusal_is_returned(tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    plugin = FakePlugin(sock_path, b'{"error":"unknown_node"}\n')
    try:
        body, status = send_canopen_request({"op": "nmt", "network": 0, "node": 9}, sock_path)
        assert (body, status) == ({"error": "unknown_node"}, 200)
    finally:
        plugin.close()


@pytest.mark.parametrize(
    "request_body, answer",
    [
        (None, ({"error": "bad_request"}, 400)),
        ([1], ({"error": "bad_request"}, 400)),
        ({"op": "status"}, ({"error": "unknown_op"}, 400)),
        ({"op": "teleport"}, ({"error": "unknown_op"}, 400)),
        ({"node": 2}, ({"error": "unknown_op"}, 400)),
        (
            {"op": "sdo_write", "data": "00" * (MAX_REQUEST // 2)},
            ({"error": "too_large"}, 413),
        ),
    ],
)
def test_refused_before_the_plugin(tmp_path: Path, request_body: object, answer: object) -> None:
    sock_path = tmp_path / "canopen.socket"
    plugin = FakePlugin(sock_path, b'{"ok":true}\n')
    try:
        assert send_canopen_request(request_body, sock_path) == answer
        assert plugin.received() == []
    finally:
        plugin.close()


def test_request_without_the_plugin(tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    assert send_canopen_request({"op": "service_close"}, sock_path) == ({"active": False}, 200)


def test_request_with_a_bad_answer(tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    plugin = FakePlugin(sock_path, b"garbage\n")
    try:
        body, status = send_canopen_request({"op": "job", "job": 1}, sock_path)
        assert status == 502 and "not valid JSON" in str(body["error"])
    finally:
        plugin.close()


def test_request_route_needs_a_login(client) -> None:
    assert client.post("/api/canopen/request", json={"op": "job", "job": 1}).status_code == 401


def test_request_route_forwards(client, monkeypatch, tmp_path: Path) -> None:
    sock_path = tmp_path / "canopen.socket"
    create_user(client, "admin", "admin-pass")
    token = token_for(client, "admin", "admin-pass")
    plugin = FakePlugin(sock_path, b'{"state":"done","data":"0a00"}\n')
    monkeypatch.setattr(
        restapi, "send_canopen_request", lambda body: send_canopen_request(body, sock_path)
    )
    try:
        resp = client.post(
            "/api/canopen/request", json={"op": "job", "job": 3}, headers=auth(token)
        )
        assert resp.status_code == 200
        assert resp.get_json() == {"state": "done", "data": "0a00"}
        resp = client.post(
            "/api/canopen/request",
            data="not json",
            headers=auth(token),
            content_type="application/json",
        )
        assert resp.status_code == 400
    finally:
        plugin.close()
