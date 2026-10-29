# SPDX-License-Identifier: Apache-2.0
"""The CANopen entry of GET /api/capabilities.

Present only when the plugin library is built, listing the canopen.json format
versions the plugin accepts, read from the plugin's own config.h.
"""

from pathlib import Path

from webserver import canopen_capability as capability_module
from webserver import restapi
from webserver.canopen_capability import CONFIG_HEADER, canopen_capability


def _library(tmp_path: Path) -> Path:
    library = tmp_path / "libcanopen_plugin.so"
    library.write_bytes(b"\x7fELF")
    return library


def test_no_entry_without_the_plugin_library(tmp_path: Path) -> None:
    assert canopen_capability(library=tmp_path / "missing.so", header=CONFIG_HEADER) is None


def test_reports_the_version_from_the_plugin_header(tmp_path: Path) -> None:
    header = tmp_path / "config.h"
    header.write_text(
        "namespace canopen {\nconstexpr int kConfigVersion = 7;\n}\n", encoding="utf-8"
    )
    assert canopen_capability(library=_library(tmp_path), header=header) == {"configVersions": [7]}


def test_matches_the_real_plugin_header(tmp_path: Path) -> None:
    result = canopen_capability(library=_library(tmp_path), header=CONFIG_HEADER)
    assert result is not None
    assert result["configVersions"] and all(v > 0 for v in result["configVersions"])


def test_no_entry_when_the_header_names_no_version(tmp_path: Path) -> None:
    header = tmp_path / "config.h"
    header.write_text("// nothing here\n", encoding="utf-8")
    assert canopen_capability(library=_library(tmp_path), header=header) is None


def test_no_entry_when_the_header_is_missing(tmp_path: Path) -> None:
    assert canopen_capability(library=_library(tmp_path), header=tmp_path / "none.h") is None


def test_endpoint_includes_the_entry_when_present(client, monkeypatch) -> None:
    monkeypatch.setattr(restapi, "canopen_capability", lambda: {"configVersions": [1]})
    body = client.get("/api/capabilities").get_json()
    assert body["canopen"] == {"configVersions": [1]}


def test_endpoint_omits_the_entry_when_absent(client, monkeypatch) -> None:
    monkeypatch.setattr(restapi, "canopen_capability", lambda: None)
    assert "canopen" not in client.get("/api/capabilities").get_json()


def test_default_paths_point_into_the_repository() -> None:
    assert capability_module.CONFIG_HEADER.is_file()
    assert capability_module.PLUGIN_LIBRARY.parent.name == "plugins"
