# SPDX-License-Identifier: Apache-2.0
"""CANopen support advertised at /api/capabilities.

A runtime has CANopen when the plugin library was built. The config format
version it accepts is read from the plugin's ``config.h`` (``kConfigVersion``),
so the advertised value cannot drift from what the plugin checks. ``features``
lists the optional CANopen routes this web server serves.
"""

import re
from pathlib import Path
from typing import List, Optional, TypedDict

from webserver.logger import get_logger

logger, _ = get_logger(__name__)

_REPO_ROOT = Path(__file__).resolve().parent.parent
PLUGIN_LIBRARY = _REPO_ROOT / "build" / "plugins" / "libcanopen_plugin.so"
CONFIG_HEADER = (
    _REPO_ROOT / "core" / "src" / "drivers" / "plugins" / "native" / "canopen" / "config.h"
)

_CONFIG_VERSION_RE = re.compile(r"constexpr\s+int\s+kConfigVersion\s*=\s*(\d+)\s*;")


# status: GET /api/canopen/status; sdo, nmt and service: POST /api/canopen/request
FEATURES = ["status", "sdo", "nmt", "service"]


class CanopenCapability(TypedDict):
    configVersions: List[int]
    features: List[str]


def canopen_capability(
    library: Path = PLUGIN_LIBRARY, header: Path = CONFIG_HEADER
) -> Optional[CanopenCapability]:
    """The CANopen entry for /api/capabilities, or None when the plugin is not built."""
    if not library.is_file():
        return None
    try:
        text = header.read_text(encoding="utf-8")
    except OSError as exc:
        logger.warning("CANopen plugin present but %s is unreadable: %s", header, exc)
        return None
    match = _CONFIG_VERSION_RE.search(text)
    if match is None:
        logger.warning("CANopen plugin present but %s names no config version", header)
        return None
    return {"configVersions": [int(match.group(1))], "features": list(FEATURES)}
