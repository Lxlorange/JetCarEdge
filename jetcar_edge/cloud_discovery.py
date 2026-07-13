from __future__ import annotations

import json
import socket
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class CloudDiscoveryConfig:
    enabled: bool = False
    port: int = 8765
    listen_seconds: float = 2.0
    service_name: str = "JetCarCloud"


def discover_cloud_host(config: CloudDiscoveryConfig) -> str:
    if not config.enabled:
        return ""

    deadline = time.monotonic() + max(0.1, config.listen_seconds)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.2)
        sock.bind(("", int(config.port)))
        while time.monotonic() < deadline:
            try:
                data, address = sock.recvfrom(4096)
            except socket.timeout:
                continue
            try:
                payload = json.loads(data.decode("utf-8"))
            except Exception:
                continue
            if not isinstance(payload, dict):
                continue
            if payload.get("service") != config.service_name:
                continue
            host = str(payload.get("host") or address[0]).strip()
            if host:
                return host
    return ""
