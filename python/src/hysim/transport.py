"""Replaceable transport boundary for the HySim runtime."""

from __future__ import annotations

import json
import socket
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from typing import Any, Mapping, Protocol

from .errors import TransportError


@dataclass(frozen=True)
class TransportResponse:
    status_code: int
    headers: Mapping[str, str]
    body: bytes


class Transport(Protocol):
    """Small protocol that permits HTTP, in-process, and test transports."""

    def request(
        self,
        method: str,
        path: str,
        *,
        json_body: Mapping[str, Any] | None = None,
        query: Mapping[str, str | int | float] | None = None,
        headers: Mapping[str, str] | None = None,
        timeout: float | None = None,
    ) -> TransportResponse: ...


class UrllibTransport:
    """Dependency-free HTTP transport.

    Requests are deliberately not retried. Model changes and analyses are not
    idempotent under the current process-global session contract.
    """

    def __init__(
        self,
        base_url: str,
        *,
        timeout: float = 600.0,
        default_headers: Mapping[str, str] | None = None,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        if not self.base_url.startswith(("http://", "https://")):
            raise ValueError("base_url must start with http:// or https://")
        self.timeout = timeout
        self.default_headers = dict(default_headers or {})

    def request(
        self,
        method: str,
        path: str,
        *,
        json_body: Mapping[str, Any] | None = None,
        query: Mapping[str, str | int | float] | None = None,
        headers: Mapping[str, str] | None = None,
        timeout: float | None = None,
    ) -> TransportResponse:
        if not path.startswith("/") or path.startswith("//"):
            raise ValueError("path must be an absolute server path")
        url = self.base_url + path
        if query:
            url += "?" + urllib.parse.urlencode(query)

        body = None
        request_headers = {"Accept": "application/json", **self.default_headers}
        if json_body is not None:
            body = json.dumps(json_body, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            )
            request_headers["Content-Type"] = "application/json"
        request_headers.update(headers or {})
        request = urllib.request.Request(
            url, data=body, headers=request_headers, method=method.upper()
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout or self.timeout) as response:
                return TransportResponse(
                    status_code=response.status,
                    headers=dict(response.headers.items()),
                    body=response.read(),
                )
        except urllib.error.HTTPError as exc:
            return TransportResponse(
                status_code=exc.code,
                headers=dict(exc.headers.items()) if exc.headers else {},
                body=exc.read(),
            )
        except (urllib.error.URLError, TimeoutError, socket.timeout, OSError) as exc:
            raise TransportError(f"request to {url} failed: {exc}") from exc

