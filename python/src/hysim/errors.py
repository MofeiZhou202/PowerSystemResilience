"""Exception hierarchy for the HySim Python API."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


class HySimError(Exception):
    """Base class for SDK errors."""


class TransportError(HySimError):
    """The C++ HTTP service could not be reached or returned invalid bytes."""


@dataclass(eq=False)
class ApiError(HySimError):
    """The service rejected a request."""

    status_code: int
    path: str
    message: str
    payload: Mapping[str, Any] | None = None

    def __str__(self) -> str:
        return f"HySim API {self.status_code} at {self.path}: {self.message}"


class BusyError(ApiError):
    """Another long-running analysis owns the process-global session."""


class InvalidResultError(HySimError):
    """A result was returned but is not scientifically usable."""


class ToolPolicyError(HySimError):
    """An AI tool call violates the configured effect or approval policy."""

