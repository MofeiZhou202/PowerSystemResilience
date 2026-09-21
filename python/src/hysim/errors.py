"""Exception hierarchy for the HySim Python API."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


class HySimError(Exception):
    """Base class for SDK errors."""


class TransportError(HySimError):
    """The C++ HTTP service could not be reached or returned invalid bytes."""


class UnknownAnalysisError(HySimError, LookupError):
    """The requested analysis is not part of the SDK's known universe."""

    def __init__(self, analysis: str) -> None:
        self.analysis = analysis
        super().__init__(f"unknown HySim analysis: {analysis!r}")


class AnalysisDisabledError(HySimError, LookupError):
    """A known analysis is unavailable in the connected runtime edition."""

    def __init__(self, analysis: str, *, edition: str | None = None) -> None:
        self.analysis = analysis
        self.edition = edition
        detail = f" in the {edition!r} edition" if edition is not None else ""
        super().__init__(f"HySim analysis {analysis!r} is disabled{detail}")


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


class JobFailedError(HySimError):
    """An asynchronous v1 analysis job reached the failed state."""

