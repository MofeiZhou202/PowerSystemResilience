"""Strict parser for the independent ``/api/v1`` discovery document."""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Iterator, Mapping

from .errors import AnalysisDisabledError, TransportError, UnknownAnalysisError


V1_DISCOVERY_PATH = "/api/v1"
V1_SCHEMA = "hysim_api_v1"
V1_VERSION = "1.0"
V1_DISCOVERY_FIELDS = frozenset(
    {"schema", "version", "features", "analyses", "known_disabled_analyses", "limits"}
)


@dataclass(frozen=True)
class V1Capabilities(Mapping[str, Any]):
    """Validated analysis availability for the isolated-session v1 API.

    The object also implements the read-only mapping interface of the historic
    ``capabilities()`` result, preserving callers that use ``get`` or indexing
    while exposing typed admission properties.
    """

    features: tuple[str, ...]
    enabled_analyses: frozenset[str]
    disabled_analyses: frozenset[str]
    limits: Mapping[str, int]

    def __post_init__(self) -> None:
        object.__setattr__(self, "limits", MappingProxyType(dict(self.limits)))

    def __getitem__(self, key: str) -> Any:
        if key == "schema":
            return V1_SCHEMA
        if key == "version":
            return V1_VERSION
        if key == "features":
            return self.features
        if key == "analyses":
            return tuple(sorted(self.enabled_analyses))
        if key == "known_disabled_analyses":
            return tuple(sorted(self.disabled_analyses))
        if key == "limits":
            return self.limits
        raise KeyError(key)

    def __iter__(self) -> Iterator[str]:
        return iter((
            "schema",
            "version",
            "features",
            "analyses",
            "known_disabled_analyses",
            "limits",
        ))

    def __len__(self) -> int:
        return len(V1_DISCOVERY_FIELDS)

    @classmethod
    def parse(cls, payload: Mapping[str, Any]) -> "V1Capabilities":
        if not isinstance(payload, Mapping):
            raise TransportError(f"{V1_DISCOVERY_PATH} must return a JSON object")
        if set(payload) != V1_DISCOVERY_FIELDS:
            raise TransportError(
                f"{V1_DISCOVERY_PATH} fields must be exactly {sorted(V1_DISCOVERY_FIELDS)}"
            )
        if payload.get("schema") != V1_SCHEMA:
            raise TransportError(f"{V1_DISCOVERY_PATH} schema must be {V1_SCHEMA!r}")
        if payload.get("version") != V1_VERSION:
            raise TransportError(f"{V1_DISCOVERY_PATH} version must be {V1_VERSION!r}")

        features = _unique_strings(payload.get("features"), "features")
        enabled = _unique_strings(payload.get("analyses"), "analyses")
        disabled = _unique_strings(
            payload.get("known_disabled_analyses"), "known_disabled_analyses"
        )
        overlap = set(enabled) & set(disabled)
        if overlap:
            raise TransportError(
                f"{V1_DISCOVERY_PATH} analyses cannot be both enabled and disabled: "
                + ", ".join(sorted(overlap))
            )
        if not enabled:
            raise TransportError(f"{V1_DISCOVERY_PATH} must enable at least one analysis")

        limits = payload.get("limits")
        if not isinstance(limits, Mapping) or set(limits) != {"sessions", "retained_jobs"}:
            raise TransportError(
                f"{V1_DISCOVERY_PATH}.limits fields must be 'sessions' and 'retained_jobs'"
            )
        checked_limits: dict[str, int] = {}
        for name in ("sessions", "retained_jobs"):
            value = limits.get(name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise TransportError(
                    f"{V1_DISCOVERY_PATH}.limits.{name} must be a positive integer"
                )
            checked_limits[name] = value

        return cls(
            features=features,
            enabled_analyses=frozenset(enabled),
            disabled_analyses=frozenset(disabled),
            limits=checked_limits,
        )

    def require(self, analysis: str) -> str:
        if not isinstance(analysis, str) or not analysis:
            raise UnknownAnalysisError(str(analysis))
        if analysis in self.enabled_analyses:
            return analysis
        if analysis in self.disabled_analyses:
            raise AnalysisDisabledError(analysis)
        raise UnknownAnalysisError(analysis)


def _unique_strings(value: Any, field: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise TransportError(f"{V1_DISCOVERY_PATH}.{field} must be an array")
    result: list[str] = []
    seen: set[str] = set()
    for index, item in enumerate(value):
        if not isinstance(item, str) or not item:
            raise TransportError(
                f"{V1_DISCOVERY_PATH}.{field}[{index}] must be a non-empty string"
            )
        if item in seen:
            raise TransportError(f"{V1_DISCOVERY_PATH}.{field} contains duplicate {item!r}")
        seen.add(item)
        result.append(item)
    return tuple(result)
