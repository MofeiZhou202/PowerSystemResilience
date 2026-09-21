"""Strict, immutable parsing for runtime edition capability discovery."""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Iterable, Mapping, Protocol

from .errors import TransportError


EDITION_PROFILE_PATH = "/api/edition"
EDITION_PROFILE_SCHEMA = "hacdcpf.edition-profile.v1"
ANALYSIS_CATALOG_FIELD = "analysis_catalog"
ANALYSIS_CATALOG_SCHEMA = "hacdcpf.edition-analysis-catalog.v1"
ANALYSIS_CATALOG_ENTRY_FIELDS = frozenset({"name", "route", "method", "enabled"})
KNOWN_EDITIONS = frozenset({"full", "trial", "resilience"})


class _KnownAnalysis(Protocol):
    name: str
    route: str
    method: str


@dataclass(frozen=True)
class AnalysisCapability:
    """One canonical legacy/session analysis advertised by the runtime."""

    name: str
    route: str
    method: str
    enabled: bool


@dataclass(frozen=True)
class EditionProfile:
    """Validated ``/api/edition`` profile and its dedicated analysis catalog.

    The profile envelope may gain unrelated metadata, but the nested analysis
    catalog is deliberately versioned and exact. This prevents a partially or
    ambiguously parsed edition response from enabling an analysis.
    """

    schema: str
    edition: str
    entries: tuple[AnalysisCapability, ...]
    _by_name: Mapping[str, AnalysisCapability] = field(
        init=False, repr=False, compare=False
    )

    def __post_init__(self) -> None:
        by_name: dict[str, AnalysisCapability] = {}
        for entry in self.entries:
            if entry.name in by_name:
                raise ValueError(f"duplicate analysis capability: {entry.name!r}")
            by_name[entry.name] = entry
        object.__setattr__(self, "_by_name", MappingProxyType(by_name))

    @property
    def catalog_schema(self) -> str:
        return ANALYSIS_CATALOG_SCHEMA

    @property
    def enabled_names(self) -> frozenset[str]:
        return frozenset(entry.name for entry in self.entries if entry.enabled)

    def capability(self, name: str) -> AnalysisCapability | None:
        return self._by_name.get(name)

    @classmethod
    def parse(
        cls,
        payload: Mapping[str, Any],
        known_analyses: Iterable[_KnownAnalysis],
    ) -> "EditionProfile":
        """Parse and reconcile a profile against the SDK's known universe.

        Every SDK-known canonical analysis must have one entry and its route and
        method must agree with the SDK catalog. Unknown or missing entries fail
        closed so a newer or partial server cannot silently expand availability.
        """

        try:
            return cls._parse(payload, tuple(known_analyses))
        except TransportError:
            raise
        except (KeyError, TypeError, ValueError) as exc:
            raise TransportError(f"{EDITION_PROFILE_PATH} profile is malformed: {exc}") from exc

    @classmethod
    def _parse(
        cls,
        payload: Mapping[str, Any],
        known_analyses: tuple[_KnownAnalysis, ...],
    ) -> "EditionProfile":
        if not isinstance(payload, Mapping):
            raise TransportError(f"{EDITION_PROFILE_PATH} must return a JSON object")

        schema = payload.get("schema")
        if schema != EDITION_PROFILE_SCHEMA:
            raise TransportError(
                f"{EDITION_PROFILE_PATH} schema must be {EDITION_PROFILE_SCHEMA!r}"
            )
        edition = payload.get("edition")
        if not isinstance(edition, str) or edition not in KNOWN_EDITIONS:
            raise TransportError(
                f"{EDITION_PROFILE_PATH} edition must be one of {sorted(KNOWN_EDITIONS)}"
            )

        catalog = payload.get(ANALYSIS_CATALOG_FIELD)
        if not isinstance(catalog, Mapping):
            raise TransportError(
                f"{EDITION_PROFILE_PATH} is missing object field {ANALYSIS_CATALOG_FIELD!r}"
            )
        catalog_keys = set(catalog)
        if catalog_keys != {"schema", "entries"}:
            raise TransportError(
                f"{ANALYSIS_CATALOG_FIELD} fields must be exactly 'schema' and 'entries'"
            )
        if catalog.get("schema") != ANALYSIS_CATALOG_SCHEMA:
            raise TransportError(
                f"{ANALYSIS_CATALOG_FIELD}.schema must be {ANALYSIS_CATALOG_SCHEMA!r}"
            )
        raw_entries = catalog.get("entries")
        if not isinstance(raw_entries, list):
            raise TransportError(f"{ANALYSIS_CATALOG_FIELD}.entries must be an array")

        entries: list[AnalysisCapability] = []
        by_name: dict[str, AnalysisCapability] = {}
        for index, raw in enumerate(raw_entries):
            prefix = f"{ANALYSIS_CATALOG_FIELD}.entries[{index}]"
            if not isinstance(raw, Mapping):
                raise TransportError(f"{prefix} must be an object")
            if set(raw) != ANALYSIS_CATALOG_ENTRY_FIELDS:
                raise TransportError(
                    f"{prefix} fields must be exactly {sorted(ANALYSIS_CATALOG_ENTRY_FIELDS)}"
                )
            name = raw.get("name")
            route = raw.get("route")
            method = raw.get("method")
            enabled = raw.get("enabled")
            if not isinstance(name, str) or not name:
                raise TransportError(f"{prefix}.name must be a non-empty string")
            if not isinstance(route, str) or not route.startswith("/api/session/"):
                raise TransportError(
                    f"{prefix}.route must be an absolute /api/session/ path"
                )
            if not isinstance(method, str) or method not in {"GET", "POST"}:
                raise TransportError(f"{prefix}.method must be 'GET' or 'POST'")
            if not isinstance(enabled, bool):
                raise TransportError(f"{prefix}.enabled must be a boolean")
            if name in by_name:
                raise TransportError(f"duplicate analysis catalog entry {name!r}")
            entry = AnalysisCapability(name, route, method, enabled)
            entries.append(entry)
            by_name[name] = entry

        known_by_name = {spec.name: spec for spec in known_analyses}
        missing = sorted(set(known_by_name) - set(by_name))
        unexpected = sorted(set(by_name) - set(known_by_name))
        if missing or unexpected:
            details: list[str] = []
            if missing:
                details.append(f"missing SDK-known analyses: {', '.join(missing)}")
            if unexpected:
                details.append(f"contains unknown analyses: {', '.join(unexpected)}")
            raise TransportError(f"{ANALYSIS_CATALOG_FIELD} " + "; ".join(details))
        for name, spec in known_by_name.items():
            entry = by_name[name]
            if entry.route != spec.route or entry.method != spec.method:
                raise TransportError(
                    f"analysis catalog entry {name!r} does not match SDK route/method"
                )

        return cls(schema=schema, edition=edition, entries=tuple(entries))
