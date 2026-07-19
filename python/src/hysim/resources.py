"""Typed resources returned by the scalable v1 read APIs."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .errors import TransportError
from .models import BusRef


def _schema(payload: Mapping[str, Any], expected: str) -> str:
    value = payload.get("schema")
    if value != expected:
        raise TransportError(f"expected {expected}, received {value!r}")
    return expected


def _integer(payload: Mapping[str, Any], key: str, default: int = 0) -> int:
    value = payload.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise TransportError(f"v1 resource field {key} must be an integer")
    return value


def _optional_integer(payload: Mapping[str, Any], key: str) -> int | None:
    value = payload.get(key)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int):
        raise TransportError(f"v1 resource field {key} must be an integer or null")
    return value


def _nonnegative(value: int, key: str) -> int:
    if value < 0:
        raise TransportError(f"v1 resource field {key} must be non-negative")
    return value


def _rows(payload: Mapping[str, Any], key: str) -> tuple[Mapping[str, Any], ...]:
    value = payload.get(key, [])
    if not isinstance(value, list) or any(not isinstance(item, Mapping) for item in value):
        raise TransportError(f"v1 resource field {key} must be an array of objects")
    return tuple(dict(item) for item in value)


def _bus_ref(value: Mapping[str, Any]) -> BusRef:
    try:
        return BusRef.from_dict(value)
    except (TypeError, ValueError) as exc:
        raise TransportError(f"invalid domain-qualified bus reference: {value!r}") from exc


@dataclass(frozen=True)
class TopologyChunk:
    session_id: str
    model_revision: int
    lod: int
    total_nodes: int
    returned_nodes: int
    offset: int
    limit: int
    next_offset: int | None
    nodes: tuple[Mapping[str, Any], ...]
    edges: tuple[Mapping[str, Any], ...]
    raw: Mapping[str, Any]

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "TopologyChunk":
        _schema(payload, "hysim_topology_chunk_v1")
        nodes = _rows(payload, "nodes")
        returned = _integer(payload, "returned_nodes", len(nodes))
        if returned != len(nodes):
            raise TransportError("topology returned_nodes does not match nodes length")
        lod = _integer(payload, "lod")
        if not 0 <= lod <= 2:
            raise TransportError("topology lod is outside [0, 2]")
        if lod == 2:
            for node in nodes:
                ref = node.get("ref")
                if not isinstance(ref, Mapping):
                    raise TransportError("LOD2 topology node is missing its stable bus ref")
                _bus_ref(ref)
        total = _nonnegative(_integer(payload, "total_nodes"), "total_nodes")
        if returned > total:
            raise TransportError("topology returned_nodes exceeds total_nodes")
        limit = _integer(payload, "limit")
        if limit < 1:
            raise TransportError("topology limit must be positive")
        return cls(
            session_id=str(payload.get("session_id", "")),
            model_revision=_integer(payload, "model_revision"),
            lod=lod,
            total_nodes=total,
            returned_nodes=returned,
            offset=_nonnegative(_integer(payload, "offset"), "offset"),
            limit=limit,
            next_offset=_optional_integer(payload, "next_offset"),
            nodes=nodes,
            edges=_rows(payload, "edges"),
            raw=dict(payload),
        )

    @property
    def bus_refs(self) -> tuple[BusRef, ...]:
        refs: list[BusRef] = []
        for node in self.nodes:
            value = node.get("ref")
            if isinstance(value, Mapping):
                refs.append(_bus_ref(value))
        return tuple(refs)


@dataclass(frozen=True)
class SubgraphView:
    session_id: str
    model_revision: int
    center: BusRef
    depth: int
    truncated: bool
    nodes: tuple[Mapping[str, Any], ...]
    edges: tuple[Mapping[str, Any], ...]
    raw: Mapping[str, Any]

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "SubgraphView":
        _schema(payload, "hysim_subgraph_v1")
        center = payload.get("center")
        if not isinstance(center, Mapping):
            raise TransportError("subgraph center must be a domain-qualified bus reference")
        truncated = payload.get("truncated", False)
        if not isinstance(truncated, bool):
            raise TransportError("subgraph truncated must be boolean")
        depth = _integer(payload, "depth")
        if not 0 <= depth <= 8:
            raise TransportError("subgraph depth is outside [0, 8]")
        return cls(
            session_id=str(payload.get("session_id", "")),
            model_revision=_integer(payload, "model_revision"),
            center=_bus_ref(center),
            depth=depth,
            truncated=truncated,
            nodes=_rows(payload, "nodes"),
            edges=_rows(payload, "edges"),
            raw=dict(payload),
        )


@dataclass(frozen=True)
class ResultFrameChunk:
    job_id: str
    session_id: str
    model_revision: int
    step: int
    domain: str
    lod: int
    total_nodes: int
    returned_nodes: int
    offset: int
    limit: int
    next_offset: int | None
    nodes: tuple[Mapping[str, Any], ...]
    branches: tuple[Mapping[str, Any], ...]
    raw: Mapping[str, Any]

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "ResultFrameChunk":
        _schema(payload, "hysim_result_frame_chunk_v1")
        nodes = _rows(payload, "nodes")
        returned = _integer(payload, "returned_nodes", len(nodes))
        if returned != len(nodes):
            raise TransportError("frame returned_nodes does not match nodes length")
        total = _nonnegative(_integer(payload, "total_nodes"), "total_nodes")
        if returned > total:
            raise TransportError("frame returned_nodes exceeds total_nodes")
        limit = _integer(payload, "limit")
        if limit < 1:
            raise TransportError("frame limit must be positive")
        return cls(
            job_id=str(payload.get("job_id", "")),
            session_id=str(payload.get("session_id", "")),
            model_revision=_integer(payload, "model_revision"),
            step=_nonnegative(_integer(payload, "step"), "step"),
            domain=str(payload.get("domain", "all")),
            lod=_integer(payload, "lod"),
            total_nodes=total,
            returned_nodes=returned,
            offset=_nonnegative(_integer(payload, "offset"), "offset"),
            limit=limit,
            next_offset=_optional_integer(payload, "next_offset"),
            nodes=nodes,
            branches=_rows(payload, "branches"),
            raw=dict(payload),
        )

    @property
    def bus_refs(self) -> tuple[BusRef, ...]:
        return tuple(_bus_ref(node) for node in self.nodes)


@dataclass(frozen=True)
class ViolationChunk:
    job_id: str
    session_id: str
    model_revision: int
    step: int
    total: int
    returned: int
    items: tuple[Mapping[str, Any], ...]
    raw: Mapping[str, Any]

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "ViolationChunk":
        _schema(payload, "hysim_result_violation_chunk_v1")
        items = _rows(payload, "items")
        returned = _integer(payload, "returned", len(items))
        if returned != len(items):
            raise TransportError("violation returned count does not match items length")
        total = _nonnegative(_integer(payload, "total"), "total")
        if returned > total:
            raise TransportError("violation returned count exceeds total")
        return cls(
            job_id=str(payload.get("job_id", "")),
            session_id=str(payload.get("session_id", "")),
            model_revision=_integer(payload, "model_revision"),
            step=_nonnegative(_integer(payload, "step"), "step"),
            total=total,
            returned=returned,
            items=items,
            raw=dict(payload),
        )
