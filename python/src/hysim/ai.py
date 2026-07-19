"""Provider-neutral AI tool registry with explicit side-effect policy."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Mapping

from .client import HySimClient
from .errors import ToolPolicyError
from .models import OPFRequest, PowerFlowOptions, PowerFlowRequest
from .v1 import HySimV1Session


class ToolEffect(str, Enum):
    READ = "read"
    ANALYZE = "analyze"
    MODIFY_MODEL = "modify_model"
    CONTROL = "control"


ToolHandler = Callable[[Mapping[str, Any]], Mapping[str, Any]]


@dataclass(frozen=True)
class ToolSpec:
    name: str
    description: str
    input_schema: Mapping[str, Any]
    effect: ToolEffect
    handler: ToolHandler = field(repr=False, compare=False)

    def function_schema(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "parameters": dict(self.input_schema),
        }


@dataclass(frozen=True)
class ToolPolicy:
    allowed_effects: frozenset[ToolEffect] = frozenset(
        {ToolEffect.READ, ToolEffect.ANALYZE}
    )
    require_approval_for: frozenset[ToolEffect] = frozenset(
        {ToolEffect.MODIFY_MODEL, ToolEffect.CONTROL}
    )

    def authorize(self, tool: ToolSpec, *, approved: bool) -> None:
        if tool.effect not in self.allowed_effects:
            raise ToolPolicyError(
                f"tool {tool.name} has disabled effect {tool.effect.value}"
            )
        if tool.effect in self.require_approval_for and not approved:
            raise ToolPolicyError(f"tool {tool.name} requires explicit approval")


class HySimToolRegistry:
    """Compact tool surface for LLM agents.

    Direct SDK calls retain full arrays. Tool calls return summaries by default
    so voltage and time-series vectors do not consume the model context window.
    """

    def __init__(
        self,
        client: HySimClient,
        *,
        policy: ToolPolicy | None = None,
    ) -> None:
        self.client = client
        self.policy = policy or ToolPolicy()
        self._tools: dict[str, ToolSpec] = {}
        self._register_builtins()

    def register(self, tool: ToolSpec) -> None:
        if tool.name in self._tools:
            raise ValueError(f"duplicate tool name: {tool.name}")
        self._tools[tool.name] = tool

    def specs(self) -> tuple[ToolSpec, ...]:
        return tuple(self._tools[name] for name in sorted(self._tools))

    def function_schemas(self) -> list[dict[str, Any]]:
        return [tool.function_schema() for tool in self.specs()]

    def openai_tools(self) -> list[dict[str, Any]]:
        """Return the common function-tool envelope without importing an SDK."""

        return [{"type": "function", "function": schema} for schema in self.function_schemas()]

    def call(
        self,
        name: str,
        arguments: Mapping[str, Any] | None = None,
        *,
        approved: bool = False,
    ) -> Mapping[str, Any]:
        try:
            tool = self._tools[name]
        except KeyError as exc:
            raise ValueError(f"unknown HySim tool: {name}") from exc
        self.policy.authorize(tool, approved=approved)
        validated = dict(arguments or {})
        _validate_arguments(tool.input_schema, validated)
        return tool.handler(validated)

    def _register_builtins(self) -> None:
        object_schema = {"type": "object", "properties": {}, "additionalProperties": False}
        self.register(
            ToolSpec(
                "hysim_list_cases",
                "List built-in power-system cases available in the runtime.",
                object_schema,
                ToolEffect.READ,
                lambda _: dict(self.client.list_cases()),
            )
        )
        self.register(
            ToolSpec(
                "hysim_session_status",
                "Report whether the current HySim session is busy or cancelled.",
                object_schema,
                ToolEffect.READ,
                lambda _: dict(self.client.status()),
            )
        )
        self.register(
            ToolSpec(
                "hysim_load_builtin",
                "Replace the current model with a named built-in case.",
                {
                    "type": "object",
                    "properties": {"case": {"type": "string", "minLength": 1}},
                    "required": ["case"],
                    "additionalProperties": False,
                },
                ToolEffect.MODIFY_MODEL,
                lambda args: dict(self.client.load_builtin(str(args["case"]))),
            )
        )
        self.register(
            ToolSpec(
                "hysim_run_power_flow",
                "Run power flow and return convergence, scope, limitations, and result sizes.",
                {
                    "type": "object",
                    "properties": {
                        "method": {"type": "string", "default": "ac_newton"},
                        "max_iter": {"type": "integer", "minimum": 1},
                        "tol": {"type": "number", "exclusiveMinimum": 0},
                        "include_raw": {"type": "boolean", "default": False},
                    },
                    "additionalProperties": False,
                },
                ToolEffect.ANALYZE,
                self._power_flow,
            )
        )
        self.register(
            ToolSpec(
                "hysim_run_optimal_power_flow",
                "Run OPF and return its scientific status, scope, limitations, and result sizes.",
                {
                    "type": "object",
                    "properties": {
                        "solver": {
                            "type": "string",
                            "enum": ["parity", "ipopt", "auto", "dc", "dispatch"],
                            "default": "parity",
                        },
                        "network_model": {
                            "type": "string",
                            "enum": [
                                "balanced_aggregate",
                                "balanced_with_three_phase_validation",
                                "three_phase_hybrid",
                            ],
                            "default": "balanced_aggregate",
                        },
                        "check_consistency": {"type": "boolean", "default": True},
                        "include_raw": {"type": "boolean", "default": False},
                    },
                    "additionalProperties": False,
                },
                ToolEffect.ANALYZE,
                self._optimal_power_flow,
            )
        )
        self.register(
            ToolSpec(
                "hysim_sppt_guard",
                "Check validation, well-posedness, and authored-component attribution.",
                object_schema,
                ToolEffect.ANALYZE,
                lambda _: self.client.run_analysis("sppt_guard").to_record(include_raw=True),
            )
        )

    def _power_flow(self, args: Mapping[str, Any]) -> Mapping[str, Any]:
        request = PowerFlowRequest(
            method=str(args.get("method", "ac_newton")),
            options=PowerFlowOptions(
                max_iter=_optional_int(args.get("max_iter")),
                tol=_optional_float(args.get("tol")),
            ),
        )
        return self.client.power_flow(request).to_record(
            include_raw=bool(args.get("include_raw", False))
        )

    def _optimal_power_flow(self, args: Mapping[str, Any]) -> Mapping[str, Any]:
        request = OPFRequest(
            solver=str(args.get("solver", "parity")),
            network_model=str(args.get("network_model", "balanced_aggregate")),
            check_consistency=bool(args.get("check_consistency", True)),
        )
        return self.client.optimal_power_flow(request).to_record(
            include_raw=bool(args.get("include_raw", False))
        )


class HySimV1ToolRegistry:
    """AI tool surface backed by one isolated, revision-aware v1 session."""

    def __init__(
        self,
        session: HySimV1Session,
        *,
        policy: ToolPolicy | None = None,
    ) -> None:
        self.session = session
        self.policy = policy or ToolPolicy()
        self._tools: dict[str, ToolSpec] = {}
        self._register_builtins()

    def specs(self) -> tuple[ToolSpec, ...]:
        return tuple(self._tools[name] for name in sorted(self._tools))

    def function_schemas(self) -> list[dict[str, Any]]:
        return [tool.function_schema() for tool in self.specs()]

    def openai_tools(self) -> list[dict[str, Any]]:
        return [
            {"type": "function", "function": schema}
            for schema in self.function_schemas()
        ]

    def call(
        self,
        name: str,
        arguments: Mapping[str, Any] | None = None,
        *,
        approved: bool = False,
    ) -> Mapping[str, Any]:
        try:
            tool = self._tools[name]
        except KeyError as exc:
            raise ValueError(f"unknown HySim v1 tool: {name}") from exc
        self.policy.authorize(tool, approved=approved)
        validated = dict(arguments or {})
        _validate_arguments(tool.input_schema, validated)
        return tool.handler(validated)

    def _register(self, tool: ToolSpec) -> None:
        if tool.name in self._tools:
            raise ValueError(f"duplicate tool name: {tool.name}")
        self._tools[tool.name] = tool

    def _register_builtins(self) -> None:
        no_arguments = {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        }
        self._register(
            ToolSpec(
                "hysim_v1_session",
                "Read the isolated session ID, model revision, ETag, and model summary.",
                no_arguments,
                ToolEffect.READ,
                lambda _: dict(self.session.refresh().raw),
            )
        )
        self._register(
            ToolSpec(
                "hysim_v1_topology",
                "Read a bounded topology LOD page with domain-qualified stable bus IDs.",
                {
                    "type": "object",
                    "properties": {
                        "lod": {"type": "integer", "minimum": 0, "maximum": 2,
                                "default": 2},
                        "offset": {"type": "integer", "minimum": 0, "default": 0},
                        "limit": {"type": "integer", "minimum": 1, "maximum": 1000,
                                  "default": 200},
                    },
                    "additionalProperties": False,
                },
                ToolEffect.READ,
                self._topology,
            )
        )
        self._register(
            ToolSpec(
                "hysim_v1_subgraph",
                "Read a bounded k-hop subgraph around one stable AC or DC bus reference.",
                {
                    "type": "object",
                    "properties": {
                        "domain": {"type": "string", "enum": ["ac", "dc"]},
                        "index": {"type": "integer"},
                        "depth": {"type": "integer", "minimum": 0, "maximum": 8,
                                  "default": 2},
                        "max_nodes": {"type": "integer", "minimum": 1,
                                      "maximum": 500, "default": 200},
                    },
                    "required": ["domain", "index"],
                    "additionalProperties": False,
                },
                ToolEffect.READ,
                self._subgraph,
            )
        )
        self._register(
            ToolSpec(
                "hysim_v1_list_jobs",
                "List compact retained-job metadata for this isolated session.",
                no_arguments,
                ToolEffect.READ,
                self._list_jobs,
            )
        )
        self._register(
            ToolSpec(
                "hysim_v1_replace_builtin",
                "Replace this session model with a built-in case using its current ETag.",
                {
                    "type": "object",
                    "properties": {"case": {"type": "string", "minLength": 1}},
                    "required": ["case"],
                    "additionalProperties": False,
                },
                ToolEffect.MODIFY_MODEL,
                lambda args: dict(
                    self.session.replace_model(case=str(args["case"])).raw
                ),
            )
        )
        self._register(
            ToolSpec(
                "hysim_v1_submit_power_flow",
                "Submit a power-flow job bound to the current model revision.",
                {
                    "type": "object",
                    "properties": {
                        "max_iter": {"type": "integer", "minimum": 1},
                        "tol": {"type": "number", "exclusiveMinimum": 0},
                    },
                    "additionalProperties": False,
                },
                ToolEffect.ANALYZE,
                self._submit_power_flow,
            )
        )
        self._register(
            ToolSpec(
                "hysim_v1_submit_optimal_power_flow",
                "Submit an OPF job bound to the current model revision.",
                {
                    "type": "object",
                    "properties": {
                        "solver": {
                            "type": "string",
                            "enum": ["parity", "ipopt", "auto", "dc", "dispatch"],
                            "default": "parity",
                        }
                    },
                    "additionalProperties": False,
                },
                ToolEffect.ANALYZE,
                self._submit_opf,
            )
        )
        job_schema = {
            "type": "object",
            "properties": {
                "job_id": {"type": "string", "minLength": 1},
                "include_result": {"type": "boolean", "default": False},
            },
            "required": ["job_id"],
            "additionalProperties": False,
        }
        self._register(
            ToolSpec(
                "hysim_v1_get_job",
                "Poll a job and return compact result metadata unless full output is requested.",
                job_schema,
                ToolEffect.READ,
                self._get_job,
            )
        )
        self._register(
            ToolSpec(
                "hysim_v1_cancel_job",
                "Request cancellation of a queued or running job.",
                {
                    "type": "object",
                    "properties": {"job_id": {"type": "string", "minLength": 1}},
                    "required": ["job_id"],
                    "additionalProperties": False,
                },
                ToolEffect.CONTROL,
                lambda args: dict(
                    self.session.client.get_job(str(args["job_id"])).cancel().raw
                ),
            )
        )
        self._register(
            ToolSpec(
                "hysim_v1_job_frame",
                "Read a bounded spatial/time result frame without downloading full vectors.",
                {
                    "type": "object",
                    "properties": {
                        "job_id": {"type": "string", "minLength": 1},
                        "step": {"type": "integer", "minimum": 0, "default": 0},
                        "domain": {"type": "string", "enum": ["all", "ac", "dc"],
                                   "default": "all"},
                        "offset": {"type": "integer", "minimum": 0, "default": 0},
                        "limit": {"type": "integer", "minimum": 1, "maximum": 500,
                                  "default": 200},
                        "indices": {"type": "array", "items": {"type": "integer"},
                                    "maxItems": 200},
                    },
                    "required": ["job_id"],
                    "additionalProperties": False,
                },
                ToolEffect.READ,
                self._job_frame,
            )
        )
        self._register(
            ToolSpec(
                "hysim_v1_job_violations",
                "Read the worst voltage and loading violations for one result step.",
                {
                    "type": "object",
                    "properties": {
                        "job_id": {"type": "string", "minLength": 1},
                        "step": {"type": "integer", "minimum": 0, "default": 0},
                        "limit": {"type": "integer", "minimum": 1, "maximum": 200,
                                  "default": 50},
                        "vmin": {"type": "number", "default": 0.9},
                        "vmax": {"type": "number", "default": 1.1},
                        "loading_limit_pct": {"type": "number", "minimum": 0,
                                              "default": 100.0},
                    },
                    "required": ["job_id"],
                    "additionalProperties": False,
                },
                ToolEffect.READ,
                self._job_violations,
            )
        )

    def _topology(self, args: Mapping[str, Any]) -> Mapping[str, Any]:
        return self.session.topology(
            lod=int(args.get("lod", 2)),
            offset=int(args.get("offset", 0)),
            limit=int(args.get("limit", 200)),
        )

    def _subgraph(self, args: Mapping[str, Any]) -> Mapping[str, Any]:
        return self.session.subgraph(
            str(args["domain"]),
            int(args["index"]),
            depth=int(args.get("depth", 2)),
            max_nodes=int(args.get("max_nodes", 200)),
        )

    def _list_jobs(self, _: Mapping[str, Any]) -> Mapping[str, Any]:
        jobs = self.session.list_jobs()
        compact = []
        for item in jobs[:100]:
            compact.append({key: item[key] for key in (
                "job_id", "analysis", "state", "model_revision",
                "created_at", "started_at", "finished_at",
                "stale_against_current_model",
            ) if key in item})
        return {"count": len(jobs), "returned": len(compact), "jobs": compact}

    def _job_frame(self, args: Mapping[str, Any]) -> Mapping[str, Any]:
        indices = args.get("indices")
        return self.session.client.get_job(str(args["job_id"])).frame(
            int(args.get("step", 0)),
            domain=str(args.get("domain", "all")),
            offset=int(args.get("offset", 0)),
            limit=int(args.get("limit", 200)),
            indices=tuple(indices) if isinstance(indices, list) else None,
        )

    def _job_violations(self, args: Mapping[str, Any]) -> Mapping[str, Any]:
        return self.session.client.get_job(str(args["job_id"])).violations(
            int(args.get("step", 0)),
            limit=int(args.get("limit", 50)),
            vmin=float(args.get("vmin", 0.9)),
            vmax=float(args.get("vmax", 1.1)),
            loading_limit_pct=float(args.get("loading_limit_pct", 100.0)),
        )

    def _submit_power_flow(self, args: Mapping[str, Any]) -> Mapping[str, Any]:
        request = PowerFlowRequest(
            options=PowerFlowOptions(
                max_iter=_optional_int(args.get("max_iter")),
                tol=_optional_float(args.get("tol")),
            )
        )
        return dict(self.session.power_flow(request).raw)

    def _submit_opf(self, args: Mapping[str, Any]) -> Mapping[str, Any]:
        request = OPFRequest(solver=str(args.get("solver", "parity")))
        return dict(self.session.optimal_power_flow(request).raw)

    def _get_job(self, args: Mapping[str, Any]) -> Mapping[str, Any]:
        job = self.session.client.get_job(str(args["job_id"]))
        if job.state != "succeeded" or bool(args.get("include_result", False)):
            return dict(job.raw)
        compact = dict(job.raw)
        compact.pop("result", None)
        compact["result_summary"] = job.result().to_record()
        return compact


def _optional_int(value: Any) -> int | None:
    return None if value is None else int(value)


def _optional_float(value: Any) -> float | None:
    return None if value is None else float(value)


def _validate_arguments(schema: Mapping[str, Any], arguments: Mapping[str, Any]) -> None:
    """Validate the small JSON-Schema subset used by built-in tools."""

    properties = schema.get("properties", {})
    if not isinstance(properties, Mapping):
        raise ValueError("tool schema properties must be an object")
    if schema.get("additionalProperties") is False:
        unknown = sorted(set(arguments) - set(properties))
        if unknown:
            raise ValueError(f"unknown tool arguments: {', '.join(unknown)}")
    for name in schema.get("required", []):
        if name not in arguments:
            raise ValueError(f"missing required tool argument: {name}")
    for name, value in arguments.items():
        field_schema = properties.get(name)
        if not isinstance(field_schema, Mapping):
            continue
        expected = field_schema.get("type")
        valid = {
            "string": isinstance(value, str),
            "boolean": isinstance(value, bool),
            "integer": isinstance(value, int) and not isinstance(value, bool),
            "number": isinstance(value, (int, float)) and not isinstance(value, bool),
            "object": isinstance(value, Mapping),
            "array": isinstance(value, list),
        }.get(expected, True)
        if not valid:
            raise ValueError(f"tool argument {name} must have type {expected}")
        choices = field_schema.get("enum")
        if isinstance(choices, list) and value not in choices:
            raise ValueError(f"tool argument {name} must be one of {choices}")
        if isinstance(value, str) and len(value) < int(field_schema.get("minLength", 0)):
            raise ValueError(f"tool argument {name} is too short")
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if "minimum" in field_schema and value < field_schema["minimum"]:
                raise ValueError(f"tool argument {name} is below its minimum")
            if "exclusiveMinimum" in field_schema and value <= field_schema["exclusiveMinimum"]:
                raise ValueError(f"tool argument {name} must exceed its minimum")
            if "maximum" in field_schema and value > field_schema["maximum"]:
                raise ValueError(f"tool argument {name} exceeds its maximum")
        if isinstance(value, list):
            if "maxItems" in field_schema and len(value) > field_schema["maxItems"]:
                raise ValueError(f"tool argument {name} has too many items")
            item_schema = field_schema.get("items")
            if isinstance(item_schema, Mapping):
                expected_item = item_schema.get("type")
                for item in value:
                    item_valid = {
                        "integer": isinstance(item, int) and not isinstance(item, bool),
                        "number": isinstance(item, (int, float)) and not isinstance(item, bool),
                        "string": isinstance(item, str),
                    }.get(expected_item, True)
                    if not item_valid:
                        raise ValueError(
                            f"tool argument {name} items must have type {expected_item}"
                        )
