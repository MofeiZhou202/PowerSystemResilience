"""Provider-neutral AI tool registry with explicit side-effect policy."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Mapping

from .client import HySimClient
from .errors import ToolPolicyError
from .models import OPFRequest, PowerFlowOptions, PowerFlowRequest


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
