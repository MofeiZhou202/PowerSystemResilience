"""Stable Python-side request and result contracts."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Mapping

from .errors import InvalidResultError


class BusDomain(str, Enum):
    AC = "ac"
    DC = "dc"
    THREE_PHASE_AC = "three_phase_ac"


@dataclass(frozen=True)
class BusRef:
    """Domain-qualified, stable authored-space bus reference."""

    domain: BusDomain
    stable_id: int

    def __post_init__(self) -> None:
        if isinstance(self.stable_id, bool) or not isinstance(self.stable_id, int):
            raise TypeError("stable_id must be an integer component index")

    def to_dict(self) -> dict[str, Any]:
        return {"domain": self.domain.value, "index": self.stable_id}

    @property
    def index(self) -> int:
        return self.stable_id

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "BusRef":
        domain = value.get("domain")
        index = value.get("index")
        try:
            typed_domain = BusDomain(str(domain))
        except ValueError as exc:
            raise ValueError(f"unsupported bus domain: {domain!r}") from exc
        if isinstance(index, bool) or not isinstance(index, int):
            raise TypeError("bus reference index must be an integer")
        return cls(typed_domain, index)


@dataclass(frozen=True)
class ComponentRef:
    """Reference to a rich-model component, never a vector position."""

    component_type: str
    stable_id: int
    domain: BusDomain | None = None

    def __post_init__(self) -> None:
        if not self.component_type:
            raise ValueError("component_type must not be empty")
        if isinstance(self.stable_id, bool) or not isinstance(self.stable_id, int):
            raise TypeError("stable_id must be an integer component index")

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "component_type": self.component_type,
            "index": self.stable_id,
        }
        if self.domain is not None:
            result["domain"] = self.domain.value
        return result


class PowerFlowMethod(str, Enum):
    AC_NEWTON = "ac_newton"
    PURE_AC = "pure_ac"
    ADAPTIVE = "adaptive"
    ISLANDED = "islanded"
    FDPF = "fdpf"
    THREE_PHASE = "three_phase"
    THREE_PHASE_HYBRID = "three_phase_hybrid"


@dataclass(frozen=True)
class PowerFlowOptions:
    max_iter: int | None = None
    tol: float | None = None
    fdpf_max_iter: int | None = None
    enable_pv_pq_conversion: bool | None = None
    enable_auto_swing_selection: bool | None = None
    enable_converter_mode_switching: bool | None = None
    enable_converter_coordination_check: bool | None = True
    enable_rigid_vdc_former: bool | None = None
    enable_coupled_jacobian: bool | None = None
    enable_augmented_equations: bool | None = None
    enable_semi_smooth_newton: bool | None = None
    enable_solver_profiling: bool | None = None
    enable_iteration_log: bool | None = None
    ac_eval_threads: int | None = None
    loss_model: str | None = None
    globalization: str | None = None
    verbose: bool | None = None
    three_phase: Mapping[str, Any] | None = None
    robust_nonlinear: Mapping[str, Any] | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        result = dict(self.extra)
        for key, value in self.__dict__.items():
            if key != "extra" and value is not None:
                result[key] = dict(value) if isinstance(value, Mapping) else value
        return result


@dataclass(frozen=True)
class PowerFlowRequest:
    method: PowerFlowMethod | str = PowerFlowMethod.AC_NEWTON
    options: PowerFlowOptions | Mapping[str, Any] = field(default_factory=PowerFlowOptions)

    def to_dict(self) -> dict[str, Any]:
        method = self.method.value if isinstance(self.method, PowerFlowMethod) else self.method
        options = (
            self.options.to_dict()
            if isinstance(self.options, PowerFlowOptions)
            else dict(self.options)
        )
        return {"method": method, "options": options}


class OPFSolver(str, Enum):
    PARITY = "parity"
    IPOPT = "ipopt"
    AUTO = "auto"
    DC = "dc"
    DISPATCH = "dispatch"


class OPFNetworkModel(str, Enum):
    BALANCED_AGGREGATE = "balanced_aggregate"
    BALANCED_WITH_THREE_PHASE_VALIDATION = "balanced_with_three_phase_validation"
    THREE_PHASE_HYBRID = "three_phase_hybrid"


@dataclass(frozen=True)
class OPFConstraints:
    branch_limits: bool = True
    converter_capacity: bool = True
    converter_current: bool = True
    converter_modulation: bool = True

    def to_dict(self) -> dict[str, bool]:
        return dict(self.__dict__)


@dataclass(frozen=True)
class OPFRequest:
    solver: OPFSolver | str = OPFSolver.PARITY
    network_model: OPFNetworkModel | str = OPFNetworkModel.BALANCED_AGGREGATE
    constraints: OPFConstraints | Mapping[str, Any] = field(default_factory=OPFConstraints)
    options: Mapping[str, Any] = field(default_factory=dict)
    three_phase: Mapping[str, Any] = field(default_factory=dict)
    check_consistency: bool = False

    def to_dict(self) -> dict[str, Any]:
        solver = self.solver.value if isinstance(self.solver, OPFSolver) else self.solver
        network_model = (
            self.network_model.value
            if isinstance(self.network_model, OPFNetworkModel)
            else self.network_model
        )
        constraints = (
            self.constraints.to_dict()
            if isinstance(self.constraints, OPFConstraints)
            else dict(self.constraints)
        )
        return {
            "solver": solver,
            "network_model": network_model,
            "constraints": constraints,
            "options": dict(self.options),
            "three_phase": dict(self.three_phase),
            "check_consistency": self.check_consistency,
        }


_SUMMARY_KEYS = (
    "schema",
    "method",
    "method_actual",
    "solver",
    "solver_backend",
    "network_model",
    "converged",
    "success",
    "feasible",
    "accepted",
    "iterations",
    "residual",
    "objective",
    "status",
    "termination_reason",
    "execution_time_sec",
    "fallback_used",
    "model_scope",
    "analysis_scope",
    "model_limitations",
    "validity_flags",
    "warnings",
)


@dataclass(frozen=True)
class AnalysisResult:
    """Raw solver result plus provenance and honest usability checks."""

    analysis: str
    route: str
    request_id: str
    model_revision: int
    data: Mapping[str, Any]
    received_at: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )

    @property
    def stale(self) -> bool:
        contract = self.data.get("_result_contract")
        return bool(isinstance(contract, Mapping) and contract.get("stale"))

    @property
    def scientific_status(self) -> str:
        if self.data.get("error"):
            return "failed"
        if self.stale:
            return "stale"
        qualification_seen = False
        for key in ("accepted", "converged", "success", "feasible"):
            value = self.data.get(key)
            if isinstance(value, bool):
                qualification_seen = True
                if not value:
                    return "invalid"
        return "qualified" if qualification_seen else "unqualified"

    @property
    def limitations(self) -> tuple[str, ...]:
        found: list[str] = []
        for container in (self.data, self.data.get("analysis_scope")):
            if not isinstance(container, Mapping):
                continue
            value = container.get("model_limitations")
            if isinstance(value, str):
                found.append(value)
            elif isinstance(value, list):
                found.extend(str(item) for item in value)
        return tuple(dict.fromkeys(found))

    def require_usable(self, *, allow_unqualified: bool = False) -> "AnalysisResult":
        status = self.scientific_status
        if status in {"failed", "stale", "invalid"} or (
            status == "unqualified" and not allow_unqualified
        ):
            raise InvalidResultError(
                f"{self.analysis} result is {status}; inspect raw data and limitations"
            )
        return self

    def summary(self) -> dict[str, Any]:
        summary = {key: self.data[key] for key in _SUMMARY_KEYS if key in self.data}
        sizes = {
            key: len(value)
            for key, value in self.data.items()
            if isinstance(value, list)
        }
        summary.update(
            {
                "analysis": self.analysis,
                "route": self.route,
                "request_id": self.request_id,
                "model_revision": self.model_revision,
                "received_at": self.received_at,
                "scientific_status": self.scientific_status,
                "limitations": list(self.limitations),
                "result_sizes": sizes,
            }
        )
        return summary

    def to_record(self, *, include_raw: bool = False) -> dict[str, Any]:
        record = self.summary()
        record["contract"] = "hysim_python_result_v1"
        if include_raw:
            record["raw"] = dict(self.data)
        return record
