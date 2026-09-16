"""Comprehensive analysis catalog and namespaced family facade.

This module layers a single authoritative analysis catalog and a typed,
namespaced facade over the process-global :class:`HySimClient`. It gives one
comprehensive Python surface for the whole ``hacdcpf`` platform without
duplicating any solver or projection logic in Python: every method resolves a
production route through the catalog and returns an honest
:class:`~hysim.models.AnalysisResult`.

See ``docs/reference/python_comprehensive_api_design.md`` for the design.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

from .client import HySimClient
from .models import AnalysisResult, OPFRequest, PowerFlowRequest
from .requests import (
    AcOpfRequest,
    AnnualProductionRequest,
    CampusIesRequest,
    CounterfactualPlanningRequest,
    DcShortCircuitRequest,
    DynamicCarbonRequest,
    EvTrafficRequest,
    FrequencyScanRequest,
    HarmonicsNewtonRequest,
    HarmonicsRequest,
    HarmonicsStateSpaceRequest,
    HostingCapacityRequest,
    LifecycleCompareRequest,
    LifecycleRequest,
    MarketPtdfRequest,
    RealTimeMarketRequest,
    ReconfigurationRequest,
    ReliabilityFdRequest,
    ReliabilityFmeaRequest,
    ReliabilityThreeStageRequest,
    RepeatedGameRequest,
    ResilienceRequest,
    ScenarioGenerationRequest,
    ShortCircuitRequest,
    SmallSignalRequest,
    SouthernMarketRequest,
    TimeSeriesConfig,
    TimeSeriesPowerFlowRequest,
    TyphoonFaultsRequest,
    WeakLinksRequest,
)
from .transport import Transport


class AnalysisEffect(str, Enum):
    """Risk class of an analysis, used for AI tool gating and audit."""

    READ = "read"
    ANALYZE = "analyze"
    MODIFY = "modify"


class AnalysisCategory(str, Enum):
    POWER_FLOW = "power_flow"
    OPTIMAL_POWER_FLOW = "optimal_power_flow"
    REACTIVE_POWER = "reactive_power"
    SHORT_CIRCUIT = "short_circuit"
    HARMONICS = "harmonics"
    DYNAMICS = "dynamics"
    TIME_SERIES = "time_series"
    CARBON = "carbon"
    RELIABILITY = "reliability"
    RESILIENCE = "resilience"
    MARKET = "market"
    INTEGRATED_ENERGY = "integrated_energy"
    EV_TRAFFIC = "ev_traffic"
    RECONFIGURATION = "reconfiguration"
    HOSTING_CAPACITY = "hosting_capacity"
    PLANNING = "planning"
    SCENARIO = "scenario"
    SPPT = "sppt"
    TOPOLOGY = "topology"
    MODEL_IO = "model_io"


@dataclass(frozen=True)
class AnalysisSpec:
    """One production analysis: stable name, route, method, effect, category.

    ``effect`` is the AI risk class (read/analyze/modify). ``mutates_model`` is a
    narrower fact: whether the call replaces or edits the authored system model
    and therefore advances the model revision. A session-config write (e.g.
    ``set_ts_config``) has ``effect == MODIFY`` but ``mutates_model == False``.
    """

    name: str
    route: str
    method: str
    effect: AnalysisEffect
    category: AnalysisCategory
    summary: str
    mutates_model: bool = False


def _spec(
    name: str,
    route: str,
    effect: AnalysisEffect,
    category: AnalysisCategory,
    summary: str,
    *,
    method: str = "POST",
    mutates_model: bool = False,
) -> AnalysisSpec:
    return AnalysisSpec(name, route, method, effect, category, summary, mutates_model)


_A = AnalysisEffect
_C = AnalysisCategory

_SPECS: tuple[AnalysisSpec, ...] = (
    # Power flow.
    _spec("power_flow", "/api/session/pf", _A.ANALYZE, _C.POWER_FLOW,
          "Balanced AC/DC power flow."),
    # Optimal power flow.
    _spec("optimal_power_flow", "/api/session/opf", _A.ANALYZE, _C.OPTIMAL_POWER_FLOW,
          "Unified AC/DC optimal power flow."),
    _spec("opf_ac", "/api/session/opf_ac", _A.ANALYZE, _C.OPTIMAL_POWER_FLOW,
          "Native AC optimal power flow."),
    _spec("opf_parity", "/api/session/opf_parity", _A.ANALYZE, _C.OPTIMAL_POWER_FLOW,
          "Parity IPM optimal power flow."),
    _spec("opf_dc", "/api/session/opf_dc", _A.ANALYZE, _C.OPTIMAL_POWER_FLOW,
          "DC optimal power flow."),
    # Reactive power optimization.
    _spec("reactive_power_optimization", "/api/session/run_rpo", _A.ANALYZE,
          _C.REACTIVE_POWER, "Reactive power optimization with discrete taps."),
    _spec("rpo_inputs", "/api/session/rpo_inputs", _A.READ, _C.REACTIVE_POWER,
          "Reactive power control inventory (read-only)."),
    # Short circuit.
    _spec("short_circuit", "/api/session/sc", _A.ANALYZE, _C.SHORT_CIRCUIT,
          "IEC 60909 simplified short circuit."),
    _spec("detailed_short_circuit", "/api/session/sc_detailed", _A.ANALYZE,
          _C.SHORT_CIRCUIT, "Detailed AC short circuit."),
    _spec("dc_short_circuit", "/api/session/dc_sc", _A.ANALYZE, _C.SHORT_CIRCUIT,
          "DC fault level."),
    # Harmonics.
    _spec("harmonics", "/api/session/harmonics", _A.ANALYZE, _C.HARMONICS,
          "Frequency-domain harmonic penetration."),
    _spec("three_phase_harmonics", "/api/session/harmonics_3ph", _A.ANALYZE,
          _C.HARMONICS, "Three-phase sequence-network harmonics."),
    _spec("harmonics_frequency_scan", "/api/session/harmonics_freqscan", _A.ANALYZE,
          _C.HARMONICS, "Harmonic frequency scan."),
    _spec("harmonics_hss", "/api/session/harmonics_hss", _A.ANALYZE, _C.HARMONICS,
          "Harmonic state-space penetration."),
    _spec("harmonics_newton", "/api/session/harmonics_newton", _A.ANALYZE,
          _C.HARMONICS, "Newton harmonic power flow."),
    _spec("harmonics_metrics", "/api/session/harmonics_metrics", _A.ANALYZE,
          _C.HARMONICS, "Harmonic distortion metrics vs standards."),
    # Dynamics.
    _spec("transient", "/api/session/run_transient", _A.ANALYZE, _C.DYNAMICS,
          "Electromechanical transient DAE simulation."),
    _spec("small_signal", "/api/session/small_signal", _A.ANALYZE, _C.DYNAMICS,
          "Small-signal stability analysis."),
    # Time series.
    _spec("unit_commitment", "/api/session/run_uc", _A.ANALYZE, _C.TIME_SERIES,
          "Unit commitment over a time-series horizon."),
    _spec("set_ts_config", "/api/session/set_ts_config", _A.MODIFY, _C.TIME_SERIES,
          "Author the session time-series profiles (session config, not model)."),
    _spec("time_series_power_flow", "/api/session/run_ts_pf", _A.ANALYZE,
          _C.TIME_SERIES, "Time-series power flow sweep."),
    _spec("annual_production", "/api/session/run_annual_sim", _A.ANALYZE,
          _C.TIME_SERIES, "Annual layered production simulation."),
    _spec("lifecycle_simulation", "/api/session/run_lifecycle_sim", _A.ANALYZE,
          _C.TIME_SERIES, "Lifecycle simulation."),
    _spec("lifecycle_compare", "/api/session/run_lifecycle_compare", _A.ANALYZE,
          _C.TIME_SERIES, "Lifecycle scenario comparison."),
    # Carbon.
    _spec("carbon_flow", "/api/session/run_carbon", _A.ANALYZE, _C.CARBON,
          "Carbon flow tracing."),
    _spec("dynamic_carbon_flow", "/api/session/run_dynamic_carbon", _A.ANALYZE,
          _C.CARBON, "Time-varying carbon flow tracing."),
    # Reliability.
    _spec("reliability_nonsequential", "/api/session/run_reliability_nsq", _A.ANALYZE,
          _C.RELIABILITY, "Non-sequential Monte Carlo reliability."),
    _spec("reliability_sequential", "/api/session/run_reliability_seq", _A.ANALYZE,
          _C.RELIABILITY, "Sequential Monte Carlo reliability."),
    _spec("reliability_fmea", "/api/session/run_reliability_fmea", _A.ANALYZE,
          _C.RELIABILITY, "FMEA reliability."),
    _spec("reliability_fd", "/api/session/run_reliability_fd", _A.ANALYZE,
          _C.RELIABILITY, "Failure and diagnosis reliability."),
    _spec("reliability_three_stage", "/api/session/run_reliability_three_stage",
          _A.ANALYZE, _C.RELIABILITY, "Three-stage MILP reliability."),
    _spec("reliability", "/api/session/run_reliability", _A.ANALYZE, _C.RELIABILITY,
          "Default reliability workflow."),
    # Resilience.
    _spec("distribution_resilience", "/api/session/run_distribution_resilience",
          _A.ANALYZE, _C.RESILIENCE, "Distribution resilience restoration."),
    # Market.
    _spec("market_clearing", "/api/session/run_market_clearing", _A.ANALYZE, _C.MARKET,
          "Day-ahead SCUC/SCED/LMP market clearing."),
    _spec("real_time_market", "/api/session/run_real_time_market", _A.ANALYZE, _C.MARKET,
          "Real-time double-settlement market."),
    _spec("repeated_market_game", "/api/session/run_repeated_market_game", _A.ANALYZE,
          _C.MARKET, "Repeated market game."),
    _spec("southern_market", "/api/session/run_southern_market", _A.ANALYZE, _C.MARKET,
          "Southern China regional market clearing."),
    _spec("market_ptdf", "/api/session/market_ptdf", _A.READ, _C.MARKET,
          "Market PTDF query (read-only)."),
    # Integrated energy.
    _spec("campus_ies", "/api/session/run_campus_ies", _A.ANALYZE, _C.INTEGRATED_ENERGY,
          "Campus electricity-heat-hydrogen multi-energy flow."),
    # EV-traffic.
    _spec("ev_power_traffic", "/api/session/run_ev_traffic", _A.ANALYZE, _C.EV_TRAFFIC,
          "EV power-traffic coupled analysis."),
    # Reconfiguration.
    _spec("reconfiguration", "/api/session/run_reconfig", _A.ANALYZE, _C.RECONFIGURATION,
          "Network reconfiguration MILP."),
    # Hosting capacity.
    _spec("hosting_capacity", "/api/session/run_hosting_capacity", _A.ANALYZE,
          _C.HOSTING_CAPACITY, "Hosting capacity assessment."),
    # Planning.
    _spec("counterfactual_planning", "/api/session/run_counterfactual_planning",
          _A.ANALYZE, _C.PLANNING, "Counterfactual planning."),
    _spec("multidimensional_weak_links", "/api/session/run_multidimensional_weak_links",
          _A.ANALYZE, _C.PLANNING, "Multi-dimensional weak-link analysis."),
    # Scenario generation.
    _spec("scenario_generation", "/api/session/generate_scenarios", _A.ANALYZE,
          _C.SCENARIO, "Scenario generation."),
    _spec("typhoon_faults", "/api/session/generate_typhoon_faults", _A.ANALYZE,
          _C.SCENARIO, "Typhoon fault scenario generation."),
    # SPPT.
    _spec("sppt_guard", "/api/session/sppt_guard", _A.ANALYZE, _C.SPPT,
          "Semantic-preserving projection guard."),
    # Topology.
    _spec("topology", "/api/session/topology", _A.READ, _C.TOPOLOGY,
          "Topology projection (read-only)."),
    _spec("network_reduction", "/api/session/network_reduction", _A.ANALYZE,
          _C.TOPOLOGY, "Network reduction."),
    # Model I/O.
    _spec("load_builtin", "/api/session/load_builtin", _A.MODIFY, _C.MODEL_IO,
          "Load a built-in case (mutates the session model).", mutates_model=True),
    _spec("load_matpower", "/api/session/load_matpower", _A.MODIFY, _C.MODEL_IO,
          "Load a MATPOWER case (mutates the session model).", mutates_model=True),
    _spec("load_json_string", "/api/session/load_json_string", _A.MODIFY, _C.MODEL_IO,
          "Load a JSON model string (mutates the session model).", mutates_model=True),
    _spec("new_empty", "/api/session/new_empty", _A.MODIFY, _C.MODEL_IO,
          "Create an empty system (mutates the session model).", mutates_model=True),
    _spec("update_components", "/api/session/update_components", _A.MODIFY, _C.MODEL_IO,
          "Edit component parameters (mutates the session model).", mutates_model=True),
    _spec("load_bpa_dat", "/api/session/load_bpa_dat", _A.MODIFY, _C.MODEL_IO,
          "Load a BPA/DSP DAT deck (mutates the session model).", mutates_model=True),
    _spec("load_cim_dist", "/api/session/load_cim_dist", _A.MODIFY, _C.MODEL_IO,
          "Load a CIM/CGMES distribution model (mutates the session model).",
          mutates_model=True),
    _spec("load_gridlabd", "/api/session/load_gridlabd", _A.MODIFY, _C.MODEL_IO,
          "Load a GridLAB-D GLM model (mutates the session model).", mutates_model=True),
    _spec("load_opendss", "/api/session/load_opendss", _A.MODIFY, _C.MODEL_IO,
          "Load an OpenDSS model (mutates the session model).", mutates_model=True),
    _spec("load_etap_xml", "/api/session/load_etap_xml", _A.MODIFY, _C.MODEL_IO,
          "Load an ETAP XML model (mutates the session model).", mutates_model=True),
    _spec("load_svg_distribution", "/api/session/load_svg_distribution", _A.MODIFY,
          _C.MODEL_IO, "Import an SVG distribution diagram (mutates the model).",
          mutates_model=True),
    _spec("export_json", "/api/session/export_json", _A.READ, _C.MODEL_IO,
          "Export the session model as JSON (read-only)."),
    _spec("export_matpower", "/api/session/export_matpower", _A.READ, _C.MODEL_IO,
          "Export the session model as MATPOWER (read-only)."),
    _spec("export_bpa_dat", "/api/session/export_bpa_dat", _A.READ, _C.MODEL_IO,
          "Export the session model as a BPA/DSP DAT deck (read-only)."),
    _spec("export_etap", "/api/session/export_etap", _A.READ, _C.MODEL_IO,
          "Export the session model as ETAP (read-only)."),
    _spec("export_etap_xml", "/api/session/export_etap_xml", _A.READ, _C.MODEL_IO,
          "Export the session model as ETAP XML (read-only)."),
    _spec("export_cim_dist", "/api/session/export_cim_dist", _A.READ, _C.MODEL_IO,
          "Export the session model as CIM/CGMES distribution (read-only)."),
    _spec("export_gridlabd", "/api/session/export_gridlabd", _A.READ, _C.MODEL_IO,
          "Export the session model as GridLAB-D GLM (read-only)."),
    _spec("export_opendss", "/api/session/export_opendss", _A.READ, _C.MODEL_IO,
          "Export the session model as OpenDSS (read-only)."),
    _spec("export_svg_distribution", "/api/session/export_svg_distribution", _A.READ,
          _C.MODEL_IO, "Export the session model as an SVG diagram (read-only)."),
)


class AnalysisCatalog:
    """Immutable registry of every production analysis, keyed by stable name."""

    def __init__(self, specs: tuple[AnalysisSpec, ...]) -> None:
        by_name: dict[str, AnalysisSpec] = {}
        for spec in specs:
            if spec.name in by_name:
                raise ValueError(f"duplicate analysis name: {spec.name}")
            by_name[spec.name] = spec
        self._by_name = by_name

    def __contains__(self, name: object) -> bool:
        return name in self._by_name

    def __iter__(self):
        return iter(self._by_name.values())

    def __len__(self) -> int:
        return len(self._by_name)

    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._by_name))

    def get(self, name: str) -> AnalysisSpec | None:
        return self._by_name.get(name)

    def require(self, name: str) -> AnalysisSpec:
        spec = self._by_name.get(name)
        if spec is None:
            choices = ", ".join(self.names())
            raise ValueError(f"unknown analysis {name!r}; choose one of: {choices}")
        return spec

    def by_category(self, category: AnalysisCategory) -> tuple[AnalysisSpec, ...]:
        return tuple(s for s in self._by_name.values() if s.category is category)

    def by_effect(self, effect: AnalysisEffect) -> tuple[AnalysisSpec, ...]:
        return tuple(s for s in self._by_name.values() if s.effect is effect)


ANALYSIS_CATALOG = AnalysisCatalog(_SPECS)


# --- Typed request models (first slice) -----------------------------------
# Fields are transcribed from the current handlers in run_gui_server.cpp. A
# ``None`` field is omitted so the C++ default in the handler stays
# authoritative; ``extra`` carries fields not yet promoted to named parameters.


def _compact(named: Mapping[str, Any], extra: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(extra)
    for key, value in named.items():
        if value is not None:
            result[key] = value
    return result


@dataclass(frozen=True)
class UnitCommitmentRequest:
    """Request for ``/api/session/run_uc`` (C++ ``solve_unit_commitment``)."""

    num_steps: int | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact({"num_steps": self.num_steps}, self.extra)


@dataclass(frozen=True)
class MarketClearingRequest:
    """Request for ``/api/session/run_market_clearing``."""

    num_steps: int | None = None
    offer_segments: int | None = None
    reserve_fraction: float | None = None
    voll_per_mwh: float | None = None
    exogenous_curtailment_penalty_per_mwh: float | None = None
    scuc_mip_relative_gap: float | None = None
    scuc_time_limit_sec: float | None = None
    scuc_max_nodes: int | None = None
    network_constraints: bool | None = None
    run_ac_validation: bool | None = None
    optimize_dc_storage: bool | None = None
    enforce_terminal_dc_storage_soc: bool | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "num_steps": self.num_steps,
                "offer_segments": self.offer_segments,
                "reserve_fraction": self.reserve_fraction,
                "voll_per_mwh": self.voll_per_mwh,
                "exogenous_curtailment_penalty_per_mwh": (
                    self.exogenous_curtailment_penalty_per_mwh
                ),
                "scuc_mip_relative_gap": self.scuc_mip_relative_gap,
                "scuc_time_limit_sec": self.scuc_time_limit_sec,
                "scuc_max_nodes": self.scuc_max_nodes,
                "network_constraints": self.network_constraints,
                "run_ac_validation": self.run_ac_validation,
                "optimize_dc_storage": self.optimize_dc_storage,
                "enforce_terminal_dc_storage_soc": self.enforce_terminal_dc_storage_soc,
            },
            self.extra,
        )


@dataclass(frozen=True)
class ReliabilityRequest:
    """Request for ``/api/session/run_reliability_nsq`` and ``_seq``."""

    max_iterations: int | None = None
    cov_threshold: float | None = None
    load_scale_factor: float | None = None
    seed: int | None = None
    compute_tail_risk: bool | None = None
    var_confidence: float | None = None
    use_importance_sampling: bool | None = None
    importance_lambda: float | None = None
    parallel: bool | None = None
    parallel_threads: int | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "max_iterations": self.max_iterations,
                "cov_threshold": self.cov_threshold,
                "load_scale_factor": self.load_scale_factor,
                "seed": self.seed,
                "compute_tail_risk": self.compute_tail_risk,
                "var_confidence": self.var_confidence,
                "use_importance_sampling": self.use_importance_sampling,
                "importance_lambda": self.importance_lambda,
                "parallel": self.parallel,
                "parallel_threads": self.parallel_threads,
            },
            self.extra,
        )


@dataclass(frozen=True)
class TransientRequest:
    """Request for ``/api/session/run_transient`` (C++ dynamics DAE)."""

    solver_type: str | None = None
    linear_solver: str | None = None
    t_start_s: float | None = None
    t_end_s: float | None = None
    dt_s: float | None = None
    abs_tol: float | None = None
    rel_tol: float | None = None
    use_adaptive_step: bool | None = None
    snapshot_budget: int | None = None
    run_power_flow_initialization: bool | None = None
    enable_der_protection: bool | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "solver_type": self.solver_type,
                "linear_solver": self.linear_solver,
                "t_start_s": self.t_start_s,
                "t_end_s": self.t_end_s,
                "dt_s": self.dt_s,
                "abs_tol": self.abs_tol,
                "rel_tol": self.rel_tol,
                "use_adaptive_step": self.use_adaptive_step,
                "snapshot_budget": self.snapshot_budget,
                "run_power_flow_initialization": self.run_power_flow_initialization,
                "enable_der_protection": self.enable_der_protection,
            },
            self.extra,
        )


@dataclass(frozen=True)
class RpoRequest:
    """Request for ``/api/session/run_rpo`` (reactive power optimization)."""

    objective: str | None = None  # "voltage" | "loss" | "combined"
    vdev_weight: float | None = None
    loss_weight: float | None = None
    v_target: float | None = None
    mip_gap: float | None = None
    time_limit_s: float | None = None
    max_evaluations: int | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "objective": self.objective,
                "vdev_weight": self.vdev_weight,
                "loss_weight": self.loss_weight,
                "v_target": self.v_target,
                "mip_gap": self.mip_gap,
                "time_limit_s": self.time_limit_s,
                "max_evaluations": self.max_evaluations,
            },
            self.extra,
        )


def _as_payload(request: Any, cls: type) -> dict[str, Any]:
    if request is None:
        return cls().to_dict()
    if isinstance(request, cls):
        return request.to_dict()
    return dict(request)


def _io_payload(request: Any) -> dict[str, Any] | None:
    if request is None:
        return None
    if hasattr(request, "to_dict"):
        return request.to_dict()
    return dict(request)


_LOAD_ROUTES: dict[str, str] = {
    "builtin": "load_builtin",
    "matpower": "load_matpower",
    "json_string": "load_json_string",
    "bpa_dat": "load_bpa_dat",
    "cim_dist": "load_cim_dist",
    "gridlabd": "load_gridlabd",
    "opendss": "load_opendss",
    "etap_xml": "load_etap_xml",
    "svg_distribution": "load_svg_distribution",
}

_EXPORT_ROUTES: dict[str, str] = {
    "json": "export_json",
    "matpower": "export_matpower",
    "bpa_dat": "export_bpa_dat",
    "etap": "export_etap",
    "etap_xml": "export_etap_xml",
    "cim_dist": "export_cim_dist",
    "gridlabd": "export_gridlabd",
    "opendss": "export_opendss",
    "svg_distribution": "export_svg_distribution",
}


def _load_name(fmt: str) -> str:
    name = _LOAD_ROUTES.get(fmt)
    if name is None:
        raise ValueError(
            f"unknown load format {fmt!r}; choose one of: {', '.join(sorted(_LOAD_ROUTES))}"
        )
    return name


def _export_name(fmt: str) -> str:
    name = _EXPORT_ROUTES.get(fmt)
    if name is None:
        raise ValueError(
            f"unknown export format {fmt!r}; choose one of: {', '.join(sorted(_EXPORT_ROUTES))}"
        )
    return name


# --- Family facades --------------------------------------------------------


class _FamilyApi:
    """Base for namespaced analysis families over one :class:`HySimClient`."""

    def __init__(self, client: HySimClient, catalog: AnalysisCatalog) -> None:
        self._client = client
        self._catalog = catalog

    def _run(
        self,
        name: str,
        payload: Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        spec = self._catalog.require(name)
        return self._client.execute(
            name=spec.name,
            route=spec.route,
            method=spec.method,
            modifies_model=spec.mutates_model,
            payload=dict(payload or {}),
            timeout=timeout,
        )


class PowerFlowApi(_FamilyApi):
    def run(
        self,
        request: PowerFlowRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._client.power_flow(request, timeout=timeout)


class OptimalPowerFlowApi(_FamilyApi):
    def run(
        self,
        request: OPFRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._client.optimal_power_flow(request, timeout=timeout)

    def ac(
        self,
        request: AcOpfRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run("opf_ac", _as_payload(request, AcOpfRequest), timeout=timeout)

    def parity(
        self, request: Mapping[str, Any] | None = None, *, timeout: float | None = None
    ) -> AnalysisResult:
        return self._run("opf_parity", request, timeout=timeout)

    def dc(
        self, request: Mapping[str, Any] | None = None, *, timeout: float | None = None
    ) -> AnalysisResult:
        return self._run("opf_dc", request, timeout=timeout)


class TimeSeriesApi(_FamilyApi):
    def configure(
        self,
        request: TimeSeriesConfig | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run("set_ts_config", _as_payload(request, TimeSeriesConfig), timeout=timeout)

    def unit_commitment(
        self,
        request: UnitCommitmentRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "unit_commitment", _as_payload(request, UnitCommitmentRequest), timeout=timeout
        )

    def power_flow(
        self,
        request: TimeSeriesPowerFlowRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "time_series_power_flow",
            _as_payload(request, TimeSeriesPowerFlowRequest),
            timeout=timeout,
        )

    def annual_production(
        self,
        request: AnnualProductionRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "annual_production", _as_payload(request, AnnualProductionRequest), timeout=timeout
        )

    def lifecycle(
        self,
        request: LifecycleRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "lifecycle_simulation", _as_payload(request, LifecycleRequest), timeout=timeout
        )

    def lifecycle_compare(
        self,
        request: LifecycleCompareRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "lifecycle_compare",
            _as_payload(request, LifecycleCompareRequest),
            timeout=timeout,
        )


class CarbonApi(_FamilyApi):
    def flow(
        self, request: Mapping[str, Any] | None = None, *, timeout: float | None = None
    ) -> AnalysisResult:
        return self._run("carbon_flow", request, timeout=timeout)

    def dynamic(
        self,
        request: DynamicCarbonRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "dynamic_carbon_flow", _as_payload(request, DynamicCarbonRequest), timeout=timeout
        )


class MarketApi(_FamilyApi):
    def clearing(
        self,
        request: MarketClearingRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "market_clearing", _as_payload(request, MarketClearingRequest), timeout=timeout
        )

    def real_time(
        self,
        request: RealTimeMarketRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "real_time_market", _as_payload(request, RealTimeMarketRequest), timeout=timeout
        )

    def repeated_game(
        self,
        request: RepeatedGameRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "repeated_market_game", _as_payload(request, RepeatedGameRequest), timeout=timeout
        )

    def southern(
        self,
        request: SouthernMarketRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "southern_market", _as_payload(request, SouthernMarketRequest), timeout=timeout
        )

    def ptdf(
        self,
        request: MarketPtdfRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run("market_ptdf", _as_payload(request, MarketPtdfRequest), timeout=timeout)


class ReliabilityApi(_FamilyApi):
    def nonsequential(
        self,
        request: ReliabilityRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "reliability_nonsequential",
            _as_payload(request, ReliabilityRequest),
            timeout=timeout,
        )

    def sequential(
        self,
        request: ReliabilityRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "reliability_sequential",
            _as_payload(request, ReliabilityRequest),
            timeout=timeout,
        )

    def fmea(
        self,
        request: ReliabilityFmeaRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "reliability_fmea", _as_payload(request, ReliabilityFmeaRequest), timeout=timeout
        )

    def failure_diagnosis(
        self,
        request: ReliabilityFdRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "reliability_fd", _as_payload(request, ReliabilityFdRequest), timeout=timeout
        )

    def three_stage(
        self,
        request: ReliabilityThreeStageRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "reliability_three_stage",
            _as_payload(request, ReliabilityThreeStageRequest),
            timeout=timeout,
        )


class ResilienceApi(_FamilyApi):
    def distribution(
        self,
        request: ResilienceRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "distribution_resilience", _as_payload(request, ResilienceRequest), timeout=timeout
        )


class DynamicsApi(_FamilyApi):
    def transient(
        self,
        request: TransientRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "transient", _as_payload(request, TransientRequest), timeout=timeout
        )

    def small_signal(
        self,
        request: SmallSignalRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "small_signal", _as_payload(request, SmallSignalRequest), timeout=timeout
        )


class ShortCircuitApi(_FamilyApi):
    def run(
        self,
        request: ShortCircuitRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "short_circuit", _as_payload(request, ShortCircuitRequest), timeout=timeout
        )

    def detailed(
        self,
        request: ShortCircuitRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "detailed_short_circuit", _as_payload(request, ShortCircuitRequest), timeout=timeout
        )

    def dc(
        self,
        request: DcShortCircuitRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "dc_short_circuit", _as_payload(request, DcShortCircuitRequest), timeout=timeout
        )


class HarmonicsApi(_FamilyApi):
    def run(
        self,
        request: HarmonicsRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run("harmonics", _as_payload(request, HarmonicsRequest), timeout=timeout)

    def three_phase(
        self,
        request: HarmonicsRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "three_phase_harmonics", _as_payload(request, HarmonicsRequest), timeout=timeout
        )

    def frequency_scan(
        self,
        request: FrequencyScanRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "harmonics_frequency_scan",
            _as_payload(request, FrequencyScanRequest),
            timeout=timeout,
        )

    def newton(
        self,
        request: HarmonicsNewtonRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "harmonics_newton", _as_payload(request, HarmonicsNewtonRequest), timeout=timeout
        )

    def state_space(
        self,
        request: HarmonicsStateSpaceRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "harmonics_hss", _as_payload(request, HarmonicsStateSpaceRequest), timeout=timeout
        )

    def metrics(
        self,
        request: HarmonicsRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "harmonics_metrics", _as_payload(request, HarmonicsRequest), timeout=timeout
        )


class ReconfigurationApi(_FamilyApi):
    def run(
        self,
        request: ReconfigurationRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "reconfiguration", _as_payload(request, ReconfigurationRequest), timeout=timeout
        )


class HostingCapacityApi(_FamilyApi):
    def run(
        self,
        request: HostingCapacityRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "hosting_capacity", _as_payload(request, HostingCapacityRequest), timeout=timeout
        )


class IntegratedEnergyApi(_FamilyApi):
    def campus(
        self,
        request: CampusIesRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run("campus_ies", _as_payload(request, CampusIesRequest), timeout=timeout)


class EvTrafficApi(_FamilyApi):
    def run(
        self,
        request: EvTrafficRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "ev_power_traffic", _as_payload(request, EvTrafficRequest), timeout=timeout
        )


class PlanningApi(_FamilyApi):
    def counterfactual(
        self,
        request: CounterfactualPlanningRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "counterfactual_planning",
            _as_payload(request, CounterfactualPlanningRequest),
            timeout=timeout,
        )

    def weak_links(
        self,
        request: WeakLinksRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "multidimensional_weak_links",
            _as_payload(request, WeakLinksRequest),
            timeout=timeout,
        )


class ScenarioApi(_FamilyApi):
    def generate(
        self,
        request: ScenarioGenerationRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "scenario_generation",
            _as_payload(request, ScenarioGenerationRequest),
            timeout=timeout,
        )

    def typhoon_faults(
        self,
        request: TyphoonFaultsRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "typhoon_faults", _as_payload(request, TyphoonFaultsRequest), timeout=timeout
        )


class ModelIoApi(_FamilyApi):
    """Model import/export. Loads mutate the session model; exports are read-only.

    ``payload``/``request`` accept either a typed I/O request (e.g.
    ``CimLoadRequest``, ``SvgImportRequest``) or a raw mapping.
    """

    def export(
        self,
        fmt: str = "json",
        request: Any = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(_export_name(fmt), _io_payload(request), timeout=timeout)

    def load(
        self,
        fmt: str,
        payload: Any = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(_load_name(fmt), _io_payload(payload), timeout=timeout)


class ReactivePowerApi(_FamilyApi):
    def optimize(
        self,
        request: RpoRequest | Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        return self._run(
            "reactive_power_optimization", _as_payload(request, RpoRequest), timeout=timeout
        )

    def inventory(
        self, request: Mapping[str, Any] | None = None, *, timeout: float | None = None
    ) -> AnalysisResult:
        return self._run("rpo_inputs", request, timeout=timeout)


class HySim:
    """Comprehensive process-global facade over the ``hacdcpf`` platform.

    Composes a :class:`HySimClient` for transport, model lifecycle, and audit,
    and attaches one typed family per analysis category. Isolated-session
    experiments use :meth:`isolated`, which returns a v1 client bound to the
    same transport.
    """

    def __init__(
        self,
        base_url: str = "http://127.0.0.1:8088",
        *,
        transport: Transport | None = None,
        timeout: float = 600.0,
        audit_hook: Any = None,
        serialize_model_access: bool = True,
    ) -> None:
        self.client = HySimClient(
            base_url,
            transport=transport,
            timeout=timeout,
            audit_hook=audit_hook,
            serialize_model_access=serialize_model_access,
        )
        self.catalog = ANALYSIS_CATALOG
        self.pf = PowerFlowApi(self.client, self.catalog)
        self.opf = OptimalPowerFlowApi(self.client, self.catalog)
        self.reactive_power = ReactivePowerApi(self.client, self.catalog)
        self.short_circuit = ShortCircuitApi(self.client, self.catalog)
        self.harmonics = HarmonicsApi(self.client, self.catalog)
        self.dynamics = DynamicsApi(self.client, self.catalog)
        self.time_series = TimeSeriesApi(self.client, self.catalog)
        self.carbon = CarbonApi(self.client, self.catalog)
        self.market = MarketApi(self.client, self.catalog)
        self.reliability = ReliabilityApi(self.client, self.catalog)
        self.resilience = ResilienceApi(self.client, self.catalog)
        self.reconfiguration = ReconfigurationApi(self.client, self.catalog)
        self.hosting_capacity = HostingCapacityApi(self.client, self.catalog)
        self.integrated_energy = IntegratedEnergyApi(self.client, self.catalog)
        self.ev_traffic = EvTrafficApi(self.client, self.catalog)
        self.planning = PlanningApi(self.client, self.catalog)
        self.scenario = ScenarioApi(self.client, self.catalog)
        self.model_io = ModelIoApi(self.client, self.catalog)

    # Model lifecycle passthroughs (mutations increment the model revision).
    def load_builtin(self, case: str) -> Mapping[str, Any]:
        return self.client.load_builtin(case)

    def load_matpower(self, filename: str) -> Mapping[str, Any]:
        return self.client.load_matpower(filename)

    def load_model(self, model: Mapping[str, Any] | str) -> Mapping[str, Any]:
        return self.client.load_model(model)

    def new_system(self) -> Mapping[str, Any]:
        return self.client.new_system()

    def update_components(self, components: Mapping[str, Any]) -> Mapping[str, Any]:
        return self.client.update_components(components)

    def export_model(self) -> Mapping[str, Any]:
        return self.client.export_model()

    def list_cases(self) -> Mapping[str, Any]:
        return self.client.list_cases()

    def status(self) -> Mapping[str, Any]:
        return self.client.status()

    @property
    def model_revision(self) -> int:
        return self.client.model_revision

    def run(
        self,
        analysis: str,
        payload: Mapping[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> AnalysisResult:
        """Run any catalog analysis by name and return an honest result."""

        spec = self.catalog.require(analysis)
        return self.client.execute(
            name=spec.name,
            route=spec.route,
            method=spec.method,
            modifies_model=spec.mutates_model,
            payload=dict(payload or {}),
            timeout=timeout,
        )

    def isolated(self) -> Any:
        """Return an isolated-session v1 client bound to the same transport."""

        from .v1 import HySimV1Client

        return HySimV1Client(
            transport=self.client.transport,
            timeout=self.client.timeout,
            audit_hook=self.client.audit_hook,
        )
