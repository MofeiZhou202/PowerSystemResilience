"""Typed request models for the comprehensive analysis families (phases 2-3).

Field names are transcribed verbatim from the current handlers in
``tests/run_gui_server.cpp``. Each model types the high-signal fields as named
parameters and carries the long tail through ``extra``; a ``None`` field is
omitted so the C++ default in the handler stays authoritative. Nested option
objects (``options``, ``sources``, ``measures``, ...) are passed through as
mappings/lists when provided.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence


def _compact(named: Mapping[str, Any], extra: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(extra)
    for key, value in named.items():
        if value is not None:
            result[key] = value
    return result


# --- Optimal power flow ----------------------------------------------------


@dataclass(frozen=True)
class AcOpfRequest:
    """Request for ``/api/session/opf_ac``."""

    solver: str | None = None  # auto | parity | ipopt | dispatch | robust
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact({"solver": self.solver}, self.extra)


# --- Short circuit ---------------------------------------------------------


@dataclass(frozen=True)
class ShortCircuitRequest:
    """Request for ``/api/session/sc`` and ``/api/session/sc_detailed``.

    ``fault_bus_ids`` is only consumed by the detailed route; leave it ``None``
    for the aggregate ``sc`` route.
    """

    fault_type: str | None = None  # ThreePhase | ...
    calc_type: str | None = None  # Max | Min
    kappa_method: str | None = None
    topology: str | None = None
    c_factor: float | None = None
    fault_impedance_pu: float | None = None
    compute_branch_flows: bool | None = None
    compute_voltage_drops: bool | None = None
    compute_ith: bool | None = None
    fault_bus_ids: Sequence[int] | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "fault_type": self.fault_type,
                "calc_type": self.calc_type,
                "kappa_method": self.kappa_method,
                "topology": self.topology,
                "c_factor": self.c_factor,
                "fault_impedance_pu": self.fault_impedance_pu,
                "compute_branch_flows": self.compute_branch_flows,
                "compute_voltage_drops": self.compute_voltage_drops,
                "compute_ith": self.compute_ith,
                "fault_bus_ids": (
                    list(self.fault_bus_ids) if self.fault_bus_ids is not None else None
                ),
            },
            self.extra,
        )


@dataclass(frozen=True)
class DcShortCircuitRequest:
    """Request for ``/api/session/dc_sc``."""

    fault_bus_ids: Sequence[int] | None = None
    fault_resistance_pu: float | None = None
    source_voltage_pu: float | None = None
    consider_dc_breakers: bool | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "fault_bus_ids": (
                    list(self.fault_bus_ids) if self.fault_bus_ids is not None else None
                ),
                "fault_resistance_pu": self.fault_resistance_pu,
                "source_voltage_pu": self.source_voltage_pu,
                "consider_dc_breakers": self.consider_dc_breakers,
            },
            self.extra,
        )


# --- Harmonics -------------------------------------------------------------


@dataclass(frozen=True)
class HarmonicsRequest:
    """Request for ``/api/session/harmonics`` and ``/api/session/harmonics_3ph``.

    ``options`` carries the harmonic option block (orders, standard, ...);
    ``sources`` and ``nics`` carry the injection and converter arrays.
    """

    options: Mapping[str, Any] | None = None
    sources: Sequence[Mapping[str, Any]] | None = None
    nics: Sequence[Mapping[str, Any]] | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "options": self.options,
                "sources": list(self.sources) if self.sources is not None else None,
                "nics": list(self.nics) if self.nics is not None else None,
            },
            self.extra,
        )


@dataclass(frozen=True)
class FrequencyScanRequest:
    """Request for ``/api/session/harmonics_freqscan``."""

    f_start: float | None = None
    f_end: float | None = None
    f_step: float | None = None
    sequence: bool | None = None
    buses: Sequence[int] | None = None
    options: Mapping[str, Any] | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "f_start": self.f_start,
                "f_end": self.f_end,
                "f_step": self.f_step,
                "sequence": self.sequence,
                "buses": list(self.buses) if self.buses is not None else None,
                "options": self.options,
            },
            self.extra,
        )


@dataclass(frozen=True)
class HarmonicsNewtonRequest:
    """Request for ``/api/session/harmonics_newton``."""

    max_iter: int | None = None
    tol: float | None = None
    mode: str | None = None  # constant_power | holomorphic
    options: Mapping[str, Any] | None = None
    resources: Sequence[Mapping[str, Any]] | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "max_iter": self.max_iter,
                "tol": self.tol,
                "mode": self.mode,
                "options": self.options,
                "resources": (
                    list(self.resources) if self.resources is not None else None
                ),
            },
            self.extra,
        )


# --- Dynamics --------------------------------------------------------------


@dataclass(frozen=True)
class SmallSignalRequest:
    """Request for ``/api/session/small_signal``."""

    t_start_s: float | None = None
    run_power_flow_initialization: bool | None = None
    project_to_canonical: bool | None = None
    include_participation: bool | None = None
    source_stiffness_pu: float | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "t_start_s": self.t_start_s,
                "run_power_flow_initialization": self.run_power_flow_initialization,
                "project_to_canonical": self.project_to_canonical,
                "include_participation": self.include_participation,
                "source_stiffness_pu": self.source_stiffness_pu,
            },
            self.extra,
        )


# --- Time series / annual / lifecycle -------------------------------------


@dataclass(frozen=True)
class TimeSeriesConfig:
    """Request for ``/api/session/set_ts_config`` (authoring the TS profiles)."""

    num_steps: int | None = None
    step_duration_hr: float | None = None
    profiles: Sequence[Mapping[str, Any]] | None = None
    load_profile_map: Sequence[Mapping[str, Any]] | None = None
    assign_all_loads_to: int | None = None
    assign_all_pv_to: int | None = None
    assign_all_renewables_to: int | None = None
    assign_all_prices_to: int | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "num_steps": self.num_steps,
                "step_duration_hr": self.step_duration_hr,
                "profiles": list(self.profiles) if self.profiles is not None else None,
                "load_profile_map": (
                    list(self.load_profile_map)
                    if self.load_profile_map is not None
                    else None
                ),
                "assign_all_loads_to": self.assign_all_loads_to,
                "assign_all_pv_to": self.assign_all_pv_to,
                "assign_all_renewables_to": self.assign_all_renewables_to,
                "assign_all_prices_to": self.assign_all_prices_to,
            },
            self.extra,
        )


@dataclass(frozen=True)
class TimeSeriesPowerFlowRequest:
    """Request for ``/api/session/run_ts_pf``."""

    num_steps: int | None = None
    step_duration_hr: float | None = None
    skip_uc: bool | None = None
    run_opf: bool | None = None
    uc_solver: str | None = None
    enable_network_constraints: bool | None = None
    reserve_fraction: float | None = None
    objective: str | None = None
    enable_external_grid: bool | None = None
    cyclic_soc: bool | None = None
    parallel_daily: bool | None = None
    daily_mode: str | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "num_steps": self.num_steps,
                "step_duration_hr": self.step_duration_hr,
                "skip_uc": self.skip_uc,
                "run_opf": self.run_opf,
                "uc_solver": self.uc_solver,
                "enable_network_constraints": self.enable_network_constraints,
                "reserve_fraction": self.reserve_fraction,
                "objective": self.objective,
                "enable_external_grid": self.enable_external_grid,
                "cyclic_soc": self.cyclic_soc,
                "parallel_daily": self.parallel_daily,
                "daily_mode": self.daily_mode,
            },
            self.extra,
        )


@dataclass(frozen=True)
class AnnualProductionRequest:
    """Request for ``/api/session/run_annual_sim``."""

    resolution: str | None = None  # "1h" | "6h"
    block_type: str | None = None
    run_opf: bool | None = None
    uc_solver: str | None = None
    enable_network_constraints: bool | None = None
    reserve_fraction: float | None = None
    objective: str | None = None
    cyclic_soc: bool | None = None
    snapshot_interval: int | None = None
    parallel_daily: bool | None = None
    daily_mode: str | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "resolution": self.resolution,
                "block_type": self.block_type,
                "run_opf": self.run_opf,
                "uc_solver": self.uc_solver,
                "enable_network_constraints": self.enable_network_constraints,
                "reserve_fraction": self.reserve_fraction,
                "objective": self.objective,
                "cyclic_soc": self.cyclic_soc,
                "snapshot_interval": self.snapshot_interval,
                "parallel_daily": self.parallel_daily,
                "daily_mode": self.daily_mode,
            },
            self.extra,
        )


@dataclass(frozen=True)
class LifecycleRequest:
    """Request for ``/api/session/run_lifecycle_sim``."""

    resolution: str | None = None
    num_years: int | None = None
    discount_rate: float | None = None
    load_growth_rate: float | None = None
    pv_annual_derating: float | None = None
    calendar_degradation: float | None = None
    run_physical_replay: bool | None = None
    run_sampled_pf_correction: bool | None = None
    pv_scale: float | None = None
    wind_scale: float | None = None
    bess_power_scale: float | None = None
    bess_energy_scale: float | None = None
    diesel_scale: float | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "resolution": self.resolution,
                "num_years": self.num_years,
                "discount_rate": self.discount_rate,
                "load_growth_rate": self.load_growth_rate,
                "pv_annual_derating": self.pv_annual_derating,
                "calendar_degradation": self.calendar_degradation,
                "run_physical_replay": self.run_physical_replay,
                "run_sampled_pf_correction": self.run_sampled_pf_correction,
                "pv_scale": self.pv_scale,
                "wind_scale": self.wind_scale,
                "bess_power_scale": self.bess_power_scale,
                "bess_energy_scale": self.bess_energy_scale,
                "diesel_scale": self.diesel_scale,
            },
            self.extra,
        )


@dataclass(frozen=True)
class LifecycleCompareRequest:
    """Request for ``/api/session/run_lifecycle_compare``."""

    sweep_param: str | None = None
    sweep_min: float | None = None
    sweep_max: float | None = None
    sweep_steps: int | None = None
    resolution: str | None = None
    num_years: int | None = None
    discount_rate: float | None = None
    load_growth_rate: float | None = None
    pv_annual_derating: float | None = None
    calendar_degradation: float | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "sweep_param": self.sweep_param,
                "sweep_min": self.sweep_min,
                "sweep_max": self.sweep_max,
                "sweep_steps": self.sweep_steps,
                "resolution": self.resolution,
                "num_years": self.num_years,
                "discount_rate": self.discount_rate,
                "load_growth_rate": self.load_growth_rate,
                "pv_annual_derating": self.pv_annual_derating,
                "calendar_degradation": self.calendar_degradation,
            },
            self.extra,
        )


# --- Carbon ----------------------------------------------------------------


@dataclass(frozen=True)
class DynamicCarbonRequest:
    """Request for ``/api/session/run_dynamic_carbon``."""

    use_last_tspf: bool | None = None
    num_steps: int | None = None
    skip_uc: bool | None = None
    run_opf: bool | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "use_last_tspf": self.use_last_tspf,
                "num_steps": self.num_steps,
                "skip_uc": self.skip_uc,
                "run_opf": self.run_opf,
            },
            self.extra,
        )


# --- Reliability extras ----------------------------------------------------


@dataclass(frozen=True)
class ReliabilityFmeaRequest:
    """Request for ``/api/session/run_reliability_fmea``.

    Reliability data-policy and cyber-physical option blocks travel through
    ``extra`` (e.g. ``extra={"reliability_template": "comprehensive"}``).
    """

    load_scale_factor: float | None = None
    verbose: bool | None = None
    parallel: bool | None = None
    parallel_threads: int | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "load_scale_factor": self.load_scale_factor,
                "verbose": self.verbose,
                "parallel": self.parallel,
                "parallel_threads": self.parallel_threads,
            },
            self.extra,
        )


@dataclass(frozen=True)
class ReliabilityFdRequest:
    """Request for ``/api/session/run_reliability_fd``."""

    load_scale_factor: float | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact({"load_scale_factor": self.load_scale_factor}, self.extra)


@dataclass(frozen=True)
class ReliabilityThreeStageRequest:
    """Request for ``/api/session/run_reliability_three_stage``."""

    max_switch_operations: int | None = None
    include_generator_faults: bool | None = None
    include_transformer_faults: bool | None = None
    include_converter_faults: bool | None = None
    include_switch_faults: bool | None = None
    include_dc_power_flow: bool | None = None
    parallel: bool | None = None
    parallel_threads: int | None = None
    revalidate_stage3_plan: bool | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "max_switch_operations": self.max_switch_operations,
                "include_generator_faults": self.include_generator_faults,
                "include_transformer_faults": self.include_transformer_faults,
                "include_converter_faults": self.include_converter_faults,
                "include_switch_faults": self.include_switch_faults,
                "include_dc_power_flow": self.include_dc_power_flow,
                "parallel": self.parallel,
                "parallel_threads": self.parallel_threads,
                "revalidate_stage3_plan": self.revalidate_stage3_plan,
            },
            self.extra,
        )


# --- Resilience ------------------------------------------------------------


@dataclass(frozen=True)
class ResilienceRequest:
    """Request for ``/api/session/run_distribution_resilience``.

    Profiles, manual faults, and detailed switch flags travel through ``extra``.
    """

    apply_demo_data: bool | None = None
    horizon_hours: int | None = None
    time_step_hr: float | None = None
    load_scale_factor: float | None = None
    allow_reconfiguration: bool | None = None
    allow_mess_dispatch: bool | None = None
    mess_travel_speed_kmph: float | None = None
    default_fault_count: int | None = None
    repair_time_hr: float | None = None
    run_power_flow: bool | None = None
    model: str | None = None
    mip_solver: str | None = None
    mip_gap: float | None = None
    mip_time_limit_s: float | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "apply_demo_data": self.apply_demo_data,
                "horizon_hours": self.horizon_hours,
                "time_step_hr": self.time_step_hr,
                "load_scale_factor": self.load_scale_factor,
                "allow_reconfiguration": self.allow_reconfiguration,
                "allow_mess_dispatch": self.allow_mess_dispatch,
                "mess_travel_speed_kmph": self.mess_travel_speed_kmph,
                "default_fault_count": self.default_fault_count,
                "repair_time_hr": self.repair_time_hr,
                "run_power_flow": self.run_power_flow,
                "model": self.model,
                "mip_solver": self.mip_solver,
                "mip_gap": self.mip_gap,
                "mip_time_limit_s": self.mip_time_limit_s,
            },
            self.extra,
        )


# --- Market extras ---------------------------------------------------------


@dataclass(frozen=True)
class RealTimeMarketRequest:
    """Request for ``/api/session/run_real_time_market``.

    Market/ancillary/realized option blocks travel through ``extra``.
    """

    num_steps: int | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact({"num_steps": self.num_steps}, self.extra)


@dataclass(frozen=True)
class RepeatedGameRequest:
    """Request for ``/api/session/run_repeated_market_game``."""

    num_steps: int | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact({"num_steps": self.num_steps}, self.extra)


@dataclass(frozen=True)
class SouthernMarketRequest:
    """Request for ``/api/session/run_southern_market`` (body is ``{revision}``)."""

    revision: int | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact({"revision": self.revision}, self.extra)


@dataclass(frozen=True)
class MarketPtdfRequest:
    """Request for ``/api/session/market_ptdf`` (read-only PTDF query)."""

    revision: int | None = None
    day: int | None = None
    period: int | None = None
    branch_ids: Sequence[int] | None = None
    config: Mapping[str, Any] | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "revision": self.revision,
                "day": self.day,
                "period": self.period,
                "branch_ids": (
                    list(self.branch_ids) if self.branch_ids is not None else None
                ),
                "config": self.config,
            },
            self.extra,
        )


# --- Integrated energy -----------------------------------------------------


@dataclass(frozen=True)
class CampusIesRequest:
    """Request for ``/api/session/run_campus_ies``.

    The multi-energy time series (loads, prices, device sizing) travels through
    ``extra`` as authored on the handler's ``campus_ies_data_from_json`` block.
    """

    solver: str | None = None
    objective: str | None = None
    num_steps: int | None = None
    step_duration_hr: float | None = None
    time_limit_sec: float | None = None
    mip_gap_tol: float | None = None
    enable_transport: bool | None = None
    enable_hydrogen_storage_layers: bool | None = None
    enforce_terminal_storage_cyclic: bool | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "solver": self.solver,
                "objective": self.objective,
                "num_steps": self.num_steps,
                "step_duration_hr": self.step_duration_hr,
                "time_limit_sec": self.time_limit_sec,
                "mip_gap_tol": self.mip_gap_tol,
                "enable_transport": self.enable_transport,
                "enable_hydrogen_storage_layers": self.enable_hydrogen_storage_layers,
                "enforce_terminal_storage_cyclic": self.enforce_terminal_storage_cyclic,
            },
            self.extra,
        )


# --- EV power-traffic ------------------------------------------------------


@dataclass(frozen=True)
class EvTrafficRequest:
    """Request for ``/api/session/run_ev_traffic``.

    ``model_options``/``ctm_options``/``due_options``/``optimizer_options`` and a
    custom ``scenario`` travel through ``extra`` or the mapping fields below.
    """

    scenario_source: str | None = None
    formulation: str | None = None
    scenario: Mapping[str, Any] | None = None
    model_options: Mapping[str, Any] | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "scenario_source": self.scenario_source,
                "formulation": self.formulation,
                "scenario": self.scenario,
                "model_options": self.model_options,
            },
            self.extra,
        )


# --- Reconfiguration -------------------------------------------------------


@dataclass(frozen=True)
class ReconfigurationRequest:
    """Request for ``/api/session/run_reconfig``."""

    num_steps: int | None = None
    v_min_pu: float | None = None
    v_max_pu: float | None = None
    mip_gap: float | None = None
    max_time_s: int | None = None
    enable_pf: bool | None = None
    enable_voltage: bool | None = None
    enable_thermal: bool | None = None
    loss_aware: bool | None = None
    solver: str | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "num_steps": self.num_steps,
                "v_min_pu": self.v_min_pu,
                "v_max_pu": self.v_max_pu,
                "mip_gap": self.mip_gap,
                "max_time_s": self.max_time_s,
                "enable_pf": self.enable_pf,
                "enable_voltage": self.enable_voltage,
                "enable_thermal": self.enable_thermal,
                "loss_aware": self.loss_aware,
                "solver": self.solver,
            },
            self.extra,
        )


# --- Hosting capacity ------------------------------------------------------


@dataclass(frozen=True)
class HostingCapacityRequest:
    """Request for ``/api/session/run_hosting_capacity``."""

    default_power_factor: float | None = None
    n1_loading_limit: float | None = None
    enable_verification: bool | None = None
    thd_limit_pct: float | None = None
    enable_harmonic: bool | None = None
    kr: float | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "default_power_factor": self.default_power_factor,
                "n1_loading_limit": self.n1_loading_limit,
                "enable_verification": self.enable_verification,
                "thd_limit_pct": self.thd_limit_pct,
                "enable_harmonic": self.enable_harmonic,
                "kr": self.kr,
            },
            self.extra,
        )


# --- Planning / weak links -------------------------------------------------


@dataclass(frozen=True)
class CounterfactualPlanningRequest:
    """Request for ``/api/session/run_counterfactual_planning``."""

    options: Mapping[str, Any] | None = None
    measures: Sequence[Mapping[str, Any]] | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "options": self.options,
                "measures": list(self.measures) if self.measures is not None else None,
            },
            self.extra,
        )


@dataclass(frozen=True)
class WeakLinksRequest:
    """Request for ``/api/session/run_multidimensional_weak_links``."""

    entities: Sequence[Mapping[str, Any]] | None = None
    periods: Sequence[Mapping[str, Any]] | None = None
    options: Mapping[str, Any] | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "entities": list(self.entities) if self.entities is not None else None,
                "periods": list(self.periods) if self.periods is not None else None,
                "options": self.options,
            },
            self.extra,
        )


# --- Scenario generation ---------------------------------------------------


@dataclass(frozen=True)
class ScenarioGenerationRequest:
    """Request for ``/api/session/generate_scenarios``.

    Each family block (``regular``, ``reliability``, ``resilience``,
    ``perturbation``, ``clustering``, ``typhoon_impact``) is a nested mapping.
    """

    regular: Mapping[str, Any] | None = None
    reliability: Mapping[str, Any] | None = None
    resilience: Mapping[str, Any] | None = None
    perturbation: Mapping[str, Any] | None = None
    clustering: Mapping[str, Any] | None = None
    typhoon_impact: Mapping[str, Any] | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "regular": self.regular,
                "reliability": self.reliability,
                "resilience": self.resilience,
                "perturbation": self.perturbation,
                "clustering": self.clustering,
                "typhoon_impact": self.typhoon_impact,
            },
            self.extra,
        )


@dataclass(frozen=True)
class TyphoonFaultsRequest:
    """Request for ``/api/session/generate_typhoon_faults``."""

    horizon_hours: int | None = None
    time_step_hr: float | None = None
    seed: int | None = None
    stochastic: bool | None = None
    month: int | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "horizon_hours": self.horizon_hours,
                "time_step_hr": self.time_step_hr,
                "seed": self.seed,
                "stochastic": self.stochastic,
                "month": self.month,
            },
            self.extra,
        )


# --- Harmonics state space (phase 3) --------------------------------------


@dataclass(frozen=True)
class HarmonicsStateSpaceRequest:
    """Request for ``/api/session/harmonics_hss``.

    ``options`` carries the HSS option block (orders, converter models, ...);
    ``injections`` and ``couplings`` carry the harmonic source and coupling
    arrays.
    """

    options: Mapping[str, Any] | None = None
    injections: Sequence[Mapping[str, Any]] | None = None
    couplings: Sequence[Mapping[str, Any]] | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "options": self.options,
                "injections": (
                    list(self.injections) if self.injections is not None else None
                ),
                "couplings": (
                    list(self.couplings) if self.couplings is not None else None
                ),
            },
            self.extra,
        )


# --- Model I/O (phase 3) ---------------------------------------------------


@dataclass(frozen=True)
class MatpowerLoadRequest:
    """Request for ``/api/session/load_matpower``."""

    filename: str | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact({"filename": self.filename}, self.extra)


@dataclass(frozen=True)
class JsonModelLoadRequest:
    """Request for ``/api/session/load_json_string``."""

    json_string: str | None = None
    preserve_reliability_configuration: bool | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "json_string": self.json_string,
                "preserve_reliability_configuration": (
                    self.preserve_reliability_configuration
                ),
            },
            self.extra,
        )


@dataclass(frozen=True)
class BpaLoadRequest:
    """Request for the JSON path of ``/api/session/load_bpa_dat``."""

    dat_string: str | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact({"dat_string": self.dat_string}, self.extra)


@dataclass(frozen=True)
class CimLoadRequest:
    """Request for ``/api/session/load_cim_dist``."""

    xml_files: Sequence[Any] | None = None
    xml_strings: Sequence[str] | None = None
    xml_string: str | None = None
    load_factor: float | None = None
    power_factor: float | None = None
    base_mva: float | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "xml_files": list(self.xml_files) if self.xml_files is not None else None,
                "xml_strings": (
                    list(self.xml_strings) if self.xml_strings is not None else None
                ),
                "xml_string": self.xml_string,
                "load_factor": self.load_factor,
                "power_factor": self.power_factor,
                "base_mva": self.base_mva,
            },
            self.extra,
        )


@dataclass(frozen=True)
class GridlabdLoadRequest:
    """Request for ``/api/session/load_gridlabd``."""

    glm_string: str | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact({"glm_string": self.glm_string}, self.extra)


@dataclass(frozen=True)
class OpenDssLoadRequest:
    """Request for ``/api/session/load_opendss``."""

    dss_string: str | None = None
    files: Sequence[Mapping[str, Any]] | None = None
    master_path: str | None = None
    dss_path: str | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "dss_string": self.dss_string,
                "files": list(self.files) if self.files is not None else None,
                "master_path": self.master_path,
                "dss_path": self.dss_path,
            },
            self.extra,
        )


@dataclass(frozen=True)
class EtapXmlLoadRequest:
    """Request for ``/api/session/load_etap_xml``."""

    xml_string: str | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact({"xml_string": self.xml_string}, self.extra)


@dataclass(frozen=True)
class SvgImportRequest:
    """Request for ``/api/session/load_svg_distribution``."""

    svg_string: str | None = None
    name: str | None = None
    base_mva: float | None = None
    nominal_mv_kv: float | None = None
    transformer_load_factor: float | None = None
    power_factor: float | None = None
    auto_complete_parameters: bool | None = None
    default_mv_cross_section_mm2: float | None = None
    synthetic_source_s_sc_max_mva: float | None = None
    synthetic_source_s_sc_min_mva: float | None = None
    synthetic_source_rx: float | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "svg_string": self.svg_string,
                "name": self.name,
                "base_mva": self.base_mva,
                "nominal_mv_kv": self.nominal_mv_kv,
                "transformer_load_factor": self.transformer_load_factor,
                "power_factor": self.power_factor,
                "auto_complete_parameters": self.auto_complete_parameters,
                "default_mv_cross_section_mm2": self.default_mv_cross_section_mm2,
                "synthetic_source_s_sc_max_mva": self.synthetic_source_s_sc_max_mva,
                "synthetic_source_s_sc_min_mva": self.synthetic_source_s_sc_min_mva,
                "synthetic_source_rx": self.synthetic_source_rx,
            },
            self.extra,
        )


@dataclass(frozen=True)
class SvgExportRequest:
    """Request for ``/api/session/export_svg_distribution``."""

    title: str | None = None
    horizontal_spacing: float | None = None
    vertical_spacing: float | None = None
    margin: float | None = None
    include_hacdcpf_parameters: bool | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return _compact(
            {
                "title": self.title,
                "horizontal_spacing": self.horizontal_spacing,
                "vertical_spacing": self.vertical_spacing,
                "margin": self.margin,
                "include_hacdcpf_parameters": self.include_hacdcpf_parameters,
            },
            self.extra,
        )
