from __future__ import annotations

import json
import unittest
from typing import Any, Mapping

from hysim import (
    ANALYSIS_CATALOG,
    AnalysisCategory,
    AnalysisEffect,
    CimLoadRequest,
    DcShortCircuitRequest,
    HarmonicsStateSpaceRequest,
    HySim,
    HySimToolRegistry,
    InvalidResultError,
    LifecycleRequest,
    MarketClearingRequest,
    ReliabilityRequest,
    ResilienceRequest,
    RpoRequest,
    ScenarioGenerationRequest,
    ShortCircuitRequest,
    SvgImportRequest,
    ToolPolicyError,
    TransientRequest,
    TransportResponse,
    UnitCommitmentRequest,
    UnknownAnalysisError,
)
from test_edition_capabilities import edition_payload


class FakeTransport:
    def __init__(self, responses: list[tuple[int, Mapping[str, Any]]]) -> None:
        self.responses = list(responses)
        self.calls: list[dict[str, Any]] = []

    def request(self, method: str, path: str, **kwargs: Any) -> TransportResponse:
        self.calls.append({"method": method, "path": path, **kwargs})
        status, payload = self.responses.pop(0)
        return TransportResponse(status, {}, json.dumps(payload).encode())


def _ok(payload: Mapping[str, Any]) -> tuple[int, Mapping[str, Any]]:
    return (200, payload)


def _profile() -> tuple[int, Mapping[str, Any]]:
    return _ok(edition_payload())


class CatalogTests(unittest.TestCase):
    def test_catalog_is_consistent(self) -> None:
        seen_routes: set[str] = set()
        for spec in ANALYSIS_CATALOG:
            self.assertTrue(spec.name)
            self.assertTrue(spec.route.startswith("/api/"))
            self.assertIn(spec.method, {"POST", "GET"})
            self.assertNotIn(spec.route, seen_routes, f"duplicate route {spec.route}")
            seen_routes.add(spec.route)

    def test_require_rejects_unknown_analysis(self) -> None:
        with self.assertRaises(UnknownAnalysisError):
            ANALYSIS_CATALOG.require("does_not_exist")

    def test_effect_and_category_filters(self) -> None:
        uc = ANALYSIS_CATALOG.require("unit_commitment")
        self.assertEqual(uc.route, "/api/session/run_uc")
        self.assertIs(uc.effect, AnalysisEffect.ANALYZE)
        self.assertIs(uc.category, AnalysisCategory.TIME_SERIES)

        modifiers = {s.name for s in ANALYSIS_CATALOG.by_effect(AnalysisEffect.MODIFY)}
        self.assertIn("load_builtin", modifiers)
        self.assertIn("update_components", modifiers)

        market = {s.name for s in ANALYSIS_CATALOG.by_category(AnalysisCategory.MARKET)}
        self.assertIn("market_clearing", market)


class RequestModelTests(unittest.TestCase):
    def test_none_fields_are_omitted(self) -> None:
        self.assertEqual(UnitCommitmentRequest().to_dict(), {})
        self.assertEqual(UnitCommitmentRequest(num_steps=12).to_dict(), {"num_steps": 12})

    def test_extra_is_preserved_and_named_fields_win(self) -> None:
        req = ReliabilityRequest(seed=7, extra={"data_template": "generic", "seed": 1})
        payload = req.to_dict()
        self.assertEqual(payload["seed"], 7)
        self.assertEqual(payload["data_template"], "generic")

    def test_market_and_transient_and_rpo_payloads(self) -> None:
        self.assertEqual(
            MarketClearingRequest(num_steps=24, reserve_fraction=0.06).to_dict(),
            {"num_steps": 24, "reserve_fraction": 0.06},
        )
        self.assertEqual(
            TransientRequest(t_end_s=5.0, dt_s=0.005).to_dict(),
            {"t_end_s": 5.0, "dt_s": 0.005},
        )
        self.assertEqual(
            RpoRequest(objective="combined").to_dict(), {"objective": "combined"}
        )


class FamilyFacadeTests(unittest.TestCase):
    def test_unit_commitment_routes_and_serializes(self) -> None:
        transport = FakeTransport([_profile(), _ok({"feasible": True, "total_cost": 1000.0})])
        sim = HySim(transport=transport)

        result = sim.time_series.unit_commitment(UnitCommitmentRequest(num_steps=24))

        self.assertEqual(transport.calls[1]["path"], "/api/session/run_uc")
        self.assertEqual(transport.calls[1]["method"], "POST")
        self.assertEqual(transport.calls[1]["json_body"], {"num_steps": 24})
        self.assertEqual(result.analysis, "unit_commitment")
        self.assertEqual(result.route, "/api/session/run_uc")
        self.assertEqual(result.scientific_status, "qualified")

    def test_market_clearing_accepts_raw_mapping(self) -> None:
        transport = FakeTransport([_profile(), _ok({"converged": True})])
        sim = HySim(transport=transport)

        sim.market.clearing({"num_steps": 6, "offer_segments": 4})

        self.assertEqual(transport.calls[1]["path"], "/api/session/run_market_clearing")
        self.assertEqual(
            transport.calls[1]["json_body"], {"num_steps": 6, "offer_segments": 4}
        )

    def test_reliability_family_reaches_both_routes(self) -> None:
        transport = FakeTransport(
            [_profile(), _ok({"converged": True}), _ok({"converged": True})]
        )
        sim = HySim(transport=transport)

        sim.reliability.nonsequential(ReliabilityRequest(seed=3))
        sim.reliability.sequential(ReliabilityRequest(seed=3))

        self.assertEqual(transport.calls[1]["path"], "/api/session/run_reliability_nsq")
        self.assertEqual(transport.calls[2]["path"], "/api/session/run_reliability_seq")

    def test_run_by_name_dispatches_through_catalog(self) -> None:
        transport = FakeTransport([_profile(), _ok({"converged": True})])
        sim = HySim(transport=transport)

        result = sim.run("small_signal", {"foo": 1})

        self.assertEqual(transport.calls[1]["path"], "/api/session/small_signal")
        self.assertEqual(result.analysis, "small_signal")

    def test_modify_effect_increments_revision(self) -> None:
        transport = FakeTransport([_profile(), _ok({"ok": True}), _ok({"converged": True})])
        sim = HySim(transport=transport)

        self.assertEqual(sim.model_revision, 0)
        sim.run("load_builtin", {"case": "ieee14_acdc"})
        self.assertEqual(sim.model_revision, 1)

        pf = sim.run("topology", {})
        # A read-only analysis must not advance the model revision.
        self.assertEqual(sim.model_revision, 1)
        self.assertEqual(pf.model_revision, 1)

    def test_invalid_result_is_not_silently_usable(self) -> None:
        transport = FakeTransport([_profile(), _ok({"feasible": False, "warnings": ["infeasible"]})])
        sim = HySim(transport=transport)

        result = sim.market.clearing(MarketClearingRequest(num_steps=24))
        self.assertEqual(result.scientific_status, "invalid")
        with self.assertRaises(InvalidResultError):
            result.require_usable()


class Phase2CatalogTests(unittest.TestCase):
    def test_io_and_ts_config_routes_present(self) -> None:
        for name, route in [
            ("set_ts_config", "/api/session/set_ts_config"),
            ("load_bpa_dat", "/api/session/load_bpa_dat"),
            ("export_bpa_dat", "/api/session/export_bpa_dat"),
            ("load_cim_dist", "/api/session/load_cim_dist"),
            ("load_opendss", "/api/session/load_opendss"),
            ("export_svg_distribution", "/api/session/export_svg_distribution"),
        ]:
            self.assertEqual(ANALYSIS_CATALOG.require(name).route, route)

    def test_mutates_model_is_narrower_than_modify_effect(self) -> None:
        # A model load mutates the authored model; a session-config write does not.
        self.assertTrue(ANALYSIS_CATALOG.require("load_matpower").mutates_model)
        self.assertTrue(ANALYSIS_CATALOG.require("load_cim_dist").mutates_model)
        set_ts = ANALYSIS_CATALOG.require("set_ts_config")
        self.assertIs(set_ts.effect, AnalysisEffect.MODIFY)
        self.assertFalse(set_ts.mutates_model)
        # Exports are read-only and never mutate.
        self.assertFalse(ANALYSIS_CATALOG.require("export_json").mutates_model)


class Phase2RequestTests(unittest.TestCase):
    def test_short_circuit_and_dc_lists(self) -> None:
        self.assertEqual(
            ShortCircuitRequest(fault_type="ThreePhase", compute_ith=True).to_dict(),
            {"fault_type": "ThreePhase", "compute_ith": True},
        )
        self.assertEqual(
            DcShortCircuitRequest(fault_bus_ids=[1, 2], source_voltage_pu=1.0).to_dict(),
            {"fault_bus_ids": [1, 2], "source_voltage_pu": 1.0},
        )

    def test_lifecycle_and_nested_scenario(self) -> None:
        self.assertEqual(
            LifecycleRequest(num_years=10, discount_rate=0.05).to_dict(),
            {"num_years": 10, "discount_rate": 0.05},
        )
        payload = ScenarioGenerationRequest(
            regular={"enabled": True, "candidate_count": 50}
        ).to_dict()
        self.assertEqual(payload, {"regular": {"enabled": True, "candidate_count": 50}})

    def test_resilience_named_and_extra(self) -> None:
        payload = ResilienceRequest(horizon_hours=8, extra={"manual_faults": []}).to_dict()
        self.assertEqual(payload["horizon_hours"], 8)
        self.assertEqual(payload["manual_faults"], [])


class Phase2FacadeTests(unittest.TestCase):
    def _sim(self, n: int = 1) -> tuple[HySim, FakeTransport]:
        transport = FakeTransport([_profile()] + [_ok({"converged": True}) for _ in range(n)])
        return HySim(transport=transport), transport

    def test_every_new_family_hits_its_route(self) -> None:
        cases = [
            (lambda s: s.short_circuit.run(), "/api/session/sc"),
            (lambda s: s.short_circuit.detailed(), "/api/session/sc_detailed"),
            (lambda s: s.short_circuit.dc(), "/api/session/dc_sc"),
            (lambda s: s.harmonics.run(), "/api/session/harmonics"),
            (lambda s: s.harmonics.frequency_scan(), "/api/session/harmonics_freqscan"),
            (lambda s: s.harmonics.newton(), "/api/session/harmonics_newton"),
            (lambda s: s.dynamics.small_signal(), "/api/session/small_signal"),
            (lambda s: s.carbon.dynamic(), "/api/session/run_dynamic_carbon"),
            (lambda s: s.reliability.fmea(), "/api/session/run_reliability_fmea"),
            (lambda s: s.reliability.three_stage(), "/api/session/run_reliability_three_stage"),
            (lambda s: s.resilience.distribution(), "/api/session/run_distribution_resilience"),
            (lambda s: s.market.southern(), "/api/session/run_southern_market"),
            (lambda s: s.market.ptdf(), "/api/session/market_ptdf"),
            (lambda s: s.integrated_energy.campus(), "/api/session/run_campus_ies"),
            (lambda s: s.ev_traffic.run(), "/api/session/run_ev_traffic"),
            (lambda s: s.reconfiguration.run(), "/api/session/run_reconfig"),
            (lambda s: s.hosting_capacity.run(), "/api/session/run_hosting_capacity"),
            (lambda s: s.planning.counterfactual(), "/api/session/run_counterfactual_planning"),
            (lambda s: s.planning.weak_links(), "/api/session/run_multidimensional_weak_links"),
            (lambda s: s.scenario.generate(), "/api/session/generate_scenarios"),
            (lambda s: s.scenario.typhoon_faults(), "/api/session/generate_typhoon_faults"),
            (lambda s: s.time_series.lifecycle(), "/api/session/run_lifecycle_sim"),
            (lambda s: s.opf.ac(), "/api/session/opf_ac"),
        ]
        for call, route in cases:
            sim, transport = self._sim()
            call(sim)
            self.assertEqual(transport.calls[1]["path"], route, route)

    def test_ts_config_write_does_not_advance_model_revision(self) -> None:
        sim, _ = self._sim()
        self.assertEqual(sim.model_revision, 0)
        sim.time_series.configure({"num_steps": 24})
        self.assertEqual(sim.model_revision, 0)

    def test_model_io_load_mutates_export_does_not(self) -> None:
        transport = FakeTransport([_profile(), _ok({"ok": True}), _ok({"json_string": "{}"})])
        sim = HySim(transport=transport)
        sim.model_io.load("matpower", {"filename": "case14.m"})
        self.assertEqual(sim.model_revision, 1)
        sim.model_io.export("json")
        self.assertEqual(sim.model_revision, 1)
        self.assertEqual(transport.calls[1]["path"], "/api/session/load_matpower")
        self.assertEqual(transport.calls[2]["path"], "/api/session/export_json")

    def test_model_io_rejects_unknown_format(self) -> None:
        sim, _ = self._sim(0)
        with self.assertRaises(ValueError):
            sim.model_io.export("nope")
        with self.assertRaises(ValueError):
            sim.model_io.load("nope")


class FamilyToolTests(unittest.TestCase):
    def test_catalog_tools_are_effect_gated_and_bounded(self) -> None:
        transport = FakeTransport([_profile(), _ok({"converged": True})])
        sim = HySim(transport=transport)
        registry = HySimToolRegistry(sim.client)

        new_tools = registry.register_family_tools()
        self.assertIn("hysim_market_clearing", new_tools)
        self.assertIn("hysim_reliability_fmea", new_tools)
        # Curated tool names are not clobbered by the generic family registration.
        self.assertNotIn("hysim_load_builtin", new_tools)

        # An ANALYZE family tool runs and returns a compact honest record.
        record = registry.call("hysim_market_clearing", {"request": {"num_steps": 4}})
        self.assertEqual(record["route"], "/api/session/run_market_clearing")
        self.assertEqual(transport.calls[1]["json_body"], {"num_steps": 4})

        # A MODIFY family tool is disabled under the default policy.
        with self.assertRaises(ToolPolicyError):
            registry.call("hysim_new_empty", {})


class Phase3IoTests(unittest.TestCase):
    def test_typed_io_requests_serialize(self) -> None:
        self.assertEqual(
            CimLoadRequest(xml_strings=["<x/>"], base_mva=100.0).to_dict(),
            {"xml_strings": ["<x/>"], "base_mva": 100.0},
        )
        self.assertEqual(
            SvgImportRequest(svg_string="<svg/>", power_factor=0.95).to_dict(),
            {"svg_string": "<svg/>", "power_factor": 0.95},
        )
        self.assertEqual(
            HarmonicsStateSpaceRequest(options={"orders": [1, 5]}).to_dict(),
            {"options": {"orders": [1, 5]}},
        )

    def test_model_io_accepts_typed_load_request(self) -> None:
        transport = FakeTransport([_profile(), _ok({"ok": True})])
        sim = HySim(transport=transport)
        sim.model_io.load("cim_dist", CimLoadRequest(xml_strings=["<x/>"]))
        self.assertEqual(transport.calls[1]["path"], "/api/session/load_cim_dist")
        self.assertEqual(transport.calls[1]["json_body"], {"xml_strings": ["<x/>"]})
        self.assertEqual(sim.model_revision, 1)

    def test_harmonics_state_space_is_typed(self) -> None:
        transport = FakeTransport([_profile(), _ok({"converged": True})])
        sim = HySim(transport=transport)
        sim.harmonics.state_space(HarmonicsStateSpaceRequest(options={"orders": [1]}))
        self.assertEqual(transport.calls[1]["path"], "/api/session/harmonics_hss")
        self.assertEqual(transport.calls[1]["json_body"], {"options": {"orders": [1]}})


if __name__ == "__main__":
    unittest.main()
