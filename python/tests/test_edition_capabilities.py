from __future__ import annotations

import json
import unittest
from typing import Any, Mapping
from unittest.mock import patch

from hysim import (
    ANALYSIS_ALIASES,
    ANALYSIS_CATALOG,
    AnalysisDisabledError,
    EditionProfile,
    HySim,
    HySimClient,
    HySimToolRegistry,
    HySimV1Client,
    HySimV1Session,
    HySimV1ToolRegistry,
    ToolEffect,
    ToolPolicy,
    TransportError,
    TransportResponse,
    UnknownAnalysisError,
    V1Capabilities,
)


def edition_payload(
    *, edition: str = "full", disabled: frozenset[str] = frozenset()
) -> dict[str, Any]:
    return {
        "schema": "hacdcpf.edition-profile.v1",
        "edition": edition,
        "analysis_catalog": {
            "schema": "hacdcpf.edition-analysis-catalog.v1",
            "entries": [
                {
                    "name": spec.name,
                    "route": spec.route,
                    "method": spec.method,
                    "enabled": spec.name not in disabled,
                }
                for spec in ANALYSIS_CATALOG
            ],
        },
    }


def v1_payload(
    *,
    enabled: tuple[str, ...] = ("power_flow", "optimal_power_flow"),
    disabled: tuple[str, ...] = (),
) -> dict[str, Any]:
    return {
        "schema": "hysim_api_v1",
        "version": "1.0",
        "features": ["multi_session", "asynchronous_jobs"],
        "analyses": list(enabled),
        "known_disabled_analyses": list(disabled),
        "limits": {"sessions": 128, "retained_jobs": 4096},
    }


class FakeTransport:
    def __init__(self, responses: list[tuple[int, Mapping[str, Any]]]) -> None:
        self.responses = list(responses)
        self.calls: list[dict[str, Any]] = []

    def request(self, method: str, path: str, **kwargs: Any) -> TransportResponse:
        self.calls.append({"method": method, "path": path, **kwargs})
        status, payload = self.responses.pop(0)
        return TransportResponse(status, {}, json.dumps(payload).encode())


class EditionProfileTests(unittest.TestCase):
    def test_strict_profile_parses_and_filters_without_mutating_universe(self) -> None:
        before = ANALYSIS_CATALOG.names()
        profile = EditionProfile.parse(
            edition_payload(edition="resilience", disabled=frozenset({"carbon_flow"})),
            ANALYSIS_CATALOG,
        )
        filtered = ANALYSIS_CATALOG.filtered(profile.enabled_names)

        self.assertEqual(profile.edition, "resilience")
        self.assertFalse(profile.capability("carbon_flow").enabled)
        self.assertNotIn("carbon_flow", filtered)
        self.assertIn("carbon_flow", ANALYSIS_CATALOG)
        self.assertEqual(ANALYSIS_CATALOG.names(), before)

    def test_profile_fails_closed_on_malformed_catalog(self) -> None:
        malformed = edition_payload()
        malformed["analysis_catalog"]["entries"][0]["extra"] = True
        with self.assertRaises(TransportError):
            EditionProfile.parse(malformed, ANALYSIS_CATALOG)

        mismatch = edition_payload()
        mismatch["analysis_catalog"]["entries"][0]["route"] = "/api/session/wrong"
        with self.assertRaises(TransportError):
            EditionProfile.parse(mismatch, ANALYSIS_CATALOG)

    def test_profile_requires_exact_known_universe(self) -> None:
        missing = edition_payload()
        missing["analysis_catalog"]["entries"].pop()
        with self.assertRaises(TransportError):
            EditionProfile.parse(missing, ANALYSIS_CATALOG)


class LegacyCapabilityTests(unittest.TestCase):
    def test_aliases_canonicalize_before_gate_and_transport(self) -> None:
        transport = FakeTransport(
            [(200, edition_payload()), (200, {"converged": True}), (200, {"converged": True})]
        )
        client = HySimClient(transport=transport)

        resilience = client.run_analysis("resilience")
        integrated = client.run_analysis("integrated_energy")

        self.assertEqual(ANALYSIS_ALIASES["resilience"], "distribution_resilience")
        self.assertEqual(resilience.analysis, "distribution_resilience")
        self.assertEqual(integrated.analysis, "campus_ies")
        self.assertEqual(transport.calls[1]["path"], "/api/session/run_distribution_resilience")
        self.assertEqual(transport.calls[2]["path"], "/api/session/run_campus_ies")

    def test_unknown_and_disabled_reject_before_analysis_transport(self) -> None:
        transport = FakeTransport(
            [(200, edition_payload(edition="resilience", disabled=frozenset({"carbon_flow"})))]
        )
        client = HySimClient(transport=transport)

        with self.assertRaises(UnknownAnalysisError):
            client.run_analysis("not_real")
        self.assertEqual(transport.calls, [])

        with self.assertRaises(AnalysisDisabledError):
            client.run_analysis("carbon_flow")
        self.assertEqual([call["path"] for call in transport.calls], ["/api/edition"])

    def test_clients_have_independent_immutable_catalogs(self) -> None:
        first_transport = FakeTransport(
            [(200, edition_payload(disabled=frozenset({"carbon_flow"})))]
        )
        second_transport = FakeTransport([(200, edition_payload())])
        first = HySimClient(transport=first_transport)
        second = HySimClient(transport=second_transport)

        self.assertNotIn("carbon_flow", first.analysis_catalog)
        self.assertIn("carbon_flow", second.analysis_catalog)
        self.assertIsNot(first.analysis_catalog, second.analysis_catalog)
        self.assertIn("carbon_flow", ANALYSIS_CATALOG)

    def test_generic_family_and_shortcuts_share_gate(self) -> None:
        disabled = frozenset({"power_flow", "small_signal", "market_clearing"})
        transport = FakeTransport([(200, edition_payload(edition="trial", disabled=disabled))])
        sim = HySim(transport=transport)

        for call in (
            lambda: sim.client.power_flow(),
            lambda: sim.run("small_signal"),
            lambda: sim.market.clearing(),
        ):
            with self.assertRaises(AnalysisDisabledError):
                call()
        self.assertEqual([call["path"] for call in transport.calls], ["/api/edition"])

    def test_execute_cannot_bypass_gate(self) -> None:
        transport = FakeTransport(
            [(200, edition_payload(disabled=frozenset({"carbon_flow"})))]
        )
        client = HySimClient(transport=transport)
        spec = ANALYSIS_CATALOG.require("carbon_flow")
        with self.assertRaises(AnalysisDisabledError):
            client.execute(
                name=spec.name,
                route=spec.route,
                method=spec.method,
                modifies_model=spec.mutates_model,
            )
        self.assertEqual(len(transport.calls), 1)

    def test_model_lifecycle_and_curated_tools_share_gate(self) -> None:
        disabled = frozenset({"load_builtin", "load_matpower", "load_json_string", "new_empty", "update_components", "export_json"})
        transport = FakeTransport([(200, edition_payload(edition="trial", disabled=disabled))])
        client = HySimClient(transport=transport)

        registry = HySimToolRegistry(client)
        self.assertNotIn("hysim_load_builtin", {tool.name for tool in registry.specs()})
        for call in (
            lambda: client.load_builtin("case9"),
            lambda: client.load_matpower("case9.m"),
            lambda: client.load_model({}),
            client.new_system,
            lambda: client.update_components({}),
            client.export_model,
        ):
            with self.assertRaises(AnalysisDisabledError):
                call()
        self.assertEqual([call["path"] for call in transport.calls], ["/api/edition"])

    def test_curated_tool_rechecks_capability_after_refresh(self) -> None:
        changing = FakeTransport(
            [
                (200, edition_payload()),
                (200, edition_payload(disabled=frozenset({"load_builtin"}))),
            ]
        )
        client = HySimClient(transport=changing)
        registry = HySimToolRegistry(
            client,
            policy=ToolPolicy(
                allowed_effects=frozenset(
                    {ToolEffect.READ, ToolEffect.ANALYZE, ToolEffect.MODIFY_MODEL}
                )
            ),
        )
        client.refresh_edition_profile()
        with self.assertRaises(AnalysisDisabledError):
            registry.call("hysim_load_builtin", {"case": "case9"}, approved=True)
        self.assertEqual(len(changing.calls), 2)

    def test_ai_registration_and_call_are_capability_gated(self) -> None:
        transport = FakeTransport(
            [(200, edition_payload(disabled=frozenset({"market_clearing", "sppt_guard"})))]
        )
        client = HySimClient(transport=transport)
        registry = HySimToolRegistry(client)
        registered = registry.register_family_tools()

        self.assertNotIn("hysim_market_clearing", registered)
        self.assertNotIn("hysim_sppt_guard", {tool.name for tool in registry.specs()})
        with self.assertRaises(AnalysisDisabledError):
            registry.register_family_tools(include=["market_clearing"])

        # A refresh can invalidate a previously registered family tool; its
        # handler rechecks capabilities independently of ToolPolicy.
        changing = FakeTransport(
            [
                (200, edition_payload()),
                (200, edition_payload(disabled=frozenset({"market_clearing"}))),
            ]
        )
        client = HySimClient(transport=changing)
        registry = HySimToolRegistry(client)
        registry.register_family_tools(include=["market_clearing"])
        client.refresh_edition_profile()
        with self.assertRaises(AnalysisDisabledError):
            registry.call("hysim_market_clearing", {})
        self.assertEqual(len(changing.calls), 2)


class V1CapabilityTests(unittest.TestCase):
    def test_v1_discovery_is_strict_and_independent(self) -> None:
        parsed = V1Capabilities.parse(
            v1_payload(disabled=("carbon_flow", "harmonics"))
        )
        self.assertIn("power_flow", parsed.enabled_analyses)
        self.assertEqual(parsed.get("schema"), "hysim_api_v1")
        self.assertIn("multi_session", parsed.get("features", ()))
        with self.assertRaises(AnalysisDisabledError):
            parsed.require("carbon_flow")
        with self.assertRaises(UnknownAnalysisError):
            parsed.require("not_real")

        malformed = v1_payload()
        malformed["extra"] = True
        with self.assertRaises(TransportError):
            V1Capabilities.parse(malformed)

    def test_v1_submit_preflights_disabled_and_unknown_without_post(self) -> None:
        transport = FakeTransport(
            [(200, v1_payload(disabled=("carbon_flow",)))]
        )
        client = HySimV1Client(transport=transport)
        session = HySimV1Session(client, "ses-1", 1, '"etag"', {})

        with self.assertRaises(AnalysisDisabledError):
            session.submit("carbon_flow")
        with self.assertRaises(UnknownAnalysisError):
            session.submit("not_real")
        self.assertEqual([call["path"] for call in transport.calls], ["/api/v1"])

    def test_v1_tools_filter_disabled_submissions(self) -> None:
        transport = FakeTransport(
            [(200, v1_payload(enabled=("power_flow",), disabled=("optimal_power_flow",)))]
        )
        session = HySimV1Session(
            HySimV1Client(transport=transport), "ses-1", 1, '"etag"', {}
        )
        registry = HySimV1ToolRegistry(session)
        names = {tool.name for tool in registry.specs()}
        self.assertIn("hysim_v1_submit_power_flow", names)
        self.assertNotIn("hysim_v1_submit_optimal_power_flow", names)


class LocalServerReadinessTests(unittest.TestCase):
    def test_readiness_uses_strict_edition_discovery(self) -> None:
        class FakeProcess:
            returncode = None

            def poll(self) -> None:
                return None

        profile = EditionProfile.parse(edition_payload(), ANALYSIS_CATALOG)
        with patch("hysim.server.subprocess.Popen", return_value=FakeProcess()), patch(
            "hysim.server.HySimClient.refresh_edition_profile", return_value=profile
        ) as discover:
            from hysim import LocalHySimServer

            runtime = LocalHySimServer(
                "run_gui_server", data_dir="data", port=18088, startup_timeout=0.5
            ).start()
            self.assertIs(runtime.edition_profile, profile)
            discover.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
