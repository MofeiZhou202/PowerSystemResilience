from __future__ import annotations

import json
import unittest
from typing import Any, Mapping

from hysim import (
    AnalysisResult,
    BusDomain,
    BusRef,
    BusyError,
    HySimClient,
    InvalidResultError,
    PowerFlowOptions,
    PowerFlowRequest,
    TransportResponse,
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


class ClientTests(unittest.TestCase):
    def test_load_and_power_flow_preserve_revision_and_payload(self) -> None:
        transport = FakeTransport(
            [
                (200, edition_payload()),
                (200, {"name": "ieee14_acdc", "counts": {"ac_buses": 14}}),
                (
                    200,
                    {
                        "schema": "power_flow_result_v1",
                        "converged": True,
                        "iterations": 4,
                        "residual": 1e-10,
                        "vm": [1.0, 0.99],
                        "model_limitations": ["balanced aggregate"],
                    },
                ),
            ]
        )
        client = HySimClient(transport=transport)

        client.load_builtin("ieee14_acdc")
        result = client.power_flow(
            PowerFlowRequest(options=PowerFlowOptions(max_iter=80, tol=1e-9))
        )

        self.assertEqual(transport.calls[0]["path"], "/api/edition")
        self.assertEqual(transport.calls[1]["path"], "/api/session/load_builtin")
        self.assertEqual(client.model_revision, 1)
        self.assertEqual(result.model_revision, 1)
        self.assertEqual(result.scientific_status, "qualified")
        self.assertEqual(result.summary()["result_sizes"]["vm"], 2)
        self.assertEqual(result.limitations, ("balanced aggregate",))
        self.assertEqual(
            transport.calls[2]["json_body"],
            {
                "method": "ac_newton",
                "options": {
                    "max_iter": 80,
                    "tol": 1e-9,
                    "enable_converter_coordination_check": True,
                },
            },
        )

    def test_busy_response_has_specific_error(self) -> None:
        client = HySimClient(
            transport=FakeTransport([(409, {"error": "Another analysis is already running"})])
        )
        with self.assertRaises(BusyError):
            client.power_flow()

    def test_invalid_result_must_not_be_silently_consumed(self) -> None:
        result = AnalysisResult(
            "power_flow",
            "/api/session/pf",
            "request-1",
            2,
            {"converged": False, "warnings": ["iteration limit"]},
        )
        with self.assertRaises(InvalidResultError):
            result.require_usable()

    def test_bus_reference_is_domain_qualified(self) -> None:
        ac = BusRef(BusDomain.AC, 7)
        dc = BusRef(BusDomain.DC, 7)
        self.assertNotEqual(ac, dc)
        self.assertEqual(dc.to_dict(), {"domain": "dc", "index": 7})


if __name__ == "__main__":
    unittest.main()

