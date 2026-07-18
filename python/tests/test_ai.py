from __future__ import annotations

import json
import unittest
from typing import Any, Mapping

from hysim import (
    HySimClient,
    HySimToolRegistry,
    ToolEffect,
    ToolPolicy,
    ToolPolicyError,
    TransportResponse,
)


class FakeTransport:
    def __init__(self, responses: list[Mapping[str, Any]]) -> None:
        self.responses = list(responses)

    def request(self, method: str, path: str, **kwargs: Any) -> TransportResponse:
        return TransportResponse(200, {}, json.dumps(self.responses.pop(0)).encode())


class AIToolTests(unittest.TestCase):
    def test_default_policy_blocks_model_mutation(self) -> None:
        registry = HySimToolRegistry(HySimClient(transport=FakeTransport([])))
        with self.assertRaises(ToolPolicyError):
            registry.call("hysim_load_builtin", {"case": "ieee14_acdc"}, approved=True)

    def test_mutation_requires_both_effect_permission_and_approval(self) -> None:
        policy = ToolPolicy(
            allowed_effects=frozenset(
                {ToolEffect.READ, ToolEffect.ANALYZE, ToolEffect.MODIFY_MODEL}
            )
        )
        registry = HySimToolRegistry(
            HySimClient(transport=FakeTransport([{"name": "ieee14_acdc"}])),
            policy=policy,
        )
        with self.assertRaises(ToolPolicyError):
            registry.call("hysim_load_builtin", {"case": "ieee14_acdc"})
        result = registry.call(
            "hysim_load_builtin", {"case": "ieee14_acdc"}, approved=True
        )
        self.assertEqual(result["name"], "ieee14_acdc")

    def test_analysis_tool_returns_compact_result_by_default(self) -> None:
        registry = HySimToolRegistry(
            HySimClient(
                transport=FakeTransport(
                    [{"converged": True, "iterations": 3, "vm": [1.0] * 100}]
                )
            )
        )
        result = registry.call("hysim_run_power_flow", {})
        self.assertNotIn("raw", result)
        self.assertEqual(result["result_sizes"]["vm"], 100)

    def test_tool_arguments_are_checked_before_execution(self) -> None:
        registry = HySimToolRegistry(HySimClient(transport=FakeTransport([])))
        with self.assertRaisesRegex(ValueError, "unknown tool arguments"):
            registry.call("hysim_run_power_flow", {"bus_id": 7})
        with self.assertRaisesRegex(ValueError, "must have type integer"):
            registry.call("hysim_run_power_flow", {"max_iter": True})


if __name__ == "__main__":
    unittest.main()
