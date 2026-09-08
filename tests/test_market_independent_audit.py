"""No solver/data dependency: validate the independent checker against hand cases."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"tools/market_validation"))
from independent_audit import audit, enumerate_uc, indexed, audit_completed_hour


def fixture():
    b = {k: [] for k in ("generators", "buses", "branches", "reservoirs", "storage", "controllable_loads", "dc_links", "dc_hubs", "external_schedules", "trades")}
    b["periods"] = [{"duration_hr": .25, "weight_hr": .25} for _ in range(98)]
    b["buses"] = [{"id": 42, "load_mw": [10]*98}]
    b["generators"] = [{"id": 9, "bus": 42, "kind": "thermal", "pmax_mw": [20]*98, "pmin_mw": [0]*98,
                         "available": [1]*98, "must_off": [0]*98, "startup_curves_mw": [[], [], []], "shutdown_curve_mw": [],
                         "segments": [{"quantity_mw": 20, "price_per_mwh": -3}]}]
    s = {k: [] for k in ("generators", "buses", "branches", "reservoirs", "storage", "controllable_loads")}
    s["generators"] = [{"id": 9, "power_mw": [10]*98, "online": [1]*98, "primary_reserve_mw": [0]*98}]
    s["buses"] = [{"id": 42, "deficit_mw": [0]*98, "surplus_mw": [0]*98}]
    s.update(day_generation_mwh=240, day_energy_bid_cost=-720)
    return b, {"schedule_feasible": True, "sced": s}


class IndependentAuditTests(unittest.TestCase):
    def test_negative_bid_full_day_hand_integral(self):
        b, r = fixture()
        self.assertTrue(audit(b, r)["passed"])

    def test_two_outlook_points_cannot_enter_realized_energy(self):
        b, r = fixture(); r["sced"]["day_generation_mwh"] = 245
        self.assertFalse(audit(b, r)["passed"])

    def test_false_success_flags_do_not_override_physics(self):
        b, r = fixture(); r["sced"]["generators"][0]["power_mw"][5] = 11
        r.update(feasible=True, max_residual=0)
        self.assertFalse(audit(b, r)["passed"])

    def test_nonfinite_missing_and_duplicate_ids(self):
        b, r = fixture()
        for value in (None, float("nan"), float("inf")):
            corrupt = copy.deepcopy(r); corrupt["sced"]["generators"][0]["power_mw"][0] = value
            self.assertFalse(audit(b, corrupt)["passed"])
        r["sced"]["generators"].append(copy.deepcopy(r["sced"]["generators"][0]))
        self.assertFalse(audit(b, r)["passed"])

    def test_uncovered_component_must_not_silently_pass(self):
        b, r = fixture(); b["dc_links"] = [{"id": 1}]
        self.assertFalse(audit(b, r)["passed"])

    def test_enumeration_startup_and_minimum_up(self):
        r = enumerate_uc([50], [100, 100], [1, 4], [5, 2], [1, 1])
        self.assertEqual(r["objective"], 17.5)
        r = enumerate_uc([50], [100, 100], [1, 4], [5, 2], [2, 1])
        self.assertEqual(r["objective"], 52)
        self.assertEqual(r["enumerated"], 4)

    def test_nonzero_no_load_prevents_free_commitment(self):
        r = enumerate_uc([50], [100, 100], [1, 4], [5, 2], [1, 1], [200, 0])
        self.assertEqual(r["objective"], 52)
        self.assertEqual(r["commitment"], (0, 1))

    def test_infeasible_enumeration_and_duplicate_identity_reject(self):
        with self.assertRaises(ValueError):
            enumerate_uc([300], [100, 100], [1, 4], [5, 2], [1, 1])
        with self.assertRaises(ValueError):
            indexed([{"id": 42}, {"id": 42}])

    def test_hour_price_reconstructed_from_executed_quarters(self):
        job = {"runs": [{"start_minute": 15*q, "executed_points": 3,
                         "dispatch": {"prices_valid": True, "lmp": {"buses": [
                             {"id": 42, "lmp_per_mwh": [p]+[999]*7}]}}}
                        for q, p in enumerate([-10, 0, 10, 40])],
               "hourly_prices": [{"bus_id": 42, "hour": 0, "observed_quarters": 4, "price_per_mwh": 10}]}
        self.assertTrue(audit_completed_hour(job, [42])["passed"])
        for field in ("runs", "hourly_prices"):
            bad = copy.deepcopy(job); bad[field] = []
            self.assertFalse(audit_completed_hour(bad, [42])["passed"])
        bad = copy.deepcopy(job); bad["runs"][1]["start_minute"] = 0
        self.assertFalse(audit_completed_hour(bad, [42])["passed"])
        for value in (None, float("nan"), 999):
            bad = copy.deepcopy(job); bad["hourly_prices"][0]["price_per_mwh"] = value
            self.assertFalse(audit_completed_hour(bad, [42])["passed"])


if __name__ == "__main__":
    unittest.main()
