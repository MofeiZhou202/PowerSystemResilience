"""Independent small numerical oracles for the empirical transfer protocol."""

import importlib.util
import csv
import io
from pathlib import Path
import tempfile
import unittest
import zipfile

spec = importlib.util.spec_from_file_location("bids", Path(__file__).resolve().parents[1]/"tools/market_bid_empirical.py")
bids = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bids)


class EmpiricalBidTests(unittest.TestCase):
    def snapshot(self, duplicate=False, future=False):
        common = {"SETTLEMENTDATE": "2026/08/28 00:00:00", "DUID": "TEST", "BIDTYPE": "ENERGY",
                  "DIRECTION": "GEN", "BIDSETTLEMENTDATE": "2026/08/27 00:00:00", "OFFERDATE": "2026/08/27 10:00:00"}
        daily = {**common, "PARTICIPANTID": "OWNER", **{f"PRICEBAND{k}": str(k-5) for k in range(1, 11)}}
        period = {**common, "BIDSETTLEMENTDATE": "2026/08/28 00:00:00", "PERIODID": "96", "MAXAVAIL": "100",
                  "INTERVAL_DATETIME": "2026/08/28 12:00:00", **{f"BANDAVAIL{k}": "10" for k in range(1, 11)}}
        if future:
            period["OFFERDATE"] = "2026/08/28 12:01:00"
        stream = io.StringIO()
        writer = csv.writer(stream)
        for table, record in (("BIDDAYOFFER_D", daily), ("BIDPEROFFER_D", period)):
            writer.writerow(["I", "BID", table, "3", *record])
            writer.writerow(["D", "BID", table, "3", *record.values()])
            if duplicate:
                writer.writerow(["D", "BID", table, "3", *record.values()])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"snapshot.zip"
            with zipfile.ZipFile(path, "w") as archive:
                archive.writestr("snapshot.CSV", stream.getvalue())
            return bids.parse_archive(path)

    def test_official_applicable_date_join_allows_carried_submission(self):
        parsed = self.snapshot()
        self.assertEqual(parsed["records"]["GEN:TEST"]["capacity_mw"], 100)
        self.assertEqual(parsed["counts"]["different_submitted_dates"], 1)

    def test_duplicate_versions_and_future_offer_are_rejected(self):
        for kwargs in ({"duplicate": True}, {"future": True}):
            with self.assertRaises(ValueError):
                self.snapshot(**kwargs)

    def test_signed_prices_availability_and_exact_integral(self):
        # 30 MW at -10 and only 20 of 70 MW at +20 can be sold.
        result = bids.curve([-10, 20], [30, 70], 50)
        self.assertEqual(result["prices_aud_mwh"], [-10]*6 + [20]*4)
        self.assertEqual(result["negative_capacity_fraction"], .6)
        self.assertEqual(sum(result["prices_aud_mwh"])*5, 100)

    def test_quantile_mapping_reports_partial_dispatch_distortion(self):
        # A single bin covers both prices: its integral is exact, its shape is not.
        result = bids.curve([0, 100], [3, 97], 100)
        self.assertEqual(result["prices_aud_mwh"][0], 70)
        self.assertAlmostEqual(result["mapping_mae_aud_mwh"], 4.2)
        self.assertAlmostEqual(sum(result["prices_aud_mwh"])*10, 9700)

    def test_load_keeps_high_willingness_to_pay_first(self):
        result = bids.curve([-10, 20], [30, 70], 50, "LOAD")
        self.assertEqual(result["prices_aud_mwh"], [20]*10)
        self.assertEqual(result["negative_capacity_fraction"], 0)

    def test_noninteger_bins_preserve_constant_price_monotonicity(self):
        result = bids.curve([22190.7], [1575], 1084)
        prices = result["prices_aud_mwh"]
        self.assertTrue(all(a <= b for a, b in zip(prices, prices[1:])))
        self.assertTrue(all(abs(p-22190.7) < 1e-9 for p in prices))

    def test_invalid_and_unavailable_offers(self):
        self.assertIsNone(bids.curve([5], [0], 10))
        self.assertIsNone(bids.curve([5], [10], 0))
        for args in (([20, 10], [1, 1], 2), ([5], [-1], 1), ([float("nan")], [1], 1)):
            with self.assertRaises(ValueError):
                bids.curve(*args)

    def test_common_factor_cannot_reverse_order_or_sign(self):
        self.assertEqual(bids.positive_factor([1, 2], [2, 4]), 2)
        self.assertEqual(bids.positive_factor([1, 2], [-1, -2]), 0)
        self.assertGreater(bids.score([2, 1], [bids.positive_factor([1, 2], [2, 1])*p for p in [1, 2]])["mae_aud_mwh"], 0)
        with self.assertRaises(ValueError):
            bids.positive_factor([0, 0], [1, 2])

    def test_merit_negative_cost_and_nondifferentiable_price(self):
        # Currency scale /100; 10 MW at -1, next 10 MW at +2.
        result = bids.merit([[-100, 200]], 10)
        self.assertEqual(result["day_cost"], -240)
        self.assertEqual(result["price_interval"], [-1, 2])
        result = bids.merit([[-100, 200]], 15)
        self.assertEqual(result["day_cost"], 0)
        self.assertEqual(result["price_interval"], [2, 2])


if __name__ == "__main__":
    unittest.main()
