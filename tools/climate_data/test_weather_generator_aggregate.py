import unittest

import weather_generator_aggregate as aggregate


class WeatherGeneratorAggregateTest(unittest.TestCase):
    def test_summary_preserves_units_and_paired_reference_delta(self):
        rows = []
        for model, site, reference, candidate in (
            ("g1", "a", 0.1, 0.2),
            ("g1", "b", 0.2, 0.25),
            ("g2", "a", 0.3, 0.4),
        ):
            rows.extend(
                (
                    {
                        "model": model,
                        "site": site,
                        "variant": "reference",
                        "diagnostic_rank": 1,
                        "composite": reference,
                    },
                    {
                        "model": model,
                        "site": site,
                        "variant": "candidate",
                        "diagnostic_rank": 2,
                        "composite": candidate,
                    },
                )
            )
        summary = aggregate.summarize_units(rows, "reference")
        self.assertEqual(summary[0]["variant"], "reference")
        self.assertEqual(summary[0]["unit_rank_one_count"], 3)
        self.assertAlmostEqual(
            summary[1]["delta_from_reference"]["mean"], (0.1 + 0.05 + 0.1) / 3
        )
        self.assertEqual(summary[1]["by_model"]["g1"]["sites"], 2)


if __name__ == "__main__":
    unittest.main()
