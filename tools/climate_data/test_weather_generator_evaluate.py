import copy
import unittest

import numpy as np

import weather_generator as wg
import weather_generator_evaluate as evaluate


class WeatherGeneratorEvaluateTest(unittest.TestCase):
    def setUp(self):
        self.study = evaluate.cd.load_json(evaluate.cd.DEFAULT_STUDY)
        self.definition = evaluate.cd.load_json(evaluate.DEFAULT_DEFINITION)
        self.score_definition = evaluate.cd.load_json(
            evaluate.gcm_screen.DEFAULT_DEFINITION
        )

    def test_definition_requires_complete_block_partition(self):
        evaluate.validate_definition(self.definition, self.study, self.score_definition)
        invalid = copy.deepcopy(self.definition)
        invalid["cross_validation"]["folds"] = 5
        with self.assertRaises(evaluate.cd.ContractError):
            evaluate.validate_definition(invalid, self.study, self.score_definition)

    def test_outer_evaluation_is_hash_locked_to_declared_outer_gcms(self):
        outer_path = (
            evaluate.cd.TOOL_DIR
            / "manifests/weather_generator_knn_outer_evaluation_calendar_amended.json"
        )
        outer = evaluate.cd.load_json(outer_path)
        evaluate.validate_definition(outer, self.study, self.score_definition)
        self.assertEqual(outer["gcm_access"]["model_set"], "outer")
        invalid = copy.deepcopy(outer)
        invalid["gcm_access"]["allowed_gcms"] = invalid["gcm_access"][
            "allowed_gcms"
        ][:-1]
        with self.assertRaises(evaluate.cd.ContractError):
            evaluate.validate_definition(invalid, self.study, self.score_definition)

    def test_monthly_iid_is_reproducible_and_retains_constraints(self):
        _, months = wg.noleap_dates(2001, 2002)
        rng = np.random.default_rng(11)
        tasmax = rng.normal(20.0, 5.0, months.size)
        values = np.column_stack(
            (
                tasmax,
                5.0 + rng.random(months.size),
                30.0 + 50.0 * rng.random(months.size),
                rng.random(months.size) * 4.0,
                rng.random(months.size) * 300.0,
                np.where(rng.random(months.size) < 0.7, 0.0, rng.random(months.size)),
            )
        )
        baseline = evaluate.cd.load_json(wg.DEFAULT_DEFINITION)
        fitted = wg.fit_generator(values, months, baseline)
        first = evaluate.sample_monthly_iid(fitted, months, 4)
        second = evaluate.sample_monthly_iid(fitted, months, 4)
        for name in wg.OUTPUT_VARIABLES:
            np.testing.assert_array_equal(first[name], second[name])
        self.assertTrue(all(wg.output_diagnostics(first)["checks"].values()))

    def test_summary_equal_weights_folds_and_ranks_lower_score_first(self):
        variants = [
            {"id": "full", "description": "full"},
            {"id": "other", "description": "other"},
        ]
        rows = []
        for variant, values in (("full", [0.2, 0.4]), ("other", [0.3, 0.7])):
            for fold, value in zip((1985, 1990), values):
                rows.append(
                    {
                        "variant": variant,
                        "fold_start_year": fold,
                        "composite": value,
                    }
                )
        summary = evaluate.summarize_rows(rows, variants, ["composite"])
        self.assertEqual(summary[0]["variant"], "full")
        self.assertAlmostEqual(summary[1]["composite_delta_from_full"]["mean"], 0.2)


if __name__ == "__main__":
    unittest.main()
