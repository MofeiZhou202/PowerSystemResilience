import copy
import unittest

import numpy as np

import gcm_screen


class GcmScreenTest(unittest.TestCase):
    def test_screen_definition_requires_weights_sum_to_one(self):
        definition = gcm_screen.cd.load_json(gcm_screen.DEFAULT_DEFINITION)
        study = gcm_screen.cd.load_json(gcm_screen.cd.DEFAULT_STUDY)
        gcm_screen.validate_definition(definition, study)
        invalid = copy.deepcopy(definition)
        invalid["categories"]["marginal_distribution"]["weight"] = 0.4
        with self.assertRaises(gcm_screen.cd.ContractError):
            gcm_screen.validate_definition(invalid, study)

    def test_average_ranks_use_midrank_for_ties(self):
        ranks = gcm_screen.average_ranks(np.asarray([4.0, 1.0, 1.0, 9.0]))
        self.assertEqual(ranks.tolist(), [2.0, 0.5, 0.5, 3.0])

    def test_spearman_matrix_preserves_monotonic_dependence(self):
        values = np.asarray([[1.0, 9.0], [2.0, 7.0], [3.0, 4.0]])
        matrix = gcm_screen.spearman_matrix(values)
        self.assertAlmostEqual(float(matrix[0, 1]), -1.0)

    def test_model_ranking_is_deterministic_for_ties(self):
        self.assertEqual(
            gcm_screen.rank_models({"B": 0.2, "A": 0.2, "C": 0.1}),
            {"C": 1, "A": 2, "B": 3},
        )

    def test_reference_scored_against_itself_is_zero(self):
        months = np.tile(np.arange(1, 13), 2)
        years = np.repeat([2001, 2002], 12)
        base = np.arange(24, dtype=float)
        arrays = {
            "tasmax": base + 20.0,
            "tasmin": base + 10.0,
            "hurs": 40.0 + base,
            "sfcWind": 1.0 + base / 10.0,
            "rsds": 100.0 + base,
            "pr": np.mod(base, 5.0),
        }
        definition = gcm_screen.cd.load_json(gcm_screen.DEFAULT_DEFINITION)
        scores = gcm_screen.score_pair(arrays, arrays, months, years, definition)
        for score in scores.values():
            self.assertAlmostEqual(score, 0.0)


if __name__ == "__main__":
    unittest.main()
