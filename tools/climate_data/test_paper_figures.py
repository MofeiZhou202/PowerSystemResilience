import unittest

import climate_data as cd
import paper_figures as figures


class PaperFiguresTest(unittest.TestCase):
    def test_pairing_retains_keys_and_computes_knn_minus_var(self):
        rows = [
            {
                "model": "M",
                "experiment": "ssp245",
                "site": "xian",
                "member": 0,
                "method": figures.VAR_METHOD,
                "metric": 2.0,
            },
            {
                "model": "M",
                "experiment": "ssp245",
                "site": "xian",
                "member": 0,
                "method": figures.KNN_METHOD,
                "metric": 1.5,
            },
        ]
        paired = figures.paired_rows(
            rows, ("model", "experiment", "site", "member"), ("metric",)
        )
        self.assertEqual(len(paired), 1)
        self.assertEqual(paired[0]["delta_metric"], -0.5)

    def test_incomplete_pair_is_rejected(self):
        rows = [
            {
                "model": "M",
                "experiment": "ssp245",
                "site": "xian",
                "member": 0,
                "method": figures.VAR_METHOD,
                "metric": 2.0,
            }
        ]
        with self.assertRaises(cd.ContractError):
            figures.paired_rows(
                rows,
                ("model", "experiment", "site", "member"),
                ("metric",),
            )


if __name__ == "__main__":
    unittest.main()
