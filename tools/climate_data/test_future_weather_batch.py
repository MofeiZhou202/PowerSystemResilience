import unittest

import climate_data as cd
import future_weather_batch as batch


class FutureWeatherBatchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.study = cd.load_json(cd.DEFAULT_STUDY)
        cls.definition = cd.load_json(batch.DEFAULT_DEFINITION)

    def test_frozen_future_contract_and_full_task_count(self):
        batch.validate_definition(self.definition, self.study)
        tasks = batch.build_tasks(self.definition, self.study)
        self.assertEqual(len(tasks), 96)
        self.assertEqual(len({task.model for task in tasks}), 6)
        self.assertEqual(len({task.site for task in tasks}), 4)
        self.assertEqual(len({task.experiment for task in tasks}), 2)
        self.assertEqual(len({task.method_id for task in tasks}), 2)

    def test_filtered_batch_preserves_declared_order(self):
        tasks = batch.build_tasks(
            self.definition,
            self.study,
            models=["FGOALS-g3"],
            experiments=["ssp585"],
            sites=["xian"],
        )
        self.assertEqual(len(tasks), 2)
        self.assertEqual(tasks[0].method_id, "seasonal_var_block_ecdf_v2")
        self.assertEqual(
            tasks[1].method_id,
            "seasonal_multivariate_knn_analog_k10_frozen_v1",
        )

    def test_unknown_filter_is_rejected(self):
        with self.assertRaises(cd.ContractError):
            batch.build_tasks(
                self.definition, self.study, models=["not-a-declared-model"]
            )


if __name__ == "__main__":
    unittest.main()
