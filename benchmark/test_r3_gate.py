"""Adversarial acceptance tests for the fixed R3 evidence contract."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import r3_gate as gate


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        archived = gate.ROOT/'benchmark/r3_reference'
        self.data = gate.load(archived/'block-01-t4.json')
        # Schema migration fixture uses archived real measurements, never release evidence.
        self.factors = gate.load(archived/'block-01-t4.factors.json')
        self.recoveries = gate.load(archived/'block-01-t4.recoveries.json')
        for record in self.factors + self.recoveries:
            record['schema_version'] = 2
        for record in self.factors:
            record['adaptive_backend_overflow'] = False

    def log(self):
        path = self.folder/'measurements.log'
        path.write_text(''.join('LP-FACTOR '+json.dumps(r)+'\n' for r in self.factors) +
                        ''.join('LP-RECOVERY '+json.dumps(r)+'\n' for r in self.recoveries))
        return path

    def validate(self):
        rows = gate.accuracy(self.data, gate.CASES, 4, 'Native-IPM[centrality-step,direct]')
        return gate.telemetry(self.log(), rows, 4, True)

    def test_complete_telemetry(self):
        self.assertEqual(len(self.validate()), 1)

    def test_legacy_schema(self):
        self.factors[0].pop('schema_version')
        with self.assertRaises(RuntimeError):
            self.validate()

    def test_nonfinite_timing(self):
        self.factors[0]['numeric_ms'] = float('nan')
        with self.assertRaises(RuntimeError):
            self.validate()

    def test_boolean_is_not_a_measurement(self):
        self.recoveries[0]['seed_capture_ms'] = True
        with self.assertRaises(RuntimeError):
            self.validate()

    def test_missing_metric_is_not_null(self):
        self.recoveries[0].pop('source_relative_gap')
        with self.assertRaises(RuntimeError):
            self.validate()

    def test_backend_overflow(self):
        self.factors[0]['adaptive_backend_overflow'] = True
        with self.assertRaises(RuntimeError):
            self.validate()

    def test_missing_backend(self):
        self.factors[0].pop('adaptive_backend_sequence')
        with self.assertRaises(RuntimeError):
            self.validate()

    def test_unpaired_recovery(self):
        self.recoveries.clear()
        with self.assertRaises(RuntimeError):
            self.validate()

    def test_duplicate_transition(self):
        self.factors.append(copy.deepcopy(self.factors[2]))
        self.recoveries.append(copy.deepcopy(self.recoveries[0]))
        with self.assertRaises(ValueError):
            self.validate()

    def test_timer_partition(self):
        self.factors[0]['other_ms'] += 1
        with self.assertRaises(RuntimeError):
            self.validate()

    def test_timer_containment(self):
        self.recoveries[0]['source_runtime_ms'] *= 2
        with self.assertRaises(ValueError):
            self.validate()

    def test_cap_not_observation(self):
        rows = {r['case']: r for r in self.data['runs']}
        with self.assertRaises(ValueError):
            gate.telemetry(self.log(), rows, 4, False)

    def test_inaccurate_despite_flag(self):
        self.data['runs'][0]['relative_gap'] = 1
        with self.assertRaises(ValueError):
            self.validate()

    def test_duplicate_run(self):
        self.data['runs'][0] = copy.deepcopy(self.data['runs'][1])
        with self.assertRaises(ValueError):
            self.validate()

    def test_summary_not_process_evidence(self):
        with self.assertRaises(ValueError):
            gate.stability(self.folder, 'a'*64, {}, False, set())

    def test_overlapping_processes(self):
        with self.assertRaises(ValueError):
            gate.ordered([dict(started_ns=1, finished_ns=4), dict(started_ns=3, finished_ns=6)])

    def test_duplicate_json(self):
        path = self.folder/'duplicate.json'
        path.write_text('{"schema_version": 1, "schema_version": 2}')
        with self.assertRaises(ValueError):
            gate.load(path)

    def test_wrong_sign_and_original_interval(self):
        contract = gate.load(gate.ROOT/'benchmark/windows_lp_r3_contract.json')
        archive = gate.load(gate.ROOT/contract['references']['recovery']['path'])
        samples = {'2': {'greenbea': [2551.] * 20}, '4': {'greenbea': [2835.] * 20}}
        records = {'2': copy.deepcopy(self.recoveries) * 20, '4': copy.deepcopy(self.recoveries) * 20}
        failures = []
        gate.greenbea(samples, records, archive, contract, failures, {})
        self.assertTrue(any('wrong direction' in f for f in failures))
        self.assertTrue(any('outside fixed' in f for f in failures))

    def test_single_case_cannot_hide_in_aggregate(self):
        samples = {'2': {'a': [120.] * 20, 'b': [10.] * 20}}
        reference = {'2': dict(cases={'a': dict(median=100., p95=100.),
                                     'b': dict(median=100., p95=100.)},
                               aggregate=dict(median=200., p95=200.))}
        limits = dict(case_limits=dict(median_ratio=1.1, p95_ratio=1.1),
                      aggregate_limits=dict(median_ratio=1.05, p95_ratio=1.1))
        failures = []
        gate.compare(samples, reference, limits, failures, {}, 'adversarial')
        self.assertEqual(len(failures), 2)
        self.assertTrue(all('.a.' in f for f in failures))


if __name__ == '__main__':
    unittest.main()
