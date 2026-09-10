"""Analytic cases for energy relaxation and GP, theory §10."""
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools/market_intelligence'))
from hydro_physics import renewable_energy, daily_surplus, physics_gain, PhysicsGP


class EnergyPhysicsTests(unittest.TestCase):
    def test_cases_cover_zero_active_and_crossing(self):
        x = np.array([[-2, -2, 1, 1, 1], [-2, 2, -1, 1, 1],
                      [2, -2, -1, 1, 1], [-.2, .1, -1, 1, 1],
                      [-.2, .1, 0, 1, 1]])
        np.testing.assert_allclose(physics_gain(x), [0, 300, -300, -210, 0], atol=1e-10)

    def test_availability_caps_each_slot_and_excludes_unavailable_units(self):
        g = {'kind': 'solar', 'forecast_mw': [80.]*96, 'pmax_mw': [100.]*96,
             'available': [1]*48+[0]*48, 'must_off': [0]*96}
        b = {'generators': [g]}
        self.assertEqual(renewable_energy(b, {'solar_scale': 2.}), 1200.)
        g['must_off'] = [1]*96
        self.assertEqual(renewable_energy(b, {'solar_scale': 2.}), 0.)

    def test_missing_load_and_external_trade_fail_loudly(self):
        b = {'areas': [{'load_mw': [0.]*96}], 'external_schedules': [], 'trades': [], 'generators': []}
        spec = {'external_days': [{'load_scale': 1.}]*8, 'weekly': {'1': 21000.}}
        with self.assertRaises(ValueError):
            daily_surplus(b, spec)
        b['areas'][0]['load_mw'] = [100.]*96
        b['trades'] = [{'id': 1}]
        with self.assertRaises(ValueError):
            daily_surplus(b, spec)

    def test_single_observation_gp_matches_scalar_conditional_formula(self):
        x = np.array([[0., 0., 1., 1., 1.]])
        gp = PhysicsGP().fit(x, np.array([-250.]))
        mean, std = gp.predict(x, return_std=True)
        self.assertAlmostEqual(mean[0], -300+2500/2525*50, places=10)
        self.assertAlmostEqual(std[0]**2, 2500-2500**2/2525, places=9)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'gp.npz'
            gp.save(path)
            loaded = PhysicsGP.load(path)
            np.testing.assert_allclose(loaded.predict(x), mean, atol=1e-12)
        self.assertEqual(gp.predict([[0., 0., 0., 1., 1.]])[0], 0.)

    def test_nonfinite_features_rejected(self):
        with self.assertRaises(ValueError):
            physics_gain([[float('nan'), 0, 0, 0, 0]])

    def test_selection_uses_prediction_and_prefers_baseline_ties(self):
        from analyze_hydro_transition import selection, evaluate
        rows = [dict(family=2, delta_mwh=delta, predicted_gain_mwh=0., actual_gain_mwh=gain)
                for delta, gain in [(-300,300), (0,0), (300,-300)]]
        result = selection(rows)[0]
        self.assertEqual(result['selected_delta_mwh'], 0)
        self.assertEqual(result['regret_mwh'], 300)
        score = evaluate(np.array([300., -300., 0.]), np.array([0., 100., 10.]))
        self.assertEqual(score['meaningful_cases'], 2)
        self.assertEqual(score['meaningful_misses'], 2)
        self.assertEqual(score['zero_response_mae_mwh'], 10.)

    def test_nan_forecast_cannot_hide_behind_capacity_cap(self):
        g = {'kind':'solar', 'forecast_mw':[float('nan')]*96, 'pmax_mw':[100.]*96,
             'available':[1]*96, 'must_off':[0]*96}
        with self.assertRaises(ValueError):
            renewable_energy({'generators':[g]}, {'solar_scale':1.})

    def test_local_inference_rejects_unsupported_action_and_quota(self):
        from predict_hydro import prepare_candidate
        for delta in (301., -301., float('nan')):
            with self.assertRaises(ValueError):
                prepare_candidate({}, {}, delta)
        with self.assertRaises(ValueError):
            prepare_candidate({}, {'weekly': {'1': 22000.}}, 100.)

    def test_transition_design_balances_independent_families_and_conserves_actions(self):
        from run_hydro_transition import build_design
        b = {'reservoirs': [], 'generators': [], 'external_schedules': [], 'trades': [],
             'areas':[{'load_mw':[1000.]*98}], 'periods':[{'duration_hr':.25}]*98}
        for rid in range(1,13):
            b['generators'].append({'id':rid,'kind':'hydro'})
            b['reservoirs'].append({'id':rid,'generators':[rid],'source':'synthetic test',
                'upstream':-1,'lag_slots':1,'area_m2':1e7,'water_m3_mwh':3600.,
                'initial_level_m':100.,'min_level_m':[0.]*98,'max_level_m':[300.]*98,
                'inflow_m3_s':[150.]*98,'min_mwh':0.,'max_mwh':10000.,'release_history_m3_s':[0.]})
        b['generators'].append({'id':201,'kind':'wind','pmax_mw':[100.]*98,
            'forecast_mw':[60.]*98,'available':[1]*98,'must_off':[0]*98})
        design = build_design(b, {'solver_options':{}}, 'test-binary')
        self.assertEqual(len(design['specs']),36)
        train={s['family'] for s in design['specs'] if s['split']=='train'}
        test={s['family'] for s in design['specs'] if s['split']=='test'}
        self.assertEqual((len(train),len(test)),(8,4))
        self.assertFalse(train & test)
        for s in design['specs']:
            self.assertEqual(sum(s['quotas']['1']),21000.)
            self.assertEqual(s['quotas']['1'][1]+s['quotas']['1'][4],6000.)
            r = daily_surplus(b,s)
            np.testing.assert_allclose(r[[1,4]],s['target_r2_r5_mwh'],atol=1e-8)
            self.assertNotIn('boundary_overrides',s['config']['days'][7])


if __name__ == '__main__':
    unittest.main()
