"""Equal-energy identifiability, state and split tests, hydro theory §12."""
import copy
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/market_intelligence'))
from hydro_temporal import variant_boundary, validate_boundary, temporal_feature, TemporalGP
from hydro_physics import feature, physics_gain
from run_hydro_temporal import build_design


def fixture():
    load = (1000+200*np.cos(np.arange(98)*2*np.pi/96)).tolist()
    b = {'reservoirs':[],'generators':[],'external_schedules':[],'trades':[],
         'areas':[{'id':1,'load_mw':load}],
         'buses':[{'id':1,'area':1,'load_mw':load.copy(),'q_load_mvar':(np.array(load)*.2).tolist()}],
         'periods':[{'duration_hr':.25}]*98}
    for rid in range(1,13):
        b['generators'].append({'id':rid,'kind':'hydro'})
        b['reservoirs'].append({'id':rid,'generators':[rid],'source':'synthetic test',
            'upstream':-1,'lag_slots':1,'area_m2':1e7,'water_m3_mwh':3600.,
            'initial_level_m':100.,'min_level_m':[0.]*98,'max_level_m':[300.]*98,
            'inflow_m3_s':[150.]*98,'min_mwh':0.,'max_mwh':10000.,'release_history_m3_s':[0.]})
    b['generators'] += [dict(id=101,kind='thermal',pmax_mw=[100.]*98,pmin_mw=[0.]*98,
        initial_on=0,initial_power_mw=0.,initial_state_minutes=1440,min_up_minutes=60,
        min_down_minutes=60,ramp_up_mw_min=5.,ramp_down_mw_min=5.),
        dict(id=201,kind='wind',pmax_mw=[100.]*98,forecast_mw=[60.]*98,available=[1]*98,must_off=[0]*98)]
    for g in b['generators']:g['bus']=1
    b['branches']=[]
    return b


class TemporalTests(unittest.TestCase):
    def test_shape_preserves_node_energy_and_renewables_without_mutating_source(self):
        b=fixture(); before=copy.deepcopy(b); v=variant_boundary(b,1,0)
        self.assertEqual(b,before)
        for key in ('load_mw','q_load_mvar'):
            self.assertAlmostEqual(sum(v['buses'][0][key][:96]),sum(b['buses'][0][key][:96]),places=8)
            self.assertNotEqual(v['buses'][0][key][:96],b['buses'][0][key][:96])
        self.assertEqual(v['generators'],b['generators'])
        validate_boundary(v)

    def test_condition_has_valid_initial_obligation_and_same_capacity(self):
        b=fixture(); v=variant_boundary(b,0,1); g=v['generators'][-2]
        self.assertEqual(g['pmax_mw'],b['generators'][-2]['pmax_mw'])
        self.assertEqual(g['initial_power_mw'],20.)
        self.assertEqual(g['min_up_minutes']-g['initial_state_minutes'],225)
        self.assertEqual(g['ramp_up_mw_min'],1.25)
        validate_boundary(v)

    def test_incoherent_load_state_and_unknown_variant_rejected(self):
        b=fixture(); b['areas'][0]['load_mw'][0]+=1
        with self.assertRaises(ValueError):validate_boundary(b)
        b=fixture(); b['generators'][-2]['initial_power_mw']=5
        with self.assertRaises(ValueError):validate_boundary(b)
        with self.assertRaises(ValueError):variant_boundary(fixture(),2,0)

    def test_full_factorial_grouping_and_action_conservation(self):
        design=build_design(fixture(),{'solver_options':{}},'test')
        self.assertEqual(len(design['specs']),72)
        for root in range(6):
            rows=[s for s in design['specs'] if s['root_family']==root]
            self.assertEqual(len(rows),12)
            self.assertEqual(len({s['split'] for s in rows}),1)
            self.assertEqual(len({s['family'] for s in rows}),4)
            for s in rows:
                self.assertEqual(sum(s['quotas']['1']),21000.)
                self.assertEqual(s['terminal'],rows[0]['terminal'])

    def test_temporal_coordinates_resolve_daily_coordinate_collision(self):
        design=build_design(fixture(),{'solver_options':{}},'test')
        daily=[]; temporal=[]
        for i in (1,4,7,10):
            s=design['specs'][i]; b=design['boundaries'][s['boundary_key']]
            daily.append(feature(b,s)); temporal.append(temporal_feature(b,s))
        np.testing.assert_allclose(daily,np.tile(daily[0],(4,1)),atol=1e-9)
        for i in range(4):
            for j in range(i):self.assertGreater(np.linalg.norm(temporal[i]-temporal[j]),1e-3)

    def test_gp_matches_scalar_posterior_and_saved_zero_invariant(self):
        x=np.array([[0,0,1,1,1,.4]])
        gp=TemporalGP().fit(x,[-250.])
        self.assertAlmostEqual(gp.predict(x)[0],-300+2500/2525*50,places=10)
        z=x.copy();z[0,2]=0
        self.assertEqual(gp.predict(z)[0],0.)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'gp.npz';gp.save(path)
            np.testing.assert_allclose(TemporalGP.load(path).predict(x),gp.predict(x),atol=1e-12)
        with self.assertRaises(ValueError):gp.predict([[float('nan')]*6])

    def test_singleton_cut_matches_analytic_import_shortfall(self):
        from hydro_temporal import local_thermal_floor
        b=fixture()
        for g in b['generators']:g['bus']=2 if g['kind']=='thermal' else 1
        b['buses'].append({'id':2,'area':1,'load_mw':[20.]*98,'q_load_mvar':[0.]*98})
        b['areas'][0]['load_mw']=[p+20 for p in b['areas'][0]['load_mw']]
        b['branches']=[{'id':1,'from_bus':1,'to_bus':2,'min_mw':[-10.]*98,'max_mw':[10.]*98,'available':[1]*98}]
        spec={'external_days':[{'load_scale':1.,'line_limit_scale':1.}]*8}
        np.testing.assert_allclose(local_thermal_floor(b,spec),[240.]*7)
        b['storage']=[{'bus':2}]
        np.testing.assert_allclose(local_thermal_floor(b,spec),[0.]*7)

    def test_positive_benefit_gate_and_collision_floor(self):
        from analyze_hydro_temporal import evaluate,collision_audit
        score=evaluate([100,99,200,-100],[99,0,210,0])
        self.assertEqual((score['positive_cases'],score['positive_misses'],score['positive_direction_misses']),(2,1,0))
        specs=[];pairs=[]
        for root in range(6):
            for delta in (-300,300):
                for k,value in enumerate((0,0,100,100)):
                    specs.append({'root_family':root,'split':'train' if root<4 else 'test','boundary_key':f'{k//2}-{k%2}'})
                    pairs.append({'index':len(specs)-1,'delta_mwh':delta,'gain_mwh':value})
        audit=collision_audit(pairs,specs)
        self.assertEqual(audit['groups'][0]['minimum_empirical_daily_only_mae_mwh'],50.)

    def test_online_contract_accepts_declared_action_and_rejects_hidden_changes(self):
        from predict_hydro_temporal import validate_candidate
        from run_price_oracle import digest
        d=build_design(fixture(),{'solver_options':{}},'test')
        frozen={'boundary_hashes':{k:digest(b) for k,b in d['boundaries'].items()},
                'operation_contract':{k:v for k,v in d['specs'][0]['config'].items() if k not in ('days','reference_days')}}
        spec=d['specs'][7];base=d['boundaries'][spec['boundary_key']]
        validate_candidate(base,spec,d,frozen)
        bad=copy.deepcopy(spec);bad['quotas']['1'][2]+=10
        with self.assertRaises(ValueError):validate_candidate(base,bad,d,frozen)
        bad=copy.deepcopy(spec);bad['external_days'][0]['hidden']=1
        with self.assertRaises(ValueError):validate_candidate(base,bad,d,frozen)

    def test_complete_context_detects_omitted_lookahead_and_other_day_line(self):
        from hydro_complete_context import complete_feature,recover_factors,FACTORS
        from hydro_temporal import network_feature
        design=build_design(fixture(),{'solver_options':{}},'test')
        s=design['specs'][1];b=design['boundaries'][s['boundary_key']]
        original=network_feature(b,s);full=complete_feature(b,s)
        expected=np.array([[day[k] for k in FACTORS] for day in s['external_days']])
        np.testing.assert_allclose(recover_factors(full),expected,atol=1e-12)
        for day,key in ((7,'wind_scale'),(0,'line_limit_scale')):
            changed=copy.deepcopy(s);changed['external_days'][day][key]+=.05
            np.testing.assert_allclose(network_feature(b,changed),original,atol=1e-12)
            self.assertGreater(np.linalg.norm(complete_feature(b,changed)-full),1e-4)

    def test_external_shape_override_is_not_silently_ignored(self):
        d=build_design(fixture(),{'solver_options':{}},'test');s=d['specs'][0]
        s['external_days'][0]['boundary_overrides']=[{'table':'areas'}]
        with self.assertRaises(ValueError):temporal_feature(d['boundaries']['0-0'],s)


if __name__=='__main__':unittest.main()
