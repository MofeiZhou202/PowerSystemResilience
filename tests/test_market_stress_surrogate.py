"""Input pooling and predeclared sampling tests; requires market intelligence requirements."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import joblib
import numpy as np
from sklearn.dummy import DummyRegressor

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools/market_intelligence'))
from build_market_label_dataset import FACTORS
from run_price_oracle import apply_design, STRESS_RANGES
from train_stress_surrogate import matrix, predict


class StressTests(unittest.TestCase):
    def row(self):
        return {'case_hash':'test', 'features':{f'd{d}.{f}':1. for d in range(8) for f in FACTORS}}

    def test_input_pooling_excludes_lookahead_from_execution_mean(self):
        row=self.row()
        row['features']['d7.load_scale']=2.
        a=matrix([row],'pooled')
        self.assertEqual(a.shape,(1,98))
        self.assertEqual(a[0,56],1.)
        self.assertEqual(a[0,91],2.)
        row['price']={'mean':99999}
        np.testing.assert_array_equal(a,matrix([row],'pooled'))

    def test_daily_change(self):
        row=self.row();row['features']['d3.load_scale']=3.
        a=matrix([row],'pooled')
        self.assertEqual(a[0,56+28],2.)
        self.assertEqual(a[0,56+21],3.)

    def test_split_is_fixed_and_all_regimes_present(self):
        counts={g:{'train':0,'test':0} for g in STRESS_RANGES}
        cfg={'marginals':[{'factor':f,'distribution':'fixed','lower':.9,'upper':1.1,'center':[1.]*8} for f in FACTORS]}
        for i in range(32):
            modified=copy.deepcopy(cfg)
            meta=apply_design(modified,i,'stress')
            counts[meta['regime']][meta['split']]+=1
            for m in modified['marginals']:
                self.assertTrue(all(m['lower']<=v<=m['upper'] for v in m['center']))
        self.assertTrue(all(v=={'train':6,'test':2} for v in counts.values()))

    def test_union_of_strata_is_not_bounding_box(self):
        row=self.row()
        model=DummyRegressor().fit(np.ones((2,56)),[1.,2.])
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp)
            joblib.dump({'case_hash':'test','support_strata':STRESS_RANGES,'threshold':59.,
                         'models':{'test':{'mode':'raw','model':model,'pilot_gate':False}}},tmp/'m.joblib')
            p=tmp/'input.json';p.write_text(json.dumps(row))
            self.assertEqual(predict(tmp/'m.joblib',p)['support_strata'],['ordinary'])
            for d in range(8):row['features'][f'd{d}.load_scale']=1.5
            p.write_text(json.dumps(row))
            with self.assertRaisesRegex(ValueError,'Outside sampled strata'):
                predict(tmp/'m.joblib',p)
            row=self.row();row['features']['unexpected']=1.
            p.write_text(json.dumps(row))
            with self.assertRaisesRegex(ValueError,'feature schema'):
                predict(tmp/'m.joblib',p)


if __name__=='__main__':unittest.main()
