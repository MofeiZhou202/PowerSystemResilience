#!/usr/bin/env python3
"""Replay heldout predictions and reject outside-domain inputs; no model fitting."""
import argparse
import json
from pathlib import Path
import tempfile

from train_stress_surrogate import predict


def validate(folder):
    rows=[json.loads(x) for x in (folder/'all_labels.jsonl').read_text().splitlines()]
    heldout=json.loads((folder/'test_predictions.json').read_text())
    sample=next(r for r in rows if r['split']=='test' and r['train_eligible'])
    payload={k:sample[k] for k in ('sample_id','case_hash','features')}
    path=folder/'example-input.json'
    path.write_text(json.dumps(payload,indent=2)+'\n')
    output=predict(folder/'model.joblib',path)
    expected={p['target']:p['prediction'] for p in heldout if p['sample_id']==sample['sample_id']}
    actual={k:v['value'] for k,v in output['predictions'].items()}
    if expected != actual:
        raise AssertionError('Reloaded inference differs from heldout predictions')
    (folder/'example-prediction.json').write_text(json.dumps(output,indent=2)+'\n')
    checks={'heldout_predictions_exact':True}
    with tempfile.TemporaryDirectory() as tmp:
        testpath=Path(tmp)/'input.json'
        for key in ('wrong_case','outside_strata','missing_feature'):
            bad=json.loads(json.dumps(payload))
            if key=='wrong_case':bad['case_hash']='wrong'
            elif key=='outside_strata':bad['features']['d0.load_scale']=9.
            else:bad['features'].pop('d0.load_scale')
            testpath.write_text(json.dumps(bad))
            try:predict(folder/'model.joblib',testpath)
            except ValueError:checks[key+'_rejected']=True
            else:raise AssertionError(key+' accepted')
    (folder/'inference-validation.json').write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps(checks))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('folder',type=Path)
    validate(p.parse_args().folder)
