#!/usr/bin/env python3
"""Check CV Ridge against augmented least squares; never reads heldout target values."""
import argparse
import json
from pathlib import Path
import warnings
import numpy as np
from sklearn.linear_model import Ridge
from sklearn.model_selection import StratifiedKFold
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_absolute_error
from train_stress_surrogate import matrix, TARGETS


def audit(folder):
    rows=[json.loads(x) for x in (folder/'all_labels.jsonl').read_text().splitlines()]
    rows=[r for r in rows if r['split']=='train' and r['train_eligible']]
    x=matrix(rows,'pooled')
    folds=list(StratifiedKFold(3,shuffle=True,random_state=917).split(x,[r['regime'] for r in rows]))
    report=json.loads((folder/'report.json').read_text())
    max_difference=0.
    discrepancies={}
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always')
        for target in TARGETS[:4]:
            cat,key=target.split('.')
            y=np.array([r[cat][key] for r in rows])
            for alpha in (1.,10.,100.):
                predictions=np.empty_like(y)
                for train,valid in folds:
                    scaler=StandardScaler().fit(x[train])
                    a=scaler.transform(x[train]);b=scaler.transform(x[valid])
                    model=Ridge(alpha=alpha).fit(a,y[train])
                    yc=y[train]-y[train].mean()
                    ac=a-a.mean(axis=0)
                    # Ridge normal equations solved independently as augmented least squares.
                    augmented=np.vstack([ac,np.sqrt(alpha)*np.eye(a.shape[1])])
                    rhs=np.r_[yc,np.zeros(a.shape[1])]
                    beta=np.linalg.lstsq(augmented,rhs,rcond=None)[0]
                    ref=np.sum((b-a.mean(axis=0))*beta[None,:],axis=1)+y[train].mean()
                    actual=model.predict(b)
                    diff=float(np.max(np.abs(ref-actual)))
                    if not np.isfinite(actual).all() or diff>1e-5:
                        raise AssertionError(f'Ridge audit mismatch {target}/{alpha}: {diff}')
                    max_difference=max(max_difference,diff)
                    predictions[valid]=ref
                diff=abs(mean_absolute_error(y,predictions)-report['targets'][target]['cv_mae'][f'ridge_{alpha:g}'])
                if diff>1e-5:raise AssertionError('Stored CV score mismatch')
                discrepancies[f'{target}/alpha={alpha}']=float(diff)
    result={'passed':True,'training_only':True,'fold_solves':36,
            'max_prediction_difference':max_difference,'score_discrepancies':discrepancies,
            'absolute_tolerance':1e-5,'warning_count':len(caught),
            'warnings':sorted({str(w.message) for w in caught}),
            'interpretation':'finite output agrees with independent augmented least-squares calculation; warning root cause not proven'}
    (folder/'ridge-numerical-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);audit(p.parse_args().folder)
