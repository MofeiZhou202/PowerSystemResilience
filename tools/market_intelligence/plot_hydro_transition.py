#!/usr/bin/env python3
"""Same-test-cohort surrogate comparisons; hydro theory §10."""
import argparse
import json
from pathlib import Path
import runpy

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from PIL import Image

from plot_identification import bbox_check
from run_price_oracle import ROOT, save

METHODS = [('zero_gain','Predict no gain','#888888','o',8),
           ('context_trees','Context trees','#D55E00','s',7),
           ('physics_only','Energy formula','#56B4E9','D',6),
           ('physics_gp','Energy + Gaussian process','#0072B2','^',4)]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,default=ROOT/'output/market-intelligence/hydro-transition-analysis-v2')
    p.add_argument('--style-kernel',type=Path,required=True)
    args=p.parse_args()
    style=runpy.run_path(str(args.style_kernel))
    style['apply_figure_style'](sizes=(10,9,8),font='DejaVu Sans')
    report=json.loads((args.input/'report.json').read_text())
    if report['test_families']!=4 or report['train_families']!=8:
        raise ValueError('Unexpected frozen cohort')
    fig,axes=plt.subplots(1,2,figsize=(10,4.8))
    fig.subplots_adjust(left=.10,right=.97,top=.74,bottom=.30,wspace=.37)
    a,b=axes
    a.set_title('Prediction versus Oracle gain',loc='left',pad=10)
    b.set_title('Error versus training simulation cost',loc='left',pad=10)
    a.plot([-330,330],[-330,330],':',color='#333333',lw=.8,label='Exact prediction')
    for model,label,color,marker,size in METHODS:
        rows=[r for r in report['predictions'] if r['n_families']==8 and r['model']==model and r['delta_mwh']!=0]
        if len(rows)!=8: raise ValueError('Missing held-out action')
        a.plot([r['actual_gain_mwh'] for r in rows],[r['predicted_gain_mwh'] for r in rows],linestyle='none',
               marker=marker,ms=size,color=color,markerfacecolor=color if model=='physics_gp' else 'none',label=label)
        points=sorted([r for r in report['results'] if r['model']==model],key=lambda r:r['n_families'])
        cost=[next(f['oracle_worker_sec'] for f in report['fits'] if f['n_families']==r['n_families'])/60 for r in points]
        b.plot(cost,[r['mae_mwh'] for r in points],marker=marker,ms=size,color=color,
               markerfacecolor=color if model=='physics_gp' else 'none',lw=1.4,label=label)
    a.set_xlabel('Oracle consumption gain (MWh)');a.set_ylabel('Predicted consumption gain (MWh)')
    b.set_xlabel('Oracle training cost (worker-minutes)');b.set_ylabel('Held-out gain MAE (MWh)')
    for ax in axes:
        ax.xaxis.set_major_locator(MaxNLocator(4,prune='both'))
        ax.yaxis.set_major_locator(MaxNLocator(4,prune='both'))
        ax.margins(.08)
    fig.suptitle('Hydro allocation: a physical energy baseline tested on new weeks',x=.1,ha='left',y=.97,fontsize=10)
    handles,labels=a.get_legend_handles_labels()
    fig.legend(handles,labels,ncol=3,loc='upper left',bbox_to_anchor=(.09,.94))
    fig.text(.1,.15,'4 independent test weeks, 8 nonzero actions; train 4 / 8 weeks (12 / 24 Oracle evaluations).',fontsize=9)
    fig.text(.1,.10,'Left: fixed 8-week models; dotted line: exact prediction. Right: lower error is better; no confidence interval.',fontsize=9)
    fig.text(.1,.05,'Synthetic transition strata; station 1, day 2 to day 5. Fixed weekly energy and terminal water; no AC certificate.',fontsize=9)
    check=bbox_check(fig)
    if check['text_collisions'] or check['out_of_frame']: raise ValueError(json.dumps(check))
    fig.savefig(args.input/'surrogate-comparison.png',dpi=300)
    fig.savefig(args.input/'surrogate-comparison.pdf')
    picture=Image.open(args.input/'surrogate-comparison.png')
    crops=[]
    for label,box in style['panel_crops'](fig).items():
        path=args.input/f'surrogate-comparison-panel-{label}.png';picture.crop(box).save(path);crops.append(path.name)
    plt.close(fig)
    save(args.input/'figure-validation.json',{'geometry':check,'crops':crops,'perceptual_review':'pending visual inspection'})
    print(json.dumps(check))


if __name__=='__main__':
    main()
