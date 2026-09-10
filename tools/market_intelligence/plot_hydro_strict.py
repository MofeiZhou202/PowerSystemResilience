#!/usr/bin/env python3
"""Fixed held-out errors and two-gap labeling cost; hydro theory §14."""
import argparse
import json
from pathlib import Path
import runpy

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np
from PIL import Image

from plot_identification import bbox_check
from run_price_oracle import ROOT,save

METHODS=[('daily_gp','Daily-energy GP','Daily','#999999','o',6),
         ('temporal_gp','Intraday + thermal GP','Intraday','#0072B2','s',5),
         ('network_gp','Network-informed GP (primary)','Network','#D55E00','^',5),
         ('complete_gp','Complete 8-day context GP','Full context','#009E73','D',4)]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,default=ROOT/'output/market-intelligence/hydro-strict-v4/analysis')
    p.add_argument('--style-kernel',type=Path,required=True)
    args=p.parse_args()
    style=runpy.run_path(str(args.style_kernel))
    style['apply_figure_style'](sizes=(10,9,8),font='DejaVu Sans')
    report=json.loads((args.input/'report.json').read_text())
    fig,axes=plt.subplots(1,2,figsize=(11,6.1));a,b=axes
    fig.subplots_adjust(left=.09,right=.97,top=.72,bottom=.34,wspace=.35)
    a.set_title('Error versus training simulation cost',loc='left',pad=10)
    b.set_title('Errors on new held-out weeks',loc='left',pad=10)
    all_mae,all_errors=[],[]
    for j,(model,label,short,color,marker,size) in enumerate(METHODS):
        points=sorted([r for r in report['results'] if r['model']==model],key=lambda r:r['n_families'])
        cost=[next(f['oracle_worker_sec'] for f in report['fits'] if f['n_families']==r['n_families'])/60 for r in points]
        mae=[r['mae_mwh'] for r in points];all_mae+=mae
        a.plot(cost,mae,color=color,marker=marker,ms=size,lw=1.4,label=label)
        rows=[r for r in report['predictions'] if r['model']==model and r['n_families']==4 and r['delta_mwh']!=0]
        if len(rows)!=8 or {r['root_family'] for r in rows}!={4,5}:raise ValueError('Wrong held-out cohort')
        errors=np.array([abs(r['predicted_gain_mwh']-r['actual_gain_mwh']) for r in rows]);all_errors+=errors.tolist()
        b.plot(j+np.linspace(-.12,.12,len(errors)),errors,ls='none',marker=marker,ms=size,color=color,
               markerfacecolor=color if model=='network_gp' else 'none',alpha=.85)
        b.plot([j-.23,j+.23],[np.median(errors)]*2,color=color,lw=1.5)
    # Gates outside the displayed data range are stated in the caption, not
    # drawn in a nonexistent coordinate interval; figure-style §3.2.
    if max(all_mae)>=10:a.axhline(25,color='#333333',lw=.8,ls=':')
    if max(all_errors)>=20:b.axhline(50,color='#333333',lw=.8,ls=':')
    for ax,values,threshold,trigger in ((a,all_mae,25,10),(b,all_errors,50,20)):
        upper=max(max(values)*1.15,.01,threshold*1.05 if max(values)>=trigger else 0)
        ax.set_ylim(-upper*.05,upper)
        ax.yaxis.set_major_locator(MaxNLocator(4,prune='both'))
    a.xaxis.set_major_locator(MaxNLocator(4,prune='both'));a.margins(x=.08)
    b.set_xticks(range(len(METHODS)),[m[2] for m in METHODS]);b.set_xlim(-.5,3.5)
    a.set_xlabel('Training Oracle cost (worker-minutes)');a.set_ylabel('Gain MAE (MWh)')
    b.set_xlabel('Fixed 4-week models');b.set_ylabel('Absolute gain error (MWh)')
    if any(abs(r['actual_gain_mwh']-150)>1e-8 for r in report['predictions'] if r['delta_mwh']):
        raise ValueError('Constant-gain caption no longer matches the plotted data')
    fig.suptitle('Small errors on a constant-gain holdout',x=.09,ha='left',y=.97,fontsize=10)
    handles,labels=a.get_legend_handles_labels()
    fig.legend(handles,labels,ncol=2,loc='upper left',bbox_to_anchor=(.08,.925))
    fig.text(.09,.22,'2 held-out exogenous weeks; 4 variants/week. All 8 nonzero gains ≈150 MWh. Train: 2 / 4 exogenous weeks.',fontsize=9)
    fig.text(.09,.17,'Right: individual errors and medians. Left: 32 / 64 training Oracle evaluations, including both gap levels.',fontsize=9)
    fig.text(.09,.12,'Gaps: 1e-5 / 1e-6. Research gates: MAE 25 MWh, maximum error 50 MWh. Initial convergence cost excluded.',fontsize=9)
    fig.text(.09,.07,'Fixed station weekly energy and terminal water; line factors 0.8-1.0. Diagnostic study; no production admission.',fontsize=9)
    check=bbox_check(fig)
    if check['text_collisions'] or check['out_of_frame']:raise ValueError(json.dumps(check))
    fig.savefig(args.input/'strict-comparison.png',dpi=300);fig.savefig(args.input/'strict-comparison.pdf')
    picture=Image.open(args.input/'strict-comparison.png');crops=[]
    for label,box in style['panel_crops'](fig).items():
        path=args.input/f'strict-comparison-panel-{label}.png';picture.crop(box).save(path);crops.append(path.name)
    plt.close(fig)
    save(args.input/'figure-validation.json',dict(geometry=check,crops=crops,perceptual_review='pending visual inspection'))
    print(json.dumps(check))


if __name__=='__main__':main()
