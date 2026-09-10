#!/usr/bin/env python3
"""Frozen held-out comparisons and learning-cost curve, hydro theory §12."""
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
from run_price_oracle import ROOT,save

METHODS=[('daily_gp','Daily-energy GP','#999999','o',7),
         ('temporal_gp','Intraday + thermal GP (primary)','#0072B2','s',5),
         ('network_gp','Network-informed GP (secondary)','#D55E00','^',4)]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,default=ROOT/'output/market-intelligence/hydro-temporal-analysis-v3')
    p.add_argument('--style-kernel',type=Path,required=True)
    p.add_argument('--complete-context-report',type=Path)
    args=p.parse_args();style=runpy.run_path(str(args.style_kernel))
    style['apply_figure_style'](sizes=(10,9,8),font='DejaVu Sans')
    report=json.loads((args.input/'report.json').read_text())
    methods=list(METHODS)
    if args.complete_context_report:
        extra=json.loads(args.complete_context_report.read_text())
        if extra['design_sha256']!=report['design_sha256']:raise ValueError('Different complete-context test design')
        for fit in extra['fits']:
            original=next(f for f in report['fits'] if f['n_families']==fit['n_families'])
            if abs(original['oracle_worker_sec']-fit['oracle_worker_sec'])>1e-6:raise ValueError('Unequal training costs')
        report['results']+=extra['results'];report['predictions']+=extra['predictions']
        methods.append(('complete_gp','Complete 8-day context GP (secondary)','#009E73','D',3))
    fig,axes=plt.subplots(1,2,figsize=(11,5.8 if args.complete_context_report else 5.2));a,b=axes
    fig.subplots_adjust(left=.09,right=.97,top=.72,bottom=.29,wspace=.35)
    a.set_title('Error versus training simulation cost',loc='left',pad=10)
    b.set_title('Held-out gain: prediction versus Oracle',loc='left',pad=10)
    ranges=[]
    for model,label,color,marker,size in methods:
        points=sorted([r for r in report['results'] if r['model']==model],key=lambda r:r['n_families'])
        cost=[next(f['oracle_worker_sec'] for f in report['fits'] if f['n_families']==r['n_families'])/60 for r in points]
        a.plot(cost,[r['mae_mwh'] for r in points],color=color,marker=marker,ms=size,lw=1.4,label=label)
        rows=[r for r in report['predictions'] if r['model']==model and r['n_families']==4 and r['delta_mwh']!=0]
        if len(rows)!=16 or {r['root_family'] for r in rows}!={4,5}:raise ValueError('Wrong test cohort')
        actual=[r['actual_gain_mwh'] for r in rows];pred=[r['predicted_gain_mwh'] for r in rows]
        ranges+=actual+pred
        b.plot(actual,pred,ls='none',marker=marker,ms=size,color=color,markerfacecolor=color if model=='network_gp' else 'none')
    lo,hi=min(ranges),max(ranges);padding=max((hi-lo)*.05,10)
    b.plot([lo-padding,hi+padding],[lo-padding,hi+padding],':',lw=.8,color='#333333')
    a.axhline(25,color='#333333',lw=.8,ls=':',label='Research MAE threshold: 25 MWh')
    a.set_xlabel('Training Oracle cost (worker-minutes)');a.set_ylabel('Gain MAE (MWh)')
    b.set_xlabel('Oracle gain (MWh)');b.set_ylabel('Predicted gain (MWh)')
    for ax in axes:
        ax.xaxis.set_major_locator(MaxNLocator(4,prune='both'));ax.yaxis.set_major_locator(MaxNLocator(4,prune='both'))
        ax.margins(.08)
    fig.suptitle('Equal daily energy: intraday shape and thermal operating conditions',x=.09,ha='left',y=.97,fontsize=10)
    handles,labels=a.get_legend_handles_labels();fig.legend(handles,labels,ncol=2,loc='upper left',bbox_to_anchor=(.08,.925))
    fig.text(.09,.16,'2 held-out exogenous weeks; 4 condition variants/week, 16 nonzero actions. Train: 2 / 4 exogenous weeks.',fontsize=9)
    fig.text(.09,.11,'Right: fixed 4-week models; dotted line: exact prediction. Left: 24 / 48 training Oracle evaluations; no CI.',fontsize=9)
    fig.text(.09,.06,'Labels use 1% Oracle gap and include flow violations. Fixed weekly water/energy; no production admission.',fontsize=9)
    check=bbox_check(fig)
    if check['text_collisions'] or check['out_of_frame']:raise ValueError(json.dumps(check))
    fig.savefig(args.input/'temporal-comparison.png',dpi=300);fig.savefig(args.input/'temporal-comparison.pdf')
    picture=Image.open(args.input/'temporal-comparison.png');crops=[]
    for label,box in style['panel_crops'](fig).items():
        path=args.input/f'temporal-comparison-panel-{label}.png';picture.crop(box).save(path);crops.append(path.name)
    plt.close(fig)
    save(args.input/'figure-validation.json',{'geometry':check,'crops':crops,'perceptual_review':'pending visual inspection'})
    print(json.dumps(check))


if __name__=='__main__':main()
