#!/usr/bin/env python3
"""Plot frozen development curves, preserving fold variability and worker-cost units."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.text import Text
from matplotlib.ticker import MaxNLocator, FuncFormatter
import numpy as np
from PIL import Image


PANELS = {
    'price.spike_fraction': ('Price-spike fraction', 'MAE (percentage points)', 100),
    'price.spike_duration_hr': ('Spike duration (47/48 at 168 h)', 'MAE (h)', 1),
    'metrics.load_loss_mwh': ('Diagnostic load deficit', 'MAE (MWh)', 1),
    'metrics.line_overload_integral_mwh': ('Active-flow overload integral', 'MAE (MWh)', 1),
    'price.mean': ('Mean price', 'MAE (currency/MWh)', 1),
    'price.p95': ('95th-percentile price', 'MAE (currency/MWh)', 1),
    'price.p99': ('99th-percentile price', 'MAE (currency/MWh)', 1),
    'price.cvar99': ('Within-week price CVaR99', 'MAE (currency/MWh)', 1),
    'metrics.renewable_utilization': ('Renewable utilization', 'MAE (percentage points)', 100),
    'metrics.renewable_curtailment_mwh': ('Renewable curtailment', 'MAE (MWh)', 1),
}
METHODS = [('median', 'Training median', '#666666', 'o'),
           ('trees_low5', 'Trees: 5 weekly levels', '#0072B2', 's'),
           ('trees_temporal17', 'Trees: 17 temporal features', '#D55E00', '^'),
           ('trees_full56', 'Trees: 56 daily inputs', '#009E73', 'D')]


def bbox_check(fig):
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    texts = [(t, t.get_window_extent(renderer)) for t in fig.findobj(Text)
             if t.get_visible() and t.get_text().strip()]
    collisions = [(a.get_text(), b.get_text()) for i, (a, ba) in enumerate(texts)
                  for b, bb in texts[i+1:] if ba.overlaps(bb)]
    outside = [t.get_text() for t, box in texts if not fig.bbox.contains(box.x0, box.y0)
               or not fig.bbox.contains(box.x1, box.y1)]
    for ax in fig.axes:
        ticks = set(ax.get_xticklabels()+ax.get_yticklabels())
        for spine in ax.spines.values():
            if not spine.get_visible():continue
            sb = spine.get_window_extent(renderer)
            for t, box in texts:
                if t not in ticks and box.overlaps(sb):collisions.append((t.get_text(), 'spine'))
    return {'text_collisions': collisions, 'out_of_frame': outside}


def plot(folder, report, keys, name, style):
    nrows = len(keys)//2
    fig, axes = plt.subplots(nrows, 2, figsize=(9, 3.1*nrows+1.25), sharex=True)
    fig.subplots_adjust(left=.105, right=.965, bottom=.18, top=.82 if nrows==2 else .865,
                        wspace=.36, hspace=.42)
    aggregates, folds = report['curves']['aggregate'], report['curves']['folds']
    for ax, key in zip(axes.flat, keys):
        title, ylabel, scale = PANELS[key]
        ax.set_title(title, pad=9)
        ax.set_ylabel(ylabel)
        for model, label, color, marker in METHODS:
            for fold in range(3):
                pts = sorted((r for r in folds if r['fold']==fold and r['model']==model and r['target']==key),
                             key=lambda r:r['n_families'])
                if len(pts)!=3:raise ValueError('Incomplete fold curve')
                ax.plot([r['worker_sec']/60 for r in pts], [r['mae']*scale for r in pts],
                        color=color, alpha=.18, lw=.7)
            pts = sorted((r for r in aggregates if r['model']==model and r['target']==key),
                         key=lambda r:r['n_families'])
            if len(pts)!=3:raise ValueError('Incomplete aggregate curve')
            ax.plot([r['worker_sec']/60 for r in pts], [r['mae']*scale for r in pts],
                    color=color, marker=marker, ms=4, lw=1.4, label=label, zorder=4)
        ax.margins(x=.07, y=.09)
        ax.yaxis.set_major_locator(MaxNLocator(4))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda value, pos: f'{value/1000:g}k' if abs(value)>=1000 else f'{value:g}'))
        ax.xaxis.set_major_locator(MaxNLocator(4))
    for ax in axes[-1]:ax.set_xlabel('Oracle training cost (worker-minutes)')
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.suptitle('Development error versus Oracle training cost', x=.105, ha='left', y=.972, fontsize=10)
    fig.legend(handles, labels, ncol=2, loc='upper left', bbox_to_anchor=(.095,.945),
               columnspacing=1.8, handletextpad=.7)
    fig.text(.105, .075, '12 input families; 3 grouped folds; training: 4, 6, 8 families (16, 24, 32 weekly evaluations).', fontsize=9)
    fig.text(.105, .05, 'Faint lines: individual folds. Bold lines: fold means. Lower error is better; these are not confidence intervals.', fontsize=9)
    fig.text(.105, .025, 'Fixed IEEE118 system and initial state. Cost excludes development weeks and repeats; worker-time is not wall time.', fontsize=9)
    check = bbox_check(fig)
    if check['text_collisions'] or check['out_of_frame']:
        plt.close(fig)
        raise ValueError(json.dumps({'figure': name, **check}))
    fig.savefig(folder/f'{name}.png', dpi=300)
    fig.savefig(folder/f'{name}.pdf')
    img = Image.open(folder/f'{name}.png')
    crops = []
    for label, box in style['panel_crops'](fig).items():
        path = folder/f'{name}-panel-{label}.png'
        img.crop(box).save(path)
        crops.append(path.name)
    plt.close(fig)
    return {'figure': name, **check, 'crops': crops,
            'source_targets': keys, 'n_families': 12, 'final_blind_test': False}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder', type=Path)
    p.add_argument('--style-kernel', type=Path, required=True)
    args = p.parse_args()
    style = runpy.run_path(str(args.style_kernel))
    style['apply_figure_style'](sizes=(10, 9, 8), font='DejaVu Sans')
    report = json.loads((args.folder/'report.json').read_text())
    if report['final_blind_test'] or report['independent_families']!=12:
        raise ValueError('Unexpected experiment scope')
    labels = [json.loads(line) for line in (args.folder/'all_labels.jsonl').read_text().splitlines()]
    labels = [r for r in labels if r['repeat_of'] is None]
    if len(labels)!=48 or sum(r['price']['spike_duration_hr']==168 for r in labels)!=47:
        raise ValueError('Update the duration panel annotation to match actual labels')
    keys = list(PANELS)
    checks = [plot(args.folder, report, keys[:4], 'learning-curves', style),
              plot(args.folder, report, keys[4:], 'learning-curves-additional', style)]
    (args.folder/'figure-validation.json').write_text(json.dumps({
        'style_kernel_sha256': hashlib.sha256(args.style_kernel.read_bytes()).hexdigest(),
        'geometric_checks': checks, 'perceptual_review': 'requires visual inspection of exported PNGs'}, indent=2)+'\n')
    print(json.dumps(checks))


if __name__ == '__main__':
    main()
