#!/usr/bin/env python3
"""Hydro paired-response and grouped error/cost figures, frozen design §8."""
import argparse
import hashlib
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

METHODS = [('zero_gain', 'Predict no gain', '#666666', 'o'),
           ('ridge', 'Ridge regression', '#0072B2', 's'),
           ('trees', 'Randomized trees', '#D55E00', '^')]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, default=ROOT/'output/market-intelligence/hydro-analysis-v1')
    p.add_argument('--style-kernel', type=Path, required=True)
    args = p.parse_args()
    style = runpy.run_path(str(args.style_kernel))
    style['apply_figure_style'](sizes=(10, 9, 8), font='DejaVu Sans')
    report = json.loads((args.input/'report.json').read_text())
    if report['curves']['status'] != 'completed_development_only':
        raise ValueError('No complete frozen cohort for learning curve')
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.7))
    fig.subplots_adjust(left=.09, right=.97, top=.77, bottom=.29, wspace=.35)
    for ax, metric, title in zip(axes, ('mae_mwh', 'max_abs_error_mwh'),
                                 ('Mean error on held-out weeks', 'Largest error on held-out weeks')):
        ax.set_title(title, loc='left', pad=10)
        for model, label, color, marker in METHODS:
            for fold in range(3):
                points = sorted([r for r in report['curves']['folds'] if r['model'] == model and r['fold'] == fold], key=lambda r: r['n_families'])
                ax.plot([r['worker_sec']/60 for r in points], [r[metric] for r in points], color=color, alpha=.2, lw=.8)
            points = sorted([r for r in report['curves']['aggregate'] if r['model'] == model], key=lambda r: r['n_families'])
            ax.plot([r['worker_sec']/60 for r in points], [r[metric] for r in points],
                    color=color, marker=marker, lw=1.5,
                    ms={'zero_gain': 8, 'ridge': 6, 'trees': 4}[model],
                    markerfacecolor=color if model == 'trees' else 'none', label=label)
        ax.set_ylabel('Consumption-gain error (MWh)')
        ax.set_xlabel('Oracle training cost (worker-minutes)')
        ax.xaxis.set_major_locator(MaxNLocator(4, prune='both'))
        ax.yaxis.set_major_locator(MaxNLocator(4, prune='both'))
        ax.margins(.08)
    fig.suptitle('Hydro allocation: development error versus simulation cost', x=.09, ha='left', y=.975, fontsize=10)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=3, loc='upper left', bbox_to_anchor=(.08, .93))
    fig.text(.09, .145, '6 external-week families; 3 grouped folds; train 2 / 3 / 4 families (10 / 15 / 20 Oracle evaluations).', fontsize=9)
    fig.text(.09, .097, 'Faint: each fold. Bold: pooled held-out error (24 nonzero actions). Lower error is better.', fontsize=9)
    fig.text(.09, .049, 'Fixed station weekly energy and terminal water; station 1, day 2 to day 5 only. Development, not certification.', fontsize=9)
    check = bbox_check(fig)
    if check['text_collisions'] or check['out_of_frame']:
        raise ValueError(json.dumps(check))
    fig.savefig(args.input/'learning-curves.png', dpi=300)
    fig.savefig(args.input/'learning-curves.pdf')
    picture = Image.open(args.input/'learning-curves.png')
    crops = []
    for name, box in style['panel_crops'](fig).items():
        path = args.input/f'learning-curves-panel-{name}.png'
        picture.crop(box).save(path)
        crops.append(path.name)
    plt.close(fig)
    save(args.input/'figure-validation.json', {'style_kernel_sha256': hashlib.sha256(args.style_kernel.read_bytes()).hexdigest(),
         'geometric_checks': check, 'crops': crops, 'perceptual_review': 'pending visual inspection'})
    print(json.dumps(check))


if __name__ == '__main__':
    main()
