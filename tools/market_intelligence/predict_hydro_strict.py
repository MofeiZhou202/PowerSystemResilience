#!/usr/bin/env python3
"""Local strict-label research estimate, always requiring Oracle confirmation (§14)."""
import argparse
import json
from pathlib import Path

from hydro_strict_model import FEATURES, StrictPredictor
from run_price_oracle import ROOT, save


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, default=ROOT/'output/market-intelligence/hydro-strict-v4')
    p.add_argument('--candidate', type=Path, required=True)
    p.add_argument('--model', choices=tuple(FEATURES), default='network_gp')
    p.add_argument('--output', type=Path)
    args = p.parse_args()
    candidate = json.loads(args.candidate.read_text())
    if candidate.get('config') != candidate['spec']['config']:
        raise ValueError('Candidate wrapper/config mismatch')
    result = StrictPredictor(args.input, args.model).predict(candidate['base'], candidate['spec'])
    if args.output:
        save(args.output, result)
    print(json.dumps(result))


if __name__ == '__main__':
    main()
