#!/usr/bin/env python3
"""Three-way Transformer2W cross-check: hacdcpf vs pandapower vs MATPOWER.

Sweeps tap side/pos, phase shift, and nameplate-vs-bus-base voltage mismatch
through a 2-bus transformer benchmark:
  - hacdcpf:    tools/transformer_tap_crosscheck (rich Transformer2W model)
  - pandapower: create_transformer with the same nameplate parameters
  - MATPOWER:   canonical branch (r/x pu + ratio + shift) hand-converted with
                the same formulas as src/model/network_utils.cpp

Usage: .venv/bin/python tools/transformer_tap_crosscheck.py [--skip-matlab]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = os.path.join(ROOT, "build", "macos-release", "transformer_tap_crosscheck")
MATLAB = "/Applications/MATLAB_R2025b.app/bin/matlab"
VENV_PY = os.path.join(ROOT, ".venv", "bin", "python")
OUT = os.path.join(ROOT, "output", "benchmarks", "transformer_tap")

VK, VKR, SN, SBASE = 10.0, 0.5, 100.0, 100.0
PD, QD = 50.0, 10.0

# (name, tap_side, tap_pos, shift_deg, vn_hv, vn_lv, bus_hv, bus_lv)
# names must be valid MATLAB function identifiers (used as .m case names)
CONFIGS = [
    ("nominal_tap0",       0,  0, 0.0, 230, 115, 230, 115),
    ("hvtap_p4",           0,  4, 0.0, 230, 115, 230, 115),
    ("hvtap_m4",           0, -4, 0.0, 230, 115, 230, 115),
    ("lvtap_p4",           1,  4, 0.0, 230, 115, 230, 115),
    ("lvtap_m4",           1, -4, 0.0, 230, 115, 230, 115),
    ("shift_p5deg",        0,  0, 5.0, 230, 115, 230, 115),
    ("shift_m5deg",        0,  0, -5.0, 230, 115, 230, 115),
    ("lvbase110_tap0",     0,  0, 0.0, 230, 115, 230, 110),
    ("lvbase110_hvtap_p4", 0,  4, 0.0, 230, 115, 230, 110),
    ("hvbase220_tap0",     0,  0, 0.0, 230, 115, 220, 115),
    ("step2p5_tap_p3",     0,  3, 0.0, 230, 115, 230, 115),
    ("lvbase110_lvtap_p4", 1,  4, 0.0, 230, 115, 230, 110),
]
TAP_STEP = {c[0]: (2.5 if c[0] == "step2p5_tap_p3" else 1.25) for c in CONFIGS}


def run_hysim(cfg):
    name, side, pos, shift, vnh, vnl, bh, bl = cfg
    cmd = [BIN, "--tap-side", str(side), "--tap-pos", str(pos),
           "--shift", str(shift), "--vn-hv", str(vnh), "--vn-lv", str(vnl),
           "--bus-hv", str(bh), "--bus-lv", str(bl),
           "--tap-step", str(TAP_STEP[name]), "--pd", str(PD), "--qd", str(QD)]
    out = subprocess.run(cmd, capture_output=True, text=True)
    return json.loads(out.stdout)


PP_SNIPPET = r"""
import json, sys
import pandapower as pp
import pandapower.networks

name, side, pos, shift, vnh, vnl, bh, bl, step = sys.argv[1:10]
pos = int(pos); shift = float(shift)
net = pp.create_empty_network(sn_mva=100.0)
b1 = pp.create_bus(net, vn_kv=float(bh), name='HV')
b2 = pp.create_bus(net, vn_kv=float(bl), name='LV')
pp.create_ext_grid(net, b1, vm_pu=1.0, va_degree=0.0)
pp.create_load(net, b2, p_mw=50.0, q_mvar=10.0)
pp.create_transformer_from_parameters(net, hv_bus=b1, lv_bus=b2, sn_mva=100.0,
                      vn_hv_kv=float(vnh), vn_lv_kv=float(vnl),
                      vk_percent=10.0, vkr_percent=0.5,
                      pfe_kw=0.0, i0_percent=0.0,
                      tap_side='hv' if side == '0' else 'lv',
                      tap_neutral=0, tap_min=-20, tap_max=20,
                      tap_step_percent=float(step), tap_pos=pos,
                      shift_degree=shift)
pp.runpp(net, numba=False)
print(json.dumps({
    'vm_lv': float(net.res_bus.vm_pu.at[1]),
    'va_lv_deg': float(net.res_bus.va_degree.at[1]),
    'vm_hv': float(net.res_bus.vm_pu.at[0]),
    'p_from_mw': float(net.res_trafo.p_hv_mw.at[0]),
    'q_from_mvar': float(net.res_trafo.q_hv_mvar.at[0]),
}))
"""


def run_pandapower(cfg):
    name, side, pos, shift, vnh, vnl, bh, bl = cfg
    cmd = [VENV_PY, "-c", PP_SNIPPET, name, str(side), str(pos), str(shift),
           str(vnh), str(vnl), str(bh), str(bl), str(TAP_STEP[name])]
    out = subprocess.run(cmd, capture_output=True, text=True)
    if out.returncode != 0:
        return {"error": out.stderr[-300:]}
    return json.loads(out.stdout)


def canonical_branch(cfg):
    """Mirror of add_equivalent_branch_from_transformer2w (network_utils.cpp)."""
    name, side, pos, shift, vnh, vnl, bh, bl = cfg
    step = TAP_STEP[name]
    r = (VKR / 100.0) * (SBASE / SN)
    x = (VK / 100.0) * (SBASE / SN)
    vbase_scale = (vnl / bl) ** 2
    r *= vbase_scale
    x *= vbase_scale
    tap_f = 1.0 + pos * step / 100.0
    tap_pos_factor = (1.0 / tap_f) if side == 1 else tap_f
    imp_scale = (tap_f * tap_f) if side == 1 else 1.0
    nominal_ratio = (vnh / vnl) / (bh / bl)
    ratio = nominal_ratio * tap_pos_factor
    return r * imp_scale, x * imp_scale, ratio, shift


def write_matpower_cases(out_dir):
    paths = []
    for cfg in CONFIGS:
        name = cfg[0]
        r, x, ratio, shift = canonical_branch(cfg)
        bh, bl = cfg[6], cfg[7]
        txt = f"""function mpc = {name}
mpc.version = '2';
mpc.baseMVA = {SBASE};
mpc.bus = [
  1 3 0 0 0 0 1 1 0 {bh} 1 1.1 0.9;
  2 1 {PD} {QD} 0 0 1 1 0 {bl} 1 1.1 0.9;
];
mpc.gen = [
  1 0 0 1e5 -1e5 1 100 1 1e5 -1e5;
];
mpc.branch = [
  1 2 {r:.12g} {x:.12g} 0 0 0 0 {ratio:.12g} {shift} 1 -360 360;
];
"""
        p = os.path.join(out_dir, name + ".m")
        with open(p, "w") as fh:
            fh.write(txt)
        paths.append(p)
    with open(os.path.join(out_dir, "caselist.txt"), "w") as fh:
        fh.write("\n".join(paths))
    return paths


def load_mp_bus(out_dir, name):
    d = {}
    with open(os.path.join(out_dir, name + ".matpower_bus.csv")) as fh:
        for row in csv.DictReader(fh):
            d[int(row["bus_i"])] = (float(row["vm"]), float(row["va_deg"]))
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-matlab", action="store_true")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)

    if not args.skip_matlab:
        write_matpower_cases(OUT)
        cmd = [MATLAB, "-batch",
               f"addpath('{os.path.join(ROOT, 'tools')}'); "
               f"matpower_benchmark('{os.path.join(OUT, 'caselist.txt')}',"
               f"'{OUT}')"]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            print(proc.stdout[-2000:], proc.stderr[-2000:], file=sys.stderr)

    hdr = (f"{'config':<20}{'hy_vm':>9} {'pp_vm':>9} {'mp_vm':>9} "
           f"{'hy_va':>8} {'pp_va':>8} {'mp_va':>8} "
           f"{'|dVm|hy-pp':>11} {'|dVa|hy-pp':>11} {'|dVm|hy-mp':>11}")
    print(hdr)
    print("-" * len(hdr))
    worst_pp, worst_mp = 0.0, 0.0
    for cfg in CONFIGS:
        name = cfg[0]
        hy = run_hysim(cfg)
        pp = run_pandapower(cfg)
        row = {"hy": hy, "pp": pp}
        mp_vm = mp_va = float("nan")
        try:
            mp = load_mp_bus(OUT, name)
            mp_vm, mp_va = mp[2]
        except Exception:
            pass
        dvm_pp = abs(hy.get("vm_lv", float("nan")) - pp.get("vm_lv", float("nan")))
        dva_pp = abs(hy.get("va_lv_deg", float("nan")) - pp.get("va_lv_deg", float("nan")))
        dvm_mp = abs(hy.get("vm_lv", float("nan")) - mp_vm)
        dva_mp = abs(hy.get("va_lv_deg", float("nan")) - mp_va)
        worst_pp = max(worst_pp, dvm_pp if dvm_pp == dvm_pp else 0)
        worst_mp = max(worst_mp, dvm_mp if dvm_mp == dvm_mp else 0)
        print(f"{name:<20}{hy.get('vm_lv', float('nan')):>9.5f} "
              f"{pp.get('vm_lv', float('nan')):>9.5f} {mp_vm:>9.5f} "
              f"{hy.get('va_lv_deg', float('nan')):>8.4f} "
              f"{pp.get('va_lv_deg', float('nan')):>8.4f} {mp_va:>8.4f} "
              f"{dvm_pp:>11.2e} {dva_pp:>11.2e} {dvm_mp:>11.2e}")
    print(f"\nworst |dVm| vs pandapower: {worst_pp:.3e}   vs MATPOWER: {worst_mp:.3e}")


if __name__ == "__main__":
    main()
