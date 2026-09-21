#!/usr/bin/env python3.8
"""
Demo 05: JSON API 求解完整 SCUC 问题

使用 mipsolvers.scuc.solve_json() 接口
自动处理：机组组合 (UC) + 安全约束经济调度 (SCED) + 边际电价 (LMP)

输入格式: JSON 字符串
输出格式: JSON 字符串 (包含调度方案、成本、LMP)
"""
import sys
sys.path.insert(0, '/Users/tianyangzhao/Codes/MIPSolvers/build_py')

import json
import mipsolvers

# ── 输入数据 ──────────────────────────────────────────────────────────────────
# 2台机组, 24小时调度
NUM_PERIODS = 24
BASE_LOAD   = 300.0   # MW (基准负荷)

# 负荷曲线系数 (标幺值 × 基准负荷)
load_profile = [
    0.80, 0.75, 0.72, 0.70, 0.72, 0.78,   # 凌晨 0-5 点
    0.85, 0.95, 1.05, 1.15, 1.20, 1.25,   # 早高峰 6-11 点
    1.20, 1.15, 1.10, 1.05, 1.10, 1.20,   # 下午 12-17 点
    1.30, 1.35, 1.40, 1.30, 1.15, 0.95,   # 晚高峰 18-23 点
]

input_data = {
    "config": {
        "num_periods":      NUM_PERIODS,
        "period_length_hr": 1.0,
        "solver":           "HiGHS",
        "mip_gap":          0.001,
        "time_limit_sec":   300.0,
        "solve_sced":       True,   # 求解安全约束经济调度
        "solve_lmp":        True,   # 计算边际电价
        "voll":             10000.0 # 失负荷价值 ($/MWh)
    },
    "generators": [
        {
            "name": "G1",  "bus": 0,
            "pmin": 50.0,  "pmax": 400.0,
            "ramp_up_mw_min":  200.0,
            "ramp_dn_mw_min":  200.0,
            "min_up_time_hr":   4.0,
            "min_dn_time_hr":   2.0,
            "startup_cost":  5000.0,
            "no_load_cost":   200.0,
            "bid_segments": [
                {"price": 20.0, "quantity": 150.0},
                {"price": 35.0, "quantity": 150.0},
                {"price": 50.0, "quantity": 100.0},
            ]
        },
        {
            "name": "G2",  "bus": 0,
            "pmin": 30.0,  "pmax": 200.0,
            "ramp_up_mw_min":  100.0,
            "ramp_dn_mw_min":  100.0,
            "min_up_time_hr":   2.0,
            "min_dn_time_hr":   1.0,
            "startup_cost":  2000.0,
            "no_load_cost":   100.0,
            "bid_segments": [
                {"price": 30.0, "quantity": 100.0},
                {"price": 50.0, "quantity":  70.0},
            ]
        },
        {
            "name": "G3",  "bus": 0,
            "pmin":  0.0,  "pmax": 150.0,
            "ramp_up_mw_min":  150.0,
            "ramp_dn_mw_min":  150.0,
            "min_up_time_hr":   1.0,
            "min_dn_time_hr":   1.0,
            "startup_cost":   500.0,
            "no_load_cost":    50.0,
            "bid_segments": [
                {"price": 60.0, "quantity": 100.0},
                {"price": 80.0, "quantity":  50.0},
            ]
        }
    ],
    "loads": [
        {"bus": 0, "p_mw": BASE_LOAD}
    ],
    "branches": [],   # 不计算网络约束
    "profiles": {
        "load": [load_profile]   # 每条负荷曲线对应一个 loads 元素
    }
}

# ── 求解 ──────────────────────────────────────────────────────────────────────
print(f"机组数量: {len(input_data['generators'])}")
print(f"时段数量: {NUM_PERIODS}")
print(f"峰值负荷: {max(load_profile)*BASE_LOAD:.0f} MW")
print(f"谷值负荷: {min(load_profile)*BASE_LOAD:.0f} MW\n")

json_input  = json.dumps(input_data)
json_output = mipsolvers.scuc.solve_json(json_input, indent=2)
result      = json.loads(json_output)

# ── 结果解析 ──────────────────────────────────────────────────────────────────
scuc = result["scuc"]
print(f"求解状态:  {'成功' if scuc['converged'] else '失败'}")
print(f"总运行成本: ${scuc['cost']['total']:,.2f}")
print(f"  ├ 启动成本: ${scuc['cost']['startup']:,.2f}")
print(f"  └ 发电成本: ${scuc['cost']['energy']:,.2f}")

print(f"\n{'时段':<5}{'负荷':>8}", end="")
gen_names = [g["name"] for g in input_data["generators"]]
for gn in gen_names:
    print(f" {gn:>8}", end="")
if "lmp" in scuc:
    print(f" {'LMP':>8}", end="")
print()

dispatch = scuc["dispatch"]    # list of per-generator dispatch arrays
lmp_arr  = scuc.get("lmp", [None]*NUM_PERIODS)

for t in range(NUM_PERIODS):
    load_t = load_profile[t] * BASE_LOAD
    print(f" {t+1:>3}   {load_t:>7.1f}", end="")
    for gi in range(len(gen_names)):
        pv = dispatch[gi][t] if gi < len(dispatch) else 0.0
        print(f" {pv:>8.1f}", end="")
    if lmp_arr[t] is not None:
        print(f" {lmp_arr[t]:>8.2f}", end="")
    print()

print(f"\n峰时 LMP: ${max(v for v in lmp_arr if v is not None):.2f}/MWh" if lmp_arr[0] is not None else "")
print(f"谷时 LMP: ${min(v for v in lmp_arr if v is not None):.2f}/MWh" if lmp_arr[0] is not None else "")

# 机组开停汇总
if "commitment" in scuc:
    print("\n机组开停计划 (1=开, 0=停):")
    for gi, gn in enumerate(gen_names):
        row = "".join(str(int(scuc['commitment'][gi][t] > 0.5)) for t in range(NUM_PERIODS))
        starts = sum(1 for t in range(1, NUM_PERIODS)
                     if scuc['commitment'][gi][t] > 0.5 and scuc['commitment'][gi][t-1] < 0.5)
        print(f"  {gn}: {row}  ({starts}次启动)")
