#!/usr/bin/env python3.8
"""
Demo 04: AML 建立简化 SCUC 模型 (机组组合问题)

问题规模: 2台机组, 6个时段 (T=6)

决策变量:
  u[g,t]  ∈ {0,1}   机组状态（1=开机，0=停机）
  p[g,t]  ≥ 0        出力 (MW)
  su[g,t] ∈ {0,1}   启动指示（t期由停转开）
  sd[g,t] ∈ {0,1}   停机指示（t期由开转停）

目标: 最小化总运行成本
  min Σ_g Σ_t [ no_load[g]*u[g,t] + bid[g]*p[g,t] + startup[g]*su[g,t] ]

约束:
  (1) 功率平衡:       Σ_g p[g,t] = load[t],  ∀t
  (2) 出力下界:       p[g,t] >= pmin[g] * u[g,t],  ∀g,t
  (3) 出力上界:       p[g,t] <= pmax[g] * u[g,t],  ∀g,t
  (4) 爬坡上升:       p[g,t] - p[g,t-1] <= ramp[g],  ∀g, t>1
  (5) 爬坡下降:       p[g,t-1] - p[g,t] <= ramp[g],  ∀g, t>1
  (6) 启停逻辑:       su[g,t] - sd[g,t] = u[g,t] - u[g,t-1],  ∀g,t
  (7) 0/1 上界:       su[g,t] + sd[g,t] <= 1,  ∀g,t
"""
import sys
sys.path.insert(0, '/Users/tianyangzhao/Codes/MIPSolvers/build_py')

import mipsolvers
aml = mipsolvers.aml

# ── 数据 ──────────────────────────────────────────────────────────────────────
T_periods = 6
gen_names = ["G1", "G2"]
t_names   = [f"t{i+1}" for i in range(T_periods)]

gen_data = {
    # pmin/pmax(MW), ramp(MW/期), no_load($/期), bid($/MWh), startup($)
    "G1": dict(pmin=50.0,  pmax=200.0, ramp=120.0,
               no_load=500.0, bid=30.0, startup=1000.0),
    "G2": dict(pmin=30.0,  pmax=150.0, ramp=90.0,
               no_load=300.0, bid=45.0, startup=800.0),
}

load_mw = {"t1": 150.0, "t2": 200.0, "t3": 260.0,
           "t4": 240.0, "t5": 180.0, "t6": 120.0}

init_u  = {"G1": 1, "G2": 0}    # 初始开停状态 (时段1之前)
init_p  = {"G1": 100.0, "G2": 0.0}  # 初始出力 (MW)

# ── 建模 ──────────────────────────────────────────────────────────────────────
m = aml.Model("mini_scuc")

G     = m.add_set("G", gen_names)
T_set = m.add_ordered_set("T", t_names)

# 决策变量
u  = m.add_var2("u",  G, T_set, aml.VarType.Binary)
p  = m.add_var2("p",  G, T_set, aml.VarType.Continuous, lb=0.0)
su = m.add_var2("su", G, T_set, aml.VarType.Binary)
sd = m.add_var2("sd", G, T_set, aml.VarType.Binary)

# ── 目标：最小化总费用 ────────────────────────────────────────────────────────
m.minimize(aml.sum_over(G, lambda g:
    aml.sum_over(T_set, lambda t:
        gen_data[g.values[0]]["no_load"] * u[(g, t)] +
        gen_data[g.values[0]]["bid"]     * p[(g, t)] +
        gen_data[g.values[0]]["startup"] * su[(g, t)])))

# ── 约束 ──────────────────────────────────────────────────────────────────────
for t in t_names:
    # (1) 功率平衡
    bal = aml.sum_over(G, lambda g: p[(g.values[0], t)] * 1.0)
    m.add_constraint(bal == load_mw[t], f"balance_{t}")

for g in gen_names:
    d = gen_data[g]
    for i, t in enumerate(t_names):
        # (2) 出力下界
        lb_c = 1.0 * p[(g, t)] - d["pmin"] * u[(g, t)]
        m.add_constraint(lb_c >= 0.0, f"pmin_{g}_{t}")

        # (3) 出力上界
        ub_c = d["pmax"] * u[(g, t)] - 1.0 * p[(g, t)]
        m.add_constraint(ub_c >= 0.0, f"pmax_{g}_{t}")

        if i == 0:
            # (4)/(5) 爬坡（与初始出力对比）
            m.add_constraint(
                1.0*p[(g,t)] - init_p[g] <= d["ramp"], f"ramp_up_{g}_{t}")
            m.add_constraint(
                init_p[g] - 1.0*p[(g,t)] <= d["ramp"], f"ramp_dn_{g}_{t}")
            # (6) 启停逻辑（与初始状态对比）
            logic = 1.0*su[(g,t)] - 1.0*sd[(g,t)] - 1.0*u[(g,t)]
            m.add_constraint(logic == -(float(init_u[g])), f"logic_{g}_{t}")
        else:
            prev_t = t_names[i - 1]
            # (4) 爬坡上升
            m.add_constraint(
                1.0*p[(g,t)] - 1.0*p[(g,prev_t)] <= d["ramp"],
                f"ramp_up_{g}_{t}")
            # (5) 爬坡下降
            m.add_constraint(
                1.0*p[(g,prev_t)] - 1.0*p[(g,t)] <= d["ramp"],
                f"ramp_dn_{g}_{t}")
            # (6) 启停逻辑
            logic = 1.0*su[(g,t)] - 1.0*sd[(g,t)] - 1.0*u[(g,t)] + 1.0*u[(g,prev_t)]
            m.add_constraint(logic == 0.0, f"logic_{g}_{t}")

        # (7) su + sd <= 1（同一时段不能同时启停）
        m.add_constraint(1.0*su[(g,t)] + 1.0*sd[(g,t)] <= 1.0, f"susd_{g}_{t}")

# ── 求解 ──────────────────────────────────────────────────────────────────────
print(f"模型规模: {m.num_vars} 变量, {m.num_constraints} 约束")
print("MILP求解器:", mipsolvers.engine.list_solvers("MILP"))

opts = aml.SolveOptions()
opts.verbosity   = 0
opts.mip_gap_tol = 0.001
opts.mip_gap_tol = 0.001
opts.solver_name = "NativeBranchAndCut"

result = m.solve(opts)

# ── 结果 ──────────────────────────────────────────────────────────────────────
print(f"\n状态:    {result.termination_status}")
print(f"总费用:  ${result.objective_value:,.2f}")
print(f"求解器:  {result.solver_used}")

print(f"\n{'时段':<6}", end="")
for t in t_names:
    print(f" {t:>6}", end="")
print()

print(f"{'负荷':<6}", end="")
for t in t_names:
    print(f" {load_mw[t]:>6.0f}", end="")
print(" MW")

for g in gen_names:
    u_row = [f"{result.var_value(u[(g,t)]):.0f}" for t in t_names]
    p_row = [f"{result.var_value(p[(g,t)]):>6.1f}" for t in t_names]
    su_row = [result.var_value(su[(g,t)]) for t in t_names]
    print(f"\n{g} 状态:", "  ".join(u_row))
    print(f"{g} 出力:", "".join(p_row), "MW")
    starts = [t_names[i] for i,v in enumerate(su_row) if v > 0.5]
    if starts:
        print(f"  → 启动于: {starts}")

total_energy_cost = sum(
    gen_data[g]["bid"] * result.var_value(p[(g,t)])
    for g in gen_names for t in t_names)
total_start_cost = sum(
    gen_data[g]["startup"] * result.var_value(su[(g,t)])
    for g in gen_names for t in t_names)
total_nolc = sum(
    gen_data[g]["no_load"] * result.var_value(u[(g,t)])
    for g in gen_names for t in t_names)
print(f"\n成本分解:")
print(f"  空载费用: ${total_nolc:,.2f}")
print(f"  发电费用: ${total_energy_cost:,.2f}")
print(f"  启动费用: ${total_start_cost:,.2f}")
