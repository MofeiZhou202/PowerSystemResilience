#!/usr/bin/env python3.8
"""
Demo 02: 底层 engine 接口求解混合整数规划 (MILP)

问题：0/1 背包问题
  背包容量 W = 10
  选择物品使总价值最大，总重量不超过容量
  每件物品只能选0次或1次
"""
import sys
sys.path.insert(0, '/Users/tianyangzhao/Codes/MIPSolvers/build_py')

import numpy as np
import mipsolvers

# ── 数据 ──────────────────────────────────────────────────────────────────────
# (名称, 重量, 价值)
items = [
    ("斧头",   3.0,  9.0),
    ("书本",   4.0,  8.0),
    ("相机",   5.0, 10.0),
    ("药品",   2.0,  5.0),
    ("望远镜", 6.0, 12.0),
    ("食物",   1.0,  3.0),
]
W = 10.0  # 背包容量（公斤）

names   = [it[0] for it in items]
weights = np.array([it[1] for it in items])
values  = np.array([it[2] for it in items])
n = len(items)

# ── 构建 MILP ─────────────────────────────────────────────────────────────────
# min  -values @ x      (最小化负价值 = 最大化价值)
# s.t. weights @ x <= W
#      0 <= x[i] <= 1, x[i] ∈ {0,1}
c = -values                         # 目标（最小化）
A = weights.reshape(1, n)           # 重量约束
b = np.array([W])
lb = np.zeros(n)
ub = np.ones(n)
vartypes = ["B"] * n                # 全部为0/1变量

print("可用MILP求解器:", mipsolvers.engine.list_solvers("MILP"))

# ── 求解 ──────────────────────────────────────────────────────────────────────
res = mipsolvers.engine.solve_milp(
    c=c, A=A, b=b, lb=lb, ub=ub,
    vartypes=vartypes,
    solver="NativeBranchAndCut",
    mip_gap=1e-6
)

# ── 结果 ──────────────────────────────────────────────────────────────────────
print(f"\n求解状态:  {res['status']}")
print(f"最优价值:  {-res['objective']:.0f}")
print(f"MIP间隙:   {res['mip_gap']:.2e}")
print(f"求解器:    {res['solver']}")
print(f"耗时:      {res['runtime_sec']*1000:.2f} ms")

selected = [(names[i], weights[i], values[i])
            for i in range(n) if res['x'][i] > 0.5]
total_w = sum(w for _, w, _ in selected)
total_v = sum(v for _, _, v in selected)

print(f"\n已选物品  (总重量={total_w:.0f}kg / {W:.0f}kg, 总价值={total_v:.0f}):")
for name, w, v in selected:
    print(f"  {name:8s} 重量={w:.0f}kg  价值={v:.0f}")

# ── 与暴力搜索对比 ────────────────────────────────────────────────────────────
from itertools import combinations
best_v, best_sel = 0.0, []
for r in range(n + 1):
    for sel in combinations(range(n), r):
        if sum(weights[i] for i in sel) <= W:
            v = sum(values[i] for i in sel)
            if v > best_v:
                best_v, best_sel = v, sel

print(f"\n暴力枚举最优值: {best_v:.0f}")
assert abs(total_v - best_v) < 0.5, "MILP结果与暴力枚举不一致！"
print("PASS: MILP与暴力枚举结果一致")
