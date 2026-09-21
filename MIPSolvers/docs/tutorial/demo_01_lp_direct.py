#!/usr/bin/env python3.8
"""
Demo 01: 底层 engine 接口求解线性规划 (LP)

问题：生产计划
  两种产品 x1, x2，最大化利润  5*x1 + 4*x2
  约束：
    6*x1 + 4*x2 <= 24   (机器A时间，小时)
     x1 + 2*x2 <= 6    (机器B时间，小时)
    x1, x2 >= 0

等价最小化: min  -5*x1 - 4*x2
"""
import sys
sys.path.insert(0, '/Users/tianyangzhao/Codes/MIPSolvers/build_py')

import numpy as np
import mipsolvers

# ── 查询可用求解器 ────────────────────────────────────────────────────────────
print("可用LP求解器:", mipsolvers.engine.list_solvers("LP"))

# ── 构建模型 ──────────────────────────────────────────────────────────────────
# 目标向量（最小化，所以取负）
c = np.array([-5.0, -4.0])

# 不等式约束 A*x <= b
A = np.array([
    [6.0, 4.0],   # 机器A: 6*x1 + 4*x2 <= 24
    [1.0, 2.0],   # 机器B:  x1 + 2*x2 <= 6
])
b = np.array([24.0, 6.0])

# 变量下界 x >= 0
lb = np.zeros(2)

# ── 求解 ──────────────────────────────────────────────────────────────────────
res = mipsolvers.engine.solve_lp(
    c=c, A=A, b=b, lb=lb,
    solver="NativeLCQP"    # "Auto" | "HiGHS" | "Gurobi" | "NativeLCQP" | ...
)

# ── 输出结果 ──────────────────────────────────────────────────────────────────
print(f"\n求解状态:   {res['status']}")
print(f"最优利润:   {-res['objective']:.4f}")
print(f"x1 = {res['x'][0]:.4f},  x2 = {res['x'][1]:.4f}")
print(f"求解器:     {res['solver']}")
print(f"迭代次数:   {res['iterations']}")
print(f"耗时:       {res['runtime_sec']*1000:.2f} ms")

if 'duals' in res and len(res['duals']) > 0:
    print(f"\n约束影子价格 (对偶变量):")
    print(f"  机器A约束: {res['duals'][0]:.4f}")
    print(f"  机器B约束: {res['duals'][1]:.4f}")
    print("  (影子价格 > 0 表示该约束为紧约束)")

# 验证
assert res['success'], "求解失败！"
x1, x2 = res['x']
profit = 5*x1 + 4*x2
print(f"\n验证: 5*{x1:.4f} + 4*{x2:.4f} = {profit:.4f}")
print("PASS" if abs(profit - 21.0) < 1e-4 else "WARN: 期望最优值=21.0")
