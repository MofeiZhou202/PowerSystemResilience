#!/usr/bin/env python3.8
"""
Demo 03: AML 接口求解运输问题 (LP)

问题：两个工厂向三个客户供货，最小化总运费
  工厂供给量、客户需求量已知，求最优运输方案
"""
import sys
sys.path.insert(0, '/Users/tianyangzhao/Codes/MIPSolvers/build_py')

import mipsolvers
aml = mipsolvers.aml

# ── 数据 ──────────────────────────────────────────────────────────────────────
supply_data = {"工厂A": 120.0, "工厂B": 80.0}

demand_data = {"客户1": 70.0, "客户2": 90.0, "客户3": 40.0}

cost_data = {
    ("工厂A", "客户1"): 4.0, ("工厂A", "客户2"): 3.0, ("工厂A", "客户3"): 5.0,
    ("工厂B", "客户1"): 5.0, ("工厂B", "客户2"): 2.0, ("工厂B", "客户3"): 3.0,
}

# ── 建模 ──────────────────────────────────────────────────────────────────────
m = aml.Model("transport")

# 1. 集合
factories = m.add_set("factories", list(supply_data.keys()))
customers = m.add_set("customers", list(demand_data.keys()))

# 2. 参数
supply = m.add_param("supply", dim=1, unit="吨")
demand = m.add_param("demand", dim=1, unit="吨")
cost   = m.add_param("cost",   dim=2, unit="元/吨")

supply.load(supply_data)
demand.load(demand_data)
cost.load(cost_data)

# 3. 决策变量: x[i,j] = 工厂i运至客户j的数量 (吨)
x = m.add_var2("x", factories, customers,
               aml.VarType.Continuous, lb=0.0)

# 4. 目标：最小化总运费
m.minimize(aml.sum_over(factories, lambda i:
    aml.sum_over(customers, lambda j:
        cost[(i, j)] * x[(i, j)])))

# 5. 约束：工厂供给上限
m.add_constraints("supply_cap", factories, lambda i:
    aml.sum_over(customers, lambda j: x[(i, j)] * 1.0) <= supply[i])

# 6. 约束：满足客户需求
m.add_constraints("demand_req", customers, lambda j:
    aml.sum_over(factories, lambda i: x[(i, j)] * 1.0) >= demand[j])

# ── 求解 ──────────────────────────────────────────────────────────────────────
opts = aml.SolveOptions()
opts.solver_name = ""      # 自动选择
opts.verbosity   = 0

result = m.solve(opts)

# ── 结果 ──────────────────────────────────────────────────────────────────────
print(f"状态:     {result.termination_status}")
print(f"最优运费: ¥{result.objective_value:.2f}")
print(f"求解器:   {result.solver_used}")

vals = result.array_values(x)
print("\n最优运输方案（吨）：")
for (i, j), v in sorted(vals.items()):
    if v > 0.01:
        print(f"  {i} → {j}: {v:.1f}")

# 对偶变量（影子价格）
supply_ca = m._Model__constraint_arrays if hasattr(m, '_Model__constraint_arrays') else None
print("\n供给约束影子价格（资源稀缺性）:")
supply_carray = None
# Access via constraint array name lookup is done through result dual_array
# In AML, add_constraints returns ConstraintArray
m2 = aml.Model("transport2")
factories2 = m2.add_set("factories", list(supply_data.keys()))
customers2  = m2.add_set("customers", list(demand_data.keys()))
supply2 = m2.add_param("supply", dim=1); supply2.load(supply_data)
demand2 = m2.add_param("demand", dim=1); demand2.load(demand_data)
cost2   = m2.add_param("cost",   dim=2); cost2.load(cost_data)
x2 = m2.add_var2("x", factories2, customers2, aml.VarType.Continuous, lb=0.0)
m2.minimize(aml.sum_over(factories2, lambda i:
    aml.sum_over(customers2, lambda j: cost2[(i,j)] * x2[(i,j)])))
sc = m2.add_constraints("supply_cap", factories2, lambda i:
    aml.sum_over(customers2, lambda j: x2[(i,j)] * 1.0) <= supply2[i])
dc = m2.add_constraints("demand_req", customers2, lambda j:
    aml.sum_over(factories2, lambda i: x2[(i,j)] * 1.0) >= demand2[j])
r2 = m2.solve(opts)
supply_duals = r2.dual_array(sc)
demand_duals = r2.dual_array(dc)

for factory, dual in supply_duals.items():
    if dual is not None:
        print(f"  {factory}: {dual:.4f}  (负值表示供给紧张)")
print("需求约束影子价格（服务重要性）:")
for cust, dual in demand_duals.items():
    if dual is not None:
        print(f"  {cust}: {dual:.4f}")
