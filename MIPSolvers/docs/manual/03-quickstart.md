# 第 3 章 快速上手

> 本章整合自: docs/archive/user_manual.md, docs/tutorial/demo_01_lp_direct.py, docs/tutorial/demo_02_milp_knapsack.py, docs/tutorial/demo_03_aml_transport.py, docs/tutorial/demo_04_aml_scuc.py, docs/tutorial/demo_05_scuc_api.py

本章用四个递进的例子带你在半小时内跑通 MIPSolvers 的主要入口：

1. Python 数组接口求解 LP（约 5 分钟）；
2. Python 数组接口求解 MILP（0/1 背包）；
3. AML 代数建模（运输问题）；
4. SCUC 场景（AML 手工建模 + JSON API 一次调用）。

每个例子都给出完整可运行代码与预期输出要点。假设你已经完成第 2 章的构建与安装（见[安装与部署](02-installation-deploy.md)），尤其是构建出 Python 扩展模块。

## 3.1 准备工作：让 Python 找到扩展模块

Python 接口来自 CMake 构建出的扩展模块。如果还没有构建：

```bash
python -m pip install numpy pybind11
cmake -S . -B build-py \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_BUILD_PYTHON=ON \
  -DPYTHON_EXECUTABLE=/path/to/python
cmake --build build-py --target mipsolvers_py
```

把生成扩展所在目录加入 `PYTHONPATH`，然后验证：

```bash
python -c "import mipsolvers; print(mipsolvers.engine.list_solvers('LP'))"
```

注意事项：

- 必须用与扩展编译时 ABI 兼容的 Python 解释器；不能把另一个 Python 版本生成的 `.pyd`/`.so` 直接复制过来。
- Windows 下还需确保依赖 DLL 在扩展同目录或 `PATH` 中。
- `docs/tutorial/` 下的 demo 脚本开头有一行 `sys.path.insert(0, '<某机器的 build_py 路径>')`，这是作者本机路径，运行前请改成你自己的构建目录，或直接配置好 `PYTHONPATH` 后删除该行。

## 3.2 第一个 LP：生产计划（5 分钟）

对应脚本：`docs/tutorial/demo_01_lp_direct.py`。

问题：两种产品 `x1`、`x2`，最大化利润 `5*x1 + 4*x2`，受两台机器的工时约束：

```text
max  5*x1 + 4*x2
s.t. 6*x1 + 4*x2 <= 24   (机器A时间，小时)
      x1 + 2*x2 <=  6    (机器B时间，小时)
     x1, x2 >= 0
```

Python 数组接口默认最小化，因此目标取负。完整代码：

```python
import numpy as np
import mipsolvers

# 查询可用求解器
print("可用LP求解器:", mipsolvers.engine.list_solvers("LP"))

# 目标向量（最小化，所以取负）
c = np.array([-5.0, -4.0])

# 不等式约束 A*x <= b
A = np.array([
    [6.0, 4.0],   # 机器A: 6*x1 + 4*x2 <= 24
    [1.0, 2.0],   # 机器B:  x1 + 2*x2 <= 6
])
b = np.array([24.0, 6.0])
lb = np.zeros(2)

# 求解
res = mipsolvers.engine.solve_lp(c=c, A=A, b=b, lb=lb, solver="HiGHS")

# 输出结果
print(f"求解状态: {res['status']}")
print(f"最优利润: {-res['objective']:.4f}")
print(f"x1 = {res['x'][0]:.4f}, x2 = {res['x'][1]:.4f}")
print(f"求解器:   {res['solver']}")
print(f"耗时:     {res['runtime_sec']*1000:.2f} ms")

if 'duals' in res and len(res['duals']) > 0:
    print(f"机器A约束影子价格: {res['duals'][0]:.4f}")
    print(f"机器B约束影子价格: {res['duals'][1]:.4f}")

assert res['success'], "求解失败！"
profit = 5 * res['x'][0] + 4 * res['x'][1]
print("PASS" if abs(profit - 21.0) < 1e-4 else "WARN: 期望最优值=21.0")
```

预期输出要点：

- `status` 表示最优，最优利润为 `21.0`（`x1=3.0, x2=1.5`），脚本自检打印 `PASS`。
- 返回字典包含 `success, objective, x, status, solver, iterations, runtime_sec`；后端提供对偶时还有 `duals`（约束影子价格，大于 0 表示该约束为紧约束）。
- `solve_lp` 的可选参数为 `A, b, Aeq, beq, lb, ub, solver, maximize`。本例也可以用 `maximize=True` 直接传正的目标系数，不必手动取负。
- 工程惯例：不要只检查 `x` 是否非空，至少检查 `success` 和 `status`。

> 源 demo 使用 `solver="NativeLCQP"` 演示原生求解路径；按用户手册的后端建议，通用 LP 生产默认应选择 `HiGHS`，上面代码已按此调整。

## 3.3 第一个 MILP：0/1 背包

对应脚本：`docs/tutorial/demo_02_milp_knapsack.py`。

背包容量 10 kg，每件物品只能选 0 次或 1 次，最大化总价值：

```python
import numpy as np
import mipsolvers

# 数据：(名称, 重量, 价值)
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

# min  -values @ x   （最小化负价值 = 最大化价值）
# s.t. weights @ x <= W,  x[i] ∈ {0,1}
c = -values
A = weights.reshape(1, n)
b = np.array([W])
lb = np.zeros(n)
ub = np.ones(n)
vartypes = ["B"] * n   # 全部为 0/1 变量

print("可用MILP求解器:", mipsolvers.engine.list_solvers("MILP"))

res = mipsolvers.engine.solve_milp(
    c=c, A=A, b=b, lb=lb, ub=ub,
    vartypes=vartypes,
    solver="StrictHiGHS",
    mip_gap=1e-6,
)

print(f"求解状态: {res['status']}")
print(f"最优价值: {-res['objective']:.0f}")
print(f"MIP间隙:  {res['mip_gap']:.2e}")
print(f"求解器:   {res['solver']}")

selected = [(names[i], weights[i], values[i])
            for i in range(n) if res['x'][i] > 0.5]
total_w = sum(w for _, w, _ in selected)
total_v = sum(v for _, _, v in selected)
print(f"已选物品 (总重量={total_w:.0f}kg / {W:.0f}kg, 总价值={total_v:.0f}):")
for name, w, v in selected:
    print(f"  {name}  重量={w:.0f}kg  价值={v:.0f}")

# 与暴力搜索对比（仅适合这种玩具规模）
from itertools import combinations
best_v = 0.0
for r in range(n + 1):
    for sel in combinations(range(n), r):
        if sum(weights[i] for i in sel) <= W:
            best_v = max(best_v, sum(values[i] for i in sel))
assert abs(total_v - best_v) < 0.5, "MILP结果与暴力枚举不一致！"
print("PASS: MILP与暴力枚举结果一致")
```

预期输出要点：

- 最优价值为 `25`（选中斧头、书本、药品、食物，总重量恰好 10 kg），`mip_gap` 接近 0，末尾打印 `PASS`。
- `vartypes` 使用 `C/I/B`（连续/整数/0-1），提供时长度必须等于变量数。
- `mip_gap` 和 `time_limit_sec` 当前只保证由 `Auto`、`StrictHiGHS` 和 `NativeBranchAndCut` 路径执行；其他外部后端会在返回字典中带提示。
- 注意 MILP 的变量对偶通常没有定义，不要假设结果里总有 `duals`。

> 源 demo 使用 `solver="NativeBranchAndCut"`（自研树搜索，适合研究和定制回调）；上面改用用户手册推荐的 MILP 生产后端 `StrictHiGHS`。

## 3.4 AML 建模入门：运输问题

对应脚本：`docs/tutorial/demo_03_aml_transport.py`。

当模型涉及集合、下标和大量同类约束时，手工拼矩阵容易出错。AML（代数建模层）负责变量编号、矩阵编译和结果映射，是新业务模型的推荐入口。

问题：两个工厂向三个客户供货，工厂供给量、客户需求量、单位运费已知，最小化总运费：

```python
import mipsolvers
aml = mipsolvers.aml

# ── 数据 ──
supply_data = {"工厂A": 120.0, "工厂B": 80.0}
demand_data = {"客户1": 70.0, "客户2": 90.0, "客户3": 40.0}
cost_data = {
    ("工厂A", "客户1"): 4.0, ("工厂A", "客户2"): 3.0, ("工厂A", "客户3"): 5.0,
    ("工厂B", "客户1"): 5.0, ("工厂B", "客户2"): 2.0, ("工厂B", "客户3"): 3.0,
}

# ── 建模 ──
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

# 3. 决策变量: x[i,j] = 工厂 i 运至客户 j 的数量（吨）
x = m.add_var2("x", factories, customers, aml.VarType.Continuous, lb=0.0)

# 4. 目标：最小化总运费
m.minimize(aml.sum_over(factories, lambda i:
    aml.sum_over(customers, lambda j: cost[(i, j)] * x[(i, j)])))

# 5. 约束：工厂供给上限 / 满足客户需求
#    add_constraints 返回 ConstraintArray，之后可用它读对偶
sc = m.add_constraints("supply_cap", factories, lambda i:
    aml.sum_over(customers, lambda j: x[(i, j)] * 1.0) <= supply[i])
dc = m.add_constraints("demand_req", customers, lambda j:
    aml.sum_over(factories, lambda i: x[(i, j)] * 1.0) >= demand[j])

# ── 求解 ──
opts = aml.SolveOptions()
opts.solver_name = ""    # 空字符串 = 自动选择
opts.verbosity   = 0
result = m.solve(opts)

# ── 结果 ──
print(f"状态:     {result.termination_status}")
print(f"最优运费: ¥{result.objective_value:.2f}")
print(f"求解器:   {result.solver_used}")

vals = result.array_values(x)
print("\n最优运输方案（吨）：")
for (i, j), v in sorted(vals.items()):
    if v > 0.01:
        print(f"  {i} → {j}: {v:.1f}")

# 对偶变量（影子价格）
print("\n供给约束影子价格:")
for factory, dual in result.dual_array(sc).items():
    if dual is not None:
        print(f"  {factory}: {dual:.4f}")
print("需求约束影子价格:")
for cust, dual in result.dual_array(dc).items():
    if dual is not None:
        print(f"  {cust}: {dual:.4f}")
```

预期输出要点：

- 总供给 200 吨等于总需求 200 吨，最优运费为 `¥670.00`；最优方案为：工厂B→客户2 运 80 吨、工厂A→客户2 运 10 吨、工厂A→客户1 运 70 吨、工厂A→客户3 运 40 吨。
- `termination_status` 说明停止原因；`result.has_primal` / `is_optimal` / `has_duals` 是常用判定；读取变量用 `var_value`（单个）和 `array_values`（整组）。
- 想保留约束对偶时，务必接住 `add_constraints` 的返回值，再调 `result.dual_array(...)`；`dual`、`reduced_cost` 也可用。
- 线性 LP/MILP 可导出供审查：`model.write_lp("model.lp")`、`model.write_mps(...)`、`model.write_json(...)`。QP/NLP/MINLP/锥模型会明确拒绝导出 LP/MPS，不会静默丢结构。

AML 完整 API 见[第 4 章 AML 建模](04-modeling-aml.md)。

## 3.5 SCUC 场景入门

机组组合（SCUC）是电力系统优化的核心场景。这里给出两条路径：先用 AML 手工搭一个最小 SCUC 理解模型结构，再改用内置 JSON API 一次完成 UC + SCED + LMP。

### 3.5.1 用 AML 手工搭一个最小 SCUC

对应脚本：`docs/tutorial/demo_04_aml_scuc.py`。2 台机组、6 个时段，变量为开停状态 `u`、出力 `p`、启动/停机指示 `su/sd`：

```text
min  Σ_g Σ_t [ no_load[g]*u[g,t] + bid[g]*p[g,t] + startup[g]*su[g,t] ]
s.t. 功率平衡、出力上下界、爬坡、启停逻辑、su+sd<=1
```

核心建模代码（数据定义与结果打印见源脚本，此处保留关键片段）：

```python
import mipsolvers
aml = mipsolvers.aml

m = aml.Model("mini_scuc")
G     = m.add_set("G", ["G1", "G2"])
T_set = m.add_ordered_set("T", [f"t{i+1}" for i in range(6)])

u  = m.add_var2("u",  G, T_set, aml.VarType.Binary)
p  = m.add_var2("p",  G, T_set, aml.VarType.Continuous, lb=0.0)
su = m.add_var2("su", G, T_set, aml.VarType.Binary)
sd = m.add_var2("sd", G, T_set, aml.VarType.Binary)

# 目标：空载费用 + 发电费用 + 启动费用
m.minimize(aml.sum_over(G, lambda g:
    aml.sum_over(T_set, lambda t:
        gen_data[g.values[0]]["no_load"] * u[(g, t)] +
        gen_data[g.values[0]]["bid"]     * p[(g, t)] +
        gen_data[g.values[0]]["startup"] * su[(g, t)])))

# 功率平衡（每个时段一条）
for t in t_names:
    bal = aml.sum_over(G, lambda g: p[(g.values[0], t)] * 1.0)
    m.add_constraint(bal == load_mw[t], f"balance_{t}")

# 出力上下界、爬坡、启停逻辑等按 (g, t) 逐条 add_constraint，
# 首个时段与 init_u/init_p 给定的初始状态衔接，详见源脚本。

opts = aml.SolveOptions()
opts.verbosity    = 0
opts.mip_gap_tol  = 0.001
opts.solver_name  = "NativeBranchAndCut"
result = m.solve(opts)
```

预期输出要点：

- 模型规模 48 个变量、78 条约束；打印各时段负荷、每台机组的开停状态与出力、启动发生的时段，以及空载/发电/启动三项成本分解。
- 源脚本把 `su/sd` 用于计启动费用和启停逻辑，是教科书式写法；理解它之后，生产场景应切换到下面的专用接口，不要手工复制这些约束。

### 3.5.2 用 JSON API 一次求解 UC + SCED + LMP

对应脚本：`docs/tutorial/demo_05_scuc_api.py`。`mipsolvers.scuc.solve_json()` 接收 JSON 字符串、返回 JSON 字符串，自动完成机组组合、安全约束经济调度和边际电价计算。

输入的最小骨架（3 台机组、24 小时、含分段报价和负荷曲线；完整数据见源脚本）：

```python
import json
import mipsolvers

input_data = {
    "config": {
        "num_periods":      24,
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
            "name": "G1", "bus": 0,
            "pmin": 50.0, "pmax": 400.0,
            "ramp_up_mw_min": 200.0, "ramp_dn_mw_min": 200.0,
            "min_up_time_hr": 4.0,   "min_dn_time_hr": 2.0,
            "startup_cost": 5000.0,  "no_load_cost": 200.0,
            "bid_segments": [
                {"price": 20.0, "quantity": 150.0},
                {"price": 35.0, "quantity": 150.0},
                {"price": 50.0, "quantity": 100.0},
            ]
        },
        # ... G2、G3 同构，见 docs/tutorial/demo_05_scuc_api.py
    ],
    "loads": [{"bus": 0, "p_mw": 300.0}],
    "branches": [],                    # 不计算网络约束
    "profiles": {"load": [load_profile]}  # 每条负荷曲线对应一个 loads 元素
}

json_input  = json.dumps(input_data)
json_output = mipsolvers.scuc.solve_json(json_input, indent=2)
result      = json.loads(json_output)

scuc = result["scuc"]
print(f"求解状态:   {'成功' if scuc['converged'] else '失败'}")
print(f"总运行成本: ${scuc['cost']['total']:,.2f}")
print(f"  ├ 启动成本: ${scuc['cost']['startup']:,.2f}")
print(f"  └ 发电成本: ${scuc['cost']['energy']:,.2f}")
```

预期输出要点：

- `result["scuc"]` 内含 `converged`、`cost`（total/startup/energy）、`dispatch`（逐机组出力数组）、`commitment`（逐时段开停 0/1）和 `lmp`（逐时段边际电价）。
- 源脚本还打印逐时段「负荷 / 各机组出力 / LMP」表格、峰谷 LMP，以及每台机组的 24 位开停串和启动次数。

同样的算例也可以用命令行跑（见[第 8 章 工业应用](08-industrial-applications.md)）：

```powershell
scuc_case_builder --case 6bus --T 24 --wind --storage --output case.json
scuc_solve case.json result.json --solver StrictHiGHS --indent 2
```

`scuc_solve` 未指定输出文件时 JSON 写到标准输出、进度写到标准错误；SCUC 未收敛时进程返回非零退出码。输入字段的完整说明见[第 4 章 建模与数据 (AML)](04-modeling-aml.md) 的 SCUC 输入 JSON 规范（历史版本见 `docs/archive/data_format_spec.md`）。

## 3.6 tutorial 脚本的位置与运行方式

`docs/tutorial/` 目录内容：

| 文件 | 说明 |
|---|---|
| `demo_01_lp_direct.py` | engine 数组接口求解 LP（生产计划） |
| `demo_02_milp_knapsack.py` | engine 数组接口求解 MILP（0/1 背包） |
| `demo_03_aml_transport.py` | AML 求解运输问题（含约束对偶读取） |
| `demo_04_aml_scuc.py` | AML 手工建立简化 SCUC（2 机 6 时段） |
| `demo_05_scuc_api.py` | `mipsolvers.scuc.solve_json` 求解完整 SCUC（3 机 24 时段） |
| `slides.tex` / `build.sh` | 配套 Beamer 幻灯片及其编译脚本 |

运行 demo：先按 3.1 节构建 Python 模块并配置 `PYTHONPATH`（或修改脚本开头的 `sys.path.insert` 行），然后：

```bash
python docs/tutorial/demo_01_lp_direct.py
python docs/tutorial/demo_02_milp_knapsack.py
python docs/tutorial/demo_03_aml_transport.py
python docs/tutorial/demo_04_aml_scuc.py
python docs/tutorial/demo_05_scuc_api.py
```

注意：`docs/tutorial/build.sh` 不是用来运行 demo 的——它是幻灯片编译脚本，依赖 `xelatex`（TeX Live 或 MacTeX），运行 `bash build.sh` 会把 `slides.tex` 编译成 `slides.pdf`（编译两遍以修正交叉引用，`--open` 可选打开 PDF）。demo 脚本本身不需要它。

## 3.7 下一步

- 更系统地选择入口（C++ Engine / AML / Python 数组 / SCUC 专用接口）：[第 1 章 总览](01-overview.md)。
- 结果判读的完整字段表与检查模式、后端选择策略：[第 5 章 求解器与引擎](05-solvers-engines.md)。
- AML 的集合、参数、表达式与锥约束：[第 4 章 AML 建模](04-modeling-aml.md)。
- 生产部署前的精度、性能与可重复性清单：[第 10 章 常见问题与排查](10-troubleshooting.md)。
