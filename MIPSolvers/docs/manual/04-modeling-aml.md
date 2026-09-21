# 第 4 章 建模与数据 (AML)

> 本章整合自: docs/archive/aml_python_api.md, docs/archive/case_builder.md, docs/archive/data_format_spec.md

本章面向集成商与现场工程师，介绍 MIPSolvers 的两条建模路径：

- **AML（Algebraic Modeling Library）**：Python 侧的高层代数建模层，用「集合 + 索引」方式声明变量、约束与目标，适用于 LP / QP / NLP / MILP，无需手工构造系数矩阵。
- **SCUC 数据格式**：电力系统机组组合（SCUC/SCED/LMP）的标准 JSON 输入/输出规范与内置算例构造器，适用于电力调度场景的直接对接。

求解器选型见 [求解器与引擎](05-solvers-engines.md)，API 速查见 [API 参考](07-api-reference.md)。

---

## 4.1 AML 代数建模层

### 4.1.1 快速上手

AML 通过 `mipsolvers.aml` 子模块访问。下面是一个完整的运输问题示例，涵盖 AML 的全部核心概念：集合、参数、变量、目标、约束族、求解与结果提取。

```python
import mipsolvers
aml = mipsolvers.aml

m = aml.Model("transport")

# 集合
plants    = m.add_set("plants",    ["P1", "P2"])
customers = m.add_set("customers", ["C1", "C2", "C3"])

# 参数
supply = m.add_param("supply", dim=1, unit="units")
demand = m.add_param("demand", dim=1, unit="units")
cost   = m.add_param("cost",   dim=2, unit="$/unit")

supply.load({"P1": 100.0, "P2": 150.0})
demand.load({"C1": 80.0, "C2": 90.0, "C3": 70.0})
cost.load({
    ("P1","C1"): 2.0, ("P1","C2"): 3.0, ("P1","C3"): 5.0,
    ("P2","C1"): 4.0, ("P2","C2"): 1.0, ("P2","C3"): 2.0,
})

# 变量
x = m.add_var2("x", plants, customers, aml.VarType.Continuous, lb=0.0)

# 目标
m.minimize(aml.sum_over(plants, lambda i:
    aml.sum_over(customers, lambda j:
        cost[i, j] * x[i, j])))

# 约束族
m.add_constraints("supply", plants, lambda i:
    aml.sum_over(customers, lambda j: x[i, j] * 1.0) <= supply[i])
m.add_constraints("demand", customers, lambda j:
    aml.sum_over(plants, lambda i: x[i, j] * 1.0) >= demand[j])

result = m.solve()
print(result)                    # <SolveResult: obj=..., OPTIMAL>
print(result.array_values(x))    # {('P1','C1'): ..., ...}
```

### 4.1.2 Key —— 多维索引

集合、参数、变量均以 `Key` 对象为索引。`Key` 可哈希，支持 `==`、`<`，可直接用作 dict/set 键。

```python
Key(atoms)                 # 从 str、tuple[str,...] 或 list[str] 构造
Key.scalar("P1")           # 一维键
Key.pair("P1", "C1")       # 二维键
Key.make(["a", "b", "c"])  # N 维键
```

| 属性 | 类型 | 说明 |
|---|---|---|
| `values` | `list[str]` | 底层原子列表 |
| `dimension` | `int` | 原子个数 |

**字符串简写**：任何需要 `Key` 的位置都可直接传 `str`（标量键）或 `tuple[str, ...]` / `list[str]`（多维键）。

### 4.1.3 集合与索引

`Model.add_set()` 返回 `ExplicitSet`（插入有序、用户管理）；`Model.add_ordered_set()` 返回 `OrderedSet`，在 `ExplicitSet` 之上增加位置导航，适合时间周期索引。

```python
s.name          # str
s.dimension     # int —— 键的维度
s.cardinality   # int —— 元素个数
s.elements      # list[Key] —— 按插入顺序
s.contains(key) # bool
s.add_element(key) -> bool   # 新增返回 True
"P1" in s       # __contains__
len(s); for k in s: ...      # __len__ / __iter__

# OrderedSet 额外方法（0 基位置）
s.at(pos)           # 该位置的 Key
s.position(key)     # int；不存在则抛异常
s.prev(key)         # 前一元素 Key 或 None
s.next(key)         # 后一元素 Key 或 None
```

### 4.1.4 参数（Parameter）

由 `Model.add_param()` / `Model.add_param_scalar()` 创建，是以 `Key` 为索引的 `float` 命名表。

```python
p.name        # str
p.dimension   # int
p.unit        # str

# 写入
p.set(key, value)
p.set_scalar(value)        # 仅 dim-0
p[key] = value             # __setitem__
p.load({"k1": 1.0, ...})   # 从 dict 批量装载

# 读取
p.get(key) -> float        # 缺失则抛异常
p.get_scalar() -> float    # 仅 dim-0
p.get_or(key, default=0.0) -> float
p.contains(key) -> bool
p[key]                     # __getitem__
p.to_dict() -> dict        # 全部值 {str|tuple: float}
```

**缺失键策略（MissingPolicy）**：`Parameter.get()` 遇到缺失键时默认 `Error`（抛异常）；枚举中另有 `ReturnZero`（静默返回 0.0），但当前版本尚不能按参数单独配置。

### 4.1.5 变量

**VarType 枚举**：

| 值 | 含义 |
|---|---|
| `VarType.Continuous` | 连续实数 |
| `VarType.Integer` | 整数 |
| `VarType.Binary` | 0/1 |

**创建与索引**：`Model.add_var()` / `Model.add_var2()` 返回 `VarArray`（集合上的变量族）；对其索引得到单个标量变量的引用 `VarRef`。

```python
a = m.add_var(name, domain, type, lb=-1e20, ub=1e20)               # VarArray
a = m.add_var2(name, domain_a, domain_b, type, lb=-1e20, ub=1e20)  # 二维域

a.name          # str
a.type          # VarType
a.total_count   # int

a["P1"]              # VarRef —— 一维查找
a[("P1", "C1")]      # VarRef —— 二维查找（tuple 简写）
a.set_lb(key, lb); a.set_ub(key, ub)
a.fix(key, value)    # 固定（lb == ub）
a.lb(key) -> float; a.ub(key) -> float
```

`VarRef` 的关键属性是 `v.id`（编译后矩阵中的列索引）。`VarRef` 支持算术（`2.0*v`、`v+w`、`-v` 等，返回 `LinearExpr`）与比较运算（`v <= 5.0` 等，返回 `TempConstr`）。

### 4.1.6 表达式与约束

- **`LinearExpr`**：稀疏仿射表达式 $c_0 + \sum_i c_i x_i$。支持 `LinearExpr()`、`LinearExpr.from_var(var_id, coef)`、`LinearExpr.const_expr(c)`、读写属性 `constant`、加减与标量乘、`+=`，以及比较运算（生成 `TempConstr`）。
- **`QuadExpr`**：线性部分加二次项。工厂方法：`QuadExpr.sq(var_id, coef)`（$c\,x_i^2$）、`QuadExpr.bilinear(var_i, var_j, coef)`（$c\,x_i x_j$）、`QuadExpr.from_linear(lin)`。
- **`NonlinearExpr`**：模型非线性表达式树中的不透明句柄，仅能通过 `Model.nl_*` 工厂方法创建。属性：`e.id`（arena 节点 ID）、`e.is_null()`。
- **`TempConstr`**：由表达式比较运算生成的临时约束，直接传给 `Model.add_constraint()`。

```python
x["P1"] + x["P2"] <= supply["P1"]   # 产生 TempConstr
```

约束的引用类型：`ConstraintRef`（`c.id`、`c.name`）与 `ConstraintArray`（`Model.add_constraints()` 返回的约束族，可按键查找 `a["C1"]`、`a[("i","j")]`）。

### 4.1.7 目标与求解

```python
m.minimize(obj) / m.maximize(obj)           # LinearExpr
m.minimize_quad(obj) / m.maximize_quad(obj) # QuadExpr
m.minimize_nl(obj) / m.maximize_nl(obj)     # NonlinearExpr

m.add_constraint(temp_constr, name="") -> ConstraintRef
m.add_constraints(family, domain, fn)  -> ConstraintArray
# fn: Key -> TempConstr，例如 m.add_constraints("balance", nodes, lambda i: flow[i] == 0)
m.add_nl_constraint(name, expr, sense="==", rhs=0.0) -> ConstraintRef
# sense: "<=", ">=", "==", "le", "ge", "eq"
```

**求解选项与调用**：

```python
opts = aml.SolveOptions()
opts.solver_name    = ""       # "" = 自动；可选 "highs"、"gurobi"、"native" 等
opts.time_limit_sec = 1e30
opts.mip_gap_tol    = 1e-4
opts.verbosity      = 0        # 0=静默, 1=摘要, 2=详细

result = m.solve()        # 或 m.solve(opts)
```

**SolveResult** 主要字段与方法：

```python
res.termination_status   # TerminationStatus 枚举
res.primal_status / res.dual_status
res.is_optimal / res.has_primal / res.has_duals   # bool
res.objective_value      # float
res.objective_bound      # float（MIP 的 LP 松弛界）
res.optimality_gap       # float
res.solve_time_sec       # float
res.simplex_iterations   # int
res.branch_and_cut_nodes # int
res.solver_used          # str

res.var_value(var_ref)        -> float
res.var_value_by_id(var_id)   -> float
res.array_values(var_array)   -> dict   # {str|tuple: float}
# 对偶（不可用时为 None）
res.dual(constraint_ref)      -> float | None
res.dual_array(constraint_array) -> dict
res.reduced_cost(var_ref)     -> float | None
```

`TerminationStatus` 取值：`Optimal`, `Infeasible`, `Unbounded`, `InfeasibleOrUnbounded`, `TimeLimit`, `IterationLimit`, `NodeLimit`, `ObjectiveLimit`, `NumericalError`, `UserInterrupt`, `SolverError`, `Unknown`。`PrimalStatus` / `DualStatus` 取值：`Optimal`, `Feasible`, `Infeasible`, `NoSolution`, `Unknown`。

### 4.1.8 非线性建模与暖启动

所有 `nl_*` 方法均返回 `NonlinearExpr`，在 `Model` 实例上调用：

```python
m.nl_var(v)          # VarRef -> NonlinearExpr
m.nl_const(c)        # float  -> NonlinearExpr
m.nl_neg(a)
m.nl_add(a, b); m.nl_sub(a, b)
m.nl_mul(a, b); m.nl_div(a, b)
m.nl_sq(a)           # a^2
m.nl_pow(a, n)       # a^n（n 为 NonlinearExpr）
m.nl_sqrt(a)
m.nl_exp(a); m.nl_log(a)
m.nl_sin(a); m.nl_cos(a); m.nl_tan(a)
m.nl_abs(a)
m.nl_max(a, b); m.nl_min(a, b)

m.set_nlp_x0(x0: list[float])   # NLP 求解器初值
```

Rosenbrock 函数最小化示例：

```python
m = aml.Model("nlp")
vars_ = m.add_set("vars", ["x", "y"])
v = m.add_var("v", vars_, aml.VarType.Continuous, lb=-10.0, ub=10.0)

x, y = m.nl_var(v["x"]), m.nl_var(v["y"])
one_minus_x = m.nl_sub(m.nl_const(1.0), x)
y_minus_xsq = m.nl_sub(y, m.nl_sq(x))
obj = m.nl_add(
    m.nl_sq(one_minus_x),
    m.nl_mul(m.nl_const(100.0), m.nl_sq(y_minus_xsq))
)
m.minimize_nl(obj)
m.set_nlp_x0([0.5, 0.5])
res = m.solve()
```

### 4.1.9 诊断、导出与辅助函数

```python
m.print_summary()
m.check_bounds()            # 存在 lb > ub 时抛异常
m.check_missing_params()    # 标量未设值、索引表为空或存在非有限值时抛异常；
                            # 参数声明只有维度、无 Set，故无法证明索引域全覆盖

m.write_lp(path)     # 仅线性 LP/MILP；不支持的模型类别抛异常
m.write_mps(path)    # free MPS，含整数性与目标常数项
m.write_json(path)   # Beta

aml.sum_over(set, fn)       -> LinearExpr   # fn: Key -> LinearExpr
aml.sum_over_quad(set, fn)  -> QuadExpr     # fn: Key -> QuadExpr
```

### 4.1.10 更多示例

**整数生产计划（MILP）**：

```python
m = aml.Model("prod")
products = m.add_set("products", ["A", "B", "C"])

profit   = m.add_param("profit", dim=1, unit="$/unit")
capacity = m.add_param_scalar("capacity", unit="hours")
hours    = m.add_param("hours", dim=1, unit="hours/unit")
profit.load({"A": 25.0, "B": 30.0, "C": 15.0})
capacity.set_scalar(100.0)
hours.load({"A": 2.0, "B": 3.0, "C": 1.0})

x = m.add_var("x", products, aml.VarType.Integer, lb=0.0)
m.maximize(aml.sum_over(products, lambda p: profit[p] * x[p]))
m.add_constraint(
    aml.sum_over(products, lambda p: hours[p] * x[p]) <= capacity.get_scalar(),
    "capacity")

res = m.solve()
if res.is_optimal:
    for p in products:
        print(p.values[0], res.var_value(x[p]))
```

**时段索引储能（OrderedSet 的 prev/next 用法）**：

```python
m = aml.Model("storage")
T = m.add_ordered_set("T", [f"t{i}" for i in range(1, 25)])

price = m.add_param("price", dim=1, unit="$/MWh")
price.load({f"t{i}": float(i % 12 + 1) * 10.0 for i in range(1, 25)})

charge    = m.add_var("charge",    T, aml.VarType.Continuous, lb=0.0, ub=50.0)
discharge = m.add_var("discharge", T, aml.VarType.Continuous, lb=0.0, ub=50.0)
soc       = m.add_var("soc",       T, aml.VarType.Continuous, lb=0.0, ub=200.0)

m.maximize(aml.sum_over(T, lambda t:
    price[t] * discharge[t] - 0.8 * price[t] * charge[t]))

for t in T:
    k = t.values[0]
    prev = T.prev(t)
    if prev is None:
        m.add_constraint(soc[k] == charge[k] - discharge[k], f"soc_init_{k}")
    else:
        prev_k = prev.values[0]
        m.add_constraint(soc[k] == soc[prev_k] + charge[k] - discharge[k], f"soc_{k}")
```

---

## 4.2 SCUC 输入数据格式规范

SCUC 模块（`mipsolvers::scuc`）以 JSON 为交换格式。对应头文件 `include/mipsolvers/scuc/scuc.hpp`，实现 `src/scuc/scuc.cpp`。

### 4.2.1 总体约定

| 约定 | 说明 |
|------|------|
| 索引 | 母线编号、机组索引均从 **0** 开始 |
| 时间步 | 计划周期共 `T = config.num_periods` 步，步长 `Δt = config.period_length_hr`（小时） |
| 提交时间步 | `T_commit = T / intervals_per_hour`（取整，默认 `intervals_per_hour = 1`） |
| 单位 | 功率 **MW**；能量 **MWh**；成本 **美元（$）**；LMP **$/MWh** |
| JSON 编码 | UTF-8；浮点数不限精度；数组用 `[]` |
| 字段缺省 | 所有字段均可选（有默认值），未提供时取下表默认值 |
| 无效值处理 | 运行时**不抛异常**（仅 `json::parse` 格式错误除外）；越界数值被 clamp 至有效区间 |

> **异常边界**：`scuc_from_json` 对非法 JSON 语法抛 `nlohmann::json::parse_error`，对字段类型不匹配抛 `nlohmann::json::type_error`。调用方应在外层捕获这两类异常。

### 4.2.2 顶层结构

```json
{
  "num_buses":         <int, 可选>,
  "config":            { ... },
  "generators":        [ ... ],
  "branches":          [ ... ],
  "loads":             [ ... ],
  "wind":              [ ... ],
  "solar":             [ ... ],
  "storage":           [ ... ],
  "dc_lines":          [ ... ],
  "generator_groups":  [ ... ],
  "sections":          [ ... ],
  "profiles":          { ... },
  "initial_status":    { ... }
}
```

`num_buses` 可省略：未提供时自动推断为所有组件中最大母线编号 + 1（至少为 1）。其余字段均可整体省略（等价于空列表或默认值）。

### 4.2.3 `config` —— 求解配置

| 字段 | 类型 | 默认值 | 有效范围 | 说明 |
|------|------|--------|----------|------|
| `solver` | `string` | `"Auto"` | `"Auto"`, `"StrictHiGHS"`, `"Gurobi"`, `"HiGHS"`, `"NativeBranchAndCut"`, `"SCIP"` | MILP 求解器。`"Auto"` 优先级：Gurobi → StrictHiGHS → HiGHS → NativeBranchAndCut；`"SCIP"` 仅支持 MINLP，不适用于 MILP 调度 |
| `allow_fallback` | `bool` | `true` | — | 主求解器失败时是否自动降级 |
| `num_periods` | `int` | `24` | ≥ 1 | 时间步数 T |
| `period_length_hr` | `float` | `1.0` | > 0 | 每时间步小时数 Δt |
| `n_segments` | `int` | `3` | ≥ 1 | 每台机组报价分段数（取 `min(n_segments, bid_segments.size())`） |
| `mip_gap` | `float` | `0.001` | [0, 1] | 相对 MIP 最优性间隙 |
| `time_limit_sec` | `float` | `300.0` | > 0 | 求解时间上限（秒） |
| `verbose` | `bool` | `false` | — | 是否输出求解器详细日志 |
| `spinning_reserve_req` | `float` | `0.10` | [0, 1] | 旋转备用需求（系统负荷分数） |
| `regulation_up_req` | `float` | `0.05` | [0, 1] | 调频上备用需求（系统负荷分数） |
| `regulation_down_req` | `float` | `0.05` | [0, 1] | 调频下备用需求（系统负荷分数） |
| `neg_reserve_req` | `float` | `0.0` | [0, 1] | 向下备用需求分数；0 = 不约束 |
| `pfr_reserve_req_mw` | `float` | `0.0` | ≥ 0 | 一次调频备用总量（MW）；0 = 不约束 |
| `voll` | `float` | `10000.0` | > 0 | 失负荷价值（$/MWh，切负荷惩罚） |
| `vocc` | `float` | `1000.0` | > 0 | 系统级弃电惩罚（$/MWh） |
| `renewable_min_output_coeff` | `float` | `0.0` | [0, 1] | 可再生能源最小出力系数 α（P_w ≥ α × 预测值） |
| `M2_renewable_curtail_penalty` | `float` | `0.0` | ≥ 0 | 可再生能源弃电惩罚（$/MW/时间步）；> 0 时激活弃电变量 |
| `M1_line_slack_penalty` | `float` | `1e5` | > 0 | 线路/断面越限松弛变量惩罚系数（Big-M） |
| `wheeling_fee_per_mwh` | `float` | `0.0` | ≥ 0 | 输电费（$/MWh，对所有火电出力叠加） |
| `enable_market_cuts` | `bool` | `true` | — | 是否在 SCUC 前添加预构型削减切面 |
| `solve_sced` | `bool` | `true` | — | 是否在 SCUC 后以固定机组状态求解 SCED（LP） |
| `solve_lmp` | `bool` | `true` | — | 是否在 SCED 后计算节点边际电价（LMP） |
| `lmp_delta` | `float` | `0.10` | [0, 1] | LMP 再调度邻域因子 δ |

### 4.2.4 `generators` —— 机组参数

```json
"generators": [
  {
    "name": "G1", "bus": 0,
    "pmin": 100.0, "pmax": 400.0,
    "ramp_up_mw_min": 5.0, "ramp_dn_mw_min": 5.0,
    "min_up_time_hr": 4.0, "min_dn_time_hr": 4.0,
    "startup_cost": 5000.0, "no_load_cost": 200.0,
    "bid_segments": [
      { "price": 25.0, "quantity": 100.0 },
      { "price": 30.0, "quantity": 150.0 },
      { "price": 38.0, "quantity": 50.0  }
    ]
  }
]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `name` | `string` | `""` | 机组标识名称 |
| `bus` | `int` | `0` | 接入母线（0 基），范围 [0, num_buses-1] |
| `pmin` / `pmax` | `float` | `0.0` | 最小/最大出力（MW），pmax ≥ pmin |
| `ramp_up_mw_min` / `ramp_dn_mw_min` | `float` | `0.0` | 爬坡/降坡速率（MW/min） |
| `min_up_time_hr` / `min_dn_time_hr` | `float` | `0.0` | 最小在线/停机时长（小时） |
| `must_run` | `bool` | `false` | 强制运行：将提交变量下界固定为 1 |
| `max_startups` / `max_shutdowns` | `int` | `0` | 规划期内最大启动/停机次数；0 = 不限 |
| `startup_cost` | `float` | `0.0` | 热启动费用（$）；也作为 warm/cold 的回退值 |
| `startup_cost_warm` / `startup_cost_cold` | `float` | `0.0` | 温/冷启动费用（$）；0 = 使用 `startup_cost` |
| `hot_start_threshold_hr` | `float` | `4.0` | 停机 ≤ 此值视为热启动 |
| `warm_start_threshold_hr` | `float` | `8.0` | 停机 ≤ 此值（> 热启动阈值）视为温启动 |
| `ud_periods` / `dd_periods` | `int` | `0` | 启动轨迹周期数（并网到 Pmin）/ 停机轨迹周期数（Pmin 到离网） |
| `no_load_cost` | `float` | `0.0` | 空载成本（$/h，在线时按时间计费） |
| `spinning_reserve_price` | `float` | `0.0` | 旋转备用报价（$/MWh） |
| `regulation_up_price` / `regulation_down_price` | `float` | `0.0` | 调频上/下备用报价（$/MWh） |
| `pfr_alpha` | `float` | `0.0` | 一次调频系数：最大一次调频 = pfr_alpha × pmax；0 = 不提供 |
| `group_id` | `int` | `-1` | 所属机组群 ID；-1 = 不属于任何群 |
| `bid_segments` | `array` | `[]` | 分段报价曲线，元素为 `{price, quantity}` |

**`bid_segments` 约定**：

- `price`（$/MWh）为第一个字段，`quantity`（MW，**相对于 pmin 的增量**）为第二个字段。
- 实际使用分段数 = `min(n_segments, bid_segments.size())`。
- 若为空，机组能以零成本出力（不推荐用于生产）。
- 各段 `quantity` 之和应等于 `pmax - pmin`；超出部分被忽略。最大总出力 = Pmin + Σ quantity_k（受 Pmax 上限约束）。
- 分段无需单调排序，但物理上应价格递增（调度优先使用低价分段）。

### 4.2.5 `branches` —— 输电线路

```json
"branches": [
  { "from": 0, "to": 1, "reactance": 0.05, "rating_mw": 200.0, "in_service": true }
]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `from` / `to` | `int` | `0` | 始端/末端母线（0 基） |
| `reactance` | `float` | `0.05` | 线路电抗（标幺值）；若 ≤ 1e-6，内部以 1e-3 替代 |
| `rating_mw` | `float` | `1e6` | 热稳极限（MW）；默认 1e6 ≈ 无限制 |
| `in_service` | `bool` | `true` | `false` 时该线路从 PTDF 计算中排除 |

> **铜排模式**：`branches` 为空列表时，系统以单母线（铜排）模式运行，不计算 PTDF，不施加线路潮流约束。

### 4.2.6 `loads` —— 负荷节点

```json
"loads": [ { "bus": 2, "p_mw": 300.0 } ]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `bus` | `int` | `0` | 负荷接入母线（0 基） |
| `p_mw` | `float` | `0.0` | 基准有功需求（MW）；运行时乘以 `profiles.load[d][t]` |

`profiles.load` 的行数应与 `loads.size()` 一致；行数不足时，缺失时间步采用基准值 `p_mw`（倍率默认 1.0）。

### 4.2.7 `wind` / `solar` —— 风光机组

```json
"wind":  [ { "bus": 5,  "pmax": 400.0 } ],
"solar": [ { "bus": 21, "pmax": 250.0 } ]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `bus` | `int` | `0` | 接入母线（0 基） |
| `pmax` | `float` | `0.0` | 额定装机容量（MW），仅在曲线缺失时作为回退值 |

- `profiles.wind[w][t]` / `profiles.solar[s][t]` 为第 w/s 台机组在第 t 步的**绝对预测出力（MW）**，应满足 0 ≤ 值 ≤ `pmax`。
- 行数不足时缺失单元以 `pmax` 替代。
- 风光出力上界 = 各时间步预测值；下界 = `renewable_min_output_coeff × 预测值`。

### 4.2.8 `storage` —— 储能机组

```json
"storage": [
  {
    "bus": 3,
    "pmax_charge": 200.0, "pmax_discharge": 200.0,
    "energy_capacity_mwh": 800.0,
    "efficiency": 0.90, "soc_init": 0.5,
    "charge_bid_price": 5.0, "discharge_bid_price": 20.0
  }
]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `bus` | `int` | `0` | 接入母线（0 基） |
| `pmax_charge` / `pmax_discharge` | `float` | `0.0` | 最大充/放电功率（MW） |
| `pmin_charge` / `pmin_discharge` | `float` | `0.0` | 最小充/放电功率（MW，工作时强制下界） |
| `energy_capacity_mwh` | `float` | `0.0` | 额定能量容量（MWh） |
| `efficiency` | `float` | `0.9` | 往返效率（传统字段），内部 clamp 至 [0.1, 1.0]；若 `eta_charge/discharge < 0`，则 η_ch = η_dis = √efficiency |
| `eta_charge` / `eta_discharge` | `float` | `-1.0` | 充/放电效率，内部 clamp 至 [0.01, 1.0]；< 0 表示从 `efficiency` 推算 |
| `soc_init` | `float` | `0.5` | 初始 SOC（容量分数）；可被 `initial_status.storage_soc` 覆盖 |
| `soc_min` | `float` | `-1.0` | SOC 下限分数；< 0 = 默认 0.10 |
| `soc_final` | `float` | `-1.0` | 期末 SOC 约束；< 0 = 等于初始 SOC |
| `charge_bid_price` / `discharge_bid_price` | `float` | `0.0` | 充电报价 / 放电报价（$/MWh） |
| `cycle_limit` | `float` | `0.0` | 每日等效满充满放次数上限；0 = 不限 |
| `use_binary_indicators` | `bool` | `false` | 使用充/放电二进制模式变量（ξ+/ξ-），精确互斥但增加模型规模 |

**SOC 初始值覆盖优先级**：`initial_status.storage_soc[s]` > `soc_init`（当 `storage_soc` 提供且长度足够时）。

### 4.2.9 `dc_lines` —— 直流线路

```json
"dc_lines": [
  { "from": 0, "to": 10, "pmin": -500.0, "pmax": 500.0, "ramp_up": 200.0, "ramp_dn": 200.0 }
]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `from` / `to` | `int` | `0` | 注入/抽出母线（0 基） |
| `pmin` / `pmax` | `float` | `-1e6` / `1e6` | 最小/最大传输功率（MW，负值表示反向） |
| `ramp_up` / `ramp_dn` | `float` | `1e6` | 爬坡/降坡上限（MW/步） |

### 4.2.10 `generator_groups` —— 机组群约束

用于建模抽水蓄能、发电-抽水联合等需共享出力限制的机组集合。

```json
"generator_groups": [
  {
    "id": 0, "name": "PSH-Group-1",
    "gen_indices": [2, 3],
    "pmin_t": [100.0], "pmax_t": [600.0],
    "emin": 0.0, "emax": 4800.0
  }
]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `id` | `int` | `-1` | 群 ID（与 `generator.group_id` 对应） |
| `name` | `string` | `""` | 群名称 |
| `gen_indices` | `int[]` | `[]` | 成员机组在 `generators` 数组中的 0 基索引 |
| `pmin_t` / `pmax_t` | `float[]` | `[]` | 群总出力下限/上限（MW）；长度为 T 或 1（标量广播）；空 = 不约束 |
| `emin` / `emax` | `float` | `0.0` | 规划期内群累计出力下限/上限（MWh）；0 = 不约束 |

### 4.2.11 `sections` —— 断面监视

```json
"sections": [
  {
    "name": "WestCorridor",
    "rating_fwd_mw": 800.0, "rating_rev_mw": 600.0,
    "line_weights": [ [0, 1.0], [3, 0.5] ]
  }
]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `name` | `string` | `""` | 断面名称 |
| `rating_fwd_mw` / `rating_rev_mw` | `float` | `1e6` | 正向/反向限额（MW）；默认无限制 |
| `line_weights` | 见下 | `[]` | 支路权重列表 |

`line_weights` 支持两种格式：数组格式 `[branch_index, weight]`，或对象格式 `{"line": branch_index, "weight": weight}`。断面潮流 = Σ weight_l × 支路 l 潮流。

### 4.2.12 `profiles` —— 时序曲线

```json
"profiles": {
  "load":  [ [1.0, 0.95, 0.9, ...], [0.8, 0.75, ...] ],
  "wind":  [ [120.0, 150.0, ...] ],
  "solar": [ [0.0, 0.0, 50.0, ...] ]
}
```

| 字段 | 类型 | 维度 | 默认行为 | 说明 |
|------|------|------|----------|------|
| `load` | `float[][]` | `[nload][T]` | 缺失时负荷固定为 `p_mw` | 负荷倍率（标幺值）；`load[d][t]` 乘以 `loads[d].p_mw` |
| `wind` | `float[][]` | `[nwind][T]` | 缺失时以 `pmax` 为预测值 | 风机绝对预测出力（MW） |
| `solar` | `float[][]` | `[nsolar][T]` | 缺失时以 `pmax` 为预测值 | 光伏绝对预测出力（MW） |

行数不足时超出索引的机组使用其 `pmax`（负荷用 `p_mw`）；列数不足（时间步缺失）时超出时间步使用最后一列值或基准值。

### 4.2.13 `initial_status` —— 初始状态

```json
"initial_status": {
  "commitment":    [1.0, 1.0, 0.0, ...],
  "dispatch":      [500.0, 300.0, 0.0, ...],
  "storage_soc":   [0.5, 0.6],
  "time_in_state": [8.0, -4.0, 3.0, ...]
}
```

| 字段 | 类型 | 维度 | 默认值 | 说明 |
|------|------|------|--------|------|
| `commitment` | `float[]` | `[ng]` | 全零（全停） | 第 0 周期前机组状态（0 或 1），内部 clamp 至 [0, 1] |
| `dispatch` | `float[]` | `[ng]` | 全零 | 第 0 周期机组出力（MW）；负值内部取 0 |
| `storage_soc` | `float[]` | `[nstorage]` | 用 `soc_init` | 储能初始 SOC；≤ 1+ε 视为分数，否则视为 MWh |
| `time_in_state` | `float[]` | `[ng]` | 全零 | 已连续处于当前状态的小时数；正值 = 在线，负值 = 停机；用于热/温/冷启动判断 |

数组长度不足时，超出索引的机组使用字段默认值，不报错。

### 4.2.14 最小完整示例

单母线、2 机、1 负荷、3 周期的最小可运行输入：

```json
{
  "config": {
    "num_periods": 3, "period_length_hr": 1.0,
    "n_segments": 2, "mip_gap": 0.001, "time_limit_sec": 60.0,
    "solve_sced": true, "solve_lmp": false
  },
  "generators": [
    {
      "name": "Coal-1", "bus": 0, "pmin": 100.0, "pmax": 400.0,
      "ramp_up_mw_min": 5.0, "ramp_dn_mw_min": 5.0,
      "min_up_time_hr": 2.0, "min_dn_time_hr": 2.0,
      "startup_cost": 3000.0, "no_load_cost": 150.0,
      "bid_segments": [
        { "price": 20.0, "quantity": 150.0 },
        { "price": 28.0, "quantity": 150.0 }
      ]
    },
    {
      "name": "Gas-1", "bus": 0, "pmin": 50.0, "pmax": 200.0,
      "ramp_up_mw_min": 10.0, "ramp_dn_mw_min": 10.0,
      "min_up_time_hr": 1.0, "min_dn_time_hr": 1.0,
      "startup_cost": 800.0,
      "bid_segments": [
        { "price": 35.0, "quantity": 100.0 },
        { "price": 45.0, "quantity": 50.0  }
      ]
    }
  ],
  "loads": [ { "bus": 0, "p_mw": 300.0 } ],
  "profiles": { "load": [ [1.0, 1.2, 0.9] ] },
  "initial_status": {
    "commitment":    [1.0, 0.0],
    "dispatch":      [250.0, 0.0],
    "time_in_state": [6.0, -3.0]
  }
}
```

该示例为铜排模式（无 `branches`，不计算 PTDF）；`num_buses` 自动推断为 1；Coal-1 初始在线 6 小时、Gas-1 停机 3 小时；3 个时间步负荷为 300、360、270 MW。

---

## 4.3 内部索引结构

理解以下内部结构有助于排查模型规模、解释输出矩阵的维度，以及在 C++ 层对接结果。

### 4.3.1 母线编号推断

未提供 `num_buses` 时自动推断：`num_buses = max(1, 全部组件最大母线编号 + 1)`，扫描范围包括 `generators.bus`、`branches.from/to`、`loads.bus`、`wind.bus`、`solar.bus`、`storage.bus`。松弛母线（参考母线）固定为 bus 0。

### 4.3.2 PTDF 矩阵

直流潮流分布因子矩阵，维度 `[nl × nb]`：

$$\mathbf{F}_\text{PTDF} = \mathbf{B}_f \cdot \mathbf{B}_\text{red}^{-1}$$

- $\mathbf{B}_f$：支路导纳矩阵（$n_l \times n_b$），元素 $b_l = 1/x_l$；
- $\mathbf{B}_\text{red}$：去除松弛母线（bus 0）后的节点导纳子矩阵，逆矩阵通过 Eigen `FullPivLU` 分解求解；
- 若网络孤岛或 $\mathbf{B}_\text{red}$ 不可逆，逆矩阵置零，退化为铜排模式；`in_service = false` 的线路不参与导纳矩阵构建；$|x_l| \le 10^{-6}$ 时以 $10^{-3}$ 代替。

断面 PTDF：$\text{SEC\_PTDF}[s][n] = \sum_l w_{sl}\,\text{PTDF}[l][n]$。

### 4.3.3 MILP 决策变量布局（VarIndex）

所有变量组织为扁平一维数组，按以下块顺序排列（提交类变量按 `T_commit`，其余按 `T`）：

| 变量块 | 含义 | 维度 |
|--------|------|------|
| `SU` / `SD` / `IG` | 启动 / 停机 / 在线二进制 | `ng × T_commit`（各一块） |
| `SEG[k]` | 第 k 段报价区间出力 | `n_seg × ng × T` |
| `PG` | 机组总有功出力 | `ng × T` |
| `RG_spin` / `RG_reg_up` / `RG_reg_down` | 旋转备用 / 调频上 / 调频下 | `ng × T`（各一块） |
| `PF` | 线路潮流（from→to，MW） | `nl × T` |
| `PW` / `PPV` | 风电 / 光伏出力 | `nw × T` / `npv × T` |
| `WIND_CURT` / `SOL_CURT` | 风 / 光弃电（仅 M2 > 0 时有效） | `nw × T` / `npv × T` |
| `PSTO_IN` / `PSTO_OUT` / `SOC` | 储能充电 / 放电 / 状态（MWh） | `nstorage × T`（各一块） |
| `XI_CH` / `XI_DIS` | 充/放电模式二进制（仅 `use_binary_indicators`） | `nstorage × T` |
| `PDC` | 直流线路潮流 | `ndc × T` |
| `LOAD_SHED` / `GEN_CURT` | 切负荷 / 系统弃电松弛（n_areas = 1） | `1 × T` |
| `SL_LINE_POS/NEG` | 线路越限松弛（仅有限额线路） | `nl × T × 2` |
| `SL_SEC_POS/NEG` | 断面越限松弛 | `nsec × T × 2` |

变量总数近似估算：

$$n_x \approx 3 n_g T_c + (n_{seg}+5) n_g T + n_l T + 2(n_w+n_{pv})T + (3+2\mathbf{1}_{xi}) n_{sto} T + n_{dc} T + 2(1+n_l+n_{sec})T$$

其中 $\mathbf{1}_{xi}$ 在有任意储能机组使用二进制模式时为 1。

### 4.3.4 约束族一览

| 约束类型 | 说明 |
|----------|------|
| 提交转换 | $I_g^h - I_g^{h-1} = u_g^h - v_g^h$；$u_g^h + v_g^h \leq 1$ |
| 最小在/停机时间 | 滑动窗口约束 |
| 最大启停次数 | $\sum_h u_g^h \leq$ maxStartups，停机同理 |
| 出力-提交联动 | $P_{min} I_g^h \leq P_g^t \leq P_{max} I_g^h$ |
| 分段出力 | $\sum_k p_g^{k,t} = P_g^t - P_{min} I_g^h$ |
| 爬坡 | $P_g^t - P_g^{t-1} \leq$ ramp_up，降坡同理 |
| 潮流平衡 | 每母线每时间步节点功率平衡（等式） |
| 线路潮流 | $P_l^t = \sum_n \text{PTDF}_{ln}\,\text{NetInj}_n^t$，含松弛变量 |
| 断面 | 加权支路潮流之和的限额约束 |
| 备用 | 系统级旋转备用、调频备用、一次调频约束 |
| 储能 SOC | $E_s^t = E_s^{t-1} + \eta^{ch} P_s^{ch,t}\Delta t - P_s^{dis,t}\Delta t/\eta^{dis}$，含期末约束 |
| 机组群 | 群出力上下限及能量限制 |
| 可再生下界 | $P_w^t \geq \alpha \hat{P}_w^t$ |
| 预构型切割面 | 仅当 `enable_market_cuts = true` |

### 4.3.5 求解流水线

```
SCUCInput
  ├─ Stage 1: SCUC MILP
  │    build_formulation()（PTDF + VarIndex + 约束矩阵）
  │    → run_milp()（solver dispatch: HiGHS→Gurobi→NativeBnC）
  │    → extract_result() → SCUCSolveResult
  │    （converged=false 时跳过后续阶段）
  ├─ Stage 2: SCED LP（仅当 solve_sced=true 且 SCUC 收敛）
  │    固定 commitment/startup/shutdown 二进制变量
  │    → run_lp()（LP solver: NativeIPM→PDLP→LCQP→Gurobi→HiGHS）
  └─ Stage 3: LMP（仅当 solve_lmp=true 且前级收敛）
       δ-邻域再调度 LP → 提取对偶 → 节点 LMP（能量+阻塞分量）
```

LMP 对偶来源：Gurobi / NativeBranchAndCut 可直接提取约束对偶；HiGHS 经 MPS 文件 I/O 不返回对偶，LMP 阶段自动回退到 Gurobi 或 NativeBranchAndCut。

---

## 4.4 输出 JSON 约定

### 4.4.1 结构总览

顶层容器 `SCUCOutput` 含三部分：`scuc`（MILP 结果）、`sced`（LP 结果，未运行时 `converged=false`）、`lmp`（节点电价结果，未运行时 `converged=false`）。`scuc_output_to_json` 输出的典型形态：

> 注意（2026-09-13 与源码核对）：C++ 层 `SCUCSolveResult` 还填充
> `segment_dispatch`、`wind_curtailment`、`solar_curtailment`、`section_flows`
> 四个矩阵，但 `solve_result_to_json` **不序列化它们**——JSON 消费方拿不到，
> 需要时只能在 C++ API 层读取。

```json
{
  "meta": { "solver": "Auto", "num_periods": 24, "num_buses": 39,
            "num_generators": 10, "num_branches": 51 },
  "scuc": {
    "converged": true, "objective": 891466.2, "solver_name": "HiGHS",
    "solve_time_sec": 3.14, "mip_gap": 0.0009, "n_cuts_added": 12,
    "commitment": [[1,1,...],[0,1,...],...],
    "startup":    [[0,0,...], ...],
    "shutdown":   [[0,0,...], ...],
    "dispatch":   [[500.0,...], ...],
    "spinning_reserve": [[50.0,...], ...],
    "regulation_up":    [[25.0,...], ...],
    "regulation_down":  [[20.0,...], ...],
    "wind_generation":  [[120.0,...], ...],
    "solar_generation": [[0.0,...], ...],
    "storage_charging":    [[0.0,...], ...],
    "storage_discharging": [[100.0,...], ...],
    "storage_soc":         [[400.0,...], ...],
    "line_flows":          [[45.2,...], ...],
    "load_shedding":  [0.0, 0.0, ...],
    "gen_curtailment":[0.0, 0.0, ...],
    "cost": { "energy": 820000.0, "startup": 15000.0, "no_load": 48000.0,
              "reserve": 8466.2, "penalty": 0.0, "total": 891466.2 }
  },
  "sced": { "...": "同 scuc 结构" },
  "lmp": {
    "converged": true, "solve_time_sec": 0.21,
    "avg_lmp": 32.5, "max_lmp": 58.3, "min_lmp": 18.1,
    "nodal":      [[32.5,...], ...],
    "energy":     [[30.0,...], ...],
    "congestion": [[2.5, ...], ...]
  }
}
```

JSON 键名与 C++ 字段的差异：`lmp.nodal/energy/congestion` 对应 `nodal_lmp/energy_lmp/congestion_lmp`；`scuc.cost.total` 对应 `total_cost`（JSON 嵌套在 `cost` 对象下）。

### 4.4.2 SCUCSolveResult 字段

| 字段 | 类型 | 维度 | 说明 |
|------|------|------|------|
| `converged` | `bool` | — | 是否找到可行解 |
| `objective` | `double` | — | 目标函数值（$），与 `total_cost` 相等 |
| `solver_name` | `string` | — | 实际使用的求解器名称 |
| `solve_time_sec` | `double` | — | 求解耗时（秒） |
| `mip_gap` | `double` | — | 最终相对 MIP 间隙（0 = 精确最优） |
| `n_cuts_added` | `int` | — | 预构型削减切面数量 |
| `commitment` / `startup` / `shutdown` | `Matrix2D` | `[ng][T_commit]` | 提交状态 / 启动 / 停机指示（0 或 1） |
| `dispatch` | `Matrix2D` | `[ng][T]` | 总有功出力（MW） |
| `spinning_reserve` / `regulation_up` / `regulation_down` | `Matrix2D` | `[ng][T]` | 各类备用（MW） |
| `segment_dispatch` | `vector<Matrix2D>` | `[n_seg][ng][T]` | 各分段出力（MW） |
| `wind_generation` / `solar_generation` | `Matrix2D` | `[nw][T]` / `[npv][T]` | 风/光实际出力（MW） |
| `wind_curtailment` / `solar_curtailment` | `Matrix2D` | `[nw][T]` / `[npv][T]` | 风/光弃电量（MW） |
| `storage_charging` / `storage_discharging` / `storage_soc` | `Matrix2D` | `[nstorage][T]` | 储能充/放功率（MW）与 SOC（MWh） |
| `line_flows` | `Matrix2D` | `[nl][T]` | 支路潮流（MW，正方向 from→to） |
| `section_flows` | `Matrix2D` | `[nsec][T]` | 断面潮流（MW） |
| `load_shedding` / `gen_curtailment` | `vector<double>` | `[T]` | 系统切负荷 / 弃电量（MW） |
| `energy_cost` / `startup_cost` / `no_load_cost` / `reserve_cost` / `penalty_cost` / `total_cost` | `double` | — | 费用分解（$），`total_cost` = 各项之和 |

### 4.4.3 SCUCLMPResult 字段

| 字段 | 类型 | 维度 | 说明 |
|------|------|------|------|
| `converged` | `bool` | — | LMP 计算是否成功 |
| `solve_time_sec` | `double` | — | LMP LP 求解耗时（秒） |
| `nodal_lmp` | `Matrix2D` | `[nb][T]` | 节点 LMP（$/MWh） |
| `energy_lmp` | `Matrix2D` | `[nb][T]` | 系统边际价格分量（$/MWh） |
| `congestion_lmp` | `Matrix2D` | `[nb][T]` | 阻塞租金分量（$/MWh） |
| `avg_lmp` / `max_lmp` / `min_lmp` | `double` | — | 全系统平均 / 最大 / 最小 LMP（$/MWh） |

LMP 符号约定：正值表示消费方支付；负值理论上可能出现（网络阻塞时接受端节点）。

### 4.4.4 维度保证与收敛语义

- `converged = true` 时，上表各矩阵维度严格保证：`commitment/startup/shutdown` 为 `[ng][T_commit]`（`T_commit = ceil(T / intervals_per_hour)`），其余时序矩阵为 `[·][T]`；无对应设备时为空数组 `[][]`；`nodal/energy/congestion_lmp` 仅当 `lmp.converged=true` 时保证维度。
- `converged = false` 时，所有矩阵为空（`[]`），所有标量成本字段为 `0.0`。

各阶段收敛组合：

| 情况 | scuc | sced | lmp | 说明 |
|------|:---:|:---:|:---:|------|
| 正常求解 | true | true | true | 三阶段均成功 |
| SCUC 超时（有可行解） | true | true | true | `mip_gap > 0` |
| SCUC 超时（无可行解） | false | false | false | 不可行或时限不足 |
| SCUC 成功、SCED 不可行 | true | false | false | 罕见 |
| `solve_sced=false` | true | false | 取决于 `solve_lmp` | SCED 跳过 |
| `solve_lmp=false` | — | — | false | LMP 跳过 |
| LMP 求解器不支持对偶 | true | true | false | HiGHS 无对偶时自动回退；无可用回退则失败 |

---

## 4.5 内置算例构造器（case_builder）

`case_builder`（源文件 `include/mipsolvers/scuc/case_builder.hpp` / `src/scuc/case_builder.cpp`）提供三个预构建的 `SCUCInput` 对象，含完整母线拓扑、支路参数、发电机报价、负荷曲线与时序曲线，可直接传入 `scuc_solve()`，用于单元测试、集成测试与性能基准。

### 4.5.1 接口一览

```cpp
#include "mipsolvers/scuc/case_builder.hpp"

namespace mipsolvers::scuc {
  SCUCInput build_3bus_case(int T = 3,  double dt = 1.0);
  SCUCInput build_6bus_case(int T = 24, double dt = 1.0,
                            bool with_wind    = false,
                            bool with_storage = false);
  SCUCInput build_ieee39_case(int T = 24, double dt = 1.0,
                              bool with_wind  = false,
                              bool with_solar = false);
  std::string scuc_input_to_json(const SCUCInput& inp, int indent = 2);
}
```

| 函数 | 参数 | 返回 |
|------|------|------|
| `build_3bus_case` | `T`（默认 3）、`dt`（小时，默认 1.0） | 3 母线 2 机组最小验证算例，用于快速冒烟测试 |
| `build_6bus_case` | `T`（默认 24）、`dt`、`with_wind`（bus 3 加 100 MW 风电）、`with_storage`（bus 5 加 240 MWh / 60 MW 储能） | 6 母线 3 机组中等规模算例 |
| `build_ieee39_case` | `T`（默认 24）、`dt`、`with_wind`（3 个风电站共 1050 MW）、`with_solar`（2 个光伏站共 450 MW） | IEEE 39 节点新英格兰系统 |
| `scuc_input_to_json` | `inp`、`indent`（≥ 0 缩进格式；= -1 紧凑格式） | 序列化 JSON 字符串 |

### 4.5.2 3-bus 标准算例

- 规模：3 母线、2 机组、2 支路（bus0–bus2 200 MW，bus1–bus2 200 MW）；总基准负荷 230 MW；双峰正弦负荷曲线（峰值系数 1.0，谷值 0.65）。
- 拓扑：G1（bus0，coal）、G2（bus1，gas）→ bus2（load）。

| 机组 | 母线 | Pmin | Pmax | 爬坡 (MW/min) | 最小开/停 (h) | 启动费 ($) | 空载 ($/h) |
|------|------|------|------|---------------|---------------|------------|------------|
| Coal_G1 | 0 | 80 | 400 | 6 | 4 / 4 | 3,000 | 150 |
| Gas_G2 | 1 | 50 | 200 | 5 | 2 / 2 | 1,200 | 60 |

报价段（超出 Pmin 的 MW 容量 @ $/MWh）：Coal_G1 为 80@22 / 200@27 / 120@33；Gas_G2 为 50@28 / 100@35 / 50@42。

默认配置：`voll` 8,000 $/MWh，`vocc` 200 $/MWh，旋转备用 10%，调频上/下各 5%，`enable_market_cuts = true`。

### 4.5.3 6-bus 标准算例

- 规模：6 母线、3 机组（Coal 400 MW、Gas 200 MW、Peaker 80 MW）、7 支路；总基准负荷 500 MW；可选风电 100 MW（bus 3）、储能 240 MWh / 60 MW（bus 5）。

| 机组 | 母线 | Pmin | Pmax | 爬坡 (MW/min) | 最小开/停 (h) | 启动费 ($) | 空载 ($/h) |
|------|------|------|------|---------------|---------------|------------|------------|
| Coal_G1 | 0 | 80 | 400 | 6 | 4 / 4 | 3,000 | 150 |
| Gas_G2 | 1 | 50 | 200 | 5 | 2 / 2 | 1,200 | 60 |
| Peaker | 3 | 10 | 80 | 8 | 1 / 1 | 500 | 20 |

储能参数（`with_storage = true`）：bus 5；最大充放电 60 MW；容量 240 MWh；综合效率 90%；初始 SOC 50%；最小 SOC 10%。

### 4.5.4 IEEE 39-bus 标准算例

基于 IEEE 39 节点新英格兰系统：39 母线、10 机组（G1–G10）、46 支路（34 输电线 + 7 升压变压器 + G10 两条联络线）；总 Pmax 7,329 MW，总 Pmin 1,300 MW；17 个负荷母线，总基准负荷 4,599 MW。

| 机组 | 母线(0-idx) | Pmin | Pmax | 爬坡 (MW/min) | 最小开/停 (h) | 启动费 ($) | 空载 ($/h) |
|------|-------------|------|------|---------------|---------------|------------|------------|
| G1 | 38 | 250 | 1,100 | 15 | 5 / 5 | 15,000 | 400 |
| G2 | 30 | 100 | 650 | 10 | 4 / 4 | 8,000 | 220 |
| G3 | 31 | 150 | 725 | 12 | 4 / 4 | 9,000 | 250 |
| G4 | 32 | 150 | 650 | 12 | 4 / 4 | 9,000 | 240 |
| G5 | 33 | 100 | 508 | 10 | 3 / 3 | 7,500 | 180 |
| G6 | 34 | 50 | 687 | 8 | 3 / 3 | 8,500 | 200 |
| G7 | 35 | 100 | 580 | 10 | 3 / 3 | 8,000 | 190 |
| G8 | 36 | 50 | 564 | 8 | 2 / 2 | 6,000 | 150 |
| G9 | 37 | 50 | 865 | 10 | 2 / 2 | 9,000 | 220 |
| G10 | 29 | 300 | 1,100 | 15 | 6 / 6 | 16,000 | 450 |

折线报价段（MW @ $/MWh，超出 Pmin 的容量）：

| 机组 | 段1 | 段2 | 段3 |
|------|-----|-----|-----|
| G1 | 250 @ 20 | 550 @ 25 | 300 @ 31 |
| G2 | 100 @ 22 | 350 @ 28 | 200 @ 34 |
| G3 | 150 @ 21 | 375 @ 27 | 200 @ 33 |
| G4 | 150 @ 23 | 300 @ 29 | 200 @ 35 |
| G5 | 100 @ 24 | 250 @ 30 | 158 @ 38 |
| G6 | 50 @ 26 | 350 @ 33 | 287 @ 41 |
| G7 | 100 @ 25 | 280 @ 31 | 200 @ 39 |
| G8 | 50 @ 27 | 300 @ 34 | 214 @ 43 |
| G9 | 50 @ 22 | 450 @ 28 | 365 @ 36 |
| G10 | 300 @ 19 | 550 @ 24 | 250 @ 29 |

**负荷分布**（基准 MW）：bus0 97.6、bus2 322.0、bus3 500.0、bus6 233.8、bus7 6.0、bus8 522.0、bus9 8.5、bus14 320.0、bus15 329.0、bus16 158.0、bus19 628.0、bus21 274.0、bus22 247.5、bus24 308.6、bus25 224.0、bus27 139.0、bus28 281.0；合计 4,599 MW（不含零值母线）。

**时序曲线**：

- 负荷曲线：各负荷点共用双峰正弦形曲线 `make_load_profile`，pu(t) = 0.65 + 0.35 · max(G₁(t), G₂(t))，G₁ 以 10:00 为中心、G₂ 以 19:00 为中心，σ = 2 h。峰值系数 1.000（t≈10h 与 t≈19h），谷值系数 0.650（夜间低谷）。
- 风电曲线（`with_wind = true`，`make_wind_profile`）：夜间高、午后低，峰值系数约 1.0，最低约 0.1（13:00 附近）。Wind 1（bus 5，400 MW）、Wind 2（bus 14，300 MW）、Wind 3（bus 26，350 MW），合计 1,050 MW。
- 光伏曲线（`with_solar = true`，`make_solar_profile`）：正午高、夜间零。PV 1（bus 21，250 MW）、PV 2（bus 36，200 MW）。

**初始机组状态**：

```cpp
inp.initial_status.commitment = {1, 1, 0, 0, 1, 0, 0, 0, 0, 1};
//                              G1 G2 G3 G4 G5 G6 G7 G8 G9 G10
inp.initial_status.dispatch   = {500, 300, 0, 0, 200, 0, 0, 0, 0, 600};
```

初始在线：G1（500 MW）、G2（300 MW）、G5（200 MW）、G10（600 MW），合计 1,600 MW。

**默认 SCUC 配置**：`n_segments` 3；旋转备用 5%、调频上 3%、调频下 2%（系统负荷分数）；`pfr_reserve_req_mw` 300 MW；`voll` 10,000 $/MWh；`vocc` 300 $/MWh；`M1_line_slack_penalty` 1e6 $/MW；`M2_renewable_curtail_penalty` 80 $/MW（`with_wind=true` 时）；`enable_market_cuts = true`。

### 4.5.5 JSON 序列化

```cpp
// 生成 IEEE 39-bus 24h 输入并写入文件
SCUCInput inp = build_ieee39_case(24, 1.0, true, false);
std::string json = scuc_input_to_json(inp, 2);
std::ofstream("/tmp/ieee39_24h.json") << json;
```

序列化输出即 4.2 节描述的输入格式，可用于复核构造器生成的数据或作为自定义算例的模板。

### 4.5.6 复现基准算例

以下命令构建并运行覆盖三个内置算例的基准测试（Release 模式）：

```bash
# Release 构建
cmake -S . -B build_rel -DCMAKE_BUILD_TYPE=Release
cmake --build build_rel --target test_market_simulation -j8

# 全求解器 benchmark（3-bus / 6-bus / IEEE 39-bus）
./build_rel/test_market_simulation "[market][benchmark]" 2>/dev/null

# IEEE 39-bus 24h 完整流程（SCUC→SCED→LMP）
./build_rel/test_market_simulation "[market][viz][ieee39]" 2>/dev/null | \
  grep -E "切负荷|SCUC|收敛|求解器"

# IPM 根节点 + 交叉迭代正确性测试（含交叉诊断日志）
MIPSOLVERS_BC_TIMELINE=1 ./build_rel/test_market_simulation "[ipm_root]" 2>&1
```

> Windows 下请使用构建目录的实际路径（如 `build\windows-msvc-release\`）替换上述 `build_rel`，并用反斜杠或引号处理路径分隔符。基准性能数字与测试矩阵见 [测试与基准](09-testing-benchmarks.md)。
