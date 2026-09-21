# MIPSolvers Python API 说明文档

本文档说明如何在 Python 3.8+ 环境中调用 `mipsolvers` 模块。  
模块由 pybind11 绑定 C++ 核心库生成，提供两个子模块：

| 子模块 | 功能 |
|--------|------|
| `mipsolvers.scuc` | 短期机组组合（SCUC/SCED/LMP）市场出清 |
| `mipsolvers.engine` | 底层 LP / MILP 求解引擎 |

---

## 1. 编译与安装

### 1.1 前置依赖

| 依赖 | 版本 | 说明 |
|------|------|------|
| Python | 3.8.x | 建议使用 pyenv 管理 |
| pybind11 | ≥ 3.0 | `pip install pybind11` |
| numpy | ≥ 1.19 | `pip install numpy` |
| CMake | ≥ 3.22 | 构建系统 |
| C++ 编译器 | C++20 | AppleClang / GCC 12+ |

### 1.2 构建步骤

```bash
# 1. 在独立 build 目录中配置（推荐与 build/ 隔离）
cmake -B build_py \
  -DCMAKE_BUILD_TYPE=Release \
  -DMIPSOLVERS_BUILD_PYTHON=ON \
  -DPYTHON_EXECUTABLE=/path/to/python3.8 \
  -DMIPSOLVERS_BUILD_TESTS=OFF \
  -DMIPSOLVERS_BUILD_SCUC=OFF \
  -DMIPSOLVERS_BUILD_SCUC_CASE_BUILDER=OFF

# 2. 仅编译 Python 绑定目标
cmake --build build_py --target mipsolvers_py -j8

# 3. 将生成的 .so 加入 Python 路径
# 文件路径: build_py/mipsolvers.cpython-38-darwin.so (macOS)
#           build_py/mipsolvers.cpython-38-x86_64-linux-gnu.so (Linux)
export PYTHONPATH=/path/to/MIPSolvers/build_py:$PYTHONPATH
```

> **注意**  
> `PYTHON_EXECUTABLE` 必须指向目标 Python 解释器，pybind11 会自动匹配扩展名后缀。

### 1.3 验证安装

```python
import mipsolvers
print(mipsolvers.__doc__)
# 预期输出: MIPSolvers Python bindings.
# 子模块: scuc, engine
```

---

## 2. 模块结构

```
mipsolvers
├── scuc
│   ├── 数据结构
│   │   ├── BidSegment          报价段（价格 + 电量）
│   │   ├── Generator           热电机组参数
│   │   ├── Branch              交流输电支路
│   │   ├── Load                负荷节点
│   │   ├── WindUnit            风电机组
│   │   ├── SolarUnit           光伏机组
│   │   ├── StorageUnit         储能单元
│   │   ├── DCLine              直流联络线
│   │   ├── GeneratorGroup      机组组合约束组
│   │   ├── Section             监测截面
│   │   ├── SCUCConfig          求解器与模型配置
│   │   ├── SCUCProfiles        时序曲线（负荷/风/光预测）
│   │   ├── SCUCInitialStatus   初始状态
│   │   ├── SCUCInput           完整输入（聚合所有结构体）
│   │   ├── SCUCSolveResult     SCUC/SCED 求解结果
│   │   ├── SCUCLMPResult       节点电价（LMP）结果
│   │   └── SCUCOutput          全流程输出（SCUC + SCED + LMP）
│   └── 函数
│       ├── from_json(json_str)                 → SCUCInput
│       ├── solve(input)                        → SCUCOutput
│       ├── output_to_json(output, input, indent) → str
│       └── solve_json(json_str, indent=2)      → str  ← 推荐入口
└── engine
    ├── solve_lp(c, A, b, Aeq, beq, lb, ub, solver, maximize)  → dict
    ├── solve_milp(c, A, b, Aeq, beq, lb, ub, vartypes,
    │             solver, mip_gap, time_limit_sec, maximize)   → dict
    └── list_solvers(problem_class)                            → list[str]
```

---

## 3. SCUC JSON API（推荐）

`solve_json` 是最简洁的调用方式：接受 JSON 字符串输入，返回 JSON 字符串结果。  
适合与其他系统集成或批处理场景。

```python
import json
import mipsolvers

# ── 构建输入 ──────────────────────────────────────────────────────────────
input_data = {
    "config": {
        "num_periods": 24,           # 调度周期数
        "period_length_hr": 1.0,     # 每期时长（小时）
        "solver": "Auto",            # 求解器："Auto"|"StrictHiGHS"|"Gurobi"|"HiGHS"
        "mip_gap": 0.001,            # MIP 相对间隙容差
        "time_limit_sec": 300.0,     # 时间限制（秒）
        "solve_sced": True,          # 是否运行 SCED
        "solve_lmp": True,           # 是否计算 LMP
        "voll": 10000.0              # 切负荷惩罚 $/MWh
    },
    "generators": [
        {
            "name": "G1",
            "bus": 0,
            "pmin": 50.0,
            "pmax": 400.0,
            "ramp_up_mw_min": 200.0,
            "ramp_dn_mw_min": 200.0,
            "min_up_time_hr": 4.0,
            "min_dn_time_hr": 2.0,
            "startup_cost": 5000.0,
            "no_load_cost": 200.0,
            "bid_segments": [
                {"price": 20.0, "quantity": 150.0},
                {"price": 35.0, "quantity": 150.0},
                {"price": 50.0, "quantity": 100.0}
            ]
        }
    ],
    "loads": [
        {"bus": 0, "p_mw": 300.0}   # 恒定负荷（或配合 profiles.load 使用）
    ],
    "branches": [],                  # 无输电约束时可省略
    "profiles": {
        # load[i][t]: 第 i 个负荷在第 t 期的标幺值倍数（省略时=1.0）
        "load": [[0.8, 0.85, 0.9, 1.0, 1.1, 1.2,
                  1.3, 1.35, 1.4, 1.3, 1.2, 1.1,
                  1.0, 0.95, 0.9, 0.85, 0.9, 1.0,
                  1.1, 1.2, 1.1, 1.0, 0.9, 0.8]]
    }
}

# ── 调用 ──────────────────────────────────────────────────────────────────
result_str = mipsolvers.scuc.solve_json(json.dumps(input_data), indent=2)
result = json.loads(result_str)

# ── 读取结果 ──────────────────────────────────────────────────────────────
scuc = result["scuc"]
print(f"收敛: {scuc['converged']}")
print(f"总费用: ${scuc['cost']['total']:,.2f}")
print(f"启动费: ${scuc['cost']['startup']:,.2f}")
print(f"能量费: ${scuc['cost']['energy']:,.2f}")
print(f"求解时间: {scuc['solve_time_sec']:.3f} s")

# 机组出力（第0台机，所有时段）
dispatch_G1 = scuc["dispatch"][0]    # list[float], 长度 = num_periods

# SCED 结果（若 solve_sced=True）
sced = result["sced"]
if sced["converged"]:
    print(f"SCED 费用: ${sced['cost']['total']:,.2f}")

# LMP 结果（若 solve_lmp=True）
lmp = result["lmp"]
if lmp["converged"]:
    nodal_lmp = lmp["nodal_lmp"]    # list[list[float]], [nb][T]
    print(f"平均 LMP: ${lmp['avg_lmp']:.2f}/MWh")
```

### JSON 输入完整字段说明

详见 [`docs/data_format_spec.md`](data_format_spec.md)。核心字段速查：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `config.num_periods` | int | 是 | 调度周期数 T |
| `config.solver` | str | 否 | `"Auto"` / `"StrictHiGHS"` / `"Gurobi"` / `"HiGHS"` |
| `generators[].name` | str | 是 | 机组名称（唯一） |
| `generators[].bus` | int | 是 | 所在母线编号（0-indexed） |
| `generators[].bid_segments` | list | 是 | 报价曲线段列表 |
| `loads[].bus` | int | 是 | 负荷母线编号 |
| `loads[].p_mw` | float | 是 | 基准负荷（MW） |
| `branches[].from_bus` | int | 是（有输电约束时） | 支路首端母线 |
| `branches[].reactance` | float | 是（有输电约束时） | 电抗（标幺值） |
| `profiles.load` | list[list[float]] | 否 | 负荷时序倍数 `[nd][T]` |

---

## 4. SCUC Struct API（结构体接口）

适合在 Python 中动态构建输入数据，或对输入进行精细控制。

```python
import mipsolvers
s = mipsolvers.scuc

# ── 构建配置 ──────────────────────────────────────────────────────────────
inp = s.SCUCInput()
inp.config.num_periods = 24
inp.config.period_length_hr = 1.0
inp.config.solver = "Auto"
inp.config.solve_sced = True
inp.config.solve_lmp = False
inp.config.voll = 10000.0

# ── 添加机组 ──────────────────────────────────────────────────────────────
g = s.Generator()
g.name = "G1"
g.bus = 0
g.pmin = 50.0
g.pmax = 400.0
g.min_up_time_hr = 4.0
g.min_dn_time_hr = 2.0
g.startup_cost = 5000.0
g.no_load_cost = 200.0
g.bid_segments = [
    s.BidSegment(price=20.0, quantity=150.0),
    s.BidSegment(price=35.0, quantity=150.0),
    s.BidSegment(price=50.0, quantity=100.0),
]
inp.generators.append(g)

# ── 添加负荷 ──────────────────────────────────────────────────────────────
ld = s.Load()
ld.bus = 0
ld.p_mw = 300.0
inp.loads.append(ld)

# ── 配置时序曲线 ──────────────────────────────────────────────────────────
T = 24
inp.profiles.load = [[0.8 + 0.5 * (t / T) for t in range(T)]]

# ── 设置初始状态（可选）──────────────────────────────────────────────────
inp.initial_status.commitment = [1]         # 第0台机初始在线
inp.initial_status.dispatch   = [200.0]     # 初始出力 MW
inp.initial_status.time_in_state = [8]      # 已连续在线 8 小时

# ── 求解 ──────────────────────────────────────────────────────────────────
out = s.solve(inp)

# ── 读取结构体结果 ─────────────────────────────────────────────────────────
print(f"SCUC 收敛: {out.scuc.converged}")
print(f"总费用: ${out.scuc.total_cost:,.2f}")

# 机组组合决策: out.scuc.commitment[ng][t] ∈ {0,1}
for t in range(T):
    print(f"  t={t:2d}  commit={out.scuc.commitment[0][t]:.0f}  "
          f"disp={out.scuc.dispatch[0][t]:.1f} MW")

# SCED 结果
if out.sced.converged:
    print(f"SCED 总费用: ${out.sced.total_cost:,.2f}")
    print(f"切负荷: {out.sced.load_shedding}")   # list[float], 长度 T

# ── 转换为 JSON ───────────────────────────────────────────────────────────
json_str = s.output_to_json(out, inp, indent=2)
```

### SCUCOutput / SCUCSolveResult 主要属性

| 属性 | 类型 | 形状 | 说明 |
|------|------|------|------|
| `scuc.converged` | bool | — | 是否找到可行解 |
| `scuc.objective` | float | — | 目标函数值（$） |
| `scuc.total_cost` | float | — | 总运行费用（=objective） |
| `scuc.energy_cost` | float | — | 能量费用 |
| `scuc.startup_cost` | float | — | 启动费用 |
| `scuc.no_load_cost` | float | — | 空载费用 |
| `scuc.reserve_cost` | float | — | 备用费用 |
| `scuc.penalty_cost` | float | — | 松弛惩罚（切负荷等） |
| `scuc.commitment` | list[list[float]] | [ng][T] | 机组组合决策 0/1 |
| `scuc.dispatch` | list[list[float]] | [ng][T] | 出力（MW） |
| `scuc.startup` | list[list[float]] | [ng][T] | 启动指示 |
| `scuc.shutdown` | list[list[float]] | [ng][T] | 停机指示 |
| `scuc.line_flows` | list[list[float]] | [nl][T] | 支路潮流（MW） |
| `scuc.load_shedding` | list[float] | [T] | 切负荷量（MW） |
| `scuc.wind_generation` | list[list[float]] | [nw][T] | 风电出力 |
| `scuc.solar_generation` | list[list[float]] | [ns][T] | 光伏出力 |
| `scuc.storage_soc` | list[list[float]] | [ne][T] | 储能 SOC（MWh） |
| `scuc.solve_time_sec` | float | — | 求解器用时（秒） |
| `scuc.mip_gap` | float | — | 最终 MIP 间隙 |
| `lmp.nodal_lmp` | list[list[float]] | [nb][T] | 节点边际电价（$/MWh） |
| `lmp.avg_lmp` | float | — | 全时段平均 LMP |

---

## 5. Engine LP API

直接调用底层求解引擎，求解标准形式 LP：

$$\min_{x} \; c^\top x \quad \text{s.t.} \quad Ax \le b, \; A_{\text{eq}}x = b_{\text{eq}}, \; \ell \le x \le u$$

```python
import numpy as np
import mipsolvers

# ── 示例：2 变量 LP ───────────────────────────────────────────────────────
c  = np.array([-1.0, -2.0])           # 目标（最小化 -x0 - 2*x1）
A  = np.array([[1.0, 1.0],            # x0 + x1 <= 4
               [2.0, 1.0]])           # 2x0 + x1 <= 6
b  = np.array([4.0, 6.0])
lb = np.zeros(2)                      # x0, x1 >= 0

res = mipsolvers.engine.solve_lp(
    c,
    A  = A,
    b  = b,
    lb = lb,
    solver = "Auto"    # "Auto"|"Gurobi"|"HiGHS"|"NativeIPMLP"|"NativePDLP"
)

print(res["success"])     # True
print(res["objective"])   # -8.0
print(res["x"])           # [0. 4.]
print(res["duals"])       # 对偶变量（若求解器支持）
print(res["solver"])      # 实际使用的求解器名

# ── 等式约束示例 ──────────────────────────────────────────────────────────
Aeq = np.array([[1.0, 1.0]])          # x0 + x1 = 3
beq = np.array([3.0])

res2 = mipsolvers.engine.solve_lp(c, A=A, b=b, Aeq=Aeq, beq=beq, lb=lb)

# ── 最大化 ────────────────────────────────────────────────────────────────
res3 = mipsolvers.engine.solve_lp(
    np.array([1.0, 2.0]),             # 最大化 x0 + 2*x1
    A=A, b=b, lb=lb,
    maximize=True
)
```

### solve_lp 返回值

| 键 | 类型 | 说明 |
|----|------|------|
| `success` | bool | 是否找到最优解 |
| `objective` | float | 最优目标值 |
| `x` | np.ndarray (n,) | 最优解向量 |
| `duals` | np.ndarray | 约束对偶变量（不等式 + 等式；HiGHS 不支持） |
| `status` | str | 求解器状态字符串 |
| `solver` | str | 实际调用的求解器名 |
| `iterations` | int | 迭代次数 |
| `runtime_sec` | float | 求解用时（秒） |

> **注意**  
> HiGHS 作为 MILP 求解器时不返回约束对偶，`duals` 键可能缺失或为空数组。  
> 如需对偶变量，建议使用 `solver="Gurobi"` 或原生 IPM 求解器。

---

## 6. Engine MILP API

```python
import numpy as np
import mipsolvers

# ── 示例：背包问题 ────────────────────────────────────────────────────────
# maximize  5*x0 + 4*x1 + 3*x2
# s.t.      2*x0 + 3*x1 + 2*x2 <= 5
#           x0, x1, x2 ∈ {0, 1}

c  = np.array([-5.0, -4.0, -3.0])    # 最小化负收益
A  = np.array([[2.0, 3.0, 2.0]])
b  = np.array([5.0])
lb = np.zeros(3)
ub = np.ones(3)

res = mipsolvers.engine.solve_milp(
    c,
    A        = A,
    b        = b,
    lb       = lb,
    ub       = ub,
    vartypes = ["B", "B", "B"],       # 二进制变量
    solver   = "Auto",                # "Auto"|"StrictHiGHS"|"Gurobi"|"HiGHS"|"NativeBranchAndCut"
    mip_gap        = 1e-4,
    time_limit_sec = 60.0
)

print(res["success"])    # True
print(-res["objective"]) # 7.0  (最大收益 5+0+3 或 0+4+3)
print(res["x"])          # [1. 0. 1.] 或 [0. 1. 1.]

# ── 混合整数示例 ──────────────────────────────────────────────────────────
# 变量类型: C = 连续, I = 整数, B = 二进制
vartypes = ["C", "I", "B"]

res2 = mipsolvers.engine.solve_milp(
    c = np.array([-1.0, -2.0, -3.0]),
    lb = np.zeros(3),
    ub = np.array([10.0, 5.0, 1.0]),
    vartypes = vartypes,
    solver = "HiGHS"
)
```

### solve_milp 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `c` | np.ndarray (n,) | 必填 | 目标系数 |
| `A` | np.ndarray (m,n) | None | 不等式约束矩阵 |
| `b` | np.ndarray (m,) | None | 不等式 RHS |
| `Aeq` | np.ndarray (p,n) | None | 等式约束矩阵 |
| `beq` | np.ndarray (p,) | None | 等式 RHS |
| `lb` | np.ndarray (n,) | None（-∞） | 变量下界 |
| `ub` | np.ndarray (n,) | None（+∞） | 变量上界 |
| `vartypes` | list[str] | [] (全连续) | 变量类型列表 |
| `solver` | str | `"Auto"` | 求解器选择 |
| `mip_gap` | float | 1e-4 | 相对 MIP 间隙容差 |
| `time_limit_sec` | float | 300.0 | 时间限制（秒） |
| `maximize` | bool | False | True = 最大化 |

### solve_milp 返回值

| 键 | 类型 | 说明 |
|----|------|------|
| `success` | bool | 是否找到可行整数解 |
| `objective` | float | 最优目标值 |
| `x` | np.ndarray (n,) | 最优解向量 |
| `mip_gap` | float | 最终 MIP 间隙 |
| `status` | str | 求解器状态 |
| `solver` | str | 实际调用的求解器 |
| `runtime_sec` | float | 求解用时 |

---

## 7. 可用求解器

```python
import mipsolvers

# 查询各问题类型的可用求解器
print(mipsolvers.engine.list_solvers("LP"))
print(mipsolvers.engine.list_solvers("MILP"))
print(mipsolvers.engine.list_solvers("NLP"))
```

| 求解器名 | 适用问题 | 说明 |
|----------|----------|------|
| `Gurobi` | LP / MILP | 商业求解器（需许可证） |
| `StrictHiGHS` | MILP | 生产默认：嵌入式 HiGHS 状态机 + MIPSolvers 合约 |
| `HiGHS` | LP / MILP | 直接 HiGHS 适配器 |
| `NativeIPMLP` | LP | 内置内点法（小型 LP） |
| `NativePDLP` | LP | 内置一阶方法（大规模 LP） |
| `NativeLCQP` | LP / QP | 内置线性化 CQP |
| `NativeBranchAndCut` | MILP | 内置分支定界 |
| `Ipopt` | NLP / MINLP | 内置非线性求解器 |
| `Auto` | 任意 | 按内置优先级自动选择 |

> **`"Auto"` 优先级（MILP）**：Gurobi → StrictHiGHS → HiGHS → NativeBranchAndCut

---

## 8. 错误处理

所有异常均以标准 Python 异常抛出：

```python
import mipsolvers

# ── JSON 解析错误 ──────────────────────────────────────────────────────────
try:
    inp = mipsolvers.scuc.from_json("invalid json{")
except RuntimeError as e:
    print(f"JSON 解析失败: {e}")

# ── 求解失败（不收敛）─────────────────────────────────────────────────────
import json
inp_infeasible = {
    "config": {"num_periods": 1},
    "generators": [{"name": "G1", "bus": 0, "pmin": 500.0, "pmax": 600.0,
                     "bid_segments": [{"price": 20.0, "quantity": 100.0}]}],
    "loads": [{"bus": 0, "p_mw": 1000.0}]   # 负荷超过供应上限
}
result = json.loads(mipsolvers.scuc.solve_json(json.dumps(inp_infeasible)))
if not result["scuc"]["converged"]:
    # 不抛异常，而是返回 converged=False
    # 检查切负荷量
    print("未收敛，切负荷:", result["scuc"]["load_shedding"])

# ── 维度不匹配 ─────────────────────────────────────────────────────────────
import numpy as np
try:
    # c 有 3 个变量，但 A 有 4 列 → 维度错误
    mipsolvers.engine.solve_lp(
        c = np.ones(3),
        A = np.ones((2, 4)),
        b = np.ones(2)
    )
except Exception as e:
    print(f"维度错误: {e}")
```

### 常见错误一览

| 异常类型 | 触发场景 |
|----------|----------|
| `RuntimeError` | JSON 格式错误、字段类型错误 |
| `ValueError` / `invalid_argument` | numpy 数组维度不符 |
| `RuntimeError` | 求解器初始化失败（如许可证无效） |
| 返回 `success=False` | 问题无解或超时（不抛异常） |

---

## 9. 完整示例

### 9.1 24 小时单节点 SCUC

```python
import json
import mipsolvers

T = 24
# 典型日负荷曲线（标幺值）
load_profile = [0.75, 0.70, 0.68, 0.67, 0.68, 0.72,
                0.80, 0.90, 0.95, 1.00, 1.05, 1.08,
                1.05, 1.00, 0.98, 0.96, 1.00, 1.10,
                1.20, 1.25, 1.20, 1.10, 0.95, 0.82]

inp = {
    "config": {
        "num_periods": T,
        "solver": "Auto",
        "solve_sced": True,
        "solve_lmp": False
    },
    "generators": [
        {
            "name": "Coal_1", "bus": 0,
            "pmin": 100.0, "pmax": 500.0,
            "ramp_up_mw_min": 150.0, "ramp_dn_mw_min": 150.0,
            "min_up_time_hr": 6.0, "min_dn_time_hr": 4.0,
            "startup_cost": 8000.0, "no_load_cost": 300.0,
            "bid_segments": [
                {"price": 25.0, "quantity": 200.0},
                {"price": 40.0, "quantity": 200.0},
                {"price": 60.0, "quantity": 100.0}
            ]
        },
        {
            "name": "Gas_1", "bus": 0,
            "pmin": 0.0, "pmax": 300.0,
            "ramp_up_mw_min": 300.0, "ramp_dn_mw_min": 300.0,
            "startup_cost": 2000.0, "no_load_cost": 100.0,
            "bid_segments": [
                {"price": 60.0, "quantity": 150.0},
                {"price": 80.0, "quantity": 150.0}
            ]
        }
    ],
    "loads": [{"bus": 0, "p_mw": 600.0}],
    "profiles": {"load": [load_profile]}
}

result = json.loads(mipsolvers.scuc.solve_json(json.dumps(inp)))
scuc = result["scuc"]

print(f"求解状态: {'收敛' if scuc['converged'] else '未收敛'}")
print(f"总费用:   ${scuc['cost']['total']:>12,.2f}")
print(f"  启动费: ${scuc['cost']['startup']:>12,.2f}")
print(f"  能量费: ${scuc['cost']['energy']:>12,.2f}")
print(f"求解耗时: {scuc['solve_time_sec']:.3f} s")
print()
print("  时段  | 煤电组合 | 煤电出力(MW) | 燃气组合 | 燃气出力(MW)")
print("  " + "-"*60)
for t in range(T):
    print(f"  t={t:02d}   |   {scuc['commitment'][0][t]:.0f}    |"
          f"   {scuc['dispatch'][0][t]:7.1f}     |"
          f"   {scuc['commitment'][1][t]:.0f}    |"
          f"   {scuc['dispatch'][1][t]:7.1f}")
```

### 9.2 Engine LP 输电潮流计算

```python
import numpy as np
import mipsolvers

# 3 节点直流潮流 LP
# 最优潮流：最小化发电费用，满足负荷平衡 + 线路容量约束

# 变量: [pg1, pg2, pg3, theta1, theta2, theta3]
n_gen = 3
n_bus = 3
n_var = n_gen + n_bus

c = np.array([20.0, 40.0, 60.0,  # 发电费用
              0.0,  0.0,  0.0])   # 相角（无费用）

# 节点功率平衡（等式约束）
# 简化：只展示 API 用法
Aeq = np.zeros((n_bus, n_var))
Aeq[0, 0] = 1.0;  Aeq[0, 3] = -2.0;  Aeq[0, 4] = 1.0
Aeq[1, 1] = 1.0;  Aeq[1, 3] = 1.0;   Aeq[1, 4] = -3.0;  Aeq[1, 5] = 1.0
Aeq[2, 2] = 1.0;  Aeq[2, 4] = 1.0;   Aeq[2, 5] = -2.0
beq = np.array([150.0, 200.0, 250.0])  # 各节点净注入

lb = np.array([0.0, 0.0, 0.0, -np.pi, -np.pi, -np.pi])
ub = np.array([400.0, 300.0, 200.0, np.pi, np.pi, np.pi])

res = mipsolvers.engine.solve_lp(
    c, Aeq=Aeq, beq=beq, lb=lb, ub=ub,
    solver="HiGHS"
)

if res["success"]:
    pg = res["x"][:n_gen]
    theta = res["x"][n_gen:]
    print(f"最优发电费用: ${res['objective']:,.2f}/h")
    print(f"发电计划: {pg} MW")
    print(f"节点相角: {theta} rad")
```

---

## 10. 性能建议

1. **批量调用**：每次调用 `solve_json` / `solve` 内部会构建完整的 LP/MIP 模型并初始化求解器。如需连续求解多个场景，建议在 Python 端并行（多进程），而非串行循环。

2. **GIL 释放**：`solve_json`、`solve`、`solve_lp`、`solve_milp` 在求解阶段均自动释放 GIL，支持多线程并发调用（求解器本身线程安全需参考各求解器文档）。

3. **大矩阵传递**：`solve_lp` / `solve_milp` 接受稠密 numpy 矩阵并内部转换为稀疏格式。对于已知稀疏结构的大型问题，可适当减少非零元素数量以提升效率。

4. **求解器选择**：
   - LP：优先 `"HiGHS"`（无许可证需求），大规模时可试 `"NativePDLP"`
   - MILP：优先 `"Gurobi"`（有许可证），无许可证时使用 `"HiGHS"`
   - SCUC 超过 100 台机组：建议 `"Gurobi"`，设置 `mip_gap=0.005`
