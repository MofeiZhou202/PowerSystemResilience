# 第 8 章 电力行业应用：SCUC 与 OPF

> 本章整合自: docs/archive/case_builder.md, docs/archive/data_format_spec.md, docs/archive/user_manual.md, docs/tutorial/demo_04_aml_scuc.py, docs/tutorial/demo_05_scuc_api.py

本章面向电力系统优化现场：先给出 SCUC（安全约束机组组合）的建模要素与三阶段求解流水线，再依次介绍内置算例构造器、命令行工具、输入 JSON 的准备要点和输出结果的判读方法，最后给出 OPF 场景的求解路径选择建议。完整的输入/输出字段表见 [SCUC 数据格式](04-modeling-aml.md)，求解器背景见 [求解器与引擎](05-solvers-engines.md)。

## 8.1 SCUC 问题的工程背景与建模要素

SCUC 在日前（或日内）时间尺度上同时决定各发电机组的开停状态（commitment）与出力计划（dispatch），目标是最小化总运行成本，同时满足功率平衡、机组物理约束和网络安全约束。`mipsolvers::scuc` 模块将该问题实现为一条三阶段流水线：

```
输入 SCUCInput
    │
    ├─[Stage 1: SCUC MILP]   PTDF 计算 + 约束矩阵构建 → MILP 求解 → 机组组合结果
    │                          （converged=false 时跳过后续阶段）
    ├─[Stage 2: SCED LP]     固定 commitment/startup/shutdown 二进制变量，
    │                          求解连续经济调度（dispatch/reserves/flows）
    └─[Stage 3: LMP]         δ-邻域再调度 LP，提取对偶 → 节点边际电价
```

Stage 2、3 分别由 `config.solve_sced`、`config.solve_lmp` 控制开关（默认均为 `true`）。对应结果容器为 `SCUCOutput.scuc` / `.sced` / `.lmp`。

### 8.1.1 机组约束

每台火电机组在 `generators` 数组中描述，核心约束族包括：

| 约束族 | 关键字段 | 说明 |
|--------|----------|------|
| 出力上下限 | `pmin` / `pmax` | 与在线状态联动：Pmin·I ≤ P ≤ Pmax·I |
| 爬坡/降坡 | `ramp_up_mw_min` / `ramp_dn_mw_min` | 单位 MW/min，相邻时段出力差受限 |
| 最小开/停机时间 | `min_up_time_hr` / `min_dn_time_hr` | 滑动窗口约束 |
| 启停逻辑 | 内部变量 `u`（启动）、`v`（停机）、`I`（在线） | I(t) − I(t−1) = u(t) − v(t)，且 u+v ≤ 1 |
| 最大启停次数 | `max_startups` / `max_shutdowns` | 0 = 不限 |
| 强制运行 | `must_run` | 将在线变量下界固定为 1 |
| 分段报价 | `bid_segments` | 折线能量报价，见下方注意 |
| 空载与启动费用 | `no_load_cost`、`startup_cost`（含温/冷启动 `startup_cost_warm/cold`） | 热/温/冷启动由 `hot/warm_start_threshold_hr` 与 `initial_status.time_in_state` 判定 |
| 备用报价 | `spinning_reserve_price`、`regulation_up/down_price` | 备用容量按报价计入目标 |

**报价段语义（容易出错）：** `bid_segments` 中 `quantity` 表示**超出** `pmin` 的容量（MW），机组最大总出力 = pmin + Σ quantity（受 pmax 上限约束）。各分段 `quantity` 之和应等于 `pmax − pmin`，超出部分被忽略；分段价格物理上应递增（调度优先使用低价段）。实际使用段数 = `min(config.n_segments, bid_segments.size())`。若 `bid_segments` 为空，机组能以零成本出力——不推荐用于生产。

### 8.1.2 网络约束

网络建模基于直流潮流（DC power flow）与 PTDF（Power Transfer Distribution Factor）矩阵：

- `branches` 数组给出线路 `from`/`to` 母线、`reactance`（标幺电抗）和 `rating_mw`（热稳极限）；`in_service=false` 的线路从 PTDF 计算中排除。电抗 ≤ 1e-6 时内部以 1e-3 替代以避免数值奇异。
- PTDF 按 F = B_f · B_red⁻¹ 计算，松弛母线固定为 bus 0；若网络孤岛导致 B_red 不可逆，逆矩阵置零，退化为铜排模式。
- **铜排模式**：`branches` 为空列表时系统按单母线运行，不计算 PTDF、不施加线路潮流约束。小规模验证或无网架数据时可直接使用。
- **断面监视**：`sections` 用 `line_weights`（支路索引 + 权重）定义断面，断面潮流 = Σ weight × 支路潮流，受 `rating_fwd_mw` / `rating_rev_mw` 限额约束。
- 线路/断面越限通过松弛变量处理，惩罚系数为 `config.M1_line_slack_penalty`（默认 1e5 $/MW）——越限以高罚函数形式允许存在，判读结果时应检查是否实际发生。
- 另有 `dc_lines`（直流线路，带 pmin/pmax 与爬坡限制）与 `generator_groups`（机组群约束，用于抽水蓄能等共享出力/能量限制的机组集合），字段细节见 [SCUC 数据格式](04-modeling-aml.md)。

### 8.1.3 备用与惩罚

系统级备用需求在 `config` 中以系统负荷分数或绝对 MW 给出：

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `spinning_reserve_req` | 0.10 | 旋转备用（负荷分数） |
| `regulation_up_req` / `regulation_down_req` | 0.05 / 0.05 | 调频上/下备用 |
| `pfr_reserve_req_mw` | 0.0 | 一次调频备用总量（MW），机组侧由 `pfr_alpha × pmax` 提供 |
| `voll` | 10000.0 | 失负荷价值（$/MWh），切负荷惩罚 |
| `vocc` | 1000.0 | 系统级弃电惩罚（$/MWh） |
| `M2_renewable_curtail_penalty` | 0.0 | 可再生弃电惩罚（$/MW/时段），> 0 时激活弃电变量 |

注意内置标准算例会覆盖部分默认值（如 IEEE 39-bus 算例 `voll`=10000、`spinning_reserve_req`=5%），以构造器实际输出为准。

## 8.2 用内置算例构造器生成算例

`case_builder` 提供三个预构建的 `SCUCInput`，包含完整的母线拓扑、支路参数、发电机报价、负荷曲线和时序曲线，可直接传入 `scuc_solve()`，适合单元测试、集成测试与性能基准。

### 8.2.1 C++ 接口

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

典型用法——生成输入、改写配置、落盘为 JSON：

```cpp
SCUCInput inp = build_ieee39_case(24, 1.0, true, false);
std::string json = scuc_input_to_json(inp, 2);  // indent=-1 为紧凑格式
std::ofstream("ieee39_24h.json") << json;
```

### 8.2.2 三个标准算例概要

| 算例 | 母线 | 机组 | 支路 | 基准负荷 | 默认 T | 可选扩展 |
|------|------|------|------|----------|--------|----------|
| 3-bus | 3 | 2（Coal 400MW + Gas 200MW） | 2 | 230 MW | 3 | — |
| 6-bus | 6 | 3（Coal/Gas/Peaker） | 7 | 500 MW | 24 | 风电 100 MW（bus 3）、储能 240 MWh/60 MW（bus 5） |
| IEEE 39-bus | 39 | 10 | 46 | 4,599 MW | 24 | 风电 3 站共 1,050 MW、光伏 2 站共 450 MW |

负荷曲线均为双峰正弦形（峰值系数 1.0 出现在约 10:00 与 19:00，谷值系数 0.65）。3-bus 用于快速冒烟测试，6-bus 用于集成测试，IEEE 39-bus 接近真实日前市场出清规模。各算例的机组参数、报价段、负荷分布与初始状态的完整表格见 [建模与数据 (AML)](04-modeling-aml.md)。

> 命令行工具 `scuc_case_builder` 还支持 `ieee118` 算例（见下节）；C++ `case_builder` 文档仅覆盖上述三个。

## 8.3 命令行工具：scuc_case_builder 与 scuc_solve

两步走：先生成算例 JSON，再求解。

```powershell
scuc_case_builder --case 6bus --T 24 --wind --storage --output case.json
```

可选算例为 `3bus`、`6bus`、`ieee39`、`ieee118`；`--dt` 设置时段小时数，`--solar` 和 `--wind` 加入新能源。

求解：

```powershell
scuc_solve case.json result.json --solver StrictHiGHS --indent 2
```

可用参数：

```text
--solver Auto|StrictHiGHS|HiGHS|SCIP|NativeBranchAndCut
--no-sced
--no-lmp
--indent <n>
```

未指定输出文件时，JSON 写到标准输出，进度与摘要写到标准错误。SCUC 未收敛时进程返回非零退出码，可直接接入 shell 脚本或调度系统做失败告警。

> **求解器选择提示**：`"Auto"` 的优先级为 Gurobi → StrictHiGHS → HiGHS → NativeBranchAndCut。源文档间存在一处不一致：用户手册的 CLI 帮助列出 `SCIP`，而数据格式规范明确指出 `SCIP` 仅支持 MINLP、不适用于 MILP 调度。以较新的数据格式规范为准——MILP 调度不要选 SCIP。另外 HiGHS 经 MPS 文件 I/O 不返回约束对偶，LMP 阶段会自动回退到 Gurobi 或 NativeBranchAndCut。

## 8.4 输入 JSON 准备要点

顶层结构（所有字段均可省略，缺省时用默认值；母线编号与索引一律从 0 开始）：

```json
{
  "num_buses":         <int, 可选，缺省自动推断为最大母线编号+1>,
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

`config` 中最常改的字段：

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `solver` | `"Auto"` | MILP 求解器 |
| `num_periods` / `period_length_hr` | 24 / 1.0 | 时段数 T 与步长 Δt（小时） |
| `mip_gap` | 0.001 | 相对 MIP 最优性间隙 |
| `time_limit_sec` | 300.0 | 求解时间上限（秒） |
| `solve_sced` / `solve_lmp` | true / true | 是否继续 SCED 与 LMP 阶段 |
| `enable_market_cuts` | true | 是否在 SCUC 前添加预构型削减切面 |
| `verbose` | false | 求解器详细日志 |

工程上需要特别注意的一致性规则：

- **profiles 维度**：`profiles.load[d][t]` 是负荷倍率（乘以 `loads[d].p_mw`），行数应与 `loads` 一致；`profiles.wind/solar` 是**绝对预测出力（MW）**，行数与 `wind`/`solar` 数组一致，值应在 [0, pmax] 内。行数不足时缺行机组用基准值/`pmax` 补齐，列数不足时用最后一列补齐——静默行为，准备数据时应自行核对维度。
- **初始状态**：`initial_status.commitment`（0/1）、`dispatch`（MW）、`time_in_state`（正=在线小时数，负=停机小时数，用于热/温/冷启动费用判定）、`storage_soc`（≤1 视为容量分数，否则视为 MWh）。数组长度不足时超出机组用默认值，不报错。
- **储能 SOC 优先级**：`initial_status.storage_soc` 优先于机组字段 `soc_init`。
- **异常语义**：JSON 语法错误抛 `nlohmann::json::parse_error`，字段类型不匹配抛 `nlohmann::json::type_error`；数值越界不抛异常而是 clamp 到有效区间。调用方（C++）应捕获前两类异常。

一个最小可运行示例（单母线 2 机 1 负荷 3 时段，铜排模式）：

```json
{
  "config": { "num_periods": 3, "period_length_hr": 1.0,
              "n_segments": 2, "mip_gap": 0.001,
              "time_limit_sec": 60.0, "solve_lmp": false },
  "generators": [
    { "name": "Coal-1", "bus": 0, "pmin": 100.0, "pmax": 400.0,
      "ramp_up_mw_min": 5.0, "ramp_dn_mw_min": 5.0,
      "min_up_time_hr": 2.0, "min_dn_time_hr": 2.0,
      "startup_cost": 3000.0, "no_load_cost": 150.0,
      "bid_segments": [ {"price": 20.0, "quantity": 150.0},
                        {"price": 28.0, "quantity": 150.0} ] },
    { "name": "Gas-1", "bus": 0, "pmin": 50.0, "pmax": 200.0,
      "ramp_up_mw_min": 10.0, "ramp_dn_mw_min": 10.0,
      "min_up_time_hr": 1.0, "min_dn_time_hr": 1.0,
      "startup_cost": 800.0,
      "bid_segments": [ {"price": 35.0, "quantity": 100.0},
                        {"price": 45.0, "quantity": 50.0} ] }
  ],
  "loads": [ { "bus": 0, "p_mw": 300.0 } ],
  "profiles": { "load": [ [1.0, 1.2, 0.9] ] },
  "initial_status": { "commitment": [1.0, 0.0],
                      "dispatch": [250.0, 0.0],
                      "time_in_state": [6.0, -3.0] }
}
```

## 8.5 输出结果判读

### 8.5.1 先判收敛，再看数值

三阶段的收敛语义：

| 情况 | `scuc.converged` | `sced.converged` | `lmp.converged` | 说明 |
|------|:---:|:---:|:---:|------|
| 正常求解 | true | true | true | 三阶段均成功 |
| SCUC 超时（有可行解） | true | true | true | `mip_gap > 0`，未证最优 |
| SCUC 超时（无可行解） | false | false | false | 不可行或时限不足 |
| SCUC 成功、SCED 不可行 | true | false | false | 罕见（固定组合后 LP 不可行） |
| LMP 求解器不支持对偶 | true | true | false | HiGHS 无对偶且无回退求解器 |

`converged=false` 时所有矩阵为空、标量成本为 0，不要误读为"零成本可行解"。SCUC 收敛但 `mip_gap > 0` 时结果是可行 incumbent 而非最优证明，是否可用由业务容差决定。

### 8.5.2 机组计划与费用分解

JSON 输出（`scuc_output_to_json`）的核心结构：

```json
{
  "meta":  { "solver": "...", "num_periods": 24, "num_buses": 39, ... },
  "scuc": {
    "converged": true, "objective": 891466.2, "solver_name": "HiGHS",
    "solve_time_sec": 3.14, "mip_gap": 0.0009, "n_cuts_added": 12,
    "commitment": [[1,1,...], ...],     // [ng][T_commit]，0/1
    "startup":    [[...], ...],
    "shutdown":   [[...], ...],
    "dispatch":   [[500.0,...], ...],   // [ng][T]，MW
    "spinning_reserve": [[...]], "regulation_up": [[...]], "regulation_down": [[...]],
    "wind_generation": [[...]], "solar_generation": [[...]],
    "storage_charging": [[...]], "storage_discharging": [[...]], "storage_soc": [[...]],
    "line_flows": [[45.2,...], ...],    // [nl][T]，MW，正方向 from→to
    "load_shedding":  [0.0, ...],       // [T]，MW
    "gen_curtailment":[0.0, ...],
    "cost": { "energy": 820000.0, "startup": 15000.0, "no_load": 48000.0,
              "reserve": 8466.2, "penalty": 0.0, "total": 891466.2 }
  },
  "sced": { "...": "同 scuc 结构" },
  "lmp":  { "converged": true, "avg_lmp": 32.5, "max_lmp": 58.3, "min_lmp": 18.1,
            "nodal": [[...]], "energy": [[...]], "congestion": [[...]] }
}
```

判读要点：

- **机组计划**：`commitment[g][t]`（0/1）与 `startup`/`shutdown` 指示构成开停计划；`dispatch[g][t]` 为出力计划（MW）。`cost.total` = energy + startup + no_load + reserve + penalty。
- **潮流**：`line_flows[l][t]` 为支路潮流（正方向 from→to），应与 `rating_mw` 对照检查；`section_flows` 为断面潮流。因线路越限以 Big-M 松弛形式允许存在，**必须检查** `load_shedding`、`gen_curtailment` 是否为零、`penalty` 成本是否为零——非零说明发生了切负荷/弃电或越限，通常意味着输入数据（限额、备用、报价）需要复核。
- **影子价格（LMP）**：`lmp.nodal[bus][t]` 为节点边际电价（$/MWh），分解为 `energy`（系统边际价格分量）与 `congestion`（阻塞分量）。`avg/max/min_lmp` 是时空汇总。正值表示消费方支付；阻塞严重时受端节点理论上可能出现负值。LMP 来自固定组合后的 LP 对偶——MILP 本身不返回有意义的行对偶，不要试图从 SCUC 阶段直接取对偶。
- 维度保证：`commitment` 等为 `[ng][T_commit]`（T_commit = T / intervals_per_hour），`dispatch` 等为 `[ng][T]`，LMP 为 `[nb][T]`；无对应设备时为空数组 `[]`。

### 8.5.3 Python 判读示例

来自教程 demo_05（`solve_json` 一次调用，自动完成 UC + SCED + LMP）的精简版：

```python
import json
import mipsolvers

with open("case.json", "r", encoding="utf-8") as f:
    result = json.loads(mipsolvers.scuc.solve_json(f.read(), indent=2))

scuc = result["scuc"]
print("收敛:", scuc["converged"])
print(f"总成本: ${scuc['cost']['total']:,.2f}")
print(f"  ├ 启动成本: ${scuc['cost']['startup']:,.2f}")
print(f"  └ 能量成本: ${scuc['cost']['energy']:,.2f}")

# 开停计划（1=开, 0=停）与启动次数统计
for gi, row in enumerate(scuc["commitment"]):
    starts = sum(1 for t in range(1, len(row))
                 if row[t] > 0.5 and row[t-1] < 0.5)
    print(f"  机组{gi}: {''.join(str(int(v > 0.5)) for v in row)}  ({starts}次启动)")
```

也可以用对象方式调用并直接读属性：

```python
case = mipsolvers.scuc.build_6bus_case(T=24, dt=1.0, with_wind=True, with_storage=True)
case.config.solver = "StrictHiGHS"
case.config.mip_gap = 1e-3

output = mipsolvers.scuc.solve(case)
print(output.scuc.converged, output.scuc.total_cost)
print(output.lmp.avg_lmp)
```

## 8.6 用 AML 自建简化 SCUC 模型

当内置 `scuc` 模块的约束体系不满足定制需求（如特殊的厂级约束、自定义目标项）时，可用 AML 自行建模。教程 demo_04 给出了 2 机组 × 6 时段的最小 UC 模型，其结构可直接扩展：

- 决策变量：`u[g,t]`（在线，Binary）、`p[g,t]`（出力，Continuous ≥ 0）、`su/sd[g,t]`（启停指示，Binary）；
- 目标：min Σ（空载·u + 报价·p + 启动费·su）；
- 约束：功率平衡、pmin·u ≤ p ≤ pmax·u、爬坡上下限、启停逻辑 su−sd = u(t)−u(t−1)、su+sd ≤ 1。

核心片段（完整脚本见 `docs/tutorial/demo_04_aml_scuc.py`）：

```python
u  = m.add_var2("u",  G, T_set, aml.VarType.Binary)
p  = m.add_var2("p",  G, T_set, aml.VarType.Continuous, lb=0.0)
su = m.add_var2("su", G, T_set, aml.VarType.Binary)
sd = m.add_var2("sd", G, T_set, aml.VarType.Binary)

m.minimize(aml.sum_over(G, lambda g:
    aml.sum_over(T_set, lambda t:
        gen_data[g.values[0]]["no_load"] * u[(g, t)] +
        gen_data[g.values[0]]["bid"]     * p[(g, t)] +
        gen_data[g.values[0]]["startup"] * su[(g, t)])))
```

求解用标准 MILP 路径（`opts.solver_name = "NativeBranchAndCut"` 或其他 MILP 后端，`opts.mip_gap_tol` 控制间隙）。工程建议：**标准 SCUC/SCED/LMP 业务直接用 SCUC 专用接口，不要手工复制其约束**——自建 AML 模型只用于内置模型确实表达不了的定制场景。AML 建模细节见 [AML 建模](04-modeling-aml.md)。

## 8.7 OPF 场景的求解路径选择建议

源文档未提供独立的 AC-OPF 模块；依据现有求解器能力（见 [求解器与引擎](05-solvers-engines.md)），可按问题形态选择路径：

- **DC-OPF / SCED（线性潮流，LP）**：优先走 `scuc` 模块的 Stage 2（固定组合后的 SCED LP），它自动处理 PTDF 网络约束并衔接 LMP 计算。单独求解 LP 时，本机 NETLIB 基准的结论是：通用 LP 默认选择 `HiGHS` 的 simplex 路径；大型稀疏 LP 可尝试 HiGHS IPM，但仍需解后审计并保留 simplex fallback。`NativeIPMLP` 为研究/影子路径，必须保留解后审计和 fallback；`NativePDLP`、`NativeLCQP` 的 LP 路由当前不应进入生产。
- **需要 LMP/影子价格**：必须在固定整数决策后的 LP/SCED 上读取对偶，MILP 阶段本身不返回有意义的行对偶；注意 HiGHS 经 MPS 文件 I/O 不返回约束对偶，LMP 需回退到 Gurobi 或 NativeBranchAndCut。
- **带二次目标的 OPF（凸 QP）**：通用 QP 可选 `Gurobi`；`NativeLCQP` 目前主要覆盖等式和箱界，属实验路径。非凸 Q 无全局最优保证。
- **AC-OPF（非线性潮流，NLP）**：生产优先 `Ipopt`（外部成熟 NLP 后端）；`NativeIPM`（原始-对偶滤子内点法）与 `NativeNLP`（轻量罚函数 Newton）为自研路径。NLP 调用必须提供解析导数并用有限差分独立核对，成功判定时同时检查原始可行度、对偶可行度和互补残差——`unknown` 状态不能解释为不可行。
- **MINLP 形态的 OPF**（如含离散变压器分接头）：可选 `NativeBranchAndCut` 或 `SCIP`；自研 MINLP 对一般非凸问题不提供全局最优保证，严格生产证明优先外部后端。

通用的结果验收纪律（适用于以上所有路径）：检查 `success`/`status`/实际 `solver_name`，拒绝尺寸错误或非有限的解向量，MILP 额外核对 `mip_gap`，并保存输入、软件版本与求解日志以便复现。详见 [测试与基准](09-testing-benchmarks.md) 与 [故障排查](10-troubleshooting.md)。

## 8.8 延伸阅读

- [SCUC 数据格式规范](04-modeling-aml.md) — 输入/输出全部字段、维度与异常语义
- [SCUC 算例构造器](04-modeling-aml.md) — 标准算例参数明细与性能基准
- [快速上手](03-quickstart.md)、[安装与部署](02-installation-deploy.md)
