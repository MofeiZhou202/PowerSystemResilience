# SCUC 数据格式规范

**版本：** 1.0  
**适用模块：** `mipsolvers::scuc`  
**对应头文件：** `include/mipsolvers/scuc/scuc.hpp`  
**对应实现：** `src/scuc/scuc.cpp`

---

## 目录

1. [总体约定](#1-总体约定)
2. [输入格式规范](#2-输入格式规范)
   - 2.1 顶层结构
   - 2.2 `config` — 求解配置
   - 2.3 `generators` — 机组参数
   - 2.4 `branches` — 输电线路
   - 2.5 `loads` — 负荷节点
   - 2.6 `wind` / `solar` — 风光机组
   - 2.7 `storage` — 储能机组
   - 2.8 `dc_lines` — 直流线路
   - 2.9 `generator_groups` — 机组群约束
   - 2.10 `sections` — 断面监视
   - 2.11 `profiles` — 时序曲线
   - 2.12 `initial_status` — 初始状态
3. [中间处理规范](#3-中间处理规范)
   - 3.1 母线编号推断
   - 3.2 PTDF 矩阵计算
   - 3.3 MILP 决策变量布局（VarIndex）
   - 3.4 约束族
   - 3.5 求解流水线
4. [输出格式规范](#4-输出格式规范)
   - 4.1 C++ 结构体
   - 4.2 JSON 输出（`scuc_output_to_json`）
   - 4.3 数组维度保证
   - 4.4 收敛语义与异常情况
5. [JSON 完整示例](#5-json-完整示例)

---

## 1. 总体约定

| 约定 | 说明 |
|------|------|
| 索引 | 所有母线编号、机组索引均从 **0** 开始 |
| 时间步 | 计划周期共 `T = config.num_periods` 个时间步，步长 `Δt = config.period_length_hr`（小时） |
| 提交时间步 | `T_commit = T / intervals_per_hour`（用电整数，默认 `intervals_per_hour = 1`） |
| 功率单位 | **MW**；能量单位 **MWh** |
| 成本单位 | **美元（$）** |
| LMP 单位 | **$/MWh** |
| JSON 编码 | UTF-8；浮点数不限精度；数组用 `[]` |
| 字段缺省 | 所有字段均可选（有默认值）；未提供时使用下方各表所列默认值 |
| 无效值处理 | 运行时 **不抛异常**（仅 `json::parse` 格式错误时除外）；超出范围的数值被 `clamp` 至有效区间 |

> **重要**：JSON 反序列化（`scuc_from_json`）对格式错误（非法 JSON 语法）会抛出 `nlohmann::json::parse_error`；对字段类型不匹配会抛出 `nlohmann::json::type_error`。调用方应在外层捕获这两类异常。

---

## 2. 输入格式规范

### 2.1 顶层结构

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

| 字段 | 类型 | 说明 |
|------|------|------|
| `num_buses` | `int` | 系统母线总数。**可省略**：若未提供，自动推断为所有组件中最大母线编号 + 1（至少为 1） |
| 其余字段 | 对象/数组 | 详见后续各节，均可整体省略（等价于空列表或默认值） |

---

### 2.2 `config` — 求解配置

```json
"config": {
  "solver":                     "Auto",
  "allow_fallback":             true,
  "num_periods":                24,
  "period_length_hr":           1.0,
  "n_segments":                 3,
  "mip_gap":                    0.001,
  "time_limit_sec":             300.0,
  "verbose":                    false,
  "spinning_reserve_req":       0.10,
  "regulation_up_req":          0.05,
  "regulation_down_req":        0.05,
  "neg_reserve_req":            0.0,
  "pfr_reserve_req_mw":         0.0,
  "voll":                       10000.0,
  "vocc":                       1000.0,
  "renewable_min_output_coeff": 0.0,
  "M2_renewable_curtail_penalty": 0.0,
  "M1_line_slack_penalty":      1.0e5,
  "wheeling_fee_per_mwh":       0.0,
  "enable_market_cuts":         true,
  "solve_sced":                 true,
  "solve_lmp":                  true,
  "lmp_delta":                  0.10
}
```

| 字段 | 类型 | 默认值 | 有效范围 | 说明 |
|------|------|--------|----------|------|
| `solver` | `string` | `"Auto"` | `"Auto"`, `"Gurobi"`, `"HiGHS"`, `"NativeBranchAndCut"`, `"SCIP"` | MILP 求解器。`"Auto"` 优先级：HiGHS → Gurobi → NativeBranchAndCut；`"SCIP"` 仅支持 MINLP 不适用于 MILP 调度 |
| `allow_fallback` | `bool` | `true` | — | 主求解器失败时是否自动降级 |
| `num_periods` | `int` | `24` | ≥ 1 | 时间步数 T |
| `period_length_hr` | `float` | `1.0` | > 0 | 每个时间步的小时数 Δt |
| `n_segments` | `int` | `3` | ≥ 1 | 每台机组的报价分段数（取 `min(n_segments, bid_segments.size())`） |
| `mip_gap` | `float` | `0.001` | [0, 1] | 相对 MIP 最优性间隙 |
| `time_limit_sec` | `float` | `300.0` | > 0 | 求解时间上限（秒） |
| `verbose` | `bool` | `false` | — | 是否输出求解器详细日志 |
| `spinning_reserve_req` | `float` | `0.10` | [0, 1] | 旋转备用需求（系统负荷分数，§2.6.3.2） |
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
| `lmp_delta` | `float` | `0.10` | [0, 1] | LMP 再调度邻域因子 δ（§2.6.5.5） |

---

### 2.3 `generators` — 机组参数

```json
"generators": [
  {
    "name":                   "G1",
    "bus":                    0,
    "pmin":                   100.0,
    "pmax":                   400.0,
    "ramp_up_mw_min":         5.0,
    "ramp_dn_mw_min":         5.0,
    "min_up_time_hr":         4.0,
    "min_dn_time_hr":         4.0,
    "must_run":               false,
    "max_startups":           0,
    "max_shutdowns":          0,
    "startup_cost":           5000.0,
    "startup_cost_warm":      3000.0,
    "startup_cost_cold":      8000.0,
    "hot_start_threshold_hr": 4.0,
    "warm_start_threshold_hr":8.0,
    "ud_periods":             0,
    "dd_periods":             0,
    "no_load_cost":           200.0,
    "spinning_reserve_price": 10.0,
    "regulation_up_price":    15.0,
    "regulation_down_price":  12.0,
    "pfr_alpha":              0.0,
    "group_id":               -1,
    "bid_segments": [
      { "price": 25.0, "quantity": 100.0 },
      { "price": 30.0, "quantity": 150.0 },
      { "price": 38.0, "quantity": 50.0  }
    ]
  }
]
```

| 字段 | 类型 | 默认值 | 有效范围 | 说明 |
|------|------|--------|----------|------|
| `name` | `string` | `""` | — | 机组标识名称 |
| `bus` | `int` | `0` | [0, num_buses-1] | 接入母线（0 基） |
| `pmin` | `float` | `0.0` | ≥ 0 | 最小出力（MW） |
| `pmax` | `float` | `0.0` | ≥ pmin | 最大出力（MW） |
| `ramp_up_mw_min` | `float` | `0.0` | ≥ 0 | 爬坡速率（MW/min） |
| `ramp_dn_mw_min` | `float` | `0.0` | ≥ 0 | 降坡速率（MW/min） |
| `min_up_time_hr` | `float` | `0.0` | ≥ 0 | 最小在线时长（小时） |
| `min_dn_time_hr` | `float` | `0.0` | ≥ 0 | 最小停机时长（小时） |
| `must_run` | `bool` | `false` | — | 强制运行：将机组提交变量下界固定为 1 |
| `max_startups` | `int` | `0` | ≥ 0 | 规划期内最大启动次数；0 = 不限 |
| `max_shutdowns` | `int` | `0` | ≥ 0 | 规划期内最大停机次数；0 = 不限 |
| `startup_cost` | `float` | `0.0` | ≥ 0 | 热启动费用（$）；也作为 `startup_cost_warm/cold` 的回退值 |
| `startup_cost_warm` | `float` | `0.0` | ≥ 0 | 温启动费用（$）；0 = 使用 `startup_cost` |
| `startup_cost_cold` | `float` | `0.0` | ≥ 0 | 冷启动费用（$）；0 = 使用 `startup_cost` |
| `hot_start_threshold_hr` | `float` | `4.0` | > 0 | 停机时间 ≤ 此值视为热启动 |
| `warm_start_threshold_hr` | `float` | `8.0` | > hot_start_threshold_hr | 停机时间 ≤ 此值（>热启动阈值）视为温启动 |
| `ud_periods` | `int` | `0` | ≥ 0 | 启动轨迹周期数（从并网到 Pmin，§2.6.3.8） |
| `dd_periods` | `int` | `0` | ≥ 0 | 停机轨迹周期数（从 Pmin 到离网，§2.6.3.8） |
| `no_load_cost` | `float` | `0.0` | ≥ 0 | 空载成本（$/h，机组在线时按时间计费） |
| `spinning_reserve_price` | `float` | `0.0` | ≥ 0 | 旋转备用报价（$/MWh） |
| `regulation_up_price` | `float` | `0.0` | ≥ 0 | 调频上备用报价（$/MWh） |
| `regulation_down_price` | `float` | `0.0` | ≥ 0 | 调频下备用报价（$/MWh） |
| `pfr_alpha` | `float` | `0.0` | [0, 1] | 一次调频系数：最大一次调频 = pfr_alpha × pmax；0 = 不提供 |
| `group_id` | `int` | `-1` | ≥ -1 | 所属机组群 ID；-1 = 不属于任何群 |
| `bid_segments` | `array` | `[]` | 见下 | 分段报价曲线 |

**`bid_segments` 元素：**

| 字段 | 类型 | 默认值 | 有效范围 | 说明 |
|------|------|--------|----------|------|
| `price` | `float` | `0.0` | ≥ 0 | 分段报价（$/MWh），**第一个字段** |
| `quantity` | `float` | `0.0` | ≥ 0 | 分段容量（MW，相对于 pmin 的增量），**第二个字段** |

> **注意事项：**  
> - 实际使用分段数 = `min(n_segments, bid_segments.size())`  
> - 如果 `bid_segments` 为空，机组能以零成本出力（不推荐用于生产）  
> - 各分段 `quantity` 之和应等于 `pmax - pmin`；超出部分被忽略  
> - 分段无需单调排序，但物理上应价格递增（调度从低价分段优先使用）

---

### 2.4 `branches` — 输电线路

```json
"branches": [
  {
    "from":       0,
    "to":         1,
    "reactance":  0.05,
    "rating_mw":  200.0,
    "in_service": true
  }
]
```

| 字段 | 类型 | 默认值 | 有效范围 | 说明 |
|------|------|--------|----------|------|
| `from` | `int` | `0` | [0, num_buses-1] | 始端母线（0 基） |
| `to` | `int` | `0` | [0, num_buses-1] | 末端母线（0 基） |
| `reactance` | `float` | `0.05` | 建议 > 1e-6 | 线路电抗（标幺值）；若 ≤ 1e-6，内部以 1e-3 替代 |
| `rating_mw` | `float` | `1e6` | > 0 | 热稳极限（MW）；默认 1e6 ≈ 无限制 |
| `in_service` | `bool` | `true` | — | `false` 时该线路从 PTDF 计算中排除 |

> **铜排模式：** 当 `branches` 为空列表时，系统以单母线（铜排）模式运行，不计算 PTDF，不施加线路潮流约束。

---

### 2.5 `loads` — 负荷节点

```json
"loads": [
  { "bus": 2, "p_mw": 300.0 }
]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `bus` | `int` | `0` | 负荷接入母线（0 基） |
| `p_mw` | `float` | `0.0` | 基准有功需求（MW）；运行时乘以 `profiles.load[d][t]` |

> **profiles 一致性**：`profiles.load` 的行数应与 `loads.size()` 一致。若行数不足，缺失时间步采用基准值 `p_mw`（倍率默认 1.0）。

---

### 2.6 `wind` / `solar` — 风光机组

```json
"wind": [
  { "bus": 5, "pmax": 400.0 }
],
"solar": [
  { "bus": 21, "pmax": 250.0 }
]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `bus` | `int` | `0` | 接入母线（0 基） |
| `pmax` | `float` | `0.0` | 额定装机容量（MW），仅在曲线缺失时作为回退值 |

> **profiles 一致性**：  
> - `profiles.wind[w][t]` 为第 w 台风机在第 t 步的**绝对预测出力（MW）**，应满足 0 ≤ 值 ≤ `pmax`  
> - `profiles.solar[s][t]` 同上  
> - 若 `profiles.wind/solar` 行数不足，缺失单元以 `pmax` 替代  
> - 风光出力上界 = 各时间步预测值；下界 = `renewable_min_output_coeff × 预测值`

---

### 2.7 `storage` — 储能机组

```json
"storage": [
  {
    "bus":                  3,
    "pmax_charge":          200.0,
    "pmax_discharge":       200.0,
    "pmin_charge":          0.0,
    "pmin_discharge":       0.0,
    "energy_capacity_mwh":  800.0,
    "efficiency":           0.90,
    "eta_charge":           -1.0,
    "eta_discharge":        -1.0,
    "soc_init":             0.5,
    "soc_min":              -1.0,
    "soc_final":            -1.0,
    "charge_bid_price":     5.0,
    "discharge_bid_price":  20.0,
    "cycle_limit":          0.0,
    "use_binary_indicators":false
  }
]
```

| 字段 | 类型 | 默认值 | 有效范围 | 说明 |
|------|------|--------|----------|------|
| `bus` | `int` | `0` | [0, num_buses-1] | 接入母线（0 基） |
| `pmax_charge` | `float` | `0.0` | ≥ 0 | 最大充电功率（MW） |
| `pmax_discharge` | `float` | `0.0` | ≥ 0 | 最大放电功率（MW） |
| `pmin_charge` | `float` | `0.0` | ≥ 0 | 最小充电功率（MW，充电时强制下界） |
| `pmin_discharge` | `float` | `0.0` | ≥ 0 | 最小放电功率（MW，放电时强制下界） |
| `energy_capacity_mwh` | `float` | `0.0` | ≥ 0 | 额定能量容量（MWh） |
| `efficiency` | `float` | `0.9` | (0, 1]，内部 clamp 至 [0.1, 1.0] | 往返效率（传统字段）；若 `eta_charge/discharge < 0`，则 η_ch = η_dis = √efficiency |
| `eta_charge` | `float` | `-1.0` | (0, 1]，内部 clamp 至 [0.01, 1.0] | 充电效率；< 0 表示从 `efficiency` 推算 |
| `eta_discharge` | `float` | `-1.0` | (0, 1]，内部 clamp 至 [0.01, 1.0] | 放电效率；< 0 表示从 `efficiency` 推算 |
| `soc_init` | `float` | `0.5` | [0, 1] | 初始 SOC（容量分数）；也可从 `initial_status.storage_soc` 覆盖 |
| `soc_min` | `float` | `-1.0` | [0, 1] 若 ≥ 0 | SOC 下限分数；< 0 = 默认 0.10（10% 容量）|
| `soc_final` | `float` | `-1.0` | [0, 1] 若 ≥ 0 | 规划期末 SOC 约束；< 0 = 等于初始 SOC |
| `charge_bid_price` | `float` | `0.0` | ≥ 0 | 充电报价（$/MWh，充电成本） |
| `discharge_bid_price` | `float` | `0.0` | ≥ 0 | 放电报价（$/MWh，放电收益） |
| `cycle_limit` | `float` | `0.0` | ≥ 0 | 每日等效满充满放次数上限；0 = 不限 |
| `use_binary_indicators` | `bool` | `false` | — | 使用充/放电二进制模式变量（ξ+/ξ-），精确互斥但增加模型规模 |

> **SOC 初始值覆盖优先级：**  
> `initial_status.storage_soc[s]` > `soc_init`（当 `initial_status.storage_soc` 提供且长度足够时）

---

### 2.8 `dc_lines` — 直流线路

```json
"dc_lines": [
  {
    "from":    0,
    "to":      10,
    "pmin":    -500.0,
    "pmax":    500.0,
    "ramp_up": 200.0,
    "ramp_dn": 200.0
  }
]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `from` | `int` | `0` | 注入母线（0 基） |
| `to` | `int` | `0` | 抽出母线（0 基） |
| `pmin` | `float` | `-1e6` | 最小传输功率（MW，负值表示反向） |
| `pmax` | `float` | `1e6` | 最大传输功率（MW） |
| `ramp_up` | `float` | `1e6` | 爬坡上限（MW/步） |
| `ramp_dn` | `float` | `1e6` | 降坡上限（MW/步） |

---

### 2.9 `generator_groups` — 机组群约束

用于建模抽水蓄能、发电-抽水联合等需要共享出力限制的机组集合（§2.6.3.9–2.6.3.10）。

```json
"generator_groups": [
  {
    "id":          0,
    "name":        "PSH-Group-1",
    "gen_indices": [2, 3],
    "pmin_t":      [100.0],
    "pmax_t":      [600.0],
    "emin":        0.0,
    "emax":        4800.0
  }
]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `id` | `int` | `-1` | 群 ID（与 `generator.group_id` 对应） |
| `name` | `string` | `""` | 群名称 |
| `gen_indices` | `int[]` | `[]` | 成员机组在 `generators` 数组中的 0 基索引 |
| `pmin_t` | `float[]` | `[]` | 群总出力下限（MW）；长度为 T 或 1（标量广播）；空 = 不约束 |
| `pmax_t` | `float[]` | `[]` | 群总出力上限（MW）；同上 |
| `emin` | `float` | `0.0` | 规划期内群累计出力下限（MWh）；0 = 不约束 |
| `emax` | `float` | `0.0` | 规划期内群累计出力上限（MWh）；0 = 不约束 |

---

### 2.10 `sections` — 断面监视

```json
"sections": [
  {
    "name":          "WestCorridor",
    "rating_fwd_mw": 800.0,
    "rating_rev_mw": 600.0,
    "line_weights": [
      [0, 1.0],
      [3, 0.5]
    ]
  }
]
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `name` | `string` | `""` | 断面名称 |
| `rating_fwd_mw` | `float` | `1e6` | 正向限额（MW）；默认无限制 |
| `rating_rev_mw` | `float` | `1e6` | 反向限额（MW）；默认无限制 |
| `line_weights` | 见下 | `[]` | 支路权重列表 |

`line_weights` 支持两种格式：
- 数组格式：`[branch_index, weight]`（整数 + 浮点）
- 对象格式：`{"line": branch_index, "weight": weight}`

断面潮流 = Σ weight_l × 支路 l 潮流

---

### 2.11 `profiles` — 时序曲线

```json
"profiles": {
  "load":  [ [1.0, 0.95, 0.9, ...], [0.8, 0.75, ...] ],
  "wind":  [ [120.0, 150.0, ...] ],
  "solar": [ [0.0, 0.0, 50.0, ...] ]
}
```

| 字段 | 类型 | 维度 | 默认行为 | 说明 |
|------|------|------|----------|------|
| `load` | `float[][]` | `[nload][T]` | 若缺失，负荷量固定为 `p_mw` | 负荷倍率（标幺值）；`load[d][t]` 乘以 `loads[d].p_mw` |
| `wind` | `float[][]` | `[nwind][T]` | 若缺失，以 `pmax` 作为预测值 | 风机绝对预测出力（MW） |
| `solar` | `float[][]` | `[nsolar][T]` | 若缺失，以 `pmax` 作为预测值 | 光伏绝对预测出力（MW） |

> **行数不足时的处理：** 若某行缺失（e.g. `profiles.wind` 只有 1 行但有 2 台风机），超出索引的机组使用其 `pmax`；若列数不足（时间步缺失），超出时间步使用最后一列值或基准值。

---

### 2.12 `initial_status` — 初始状态

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
| `commitment` | `float[]` | `[ng]` | 全零（全停） | 第 0 周期前机组状态，0 或 1；内部 clamp 至 [0, 1] |
| `dispatch` | `float[]` | `[ng]` | 全零 | 第 0 周期机组出力（MW）；负值内部取 0 |
| `storage_soc` | `float[]` | `[nstorage]` | 使用 `soc_init` 字段 | 储能初始 SOC（分数或 MWh）；≤ 1+ε 视为分数，否则视为 MWh |
| `time_in_state` | `float[]` | `[ng]` | 全零 | 已连续处于当前状态的小时数；正值 = 在线，负值 = 停机；用于热/温/冷启动判断 |

> **数组长度不足：** 若数组长度 < ng（或 nstorage），超出索引的机组使用字段默认值，不报错。

---

## 3. 中间处理规范

### 3.1 母线编号推断

若输入中未提供 `num_buses`，自动推断：

```
num_buses = max(1, max_bus_index_across_all_components + 1)
```

扫描范围：`generators.bus`、`branches.from/to`、`loads.bus`、`wind.bus`、`solar.bus`、`storage.bus`。

松弛母线（参考母线）固定为母线 0（bus 0）。

---

### 3.2 PTDF 矩阵计算

直流潮流分布因子矩阵（Power Transfer Distribution Factor），维度 `[nl × nb]`。

**公式：**

$$\mathbf{F}_\text{PTDF} = \mathbf{B}_f \cdot \mathbf{B}_\text{red}^{-1}$$

其中：
- $\mathbf{B}_f$：支路导纳对角矩阵（$n_l \times n_b$），元素 $b_l = 1/x_l$
- $\mathbf{B}_\text{bus}$：节点导纳矩阵（$n_b \times n_b$）
- $\mathbf{B}_\text{red}$：去除松弛母线（bus 0）后的 $(n_b-1) \times (n_b-1)$ 子矩阵
- 逆矩阵通过 Eigen `FullPivLU` 分解求解

**特殊情况：**
- 若网络孤岛或 $\mathbf{B}_\text{red}$ 不可逆（`lu.isInvertible() == false`），逆矩阵置零，有效退化为铜排模式
- 线路停运（`in_service = false`）从导纳矩阵构建中排除
- 若 `|x_l| ≤ 1e-6`，以 `1e-3` 代替（避免数值奇异）

**断面 PTDF：**

$$\text{SEC\_PTDF}[s][n] = \sum_l w_{sl} \cdot \text{PTDF}[l][n]$$

---

### 3.3 MILP 决策变量布局（VarIndex）

变量以扁平一维数组组织，按以下顺序排列：

| 变量块 | 符号 | 维度 | 说明 |
|--------|------|------|------|
| `SU` | $u_g^t$（启动二进制） | `ng × T_commit` | 第 g 台机组在提交步 h 启动 |
| `SD` | $v_g^t$（停机二进制） | `ng × T_commit` | 第 g 台机组在提交步 h 停机 |
| `IG` | $I_g^t$（在线二进制） | `ng × T_commit` | 第 g 台机组在提交步 h 的状态 |
| `SEG[k]` | $p_g^{k,t}$（分段出力） | `n_seg × ng × T` | 第 k 段报价区间出力 |
| `PG` | $P_g^t$（总出力） | `ng × T` | 机组总有功出力 |
| `RG_spin` | $R_g^{s,t}$（旋转备用） | `ng × T` | |
| `RG_reg_up` | $R_g^{u,t}$（调频上） | `ng × T` | |
| `RG_reg_down` | $R_g^{d,t}$（调频下） | `ng × T` | |
| `PF` | $P_l^t$（线路潮流） | `nl × T` | 从始端到末端（MW）|
| `PW` | $P_w^t$（风电出力） | `nw × T` | |
| `PPV` | $P_s^t$（光伏出力） | `npv × T` | |
| `WIND_CURT` | 风电弃电 | `nw × T` | 仅 M2 > 0 时有效 |
| `SOL_CURT` | 光伏弃电 | `npv × T` | 仅 M2 > 0 时有效 |
| `PSTO_IN` | $P_s^{ch,t}$（充电功率） | `nstorage × T` | |
| `PSTO_OUT` | $P_s^{dis,t}$（放电功率） | `nstorage × T` | |
| `SOC` | $E_s^t$（储能状态） | `nstorage × T` | 单位 MWh |
| `XI_CH` | $\xi_s^{ch,t}$（充电模式二进制） | `nstorage × T` | 仅 `use_binary_indicators = true` |
| `XI_DIS` | $\xi_s^{dis,t}$（放电模式二进制） | `nstorage × T` | 同上 |
| `PDC` | $P_{dc}^t$（直流线路潮流） | `ndc × T` | |
| `LOAD_SHED` | 切负荷松弛 | `n_areas × T` | n_areas = 1 |
| `GEN_CURT` | 系统弃电松弛 | `n_areas × T` | n_areas = 1 |
| `SL_LINE_POS/NEG` | 线路越限松弛 | `nl × T` × 2 | 仅对有限额线路 |
| `SL_SEC_POS/NEG` | 断面越限松弛 | `nsec × T` × 2 | |

**变量总数估算（近似）：**

$$n_x \approx 3 n_g T_c + (n_{seg}+5) n_g T + n_l T + 2(n_w+n_{pv})T + (3+2\mathbf{1}_{xi}) n_{sto} T + n_{dc} T + 2(1+n_l+n_{sec})T$$

其中 $\mathbf{1}_{xi}$ 在有任意储能机组使用二进制模式时为 1，否则为 0。

---

### 3.4 约束族

| 约束类型 | 对应章节 | 说明 |
|----------|---------|------|
| 机组提交转换约束 | §2.6.3.12 | $I_g^h - I_g^{h-1} = u_g^h - v_g^h$；$u_g^h + v_g^h \leq 1$ |
| 最小在/停机时间 | §2.6.3.12 | 滑动窗口约束 |
| 最大启停次数 | §2.6.3.13 | $\sum_h u_g^h \leq \text{maxStartups}$，同理停机 |
| 出力-提交联动 | §2.6.3.1 | $P_{min} I_g^h \leq P_g^t \leq P_{max} I_g^h$ |
| 分段出力约束 | §2.6.3.1 | $\sum_k p_g^{k,t} = P_g^t - P_{min} I_g^h$ |
| 爬坡约束 | §2.6.3.11 | $P_g^t - P_g^{t-1} \leq \text{ramp\_up}$，类似降坡 |
| 潮流平衡 | §2.6.3.0 | 每母线每时间步节点功率平衡（等式约束） |
| 线路潮流约束 | §2.6.3.14 | $P_l^t = \sum_n \text{PTDF}_{ln} \cdot \text{NetInj}_n^t$；含松弛变量 |
| 断面约束 | §2.6.3.15 | 加权支路潮流之和的限额约束 |
| 备用约束 | §2.6.3.2–4 | 系统级旋转备用、调频备用、一次调频约束 |
| 储能 SOC | §2.6.3.16 | $E_s^t = E_s^{t-1} + \eta^{ch} P_s^{ch,t} \Delta t - P_s^{dis,t}/\eta^{dis} \Delta t$；SOC 终值约束 |
| 机组群约束 | §2.6.3.9–10 | 群出力上下限及能量限制 |
| 可再生下界 | §2.6.3.20 | $P_w^t \geq \alpha \cdot \hat{P}_w^t$ |
| 预构型切割面 | — | 仅当 `enable_market_cuts = true`；LP 松弛有效的不等式 |

---

### 3.5 求解流水线

```
输入 SCUCInput
    │
    ├─[Stage 1: SCUC MILP]────────────────────────────────────────
    │  build_formulation() → PTDF计算 + VarIndex + 约束矩阵构建
    │  → run_milp()（solver dispatch: HiGHS→Gurobi→NativeBnC）
    │  → extract_result() → SCUCSolveResult (commitment矩阵等)
    │  如果 converged=false → 跳过后续阶段
    │
    ├─[Stage 2: SCED LP]（仅当 solve_sced=true 且 SCUC converged）
    │  固定 commitment/startup/shutdown 二进制变量
    │  → run_lp()（LP solver: NativeIPM→PDLP→LCQP→Gurobi→HiGHS）
    │  → 提取连续变量结果（dispatch/reserves/flows）
    │
    └─[Stage 3: LMP]（仅当 solve_lmp=true 且 SCED/SCUC converged）
       δ-邻域再调度 LP（§2.6.5.5）
       → 提取对偶变量 → 计算节点 LMP（能量分量+阻塞分量）
       → SCUCLMPResult (nodal_lmp[nb][T])
```

**LMP 来源说明：**
- Gurobi / NativeBranchAndCut：可直接从 SCED LP 提取约束对偶
- HiGHS：通过 MPS 文件 I/O，**不返回约束对偶**，LMP 阶段自动回退至 Gurobi 或 NativeBranchAndCut

---

## 4. 输出格式规范

### 4.1 C++ 结构体

**`SCUCOutput`** 是顶层结果容器，包含：
- `scuc`: `SCUCSolveResult` — SCUC（MILP）结果
- `sced`: `SCUCSolveResult` — SCED（LP）结果（若未运行，`converged=false`）
- `lmp`: `SCUCLMPResult` — 节点电价结果（若未运行，`converged=false`）

**`SCUCSolveResult` 字段详解：**

| 字段 | 类型 | 维度 | 说明 |
|------|------|------|------|
| `converged` | `bool` | — | 是否找到可行解 |
| `objective` | `double` | — | 目标函数值（$），与 `total_cost` 相等 |
| `solver_name` | `string` | — | 实际使用的求解器名称 |
| `solve_time_sec` | `double` | — | 求解耗时（秒） |
| `mip_gap` | `double` | — | 最终相对 MIP 间隙（0 = 精确最优） |
| `n_cuts_added` | `int` | — | 预构型削减切面数量 |
| `commitment` | `Matrix2D` | `[ng][T_commit]` | 机组提交状态（0 或 1） |
| `startup` | `Matrix2D` | `[ng][T_commit]` | 启动指示（0 或 1） |
| `shutdown` | `Matrix2D` | `[ng][T_commit]` | 停机指示（0 或 1） |
| `dispatch` | `Matrix2D` | `[ng][T]` | 总有功出力（MW） |
| `spinning_reserve` | `Matrix2D` | `[ng][T]` | 旋转备用（MW） |
| `regulation_up` | `Matrix2D` | `[ng][T]` | 调频上备用（MW） |
| `regulation_down` | `Matrix2D` | `[ng][T]` | 调频下备用（MW） |
| `segment_dispatch` | `vector<Matrix2D>` | `[n_seg][ng][T]` | 各分段出力（MW） |
| `wind_generation` | `Matrix2D` | `[nw][T]` | 风电实际出力（MW） |
| `solar_generation` | `Matrix2D` | `[npv][T]` | 光伏实际出力（MW） |
| `wind_curtailment` | `Matrix2D` | `[nw][T]` | 风电弃电量（MW） |
| `solar_curtailment` | `Matrix2D` | `[npv][T]` | 光伏弃电量（MW） |
| `storage_charging` | `Matrix2D` | `[nstorage][T]` | 储能充电功率（MW） |
| `storage_discharging` | `Matrix2D` | `[nstorage][T]` | 储能放电功率（MW） |
| `storage_soc` | `Matrix2D` | `[nstorage][T]` | 储能 SOC（MWh） |
| `line_flows` | `Matrix2D` | `[nl][T]` | 支路潮流（MW，正方向为 from→to） |
| `section_flows` | `Matrix2D` | `[nsec][T]` | 断面潮流（MW） |
| `load_shedding` | `vector<double>` | `[T]` | 系统切负荷量（MW） |
| `gen_curtailment` | `vector<double>` | `[T]` | 系统弃电量（MW） |
| `energy_cost` | `double` | — | 能量成本（$） |
| `startup_cost` | `double` | — | 启动成本（$） |
| `no_load_cost` | `double` | — | 空载成本（$） |
| `reserve_cost` | `double` | — | 备用成本（$） |
| `penalty_cost` | `double` | — | 惩罚成本（切负荷+弃电，$） |
| `total_cost` | `double` | — | 总成本 = 各项之和（$） |

**`SCUCLMPResult` 字段详解：**

| 字段 | 类型 | 维度 | 说明 |
|------|------|------|------|
| `converged` | `bool` | — | LMP 计算是否成功 |
| `solve_time_sec` | `double` | — | LMP LP 求解耗时（秒） |
| `nodal_lmp` | `Matrix2D` | `[nb][T]` | 节点 LMP（$/MWh） |
| `energy_lmp` | `Matrix2D` | `[nb][T]` | 系统边际价格分量（$/MWh） |
| `congestion_lmp` | `Matrix2D` | `[nb][T]` | 阻塞租金分量（$/MWh） |
| `avg_lmp` | `double` | — | 全系统时空平均 LMP（$/MWh） |
| `max_lmp` | `double` | — | 全系统最大 LMP（$/MWh） |
| `min_lmp` | `double` | — | 全系统最小 LMP（$/MWh） |

> **LMP 符号约定：** 正值表示消费方支付，负值理论上可能出现（网络阻塞时接受端节点）。

---

### 4.2 JSON 输出（`scuc_output_to_json`）

```json
{
  "meta": {
    "solver":         "Auto",
    "num_periods":    24,
    "num_buses":      39,
    "num_generators": 10,
    "num_branches":   51
  },
  "scuc": {
    "converged":      true,
    "objective":      891466.2,
    "solver_name":    "HiGHS",
    "solve_time_sec": 3.14,
    "mip_gap":        0.0009,
    "n_cuts_added":   12,
    "commitment":     [[1,1,...],[0,1,...],...],
    "startup":        [[0,0,...],           ...],
    "shutdown":       [[0,0,...],           ...],
    "dispatch":       [[500.0,...],         ...],
    "spinning_reserve": [[50.0,...],        ...],
    "regulation_up":    [[25.0,...],        ...],
    "regulation_down":  [[20.0,...],        ...],
    "wind_generation":  [[120.0,...],       ...],
    "solar_generation": [[0.0,...],         ...],
    "storage_charging":    [[0.0,...],      ...],
    "storage_discharging": [[100.0,...],    ...],
    "storage_soc":         [[400.0,...],    ...],
    "line_flows":          [[45.2,...],     ...],
    "load_shedding":  [0.0, 0.0, ...],
    "gen_curtailment":[0.0, 0.0, ...],
    "cost": {
      "energy":   820000.0,
      "startup":  15000.0,
      "no_load":  48000.0,
      "reserve":  8466.2,
      "penalty":  0.0,
      "total":    891466.2
    }
  },
  "sced": { "...": "...（同 scuc 结构）..." },
  "lmp": {
    "converged":      true,
    "solve_time_sec": 0.21,
    "avg_lmp":        32.5,
    "max_lmp":        58.3,
    "min_lmp":        18.1,
    "nodal":      [[32.5,...], ...],
    "energy":     [[30.0,...], ...],
    "congestion": [[2.5, ...], ...]
  }
}
```

**JSON 键名与 C++ 字段对应关系（差异说明）：**

| JSON 键 | C++ 字段 | 说明 |
|---------|---------|------|
| `scuc` | `output.scuc` | SCUC MILP 结果 |
| `sced` | `output.sced` | SCED LP 结果（仅当 `solve_sced=true`） |
| `lmp` | `output.lmp` | LMP 结果（仅当 `solve_lmp=true`） |
| `lmp.nodal` | `lmp.nodal_lmp` | JSON 中简化键名 |
| `lmp.energy` | `lmp.energy_lmp` | JSON 中简化键名 |
| `lmp.congestion` | `lmp.congestion_lmp` | JSON 中简化键名 |
| `scuc.cost.total` | `scuc.total_cost` | JSON 嵌套在 `cost` 对象下 |

---

### 4.3 数组维度保证

当 `converged = true` 时，以下维度严格保证：

| 字段 | 维度 | 条件 |
|------|------|------|
| `commitment[ng][T_commit]` | `T_commit = ceil(T / intervals_per_hour)` | 始终 |
| `startup[ng][T_commit]` | 同上 | 始终 |
| `shutdown[ng][T_commit]` | 同上 | 始终 |
| `dispatch[ng][T]` | T = `config.num_periods` | 始终 |
| `spinning_reserve[ng][T]` | — | 始终 |
| `regulation_up[ng][T]` | — | 始终 |
| `regulation_down[ng][T]` | — | 始终 |
| `wind_generation[nw][T]` | — | 始终（空则 `[][]`） |
| `solar_generation[npv][T]` | — | 始终 |
| `storage_charging[nstorage][T]` | — | 始终 |
| `storage_discharging[nstorage][T]` | — | 始终 |
| `storage_soc[nstorage][T]` | — | 始终 |
| `line_flows[nl][T]` | — | 始终（空则 `[][]`） |
| `section_flows[nsec][T]` | — | 始终 |
| `load_shedding[T]` | — | 始终 |
| `gen_curtailment[T]` | — | 始终 |
| `nodal_lmp[nb][T]` | — | 仅当 `lmp.converged=true` |
| `energy_lmp[nb][T]` | — | 同上 |
| `congestion_lmp[nb][T]` | — | 同上 |

当 `converged = false` 时，所有矩阵均为空（`[]`），所有标量成本字段为 `0.0`。

---

### 4.4 收敛语义与异常情况

| 情况 | `scuc.converged` | `sced.converged` | `lmp.converged` | 说明 |
|------|:---:|:---:|:---:|------|
| 正常求解 | `true` | `true` | `true` | 三阶段均成功 |
| SCUC 超时（有可行解） | `true` | `true` | `true` | `mip_gap > 0` 表示未到最优 |
| SCUC 超时（无可行解） | `false` | `false` | `false` | 问题不可行或时限不足 |
| SCUC 成功，SCED 不可行 | `true` | `false` | `false` | 罕见（固定提交后 LP 不可行） |
| `solve_sced=false` | `true` | `false` | 取决于 `solve_lmp` | SCED 跳过，`sced` 结构默认值 |
| `solve_lmp=false` | — | — | `false` | LMP 跳过 |
| LMP 求解器不支持对偶 | `true` | `true` | `false` | HiGHS 无对偶时自动回退；若无其他求解器则失败 |

**异常处理：**

| 异常类型 | 触发条件 | 建议处理 |
|---------|---------|---------|
| `nlohmann::json::parse_error` | `scuc_from_json` 收到非法 JSON 字符串 | 捕获并返回错误码 |
| `nlohmann::json::type_error` | JSON 字段类型与期望不符 | 同上 |
| 无异常（仅 `converged=false`） | 求解失败、不可行、数值问题 | 检查 `converged` 字段 |

---

## 5. JSON 完整示例

以下为一个最小可运行的单母线 2 机 1 负荷 3 周期示例：

```json
{
  "config": {
    "num_periods": 3,
    "period_length_hr": 1.0,
    "n_segments": 2,
    "mip_gap": 0.001,
    "time_limit_sec": 60.0,
    "solve_sced": true,
    "solve_lmp": false
  },
  "generators": [
    {
      "name": "Coal-1",
      "bus": 0,
      "pmin": 100.0,
      "pmax": 400.0,
      "ramp_up_mw_min": 5.0,
      "ramp_dn_mw_min": 5.0,
      "min_up_time_hr": 2.0,
      "min_dn_time_hr": 2.0,
      "startup_cost": 3000.0,
      "no_load_cost": 150.0,
      "bid_segments": [
        { "price": 20.0, "quantity": 150.0 },
        { "price": 28.0, "quantity": 150.0 }
      ]
    },
    {
      "name": "Gas-1",
      "bus": 0,
      "pmin": 50.0,
      "pmax": 200.0,
      "ramp_up_mw_min": 10.0,
      "ramp_dn_mw_min": 10.0,
      "min_up_time_hr": 1.0,
      "min_dn_time_hr": 1.0,
      "startup_cost": 800.0,
      "bid_segments": [
        { "price": 35.0, "quantity": 100.0 },
        { "price": 45.0, "quantity": 50.0  }
      ]
    }
  ],
  "loads": [
    { "bus": 0, "p_mw": 300.0 }
  ],
  "profiles": {
    "load": [ [1.0, 1.2, 0.9] ]
  },
  "initial_status": {
    "commitment":    [1.0, 0.0],
    "dispatch":      [250.0, 0.0],
    "time_in_state": [6.0, -3.0]
  }
}
```

该示例：
- 无输电网络（铜排模式，不计算 PTDF）
- 无 `num_buses`（自动推断为 1）
- 无风光储（不包含可再生能源）
- 机组 Coal-1 初始在线 6 小时，Gas-1 停机 3 小时
- 3 个时间步负荷为 300、360、270 MW
