# SCUC 模块技术文档

> 实现依据：《南方区域电力市场现货交易规则（2025版V1.0）》§2.6 日前市场出清

---

## 目录

1. [模块概述](#1-模块概述)
2. [输入数据结构](#2-输入数据结构)
3. [输出数据结构](#3-输出数据结构)
4. [数学模型](#4-数学模型)
5. [求解流程](#5-求解流程)
6. [JSON 接口格式](#6-json-接口格式)
7. [案例生成器](#7-案例生成器)
8. [测试结果](#8-测试结果)

---

## 1. 模块概述

SCUC 模块实现了安全约束机组组合（Security-Constrained Unit Commitment）+ 经济调度（SCED）+ 节点边际电价（LMP）三阶段求解流程。

**主要特性**

| 特性 | 说明 |
|------|------|
| 约束来源 | 南方区域现货规则 §2.6 全部约束条款 |
| 网络模型 | B-θ直流潮流（PTDF矩阵），支持监测断面 |
| 机组模型 | 含爬坡、最小启停时、状态相关启动费用、折线报价 |
| 储能模型 | 分离充放电效率、SOC 约束、二进制充放电指示变量 |
| 新能源 | 风电/光伏预测出力，可选消纳下限与弃电惩罚 |
| 备用 | 正备用（旋转/调频上/调频下）、负备用、一次调频（PFR）需求 |
| 求解器 | 自动选择 Gurobi / SCIP / HiGHS / NativeBranchAndCut |
| 接口 | JSON 输入/输出，CLI 工具 `scuc_solve` |

---

## 2. 输入数据结构

所有输入封装在 `SCUCInput` 结构体中，对应 JSON 顶层对象。

### 2.1 求解配置（`config`）

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `solver` | string | `"Auto"` | 求解器：Auto / Gurobi / HiGHS / SCIP |
| `allow_fallback` | bool | true | 首选求解器失败时自动回退 |
| `num_periods` | int | 24 | 调度时段数 $T$ |
| `period_length_hr` | float | 1.0 | 每时段小时数 $\Delta t$ |
| `n_segments` | int | 3 | 每台机组折线报价段数 |
| `mip_gap` | float | 0.001 | MILP 相对最优间隙 |
| `time_limit_sec` | float | 300 | 求解时限（秒） |
| `spinning_reserve_req` | float | 0.10 | 旋转备用需求（系统负荷比例）|
| `regulation_up_req` | float | 0.05 | 调频上备用需求比例 |
| `regulation_down_req` | float | 0.05 | 调频下备用需求比例 |
| `neg_reserve_req` | float | 0.0 | 负备用需求（§2.6.3.3），0=不约束 |
| `pfr_reserve_req_mw` | float | 0.0 | 一次调频备用需求 MW（§2.6.3.4），0=不约束 |
| `voll` | float | 10000 | 缺电惩罚价格 \$/MWh |
| `vocc` | float | 1000 | 弃电惩罚价格 \$/MWh |
| `M1_line_slack_penalty` | float | 1e5 | 线路/断面越限松弛惩罚 |
| `M2_renewable_curtail_penalty` | float | 0.0 | 新能源弃电惩罚 \$/MW·时段（>0 时启用弃电变量）|
| `renewable_min_output_coeff` | float | 0.0 | 新能源最小出力系数 $\alpha$（§2.6.3.20）|
| `wheeling_fee_per_mwh` | float | 0.0 | 过网费 \$/MWh（§2.6.3.7 目标函数 $P_{gwf}$ 项）|
| `enable_market_cuts` | bool | true | 是否添加预建 LP 有效不等式（见§4.6）|
| `solve_sced` | bool | true | SCUC 后是否求解 SCED（LP，固定组合）|
| `solve_lmp` | bool | true | 是否计算节点 LMP |
| `lmp_delta` | float | 0.10 | LMP 再调度邻域系数（§2.6.5.5）|

### 2.2 发电机（`generators[]`）

| 字段 | 说明 |
|------|------|
| `name` | 机组名称 |
| `bus` | 接入母线（0-based）|
| `pmin` / `pmax` | 最小/最大出力 MW |
| `ramp_up_mw_min` / `ramp_dn_mw_min` | 上/下爬坡率 MW/min |
| `min_up_time_hr` / `min_dn_time_hr` | 最小开机/停机时间（小时）|
| `must_run` | 是否强制在线 |
| `max_startups` / `max_shutdowns` | 最大启动/停机次数（0=不限）|
| `startup_cost` | 热启动费 \$（同时作为默认启动费）|
| `startup_cost_warm` / `startup_cost_cold` | 温启/冷启费 \$（§2.6.3.13）|
| `hot_start_threshold_hr` / `warm_start_threshold_hr` | 热/温启判定停机时长阈值（小时）|
| `ud_periods` / `dd_periods` | 启动/停机过渡时段数（§2.6.3.8）|
| `no_load_cost` | 空载费 \$/h（最小出力运行成本）|
| `spinning_reserve_price` / `regulation_up_price` / `regulation_down_price` | 各类备用报价 \$/MWh |
| `pfr_alpha` | 一次调频系数，最大 PFR = $\alpha \cdot P_{\max}$（§2.6.3.4）|
| `group_id` | 所属发电机组 ID（-1=不属于任何组）|
| `bid_segments[]` | 折线报价段：`price`（\$/MWh）+ `quantity`（MW，超出 $P_{\min}$ 的容量）|

### 2.3 支路（`branches[]`）

| 字段 | 说明 |
|------|------|
| `from` / `to` | 首/末母线（0-based）|
| `reactance` | 电抗（标幺值，用于 PTDF 计算）|
| `rating_mw` | 热稳定极限 MW |
| `in_service` | 是否在运 |

### 2.4 负荷（`loads[]`）

| 字段 | 说明 |
|------|------|
| `bus` | 接入母线 |
| `p_mw` | 基准有功需求 MW；与 `profiles.load[d][t]` 逐时段相乘得实际负荷 |

### 2.5 风电/光伏（`wind[]` / `solar[]`）

| 字段 | 说明 |
|------|------|
| `bus` | 接入母线 |
| `pmax` | 装机容量 MW |

`profiles.wind[w][t]` 和 `profiles.solar[s][t]` 给出 **绝对值** 预测出力（MW）。

### 2.6 储能（`storage[]`）

| 字段 | 说明 |
|------|------|
| `pmax_charge` / `pmax_discharge` | 最大充/放电功率 MW |
| `pmin_charge` / `pmin_discharge` | 最小充/放电功率 MW（0=不约束）|
| `energy_capacity_mwh` | 储能容量 MWh |
| `efficiency` | 综合效率（传统字段，$\eta^{ch} = \eta^{dis} = \sqrt{\text{efficiency}}$）|
| `eta_charge` / `eta_discharge` | 独立充/放电效率（<0 时回退到 `efficiency` 开方）|
| `soc_init` | 初始 SOC（容量分数，>1 时解释为 MWh）|
| `soc_min` | 最小 SOC 分数（<0 → 取 0.10）|
| `soc_final` | 要求末态 SOC（<0 → 取 `soc_init`）|
| `charge_bid_price` / `discharge_bid_price` | 充/放电报价 \$/MWh |
| `cycle_limit` | 等效全充满次数限制（0=不限，§2.6.3.16(5)）|
| `use_binary_indicators` | 是否引入充/放电二进制互斥变量 $\xi^+/\xi^-$ |

### 2.7 发电机组（`generator_groups[]`）

对应 §2.6.3.9–2.6.3.10，用于约束一组机组的合计出力和总电量。

| 字段 | 说明 |
|------|------|
| `id` | 组 ID |
| `name` | 组名称 |
| `gen_indices[]` | 成员发电机索引（0-based）|
| `pmin_t[]` / `pmax_t[]` | 逐时段合计出力下/上限 MW（长度1时对所有时段生效）|
| `emin` / `emax` | 总电量下/上限 MWh（0=不限）|

### 2.8 监测断面（`sections[]`）

对应 §2.6.3.15，用于约束多条支路加权潮流之和。

| 字段 | 说明 |
|------|------|
| `name` | 断面名称 |
| `line_weights[]` | 支路索引与权重的列表 `[branch_idx, weight]` |
| `rating_fwd_mw` | 正方向潮流限额 MW |
| `rating_rev_mw` | 反方向潮流限额 MW |

### 2.9 DC 线路（`dc_lines[]`）

| 字段 | 说明 |
|------|------|
| `from` / `to` | 接入母线 |
| `pmin` / `pmax` | 有功传输范围 MW |
| `ramp_up` / `ramp_dn` | 爬坡率 MW/时段 |

### 2.10 时序曲线（`profiles`）

```json
"profiles": {
  "load":  [[pu_d0_t0, pu_d0_t1, ...], [pu_d1_t0, ...]],
  "wind":  [[mw_w0_t0, ...], ...],
  "solar": [[mw_s0_t0, ...], ...]
}
```

### 2.11 初始状态（`initial_status`）

```json
"initial_status": {
  "commitment":   [1.0, 0.0, ...],   // 机组上时段开机状态 (0/1)
  "dispatch":     [120.0, 0.0, ...], // 机组上时段出力 MW
  "storage_soc":  [0.5, ...],        // 储能初始 SOC 分数
  "time_in_state":[4.0, -2.0, ...]   // 已连续处于当前状态时间(h)，正=在线，负=离线
}
```

---

## 3. 输出数据结构

`scuc_solve()` 返回 `SCUCOutput`，由三个子结果组成：

### 3.1 `scuc`（MILP 组合结果）

| 字段 | 类型 | 说明 |
|------|------|------|
| `converged` | bool | 是否收敛 |
| `objective` | float | 目标函数值（\$）|
| `solver_name` | string | 实际使用的求解器 |
| `mip_gap` | float | 相对 MIP 间隙 |
| `n_cuts_added` | int | 添加的预建有效不等式数量 |
| `commitment[g][t]` | float | 机组开机状态 0/1 |
| `startup[g][t]` / `shutdown[g][t]` | float | 启动/停机指示变量 |
| `dispatch[g][t]` | float | 发电出力 MW |
| `spinning_reserve[g][t]` | float | 旋转备用 MW |
| `regulation_up[g][t]` / `regulation_down[g][t]` | float | 调频备用 MW |
| `wind_generation[w][t]` / `solar_generation[s][t]` | float | 新能源出力 MW |
| `wind_curtailment[w][t]` / `solar_curtailment[s][t]` | float | 弃风/弃光 MW |
| `storage_charging[s][t]` / `storage_discharging[s][t]` | float | 充/放电功率 MW |
| `storage_soc[s][t]` | float | 储能 SOC（MWh）|
| `line_flows[l][t]` | float | 支路潮流 MW |
| `section_flows[sec][t]` | float | 断面潮流 MW |
| `load_shedding[t]` | float | 切负荷量 MW |
| `gen_curtailment[t]` | float | 弃电量 MW |
| `energy_cost` / `startup_cost` / `no_load_cost` / `reserve_cost` / `penalty_cost` / `total_cost` | float | 费用分解 \$ |

### 3.2 `sced`（LP 经济调度结果）

字段与 `scuc` 相同，区别：机组开机状态固定为 MILP 解，连续变量重新优化。

### 3.3 `lmp`（节点边际电价结果）

| 字段 | 类型 | 说明 |
|------|------|------|
| `converged` | bool | 是否收敛 |
| `nodal_lmp[b][t]` | float | 节点电价 \$/MWh |
| `energy_lmp[b][t]` | float | 能量分量 \$/MWh |
| `congestion_lmp[b][t]` | float | 阻塞分量 \$/MWh |
| `avg_lmp` / `max_lmp` / `min_lmp` | float | 系统平均/最大/最小电价 \$/MWh |

---

## 4. 数学模型

### 4.1 决策变量

| 符号 | 类型 | 说明 |
|------|------|------|
| $I_{g,t} \in \{0,1\}$ | 二值 | 机组 $g$ 在时段 $t$ 的开机状态 |
| $Y_{g,t}, Z_{g,t} \in \{0,1\}$ | 二值 | 启动 / 停机指示 |
| $P_{g,t} \geq 0$ | 连续 | 机组出力 MW |
| $p^k_{g,t} \geq 0$ | 连续 | 第 $k$ 段折线变量（$\sum_k p^k_{g,t} = P_{g,t} - P^{\min}_g I_{g,t}$）|
| $R^{spin}_{g,t}, R^{up}_{g,t}, R^{dn}_{g,t} \geq 0$ | 连续 | 旋转备用 / 调频上 / 调频下 MW |
| $P^w_{w,t}, P^{pv}_{s,t}$ | 连续 | 风电/光伏出力 MW |
| $C^w_{w,t}, C^{pv}_{s,t} \geq 0$ | 连续 | 弃风/弃光 MW（M2>0 时激活）|
| $P^{ch}_{s,t}, P^{dis}_{s,t} \geq 0$ | 连续 | 储能充/放电功率 MW |
| $\xi^+_{s,t}, \xi^-_{s,t} \in \{0,1\}$ | 二值（可选）| 充/放电模式指示 |
| $E_{s,t} \geq 0$ | 连续 | 储能 SOC（MWh）|
| $P^{dc}_{l,t}$ | 连续 | DC 线路传输功率 MW |
| $S^{ls}_t, S^{gc}_t \geq 0$ | 连续 | 切负荷 / 系统弃电 MW |
| $sl^+_{l,t}, sl^-_{l,t} \geq 0$ | 连续 | 支路越限松弛 MW |
| $sl^{sec+}_{sec,t}, sl^{sec-}_{sec,t} \geq 0$ | 连续 | 断面越限松弛 MW |

### 4.2 目标函数（§2.6.3 运营费用最小化）

$$\min \sum_{t=1}^{T} \left[ \sum_{g} \left( \sum_k \lambda^k_g p^k_{g,t} \Delta t + P_{gwf} P_{g,t} \Delta t + C^{nl}_g I_{g,t} \right) + \sum_g \left( C^{su}_g Y_{g,t} \right) \right]$$

$$+ \sum_t \left[ c^{voll} S^{ls}_t \Delta t + c^{vocc} S^{gc}_t \Delta t \right] + M_1 \sum_{l,t}(sl^+_{l,t} + sl^-_{l,t}) + M_1 \sum_{sec,t}(sl^{sec+}_{sec,t} + sl^{sec-}_{sec,t})$$

$$+ M_2 \sum_{w,t} C^w_{w,t} \Delta t + M_2 \sum_{s,t} C^{pv}_{s,t} \Delta t + \sum_s \sum_t \left( \lambda^{dis}_s P^{dis}_{s,t} + \lambda^{ch}_s P^{ch}_{s,t} \right) \Delta t$$

其中：
- $\lambda^k_g$：机组 $g$ 第 $k$ 段边际报价（\$/MWh）
- $P_{gwf}$：过网费（\$/MWh），配置项 `wheeling_fee_per_mwh`
- $C^{nl}_g$：空载费（\$/h）
- $C^{su}_g$：启动费（含状态相关热/温/冷分类）
- $M_1$：线路/断面越限大惩罚系数
- $M_2$：新能源弃电惩罚系数

### 4.3 系统功率平衡（§2.6.3.1）

$$\sum_g P_{g,t} + \sum_w P^w_{w,t} + \sum_s P^{pv}_{s,t} + \sum_s (P^{dis}_{s,t} - P^{ch}_{s,t}) + \sum_{dc} P^{dc}_{dc,t} + S^{ls}_t - S^{gc}_t = D_t \quad \forall t$$

### 4.4 出力约束（§2.6.3.6, §2.6.3.8）

折线分段表达：

$$P_{g,t} = P^{\min}_g I_{g,t} + \sum_k p^k_{g,t}, \quad 0 \leq p^k_{g,t} \leq q^k_g \quad \forall g,t,k$$

出力界限：

$$P^{\min}_g I_{g,t} \leq P_{g,t} \leq P^{\max}_g I_{g,t} \quad \forall g,t$$

### 4.5 机组组合约束

**承诺切换一致性（§2.6.3.12）：**

$$I_{g,t} - I_{g,t-1} = Y_{g,t} - Z_{g,t}, \quad Y_{g,t} + Z_{g,t} \leq 1 \quad \forall g,t$$

**最小开停机时间（§2.6.3.12）：**

$$\sum_{\tau=t}^{t+T^{up}_g - 1} I_{g,\tau} \geq T^{up}_g \cdot Y_{g,t} \quad \forall g,t \quad \text{（最小开机时间）}$$

$$\sum_{\tau=t}^{t+T^{dn}_g - 1} (1 - I_{g,\tau}) \geq T^{dn}_g \cdot Z_{g,t} \quad \forall g,t \quad \text{（最小停机时间）}$$

**最大启停次数（§2.6.3.13）：**

$$\sum_t Y_{g,t} \leq N^{su}_g, \quad \sum_t Z_{g,t} \leq N^{sd}_g \quad \forall g \quad (N=0 \text{ 时不约束})$$

**强制在线（§2.6.3.5）：**

$$I_{g,t} = 1 \quad \forall t, \quad \text{若 must\_run = true}$$

### 4.6 爬坡约束（§2.6.3.11）

$$P_{g,t} - P_{g,t-1} \leq R^{up}_g \Delta t + P^{\max}_g Y_{g,t} + P^{\min}_g Z_{g,t-1}$$

$$P_{g,t-1} - P_{g,t} \leq R^{dn}_g \Delta t + P^{\max}_g Z_{g,t} + P^{\min}_g Y_{g,t-1}$$

其中 $R^{up}_g = r^{up}_{mw/min} \times 60 \times \Delta t$（以 MW/时段计）。

### 4.7 发电机组出力 / 电量约束（§2.6.3.9, §2.6.3.10）

出力约束（每时段）：

$$GP^{\min}_{grp,t} \leq \sum_{g \in grp} P_{g,t} \leq GP^{\max}_{grp,t} \quad \forall grp, t$$

总电量约束（全周期）：

$$E^{\min}_{grp} \leq \sum_t \sum_{g \in grp} P_{g,t} \Delta t \leq E^{\max}_{grp} \quad \forall grp$$

### 4.8 网络约束（§2.6.3.14, §2.6.3.15）

基于 PTDF 的支路潮流（B-θ直流潮流）：

$$f_{l,t} = \sum_b PTDF_{l,b} \cdot \text{NetInj}_{b,t}$$

支路约束（含松弛变量）：

$$-F^{\max}_l - sl^-_{l,t} \leq f_{l,t} \leq F^{\max}_l + sl^+_{l,t} \quad \forall l,t$$

断面潮流（§2.6.3.15）：

$$f^{sec}_{sec,t} = \sum_l w_{sec,l} \cdot f_{l,t}$$

$$-F^{rev}_{sec} - sl^{sec-}_{sec,t} \leq f^{sec}_{sec,t} \leq F^{fwd}_{sec} + sl^{sec+}_{sec,t} \quad \forall sec,t$$

### 4.9 备用约束（§2.6.3.2, §2.6.3.3, §2.6.3.4）

**正备用（上调）：**

$$\sum_g R^{spin}_{g,t} \geq r^{spin} D_t, \quad \sum_g R^{up}_{g,t} \geq r^{up} D_t, \quad \sum_g R^{dn}_{g,t} \geq r^{dn} D_t$$

备用耦合约束：

$$P_{g,t} + R^{spin}_{g,t} + R^{up}_{g,t} \leq P^{\max}_g I_{g,t}, \quad R^{dn}_{g,t} \leq P_{g,t} - P^{\min}_g I_{g,t}$$

**负备用（§2.6.3.3）：**

$$\sum_g \left(P_{g,t} - P^{\min}_g I_{g,t}\right) \leq D_t - r^{neg} D_t$$

**一次调频备用（§2.6.3.4）：**

$$\sum_g \alpha^{pf}_g P^{\max}_g I_{g,t} \geq F^{pfr}_{req} \quad \forall t$$

### 4.10 新能源约束（§2.6.3.20）

$$\alpha \cdot \hat{P}^w_{w,t} \leq P^w_{w,t} \leq \hat{P}^w_{w,t} \quad \forall w,t$$

其中 $\hat{P}^w_{w,t}$ 为预测出力，$\alpha$ 为配置项 `renewable_min_output_coeff`。

启用弃电变量时（$M_2 > 0$）：

$$P^w_{w,t} = \hat{P}^w_{w,t} - C^w_{w,t}, \quad C^w_{w,t} \geq 0$$

### 4.11 储能约束（§2.6.3.16）

**SOC 动态：**

$$E_{s,t} = E_{s,t-1} + \eta^{ch}_s P^{ch}_{s,t} \Delta t - \frac{P^{dis}_{s,t}}{\eta^{dis}_s} \Delta t \quad \forall s,t$$

**SOC 界限：**

$$E^{\min}_s \leq E_{s,t} \leq E^{cap}_s, \quad E_{s,0} = E^{init}_s$$

**末态 SOC：**

$$E_{s,T} \geq E^{final}_s$$

**充放电功率界限：**

$$0 \leq P^{ch}_{s,t} \leq P^{ch,\max}_s, \quad 0 \leq P^{dis}_{s,t} \leq P^{dis,\max}_s$$

**二进制互斥（可选，`use_binary_indicators = true`）：**

$$\xi^+_{s,t} + \xi^-_{s,t} \leq 1, \quad P^{ch}_{s,t} \leq P^{ch,\max}_s \xi^+_{s,t}, \quad P^{dis}_{s,t} \leq P^{dis,\max}_s \xi^-_{s,t}$$

**等效循环次数限制（§2.6.3.16(5)）：**

$$\sum_t \left( \frac{P^{dis}_{s,t}}{\eta^{dis}_s} + \eta^{ch}_s P^{ch}_{s,t} \right) \Delta t \leq 2 N^{cycle}_s E^{cap}_s$$

### 4.12 DC 线路（§2.6.3.17）

$$P^{\min}_{dc} \leq P^{dc}_{dc,t} \leq P^{\max}_{dc}$$

$$|P^{dc}_{dc,t} - P^{dc}_{dc,t-1}| \leq R^{dc}_{up/dn}$$

### 4.13 预建有效不等式（LP Cuts，`enable_market_cuts = true`）

| 类型 | 说明 |
|------|------|
| Family 6（对称性削减）| $I_{g,t} \geq I_{g,t-1} - Z_{g,t}$，减少对称解 |
| Family A（分段耦合）| $p^k_{g,t} \leq q^k_g \cdot I_{g,t}$，加强 LP 松弛 |
| Family G（拓展 SU clique）| $Y_{g,t} + I_{g,t-1} \leq 1$（适用于单时段最小开机时间机组）|
| Family 11（循环 SOC）| $E_{s,T} = E_{s,0}$ 等价不等式 |

---

## 5. 求解流程

```
SCUCInput (JSON)
       │
       ▼
  scuc_from_json()
       │
       ▼
  build_formulation()          ← 构建 MILP
       │  - 变量分配 (VarIndex)
       │  - 目标系数
       │  - 等式/不等式约束矩阵
       │  - 预建 LP 割平面
       ▼
  SolverEngine::solve_milp()   ← SCUC (MILP)
       │
       ▼
  extract_result()             ← 提取 MILP 解 → scuc
       │
       ├─ solve_sced == true:
       │       │
       │       ▼
       │  fix_commitment() + solve_lp()  ← SCED (LP，固定 I_{g,t})
       │       ▼
       │  extract_result() → sced
       │
       └─ solve_lmp == true:
               │
               ▼
          solve_lmp_problem()  ← 对偶提取 + 节点 LMP 分解
               ▼
          SCUCLMPResult → lmp
       │
       ▼
  scuc_output_to_json()        ← 序列化输出 (JSON)
```

**MILP 求解器优先级**（`solver = "Auto"`）：HiGHS → Gurobi → NativeBranchAndCut

> 注：SCIP 在本框架中仅支持 MINLP，不参与 MILP 调度。HiGHS 不返回对偶变量，
> 因此当 `solve_lmp = true` 时 LMP 阶段会自动回退到 Gurobi 或 NativeBranchAndCut。

---

## 6. JSON 接口格式

### 6.1 输入 JSON 最小示例（2 母线，2 机组，3 时段）

```json
{
  "config": {
    "solver": "Auto",
    "num_periods": 3,
    "period_length_hr": 1.0,
    "n_segments": 2,
    "mip_gap": 0.005,
    "spinning_reserve_req": 0.10,
    "voll": 5000.0,
    "solve_sced": true,
    "solve_lmp": true
  },
  "num_buses": 2,
  "generators": [
    {
      "name": "G1", "bus": 0,
      "pmin": 50.0, "pmax": 200.0,
      "ramp_up_mw_min": 5.0, "ramp_dn_mw_min": 5.0,
      "min_up_time_hr": 2.0, "min_dn_time_hr": 2.0,
      "startup_cost": 300.0, "no_load_cost": 80.0,
      "bid_segments": [
        {"price": 25.0, "quantity": 100.0},
        {"price": 40.0, "quantity": 50.0}
      ]
    }
  ],
  "branches": [
    {"from": 0, "to": 1, "reactance": 0.1, "rating_mw": 150.0, "in_service": true}
  ],
  "loads": [{"bus": 0, "p_mw": 150.0}],
  "wind": [], "solar": [], "storage": [], "dc_lines": [],
  "profiles": {
    "load": [[0.9, 1.0, 0.85]],
    "wind": [], "solar": []
  },
  "initial_status": {
    "commitment": [1.0], "dispatch": [120.0], "storage_soc": []
  }
}
```

### 6.2 输出 JSON 结构

```json
{
  "meta": {
    "num_buses": 2, "num_generators": 2, "num_branches": 1,
    "num_periods": 3, "period_length_hr": 1.0
  },
  "scuc": {
    "converged": true,
    "objective": 12450.3,
    "solver_name": "Gurobi",
    "mip_gap": 0.00031,
    "commitment":  [[1,1,1], [0,1,1]],
    "dispatch":    [[...], [...]],
    "load_shedding": [0.0, 0.0, 0.0],
    "energy_cost": 9800.0,
    "startup_cost": 450.0,
    "total_cost": 12450.3,
    ...
  },
  "sced": { ... },
  "lmp": {
    "converged": true,
    "nodal_lmp": [[31.2, 38.5, 29.1], [30.8, 37.9, 28.4]],
    "avg_lmp": 32.7, "max_lmp": 38.5, "min_lmp": 28.4
  }
}
```

---

## 7. 案例生成器

`case_builder` 提供三个预构建的 `SCUCInput` 对象，可直接传入 `scuc_solve()` 进行测试与基准测试。
详细接口文档见 [docs/case_builder.md](case_builder.md)。

### C++ API

```cpp
#include "mipsolvers/scuc/case_builder.hpp"
using namespace mipsolvers::scuc;

// 3-bus 最小验证算例（2 机、2 支路）
SCUCInput inp3 = build_3bus_case(/*T=*/6);

// 6-bus 中等算例（3 机、7 支路，含可选风电/储能）
SCUCInput inp6 = build_6bus_case(/*T=*/24, /*dt=*/1.0,
                                 /*with_wind=*/true, /*with_storage=*/true);

// IEEE 39-bus 标准算例（10 机、51 支路，含可选风电/光伏）
SCUCInput inp39 = build_ieee39_case(/*T=*/24, /*dt=*/1.0,
                                    /*with_wind=*/true, /*with_solar=*/false);

// JSON 序列化
std::string json = scuc_input_to_json(inp39, /*indent=*/2);
```

### 内置算例规模

| 算例 | 母线数 | 机组数 | 支路数 | 可选资源 | 备注 |
|------|--------|--------|--------|----------|------|
| `build_3bus_case` | 3 | 2（煤电+燃气）| 2 | — | 快速验证用 |
| `build_6bus_case` | 6 | 3 | 7 | 风电（bus 3）、储能（bus 5, 240 MWh）| 中等规模 |
| `build_ieee39_case` | 39 | 10 | **51** | 3 个风电站（1050 MW）+ 2 个光伏站（450 MW）| IEEE 39节点标准系统，PFR需求 300 MW，总负荷 4599 MW |

> **注（ieee39 支路数）**：51 条 = 41 条输电线 + 2 条 G10 联络线 + 1 条 G9 升压变 + 7 条 G2–G8 升压变。
> （`case_builder.hpp` 注释中的 "46" 为历史遗留，代码注释已改正为 "51 lines"。）

### 算例中的 BidSegment 含义

```cpp
// BidSegment{price, quantity}
// price    = 边际报价（$/MWh）
// quantity = 超出 Pmin 的分段容量（MW）
g.bid_segments.push_back({25.0, 250.0});  // 正确：先报价后容量
```

### IEEE 39-bus 初始状态

```cpp
// 开机机组：G1(500MW), G2(300MW), G5(200MW), G10(600MW)，合计 1600 MW
inp.initial_status.commitment = {1,1,0,0,1,0,0,0,0,1};
inp.initial_status.dispatch   = {500,300,0,0,200,0,0,0,0,600};
```

---

## 8. 测试结果

测试框架：**Catch2**，测试文件：`tests/test_market_simulation.cpp`

### 8.1 测试套件概览

| 标签 | 测试名称 | 说明 |
|------|---------|------|
| `[market][viz]` | TC-1 … TC-9 | 端到端功能验证（共 161 个断言，100% 通过）|
| `[market][benchmark]` | TEST 1b … TEST 3b | 各求解器性能基准（HiGHS / Gurobi / NativeBranchAndCut）|

### 8.2 多求解器性能基准

**测试环境**：macOS ARM64（Apple M4），Release 模式，MIP 间隙 1%，HiGHS 4.x / Gurobi 12

| 算例 | T | HiGHS | Gurobi | NativeBranchAndCut | 目标函数 ($) | 割平面数 |
|------|---|-------|--------|--------------------|------------|---------|
| 3-bus | 6 | 22 ms | 4 ms | 50 ms | 746,316 | 32 |
| 6-bus（风电+储能）| 8 | 22 ms | 6 ms | 73 ms | 76,478 | 86 |
| IEEE 39-bus | 4 | 21 ms | 5 ms | 15 ms | 203,130 | 122 |
| **IEEE 39-bus** | **24** | **81 ms** | **73 ms** | **85 ms** | **976,359** | **898** |

> **说明**：
> - SCIP 在本框架中仅支持 MINLP，不参与 MILP 基准。
> - 三个求解器目标函数差异 < 1 $（MIP 间隙范围内），结果一致。
> - Auto 模式（`solver="Auto"`）默认优先 HiGHS，不可用时依次回退到 Gurobi → NativeBranchAndCut。

### 8.3 IEEE 39-bus 24h 费用分解（SCUC MILP 阶段）

| 费用项目 | 金额 ($) | 占比 |
|---------|---------|------|
| 能量费用 | 830,936 | 85.1% |
| 启动费用 | 49,500 | 5.1% |
| 空载费用 | 60,000 | 6.1% |
| 备用费用 | 35,923 | 3.7% |
| 惩罚费用 | 0 | 0.0% |
| **合计** | **976,359** | 100% |

切负荷：**0 MWh**（无切负荷）

### 8.4 关键功能验证项（`[market][viz]` 套件）

- **功率平衡**：每时段 $\sum_g P_{g,t}$ 与 $D_t$ 误差 < 5 MW
- **切负荷为零**：24 个时段切负荷均 < 1 MW（ieee39 算例）
- **SCED 组合一致**：$\lfloor I^{SCED}_{g,t} \rceil = \lfloor I^{SCUC}_{g,t} \rceil$
- **LMP 范围**：$0 \leq \text{LMP}_{avg} \leq 300$ \$/MWh
- **JSON 结构**：输出含 `scuc`、`sced`、`lmp`、`meta` 顶层键，数组维度正确

### 8.5 测试命令

```bash
# Release 模式构建
cmake -S . -B build_rel -DCMAKE_BUILD_TYPE=Release
cmake --build build_rel --target test_market_simulation -j8

# 功能验证测试
./build_rel/test_market_simulation "[market][viz]" 2>/dev/null

# 多求解器基准
./build_rel/test_market_simulation "[market][benchmark]" 2>/dev/null

# IEEE 39-bus 完整流程
./build_rel/test_market_simulation "[market][viz][ieee39]" 2>/dev/null

---

*文档更新日期：2026-05-06*  
*对应源文件：`include/mipsolvers/scuc/scuc.hpp`，`src/scuc/scuc.cpp`，`src/scuc/case_builder.cpp`*
