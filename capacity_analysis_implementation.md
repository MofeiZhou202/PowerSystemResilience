# 承载力分析实现说明 (DL/T 2041-2025)
# Hosting-Capacity Assessment — Implementation & Procedure

本文档说明本次将 **分布式资源承载力评估 (DL/T 2041-2025)** 落地到项目中的实现细节、涉及的改动、以及承载力分析的计算流程。它**替换**了原有的 DL/T 2041-2019 “承载力评估” 功能（代码与网页）。

---

## 1. 范围 (Scope)

按与您确认的方案，实现 **设备级（变压器）承载力 + 工程校验** 路径（规范 §7、§10、§11，区域聚合 §8，工程校验 §12），即规范 §19 推荐的 MVP。
系统级生产模拟 / 县级分解 / 系统-设备协调（§5/§6/§9）**未纳入**，因其需要行政区、发电机群、8760h 分区数据，当前数据模型不具备。

核心公式（设备级，§7.4/§7.5）在**变压器下游（低压侧）供电区域**的聚合量上代数求解：

```
S_d = ( P − P_G + β·S·cosθ + P_ESS + ΔP_ESS ) / τ_max
```

| 符号 | 含义 | 来源 |
|---|---|---|
| P | 供电区域负荷 | 由已加载系统聚合（下游 BFS） |
| P_G | 非分布式（常规）发电出力 | 同步机 `generators` 聚合 |
| S | 变压器额定容量 (MVA) | `Transformer2W.sn_mva`（并联时 ×`n_parallel`） |
| cosθ | 功率因数 | `cap_power_factor`（缺省 0.95） |
| β | 最大反向负载率 | `cap_max_reverse_load_rate`（0=按 N-1 自动） |
| τ_max | 最大出力系数 | `cap_dr_max_output_coeff`（缺省 1.0） |
| P_ESS | 现有储能充电功率 | 见 §3 充电策略 |
| ΔP_ESS | 预期新增储能充电区间 | `cap_expected_new_storage_min/max_mw` |

潮流 / 短路 / 谐波引擎**仅在工程校验阶段**被调用（规范将工程校验列为可选）。

---

## 2. 新增的构件属性 (Component properties)

按您的要求以真实属性方式落地（非仅对话框参数），且已与您确认属性归属。所有字段均以 `cap_` 前缀、带向后兼容缺省值，**未删除任何既有属性**。

**`Transformer2W`**（`include/hacdcpf/model/ac_components.hpp`）— 6 个新字段：
`cap_power_factor` (0.95)、`cap_max_reverse_load_rate` (0=N-1自动)、`cap_dr_max_output_coeff` (1.0)、
`cap_registered_dr_mw` (0)、`cap_expected_new_storage_min_mw` (0)、`cap_expected_new_storage_max_mw` (0)。

**`Storage`** 与 **`DCStorage`**（`ac_components.hpp` / `dc_components.hpp`）— 各 2 个新字段（对应您提出的 ESS 问题）：
`cap_charging_strategy`（`"opf"` | `"static"`，缺省 `"opf"`）、`cap_static_charging_mw`（静态充电功率，缺省 0）。

派生量（P、P_G、已接入 DR、opf 模式下的 P_ESS）由模型 + 潮流/OPF 计算得到，**不新增字段**。

---

## 3. ESS 充电策略：static 从 OPF 中剔除

您指出：时序分析中 ESS 出力由 OPF 决定；若采用静态策略，则 OPF 不应包含该 ESS。经核实，既有 `Storage.controllable` 并**不**参与 OPF（它被可靠性/停运逻辑占用），因此新增了专用字段 `cap_charging_strategy`：

- **`"opf"`（缺省）**：与现状完全一致，储能作为 OPF/UC 决策变量被优化。既有算例零行为变化。
- **`"static"`**：储能作为**固定注入**，不参与优化：
  - OPF 每步：在 `src/optimal_power_flow/parity_formulation.cpp` 中将该储能变量上下界**钉定**在 `−cap_static_charging_mw`（充电为负），仍保留在节点功率平衡中（物理正确）。
  - UC/时序：在 `src/time_series/time_series_pf.cpp` 中将 static 单元**排除出被优化集合**（避免把恒定充电强加进带 SOC 约束的 8760h 时段导致不可行）。
  - 承载力评估读取 P_ESS：static → `cap_static_charging_mw`；opf → 取快照 `−p_mw` 的充电量（反映上一次 OPF/时序调度）。
- DC 储能经 `materialize_dc_storage`（`hybrid_power_system.hpp`）折叠为 `Storage` 前，已一并复制这两个新字段。

---

## 4. 后端模块与接口

新增分析模块（模仿 scenario-generation 模式）：
- `include/hacdcpf/analysis/hosting_capacity.hpp`、`src/analysis/hosting_capacity.cpp`（已登记进根 `CMakeLists.txt`）。
- 命名空间 `hacdcpf::analysis`：
  - `HostingCapacityOptions` / `hosting_capacity_options_from_json`
  - `HostingCapacityResult` / `hosting_capacity_result_to_json`
  - `assess_hosting_capacity(sys, opt)`
- 复用引擎：`hacdcpf::solve_power_flow`、`analysis::run_short_circuit_detailed_batch`、`harmonics::solve_harmonic_power_flow`。

HTTP 接口（`tests/run_gui_server.cpp`）：新增 `POST /api/session/run_hosting_capacity`，**移除**旧的 `run_bearing_capacity`（约 456 行内联实现）。

序列化：`src/io/json_io.cpp` 的 `transformer2w_/storage_/dc_storage_` 编解码器均已补齐新字段（另补齐了此前缺失的 `Transformer2W.n_parallel` 以支撑 N-1 自动计算）。所有服务端摄入均经 `hacdcpf::io::from_json`，无需改动各 handler。

---

## 5. 前端（网页）

在原 `hosting`（承载力分析）模块内**原地升级**，Tab/按钮/结果分组保持不变：
- `web/index.html`：`#bcDialog` 改为 2025 参数（cosθ、τ_max、单台 β、N-1 上限、工程校验开关、k_r、ΔU、谐波开关、THD）；`#bearingCapSection` 结果区改为承载力表格。
- `web/js/app.js`：`runBearingCapacity` 改调 `run_hosting_capacity`；`showBearingCapResults` 重写为渲染：区域（县级）表、变压器 S_d/C_d1/C_d2 表、工程校验（潮流/短路/电压/谐波 + 违规清单）、预警清单。
- `web/js/components.js`：`transformer_2w`/`storage`/`dc_storage` 的 `defaults` 加入 `cap_*`（属性编辑器为通用渲染，自动显示）+ 中文 `fieldLabels`。
- `web/js/canvas.js`：`buildSystemJson` 导出与导入均显式补齐 `cap_*`（画布是有损层，必须显式列出方能往返）。

---

## 6. 承载力分析计算流程 (Procedure)

`assess_hosting_capacity` 的步骤：

1. **母线聚合**：按母线累加负荷、非 DR 发电（同步机）、已接入 DR（PV+可再生+静止发电机）、ESS 充电（static/opf）。
2. **供电区域**：以“线路邻接”（两端 `base_kv` 相同的支路）建图；变压器作为边界不穿越。从每台变压器 `lv_bus` 做 BFS 得到其**下游供电区域**母线集合，聚合上述量得到 P、P_G、已接入 DR、P_ESS。
3. **设备级承载力**（§7.4/§7.5）：`S_d = (P − P_G + β·S·cosθ + P_ESS + ΔP_ESS)/τ_max`，由 ΔP_ESS 上下限得到区间，钳位 ≥0。
   - β 选取（§7.6/§7.7）：手动 `cap_max_reverse_load_rate>0` 直接用；否则自动——单台 β=0.8，多台并联 β=(n−1)/n×负载上限。
4. **可接入能力**（§10）：`C_d1 = S_d − 已接入DR`，`C_d2 = C_d1 − 已备案DR`（允许为负）。
5. **变压器分级**（§11，取下限）：`C_d2,min>0` 绿；`C_d2,min≤0 且 C_d1,min>0` 黄；`C_d1,min≤0` 红。
6. **区域（县级）聚合与分级**（§8/§11.1）：按 `bus.area` 汇总 S_D、已接入/已备案 DR，得区域 C_s1/C_s2 并分级；产生黄/红预警。
7. **下级服从上级**（§11.2）：按电压等级 330→10kV 排序，若所属区域为红，则变压器判红。
8. **工程校验（可选，§12）**：运行潮流（电压偏差、支路/变压器越限、反向潮流），三相短路对比断路器遮断能力 `i_breaker_ka`（Eq.3），可选谐波 THD 校核；汇总违规并给出接入建议（allow / allow_with_mitigation / suspend / require_further_study）。

---

## 7. 使用方式

**网页**：稳态分析 → 承载力分析 → 运行承载力评估 → 在对话框中设置全局缺省参数与工程校验开关 → 运行；结果在“结果”页展示。变压器/储能的 `cap_*` 属性可在属性面板中逐个设置（优先于全局缺省），并随算例导出/导入往返保存。

**API**：
```
POST /api/session/run_hosting_capacity
{
  "default_power_factor": 0.95, "default_dr_max_output_coeff": 1.0,
  "single_transformer_beta": 0.8, "n1_loading_limit": 1.0,
  "enable_verification": true, "kr": 0.8,
  "delta_UH_pct": 7, "delta_UL_pct": 7,
  "enable_harmonic": false, "thd_limit_pct": 5
}
```
返回 `transformers[]`、`areas[]`、`warnings[]`、`verification{}`。

---

## 8. 构建与验证结果

- 构建：`run_gui_server` 目标，使用 MSYS2 UCRT64 工具链（`-DMIPSOLVERS_SOURCE_DIR`、`-DSCIP_NO_SIGACTION`），编译链接通过、无错误。
- 端到端（`comprehensive_hybrid_acdc`）：
  - 变压器识别去重正确（1 台，避免 Transformer2W 与其对应支路重复计数）。
  - `cap_*` 属性经 `from_json`→`to_json` 往返保存成功。
  - 覆盖参数生效（cosθ=0.90、τ=0.80）；静态储能 P_ESS 聚合正确（3×4=12 MW）；ΔP_ESS 区间宽度 = (10−0)/0.8 = 12.5，与 `S_d=[67.87, 80.37]` 一致；可接入能力与分级逐项校核正确。
  - 工程校验：潮流 4 次迭代收敛，短路/电压校核通过，建议 allow_connection。
  - OPF 回归：默认（opf）与静态（static）储能两种情形 OPF 均收敛，静态储能不再被优化。

---

## 9. 说明与限制

- 供电区域采用“线路同压邻接、变压器为边界”的下游 BFS；若同一低压母线由多台变压器并联供电，请以单条 `Transformer2W` + `n_parallel` 表示该并联站，以避免供电区域重复计入。
- 上级对象目前取**所属区域（bus.area）**；规范中完整的变压器父级层级（330→10kV 逐级父子）未建父指针，采用区域级“下级服从上级”的合理简化。
- 谐波校核为尽力而为（默认关闭）；潮流与短路为主校核项，均复用项目既有引擎。
- static-ESS 在 UC 中被排除出优化集合而非注入到 UC 粗粒度平衡中，可能使 UC 少计该固定充电负荷；每步 OPF 会据 `cap_static_charging_mw` 修正实际调度。
