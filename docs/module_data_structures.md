# 各分析模块内部数据结构（深化参考）

Updated: 2026-08-17
Class: Implementation reference（源代码派生；行级细节在代码变更后需复核）

本文件在 [数据结构与 API 契约](data_structure_api_contract.md)（核心骨干）之上，逐个
深入六大分析/求解模块的**内部公共数据结构**（选项 / 结果 / 记录 / DTO），说明它们如何
引用核心 `HybridPowerSystem`，以及贯穿全部模块的三大横切约定。这是"逐模块内部数据结构"
的落地参考。设计评审见 [数据结构设计评审](data_structure_design_review.md)。

## 0. 分析模块如何消费核心模型

所有分析模块统一"富模型进、稳定 ID 出"，中间层各自拥有临时索引空间：

```mermaid
flowchart TD
    HPS["HybridPowerSystem<br/>(稳定 .index)"]
    HPS --> DYN["dynamics<br/>DynamicSystem"]
    HPS --> OPF["optimal_power_flow<br/>ACOPFResult"]
    HPS --> TS["time_series<br/>AnnualProductionSimResult"]
    HPS --> REL["reliability<br/>ReliabilityParams / ComponentRef"]
    HPS --> RES["resilience<br/>DistributionResilience*"]
    HPS --> MKT["market<br/>PricingPeriod / Settlement"]
    DYN -. "结果回稳定 ID" .-> OUT["结果证书<br/>(.index / stable_id)"]
    OPF -. unproject + ComponentRef .-> OUT
    TS -. gen_index/storage_index .-> OUT
    REL -. ComponentRef.stable_id .-> OUT
    RES -. branch_index + AC/DC 限定 .-> OUT
    MKT -. position + index 双键 .-> OUT
```

---

## 1. dynamics（机电暂态 DAE）

头文件：[DynamicSystem.hpp](../include/hacdcpf/dynamics/DynamicSystem.hpp)、
[DynamicSolverOptions.hpp](../include/hacdcpf/dynamics/DynamicSolverOptions.hpp)、
[DynamicResults.hpp](../include/hacdcpf/dynamics/DynamicResults.hpp)、
[SmallSignal.hpp](../include/hacdcpf/dynamics/SmallSignal.hpp)

| 结构 | 角色 | 身份 / 耦合 |
|---|---|---|
| `DynamicSystem` | 顶层暂态容器：`canonical_system` + `network` + `devices` + 微分态 `x` + 代数态 `y` + `options` + 因子缓存 | 持有投影后的 `HybridPowerSystem`；设备为 `unique_ptr<DynamicDevice>` 多态 |
| `DynamicNetwork` | 相节点网络：`Yac_base`/`Gdc_base`、相节点编号 `3*bus_pos+phase` | `ac_bus_pos_by_id` / `dc_bus_pos_by_id` 域限定位映射；`DynamicACBranch` 带 `from_pos`/`to_pos`（0-based）+ `from_bus`/`to_bus`（稳定） |
| `DynamicSolverOptions` | 求解器全部旋钮（求解器类型、DAE/Jacobian 策略、容差、保护、小信号开关） | 内嵌 `PowerFlowOptions`；`project_to_canonical` 默认 true |
| `DynamicResults` / `DynamicSnapshot` | 逐步轨迹 + 每步快照（相电压、频率、COI、设备输出） | 快照为向量；`DynamicInitializationSummary` 带稳定 ID 匹配 |
| `DynamicModalSummary` / `DynamicModalMode` | 小信号模态屏（特征值、阻尼、主导态） | `dominant_state` 为 `type#component:sLocal` 标签 |
| `SmallSignalResult` / `SmallSignalStateInfo` | 完整小信号分析（约化 Jacobian、参与因子） | `SmallSignalStateInfo` 带 `component_index`（稳定）+ `local_index`（设备内） |

> 关键：微分态**全局索引**是纯内部量；对外定位靠设备携带的 `component_index` 与
> `state_label`。DAE 铁律"求解器选择而非模型 bug"见 repo 记忆（显式 Heun 对刚性 AVR 发散）。

---

## 2. optimal_power_flow（OPF / RPO）

头文件：[opf_result.hpp](../include/hacdcpf/optimal_power_flow/opf_result.hpp)、
[opf_options.hpp](../include/hacdcpf/optimal_power_flow/opf_options.hpp)

| 结构 | 角色 | 身份 / 耦合 |
|---|---|---|
| `ACOPFResult` | AC/DC OPF 主结果：`vm/va/vdc/pg/qg/...` + LMP + IPM 续解态 | 结果向量经 `unproject_bus_vector` 回到**授权母线序**；`ComponentRef gen_map/ren_map/stor_map/...` 把向量位→`original_index`+`source_type` |
| `ACOPFResult::ComponentRef` | 结果向量位→源组件 | `original_index`（稳定）+ `source_type` |
| LMP 诚实字段 | `lmp_valid` + `lmp_validity_reason` | 仅当来自该表述的认证等式乘子才 `valid` |
| IPM 续解态 | `ipm_primal_state` 等 + `ipm_layout_signature` | 进程局部不透明；`layout_signature` 防维度兼容但语义错配的复用 |
| `ACOPFOptions` | 后端选择（Auto/ParityIPM/Ipopt/EconomicDispatch）+ 目标 + IPM 旋钮 + `warm_start` | `warm_start` 为**非拥有**指针，必须存活至 `solve_ac_opf` 返回 |
| `ACOPFObjective` | Economic / VoltageDeviation / ActiveLoss / VoltageDeviationAndLoss | RPO 用非 Economic 目标由内层 NLP 优化 |

---

## 3. time_series（UC→OPF→PF / 年度生产模拟）

头文件：[time_series_pf.hpp](../include/hacdcpf/time_series/time_series_pf.hpp)、
[annual_production_sim.hpp](../include/hacdcpf/time_series/annual_production_sim.hpp)

| 结构 | 角色 | 身份 / 耦合 |
|---|---|---|
| `TimeSeriesPFOptions` | UC→OPF→PF 流水线控制（UC 求解器/目标、网络约束、跟踪带、周期 SOC） | 内嵌 `PowerFlowOptions` + `ACOPFOptions`；`fixed_commitment_schedule` / `precomputed_uc_schedule` 为**非拥有**指针 |
| `AnnualProductionSimResult` | 完整年度结果：逐步 + 月块 + 组件年统计 + 采样 PF 快照 | 组件统计带 `gen_index`/`storage_index`/`ren_index`（稳定） |
| `AnnualStepResult` / `BlockSummary` | 逐步 / 逐块聚合量（发电/负荷/新能源/储能/ENS/成本） | 标量聚合，无组件身份 |
| `PFSnapshot` | 仪表盘可视化采样（`vm`/`vdc`/`branch_flows`/`vsc_transfers`） | 向量为**授权序**；`VSCTransfer` 带稳定 VSC/母线 ID |
| `AnnualProductionSimOptions` | L0-L3 分解（块类型、并行日、每日模式） | 内嵌 `ts_pf_options`；`DailySimMode`=SCUC/DynamicSCED/DynamicOPF |
| `UCSchedule` | UC 完整调度（承诺 + 出力 + SOC 轨迹） | `gen_commit[g_active][t]` 按在役机组自然序 |

> 年度分层：L0 年计划 → L1/L2 月/周滚动 UC → L3 日 OPF/PF 回放；每日"能量中性 + 周期 SOC"
> 使日间独立、可并行（`enforce_terminal_soc_cyclic`）。

---

## 4. reliability（可靠性 / FMEA / 信息物理）

头文件：[reliability_assessment.hpp](../include/hacdcpf/reliability/reliability_assessment.hpp)、
[failure_mode.hpp](../include/hacdcpf/reliability/failure_mode.hpp)

| 结构 | 角色 | 身份 / 耦合 |
|---|---|---|
| `ReliabilityRawFields` → `ReliabilityParams` | 统一解析器输入/输出：把异构可靠性字段归一为 λ/repair/unavailability | `resolve_reliability_params()` 单一解析；`data_source`="case/template/default/missing" 诚实标注 |
| `ReliabilityDataPolicy` | 缺失数据策略（StrictCaseDataOnly / MissingOnly / OverwriteTemplate）+ MTBF 约定 | GUI 默认 StrictCaseDataOnly（不发明数据） |
| `ReliabilityOptions` | MC/SEQ 迭代、CoV、尾部风险、分布指标、并行 | 内嵌 `DCOPFOptions` + `data_policy` |
| `ComponentRef` | **可靠性稳定组件引用** | `element_index`（0-based 位）+ `component_index`（稳定）+ `stable_id`（如 `ac_branch:5`）——双键+字符串 ID 典范 |
| `FailureModeRef` / `FailureModeReliability` | 故障模式（激活类/成因类/后果类）+ 解析参数 + 三阶段时长 | `mode_id`（如 `vsc_converter:12/grid_forming_lost`）；成因含 Cyber/Protection |
| `FailureModeParameterOverride` / `ProtectionConfiguration` | 用户覆盖（`optional` 叠加）/ 保护链 | 覆盖绝不写回源模型；保护链用稳定组件 ID |
| `DistributionIndices` / `TailRiskMetrics` | SAIFI/SAIDI/ASAI + VaR/CVaR | `nodal_cif`/`nodal_cid` 按母线序（DC 感知，见 repo 记忆） |

---

## 5. resilience（弹性恢复 / 认证恢复）

头文件：[resilience_assessment.hpp](../include/hacdcpf/resilience/resilience_assessment.hpp)、
[certified_restoration.hpp](../include/hacdcpf/resilience/certified_restoration.hpp)、
[resilience_dynamic_certification.hpp](../include/hacdcpf/resilience/resilience_dynamic_certification.hpp)

| 结构 | 角色 | 身份 / 耦合 |
|---|---|---|
| `DistributionResilienceOptions` | 多时段研究配置（模型、时域、重构、MESS、负荷/新能源曲线） | 负荷曲线三套映射：`*_by_position` / `*_by_index` / `*_by_bus`（域限定） |
| `DistributionResilienceMIPOptions` | 严格 MIP 骨架（电压界、切负荷优先级罚、开关/MESS 成本、gap/时限） | 求解器 Native/HiGHS/Gurobi |
| `DistributionResilienceModelStats` + `ValidityFlags` | 模型规模 + **诚实能力声明** | `model_scope`="hybrid-acdc-restoration-milp"；`ValidityFlags` 逐特征声明是否真正建模 |
| `DistributionResilienceFault` | 单支路故障 | `branch_kind`（**AC/DC 限定**）+ `branch_index`；`ac_branch_index` 为遗留兼容 |
| `CertifiedRestorationAction` | 恢复动作 + 动态事件 + 赛博命令链 + MESS 契约 | 携带 `dynamics::DynamicEvent`；MESS `mess_storage_index`/`mess_target_ac_bus` |
| `MultiFidelityCertificate` | 多保真证书（L1/L2 采样 Jacobian、L3 全 DAE 阈值）+ 残差/Lipschitz/margin | `label`=Safe/Unsafe/Unresolved/Failed；`limitations` 显式 |
| `CertifiedRestorationResult` / `...DAECertificateResult` | 主 MIP + 认证结果 | `model_scope` 显式；`proof_valid` 仅限所模拟场景 |

---

## 6. market（日前/实时市场 / 重复博弈）

头文件：[market_simulation.hpp](../include/hacdcpf/market/market_simulation.hpp)

| 结构 | 角色 | 身份 / 耦合 |
|---|---|---|
| `MarketParticipant` / `ParticipantBehavior` | 市场主体 + 行为策略（加价/withholding） | `generator_positions` 对齐 `ac.generators`，提交时审计 |
| `GeneratorOffer` / `OfferSegment` | 机组报价（分段能量 + 承诺 + 备用） | `generator_position`（位）+ `generator_index`（稳定）**双键并存** |
| `PricingPeriod` | 固定承诺 SCED + 价格（逐区间） | 全部向量按**授权序**并显式 DC 限定（`dc_lmp`/`dc_bus_voltage`/`vsc_*`/`dcdc_*`/两类 dc_storage） |
| `MarketModelScope` | **诚实能力声明** | `model_scope`="ac-dc-linear-v1"；逐项 `*_modelled` 标志 + `energy_prices_valid` |
| `UnsupportedMarketAsset` | 当前表述无法表示的在役资产 | 显式列出（`component_position`+`component_index`+`reason`），避免泛化 scope 失败 |
| 结算族 | `GeneratorSettlement`/`DCStorageSettlement`/`ParticipantSettlement`/`SettlementLedger` | 位+索引双键；真成本与报价成本分离防串味 |
| 安全族 | `SecurityResult`/`N1Violation`/`ComponentContingencyCheck`/`ACContingencyCheck` | 支路 outage/monitored 均带 position+index |
| `MarketPerformanceProfile` | 规模估计 + 各阶段墙钟 | 可扩展性审计用 |

---

## 7. 三大横切约定（跨全部模块）

评审确认以下三条约定在六大模块中一致落地，是全仓数据结构最值得保留的工程规范：

1. **位 + 稳定索引双键**：`reliability::ComponentRef`（`element_index`+`component_index`+`stable_id`）、
   `market::GeneratorOffer`/结算族（`*_position`+`*_index`）、`opf::ACOPFResult::ComponentRef`
   （`original_index`+`source_type`）。凡对外报告一律用稳定键，向量位仅内部。对应
   [铁律 #2](data_structure_api_contract.md#4-三类-id-与域限定映射)。

2. **诚实能力声明**：`resilience::ValidityFlags`+`model_scope`、`market::MarketModelScope`
   +`UnsupportedMarketAsset`、`opf` 的 `lmp_valid`/`lmp_validity_reason`、
   `reliability` 的 `data_source`/`ReliabilityDataQuality`、`dynamics` 的 `DynamicModalSummary`。
   近似/覆盖不足写进结果而非隐藏。对应
   [铁律 #3](data_structure_api_contract.md#1-分层数据架构总览)。

3. **非拥有调度/热启动指针**：`opf::ACOPFOptions::warm_start`、
   `time_series` 的 `precomputed_uc_schedule`/`fixed_commitment_schedule`、
   `market::MarketOptions` 的 `*_dispatch_schedule_mw`。约定统一为"**必须存活至调用返回**"，
   避免大对象拷贝，同时不获取所有权。

此外，**域限定**在中间层同样贯穿：`dynamics` 的 `ac_bus_pos_by_id`/`dc_bus_pos_by_id`、
`resilience` 负荷曲线的 `by_position`/`by_index`/`by_bus` 三套映射、`market`/`opf` 结果向量
的 AC/DC 分列，均遵循[铁律 #1](data_structure_api_contract.md#1-分层数据架构总览)。
