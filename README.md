# Hybrid AC/DC Distribution Systems Simulation 项目系统说明

本文档面向工程使用者和开发者，说明本项目从“工程场景建模”到“规范模型求解”、再到“结果回投”的完整链路。更底层的公式、接口和实现审计见 `docs/technical_notebook/`。

## 1. 工程场景

本项目是一个 C++20 静态库，核心目标是支撑混合 AC/DC 配电系统的稳态仿真、优化、可靠性和弹性分析。典型工程对象包括：

- 城市/园区配电网：AC 馈线、DC 母线、VSC 换流器、DC/DC 变换器、联络开关、断路器、分布式电源、储能和充电设施。
- 主动配电网：PV、风电等可再生电源，静态发电机、柔性负荷、可控负荷、移动储能、微电网和虚拟电厂。
- 故障恢复和运行优化：N-1 故障枚举、三阶段故障恢复、网络重构、弹性恢复、多时段生产模拟、OPF 和碳流追踪。
- 标准算例和工程导入：MATPOWER、JPC JSON、Excel/OpenDSS 可选接口，以及项目内部 rich component schema。

工程上，本项目不直接把所有复杂设备塞进一个求解器模型，而是采用分层流程：

```text
Rich HybridPowerSystem
  -> validation / diagnostics
  -> pre-formulation: aggregation + graph analysis + network merging
  -> canonical projection
  -> solver data / optimization model assembly
  -> analysis or numerical solve
  -> projection back / result attribution
```

这种设计的核心价值是：输入层保留工程语义，求解层保持数值模型简洁，输出层再把电压、潮流、可靠性贡献和拓扑动作映射回原始设备。

## 2. Rich Component 与标准模型层

项目的顶层数据结构是 `HybridPowerSystem`，定义在 `include/hacdcpf/model/hybrid_power_system.hpp`。它包含 AC、DC、换流器、微电网、VPP、移动储能和可选三相系统。

| 模型域 | Rich component / 标准模型 | 工程语义 | Canonical 处理 |
|---|---|---|---|
| AC 网络 | `ACBus`, `ACBranch` | 母线、线路、基础负荷/并联参数 | 直接进入 AC 导纳矩阵和潮流/OPF 模型 |
| AC 电源 | `Generator`, `StaticGenerator`, `RenewableGen`, `PVSystem`, `ExternalGrid` | 常规机组、分布式电源、可再生电源、外部电网 | 聚合成每母线注入；外部电网可设置 slack / 电压设定 |
| AC 负荷 | `Load`, `FlexibleLoad`, `AsymmetricLoad`, `AsynchronousMotor` | 静态负荷、柔性负荷、不对称负荷、电机 | 转换为 canonical load 或等效注入；ZIP 系数在组装阶段加权聚合 |
| AC 设备 | `Transformer2W`, `Transformer3W`, `Switch`, `CircuitBreaker`, `Shunt` | 变压器、开关、断路器、并联补偿 | 变压器/开关可展开为等效 `ACBranch`；保留 provenance 映射 |
| 充电设施 | `ChargingStation`, `Charger` | 站级或桩级 EV 负荷 | 桩级可投影到站级；作为恒功率负荷进入组装 |
| DC 网络 | `DCBus`, `DCBranch`, `DCLoad` | DC 母线（`DC_P` 有功母线 / `DC_V` 电压参考母线 / `DC_ISOLATED` 停电隔离母线）、线路、负荷 | 进入 DC 电导矩阵和混合潮流/优化模型；`DC_ISOLATED` 母线在图/孤岛分析中按停电处理，不作为电压参考，且在潮流方程组中以固定电压剔除，避免雅可比奇异 |
| DC 电源/设备 | `StaticGeneratorDC`, `PVArrayDC`, `DCDCConverter`, `DCCircuitBreaker`, DC storage | DC 电源、PV 阵列、DC/DC、DC 开断设备 | 投影到 DC 注入、DC 边或耦合设备 |
| AC/DC 耦合 | `VSCConverter`, `EnergyRouter` | 换流器、能量路由器、多端口耦合 | VSC 保留为耦合元件；EnergyRouter 展开为内部 DC 母线、VSC 和 DC/DC |
| 聚合资源 | `VirtualPowerPlant`, `Microgrid`, `MobileStorage` | VPP、微电网、移动储能 | VPP/Microgrid 可转换为 PCC 注入；移动储能按位置和状态注入 |
| 三相系统 | `ThreePhaseACSystem` | abc 三相馈线和设备 | 可投影或单独由三相 NR 分析处理 |

因此，rich component 层偏工程数据模型，canonical 层偏求解器模型。二者之间不能简单等同，必须通过 projection 和 mapping 保持可追溯性。

## 3. Pre-Formulation：聚合、图分析与网络合并

Pre-formulation 是正式建立 PF/OPF/MILP 前的结构化预处理，主要解决三个问题：设备聚合、拓扑图表达、数值网络合并。

### 3.1 节点和设备聚合

求解器通常需要“每个节点的净注入”或“每个节点的一组受约束设备”。项目在 `src/power_flow/solver_data.cpp` 中执行以下聚合：

- `aggregate_generation`：把 `Generator`、`StaticGenerator`、`RenewableGen`、`PVSystem`、`Storage`、`VPP`、`Microgrid`、`MobileStorage` 聚合为每个 AC 母线的 `pg/qg`。
- `aggregate_load_demand`：把 `Load` 和 `ChargingStation` 聚合为每母线 `pd/qd`，并按负荷权重计算 ZIP 系数。
- `make_solver_data`：先调用 `project_to_canonical_models`，再移动 canonical component 到 `SolverData`，最后建立 AC `Ybus` 和 DC `gdc`。

这一步要求 bus index、设备 bus 引用、base MVA 和 in-service 状态一致，否则会出现 silent drop 或结果错位。因此正式求解前建议使用 `validate_full` 或 `validation::validate(..., ValidationLevel::Strict)`。

### 3.2 图建模与拓扑分析

图层入口是 `include/hacdcpf/graph/power_system_graph.hpp` 和 `src/graph/power_system_graph.cpp`。`build_power_system_graph` 将 `HybridPowerSystem` 转为：

- `GraphNode`：AC/DC 母线节点，带有 domain、slack、load、generator、storage、converter 等标记。
- `GraphEdge`：AC line、transformer、switch、breaker、DC line、DC switch、VSC coupling、DCDC coupling。
- domain-qualified maps：`ac_bus_id_to_node_idx` 和 `dc_bus_id_to_node_idx`，用于避免 AC/DC 母线使用相同数字 ID 时发生混淆。

图分析支撑：

- 连通性、孤岛、径向性、桥边、割点等拓扑诊断。
- 开关收缩、零阻抗边识别、Kron/series/pendant reduction。
- 网络重构和弹性恢复中的连通性约束、候选开关动作和故障隔离。

### 3.3 节点聚合与网络合并

项目有两类容易混淆但用途不同的“合并”：

- Projection 里的 zero-impedance bus merge：`merge_zero_impedance_buses` 将由零阻抗支路连接的 AC 母线合并，避免 `Ybus` 病态。映射记录在 `BusMergeMap`。
- Graph 里的 switch contraction：`contract_zero_impedance_edges` 根据闭合开关、闭合断路器和零阻抗边形成 super-node，同时聚合负荷/电源/并联元件，并保留 `ContractionResult`。

二者都属于 pre-formulation，但服务对象不同：前者偏数值求解稳定性，后者偏拓扑/图算法和恢复过程。开发时应优先使用 domain-qualified map，避免用裸 `int bus_id` 跨 AC/DC 域传递。

## 4. Canonical Models（规范模型）

Canonical projection 的入口是 `project_to_canonical_models`，定义在 `include/hacdcpf/projection/project_to_canonical.hpp`，实现主要在 `src/model/network_utils.cpp`。其职责是把 rich component 展开或折叠为求解器可以直接处理的扁平模型。

典型转换包括：

- 统一 base MVA，并把 bus-level load 与显式 `Load` 表保持一致，避免双计。
- 实际工程值到标幺值转换：当支路只给出实际值（`r_ohm_per_km`、`x_ohm_per_km`、`b_us_per_km`、`length_km` 和母线 `base_kv`）而 `r_pu/x_pu` 仍为零时，`convert_actual_to_per_unit`（在 `project_to_canonical_models` 入口执行）按 `Z_base = base_kv² / base_mva` 计算 `r_pu/x_pu/b_pu`，AC 与 DC 支路同理。该步骤是非破坏性且幂等的：已给定标幺值的支路保持不变，因此既支持 ETAP/OpenDSS 风格的实际值输入，也完全兼容既有标幺值算例。
- `FlexibleLoad`、`AsymmetricLoad`、`AsynchronousMotor` 转换为等效 `Load`。
- `Transformer2W`、`Transformer3W`、`Switch`、`CircuitBreaker` 转换为等效 branch，并通过 `BranchExpandMap` 记录来源。
- `EnergyRouter` 展开为内部 DC 母线、VSC 和 DC/DC 耦合。
- `VirtualPowerPlant`、`Microgrid`、`MobileStorage` 投影为 PCC 注入或储能等规范设备。
- 执行零阻抗母线合并，生成 `BusMergeMap`，并剔除无供电路径的 dead islands。
- 生成 `ComponentMapping` / `ProjectionReport` 能力，用于诊断和结果归因。

Canonical 层的一个重要设计原则是：求解器只看到必要的数学对象，但用户仍能通过 mapping 理解某条 branch、某个注入或某个恢复动作来自哪个原始组件。

## 5. 现有分析能力

| 模块 | 入口/路径 | 使用模型 | 输出 |
|---|---|---|---|
| AC/DC Power Flow | `solve_power_flow`, `solve_dc_power_flow`, `solve_power_flow_fdpf`, `solve_ac_dc_power_flow` | canonical AC/DC network + converter coupling | 电压、相角、潮流、收敛状态 |
| 三相潮流 | `analysis::solve_three_phase_nr` | `ThreePhaseACSystem` | abc 相电压、电流和三相收敛信息 |
| OPF | `solve_ac_opf`, `solve_dc_opf`, `solve_rpo` | AC/IPM、DC LP/QP、无功优化模型 | 调度、目标值、节点 LMP、约束诊断；DCOPF branch congestion dual 仅在 `branch_mu_valid=true` 时可作工程解释 |
| 网络重构 | `solve_optimal_reconfiguration`, `run_topology_reconfiguration` | LinDistFlow MILP + graph connectivity | 开/合支路集合、损耗 proxy、PF 校验 |
| 图分析/降阶 | `build_power_system_graph`, `contract_zero_impedance_edges`, Kron/series/pendant recovery | graph abstraction | 连通性、径向性、super-node、恢复映射 |
| 可靠性 MC | `run_nonsequential_mc`, `run_sequential_mc` | component outage sampling + DC OPF state evaluation | EENS、LOLE、LOLF、CoV、关键元件 |
| FMEA 可靠性 | `run_distribution_fmea` | N-1 enumeration + switching/repair stage evaluation | contingency detail、EENS/EDNS/SAIFI/SAIDI |
| 三阶段可靠性 | `run_three_stage_reliability` | native C++ MILP via MIPSolvers | 三阶段失负荷、SOP 动作、节点可靠性指标 |
| 配电弹性 | `run_distribution_resilience_assessment`, `run_distribution_resilience_mip_assessment` | heuristic sequential 或 multi-period MIP LinDistFlow | 恢复曲线、MESS 状态、故障序列、弹性指标 |
| 短路分析 | `run_short_circuit_analysis` | canonical network / sequence approximation | 故障电流和节点短路指标 |
| 碳分析 | `run_carbon_analysis` | PF result + proportional / matrix tracing | 节点、支路、负荷碳流 |
| 时序/生产模拟 | `solve_time_series_pf`, `solve_unit_commitment`, `solve_annual_production_simulation`, `run_lifecycle_simulation` | 多时段负荷/资源曲线 + OPF/UC | 年度生产、成本、生命周期指标 |

优化和 MILP 模块依赖 sibling directory `../MIPSolvers` 提供的 Eigen、HiGHS、Ipopt 和 native branch-and-cut 后端。项目 CMake 默认从该 sibling 路径解析依赖，而不是搜索系统 solver。

## 6. Validation 与诊断

Validation 是从 rich component 到 canonical model 的安全门。主要入口：

- `hacdcpf::validate_full(sys)`：公共 API 中的完整校验。
- `validation::validate(sys, ValidationLevel::...)`：可选择 `Basic`、`Electrical`、`SolverReady`、`Strict`。
- `project_to_canonical_models` 内部也会生成 projection mapping 和 dead-island / merge 相关信息。

当前静态校验覆盖：

- AC/DC bus ID 重复、branch/converter 引用不存在、孤岛、slack 缺失或多 slack。
- 电压上下限、机组 P/Q 限值、branch 阻抗、transformer tap、base MVA 不一致。
- VSC、DC branch、DC converter 引用错误。
- 零阻抗和死岛等会影响数值稳定性的拓扑问题。

建议的工程流程是：

```text
import/load system
  -> validation::validate(sys, SolverReady or Strict)
  -> project_to_canonical_models(sys)
  -> optional graph diagnostics
  -> solve / analysis
  -> result audit and projection back
```

对于可靠性、弹性和网络重构这类组合优化任务，还应把求解器状态、MIP gap、time limit、模型规模和 fallback 状态写入结果对象，避免把启发式、近似可行和最优解混为一谈。

## 7. Projection Back 与结果归因

Projection back 的目标是把 solver 内部的 compact/canonical 结果映射回用户输入的 rich model。主要机制包括：

- `BusMergeMap`：记录 external bus index 到 internal merged position 的映射、合并组、dead bus、branch 原始位置到 projected 位置。
- `unproject_bus_vector`：把求解器返回的电压、相角、LMP 等 bus vector 扩展回原始 bus 数量；dead-island bus 返回 0。
- `BranchExpandMap`：记录 canonical `ACBranch` 来自 `Transformer2W`、`Transformer3W` 或 `Switch` 的哪一个原始组件。
- `ComponentMapping`：记录 rich component 到 canonical component 的来源关系，服务于诊断、碳流和用户可读报告。
- `ContractionResult` 与 `FullNetworkVoltages`：用于图降阶后的电压恢复，特别是 switch contraction、series reduction、pendant reduction 和 Kron reduction。

开发时需要区分三类 ID：

- component `.index`：用户/模型层的稳定组件编号。
- vector position：数组中的 0-based 位置，适合内部循环，不适合对外报告。
- graph node/edge index：图构建后的临时下标，必须通过 mapping 回到模型组件。

任何跨越 projection、graph contraction 或 canonical solver 的结果，都不应直接用数组下标对外解释，必须经过对应 map。

## 8. 典型端到端流程

### 8.1 潮流/OPF 流程

```text
HybridPowerSystem
  -> validate_full
  -> project_to_canonical_models
  -> make_solver_data
  -> aggregate_generation / aggregate_load_demand
  -> build_admittance_matrix / build_dc_conductance
  -> PF or OPF solve
  -> unproject_bus_vector / branch provenance
```

### 8.2 网络重构/弹性恢复流程

```text
HybridPowerSystem
  -> project_to_canonical_models
  -> build_power_system_graph
  -> identify faults, switches, islands, roots
  -> LinDistFlow / connectivity MILP or heuristic restoration
  -> solve with native B&C / HiGHS / selected backend
  -> report BranchRef / switch actions / restoration metrics
  -> optional PF validation on restored topology
```

### 8.3 可靠性流程

```text
HybridPowerSystem + reliability data
  -> validate component failure rates, MTTR, customer/load metadata
  -> enumerate or sample component outages
  -> stage evaluation using OPF / reconfiguration / restoration model
  -> accumulate frequency-weighted EENS, LOLE, SAIFI, SAIDI
  -> expose model_limitations when physics coverage is approximate
```

### 8.4 I/O 流程

```text
MATPOWER / JPC JSON / Excel / OpenDSS
  -> rich HybridPowerSystem
  -> validation and projection
  -> analysis
  -> JSON result export or report generation
```

JPC JSON export should be treated as a schema-preserving operation: rich component arrays must either be written faithfully or explicitly diagnosed as unsupported, because silently writing empty arrays can lose engineering data.

### 8.5 ETAP I/O（导入/导出）

ETAP 互操作由 `include/hacdcpf/io/etap_io.hpp` / `src/io/etap_io.cpp` 提供，编译开关
`-DHACDCPF_ENABLE_ETAP=ON`（依赖 OpenXLSX；默认 OFF）。

支持三条输入路径，全部映射到同一 `HybridPowerSystem`：

1. **规范 ETAP 工作簿**（`save_etap` 产生、可无损 round-trip 的 schema）。
2. **原始 ETAP 工具箱导出**（`etap-main/etap_output.py` 的列名与单位：`OpVMag`/`VMag`
   为百分比、`NominalkV`、`RPos`/`XPos` 为欧姆、`AnsiPosZ`/`PosR` 为变压器 %Z/%R、
   `ZBaseMVA` 为 kVA、`LUMPEDLOAD` 用 `MVA`+`PF`）。导入器通过别名表同时识别两套列名。
3. **原生 ETAP 工程 XML**（如 `Feeder.xml`）：`load_etap_xml()` 直接解析 `<COMPONENTS>`
   元素属性，无需 Python 工具箱。

主要 API：

| 功能 | 入口 |
|---|---|
| Excel 导入 | `load_etap(path[, mode, report])` |
| Excel 导出 | `save_etap(sys, path[, report])` |
| 原生 XML 导入 | `load_etap_xml(path[, mode, report])` |
| 严格度 | `EtapImportMode::{Strict, Permissive}`（Strict 对悬空母线引用抛错，Permissive 记 warning） |
| 往返保真度报告 | `etap_fidelity_check(sys)` → `EtapFidelityReport`（逐字段 before→after 差异） |
| 诊断 | `EtapIoReport`（每个 sheet 计数 + warnings） |

无显式 `Type` 列时，母线类型由所连 utility（→SLACK）/generator（→PV）推导；标幺↔欧姆
阻抗用 `Z_base = base_kV² / base_MVA`（取自支路 from 母线），保证往返精确。

支持的元件 sheet：`BUS, XLINE, CABLE, XFORM2W, XFORM3W, UTIL, SYNGEN, MGSET(仅导入),
PVARRAY, WIND, LUMPEDLOAD(ZIP), CAPACITOR, HVCB, INDMOTOR, DCBUS, DCIMPEDANCE,
DCLUMPLOAD, DCCONVERTER, DCCB, INVERTER, CHARGER, BATTERY` 外加 `PROJECT`。

**逐字段保真**：除拓扑与核心电气量外，往返还无损保留——2 绕组/3 绕组变压器分接头
（`TapSide/TapPos/TapStepPct`，3W 含 `ShiftMV/LV`）、负荷 ZIP 模型与优先级、VSC 控制模式
与设定点、电池 SoC/效率，以及**短路数据**：外部电网 `S_sc_max/min_MVA`、`RX_max/min`、
零序 `R0/X0`；同步机次暂态/暂态/同步电抗 `Xdpp/Xdp/Xd`、`Ra`、零序 `R0/X0`；断路器额定/
开断电流 `I_rated_kA`/`I_breaking_kA`；变压器零序 `Z0_percent`。原生 XML 同时识别 ETAP
原始属性（`ZeroR/ZeroX`、断路器 `Rated`、`AnsiPosXR` 反推 %R 等）。

**3 绕组变压器潮流**：投影时 3W 被展开为三条等效支路（成对短路阻抗构成的 Δ）。有载调压
（OLTC）仅作用于与受调绕组端子相连的两条支路（`tap_side`：0=HV，1=MV，2=LV），不影响对边
支路，物理上更准确。

命令行工具 `etap_convert`（`-DHACDCPF_ENABLE_ETAP=ON` 时构建）：

```text
etap_convert etap2json  in.xlsx  out.json   [--strict]
etap_convert xml2json   in.xml   out.json   [--strict]
etap_convert json2etap  in.json  out.xlsx
etap_convert etap2etap  in.xlsx  out.xlsx   [--strict]   # 规范化
etap_convert fidelity   in.xlsx                          # 报告再导出会丢失的字段
```

Python 侧 `etap-main/src/canonical_schema.py` 提供与 C++ 完全一致的列定义
（`CANONICAL_COLUMNS`）、`write_canonical_workbook()` 与 `convert_raw_export()`，
用于从工具箱直接产出规范工作簿。

**Web GUI 集成**（`tests/run_gui_server.cpp`，`web/` 下的画布编辑器挂载于 `/xjtu/`）：

| 操作 | 入口 |
|---|---|
| 导出当前系统为 ETAP `.xlsx` | `POST /api/session/export_etap`（二进制下载）；工具栏「导出ETAP」按钮 |
| 导入 ETAP `.xlsx`（二进制上传） | `POST /api/session/load_etap_xlsx`；「加载算例」对话框「导入ETAP工作簿 (.xlsx)」 |
| 导入原生 ETAP `.xml` | `POST /api/session/load_etap_xml`；「加载算例」对话框「导入ETAP工程 (.xml)」 |

往返与摄入由 `tests/test_io_etap.cpp` 覆盖（Excel round-circle、真实导出摄入、
case14 潮流一致性、逐字段保真度、原生 XML、3 绕组变压器分接头/潮流、短路数据），
fixtures 见 `data/etap_sample.xlsx`、`data/etap_feeder.xml`。GUI 后端端到端冒烟测试见
`tools/gui_api_e2e.py`（启动服务并驱动 加载/导出ETAP/重新导入/XML导入/潮流/短路 全链路，
已接入 ctest 目标 `gui_api_e2e`）。画布层浏览器端到端测试见 `tests/e2e/canvas_3w_e2e.mjs`
（Playwright：在画布上放置并连线一台三绕组变压器+外网+负荷，同步后端并跑潮流；需
`npm i -D playwright && npx playwright install chromium`）。

## 9. 实现地图

| 主题 | 主要文件 |
|---|---|
| 顶层模型 | `include/hacdcpf/model/hybrid_power_system.hpp` |
| AC/DC/rich components | `include/hacdcpf/model/ac_components.hpp`, `include/hacdcpf/model/dc_components.hpp`, `include/hacdcpf/model/converter_components.hpp` |
| Canonical projection | `include/hacdcpf/projection/project_to_canonical.hpp`, `src/model/network_utils.cpp` |
| Projection mapping | `include/hacdcpf/projection/canonical_network.hpp` |
| Solver data assembly | `include/hacdcpf/assembly/solver_data.hpp`, `src/power_flow/solver_data.cpp` |
| Graph model | `include/hacdcpf/graph/power_system_graph.hpp`, `src/graph/power_system_graph.cpp` |
| Graph reduction/recovery | `include/hacdcpf/graph/switch_contraction.hpp`, `include/hacdcpf/graph/result_recovery.hpp`, `src/graph/` |
| Validation | `include/hacdcpf/validation/validate_system.hpp`, `src/validation/validate_system.cpp` |
| PF/OPF | `include/hacdcpf/power_flow/`, `include/hacdcpf/optimal_power_flow/`, `src/power_flow/`, `src/optimal_power_flow/` |
| 网络重构 | `include/hacdcpf/network_reconfiguration/`, `src/network_reconfiguration/` |
| 可靠性 | `include/hacdcpf/reliability/`, `include/hacdcpf/analysis/three_stage_reliability.hpp`, `src/reliability/` |
| 弹性恢复 | `include/hacdcpf/resilience/resilience_assessment.hpp`, `src/resilience/` |
| I/O | `include/hacdcpf/io/`, `src/io/` |
| 技术笔记 | `docs/technical_notebook/` |

## 10. 维护原则

- Rich component 层尽量保留工程语义，不把设备信息过早丢弃。
- Canonical model 层只保留求解必要变量，并通过 mapping 保留来源。
- Graph 层所有 AC/DC lookup 应使用 domain-qualified map。
- 对外报告使用稳定 component ID 或结构化引用，例如 `BranchRef`，不要暴露临时 vector position。
- 任何 fallback、近似模型、time-limit 解和模型覆盖不足都应进入 result status / diagnostics。
- 修改 projection 或 graph 代码后，应优先补充 round-trip、result recovery、AC/DC 同号 bus ID、dead-island、zero-impedance merge 等测试。
