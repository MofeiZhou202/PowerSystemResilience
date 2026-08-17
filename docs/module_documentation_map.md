# 模块文档覆盖地图

最后核实：2026-08-18

本文件评价文档覆盖，不评价实现质量。源码、公共头文件、注册测试和运行时响应始终是行为权威。
模块所有权或主文档变化时应原地更新本文件，不新增日期化快照。

## 覆盖等级

| 等级 | 含义 |
|---|---|
| 完整手册 | 具有独立中文 LaTeX 主文档，覆盖模型、接口、算法、验证、工业指标和局限 |
| 专项契约 | 具有面向当前运行行为的独立契约或实现说明 |
| 分散资料 | 信息散布在相邻模块文档，尚无单一所有者 |
| 研究资料 | 详细推导存在，但运行时覆盖更窄，不构成当前契约 |

覆盖等级不表示求解器正确、完整或生产就绪，只表示维护者定位当前模型与边界的效率。

## 核心模型与求解模块

| 源码模块 | 覆盖 | 主文档 | 说明 |
|---|---|---|---|
| `model/`、`validation/` | 完整手册 | [元件模型手册](ComponentModels/component_models_math_audit.tex) | 配合[参数系统](parameter_system.md)和[跨模块语义](model_data_semantics_contract.md)使用 |
| `projection/`、`assembly/` | 专项契约 | [投影与结果恢复](projection_and_results.md) | 与潮流手册共同规定索引、装配和恢复 |
| `power_flow/` | 完整手册 | [潮流手册](PowerFlow/power_flow_manual1.tex) | 覆盖平衡、三相、鲁棒求解、CPF 与换流器限制 |
| `optimal_power_flow/` | 完整手册 | [OPF 手册](OptimalPowerFlow/opf_manual.tex) | 覆盖 Native、Parity、DC、RPO 与三相路径 |
| `power_models/` | 完整手册 | [AML 建模层手册](PowerModels/power_models_manual.tex) | 明确实验性建模层与生产路径边界 |
| `graph/` | 完整手册 | [图模块手册](Graph/graph_manual.tex) | 覆盖域限定 ID、拓扑、降阶和结果恢复 |
| `network_reconfiguration/` | 完整手册 | [网络重构手册](NetworkReconfiguration/network_reconfiguration_manual.tex) | 区分混合重构 MILP 与 AC-only ONR |

## 分析、规划与运行模块

| 源码模块 | 覆盖 | 主文档 | 说明 |
|---|---|---|---|
| `analysis/` | 完整手册 | [分析手册](Analysis/analysis_manual.tex) | 承载力、薄弱环节与反事实规划统一归口 |
| `reliability/` | 完整手册 | [可靠性手册](Reliability/reliability_manual.tex) | MC、FMEA、三阶段和频率—持续时间分章说明 |
| `resilience/` | 完整手册 | [弹性手册](Resilience/resilience_manual.tex) | 启发式、恢复 MILP、MESS 与认证恢复统一归口 |
| `scenario_generation/` | 完整手册 | [场景生成手册](ScenarioGeneration/scenario_generation_manual.tex) | 覆盖三类场景、台风和交通影响 |
| `time_series/` | 完整手册 | [时序手册](TimeSeries/time_series_manual.tex) | 覆盖 UC→OPF→PF、年度与生命周期 |
| `carbon_analysis/` | 完整手册 | [碳分析手册](CarbonAnalysis/carbon_analysis_manual.tex) | 覆盖快照碳流、年度核算与 GEC |
| `ev_power_traffic/` | 完整手册 | [电—交通手册](EvPowerTraffic/ev_power_traffic_manual.tex) | 覆盖 A–H 层级并明确启发式/认证边界 |
| `integrated_energy/` | 完整手册 | [综合能源手册](IntegratedEnergy/integrated_energy_manual.tex) | 明确园区聚合多能流而非 AC 网络模型 |
| `market/` | 完整手册 | [市场手册](Market/market_manual.tex) | 覆盖日前、实时、安全和结算，保持 AC-only 口径 |

## 动态、故障与电能质量模块

| 源码模块 | 覆盖 | 主文档 | 说明 |
|---|---|---|---|
| `dynamics/` | 完整手册 | [动力学手册](Dynamics/dynamics_manual.tex) | 覆盖相量域 DAE、设备、积分器与小信号 |
| `short_circuit/` | 完整手册 | [短路手册](ShortCircuit/short_circuit_manual.tex) | 覆盖 IEC 60909、序网、换流器和直流故障 |
| `harmonics_power_flow/` | 完整手册 | [谐波手册](HarmonicsPowerFlow/harmonics_power_flow_manual.tex) | 覆盖频域、三相、NIC 与标准校核 |

## 数据、接口与可执行验证模块

| 源码模块 | 覆盖 | 主文档 | 说明 |
|---|---|---|---|
| `io/` | 完整手册 | [I/O 手册](IO/io_manual.tex) | 覆盖格式映射、导入报告、可选后端和往返 |
| `api/` | 完整手册 | [公共 API 手册](Api/api_manual.tex) | C++ 门面；HTTP/Python 由专项契约补充 |
| `sppt/` | 完整手册 | [SPPT 手册](SPPT/sppt_manual.tex) | MR1–MR8、证书、独立残差、守卫和代理循环 |
| `src/server/`、`web/` | 专项契约 | [运行时 API](runtime_api.md)、[Canvas 运行时](gui_canvas_runtime.md) | 不属于本次要求的独立源码模块手册清单 |

## 文档所有权决策

- `docs/README.md` 是唯一导航入口和分类政策所有者。
- `document_catalog.md` 是 Markdown 与 LaTeX 资产归属台账。
- `development_status.md` 独占易变的构建、测试、依赖和当前工作证据。
- [模块手册索引](modules/README.md) 独占 18 部新增手册与 3 部标杆手册的映射。
- 专项 Markdown 契约描述公共运行行为；LaTeX 手册负责完整数学模型、算法、数据和工业评价。
- `archive/` 与 `latex/paper/` 分别管理历史资料和研究论文，不得自动升级为生产契约。

## 维护优先级

| 优先级 | 工作 | 完成条件 |
|---|---|---|
| P0 | 保持索引在干净检出中可用 | `docs/README.md` 的本地链接全部存在 |
| P0 | 保持 18 部手册与源码同步 | 每部主 `.tex` 可编译，模型范围、字段和测试引用无漂移 |
| P1 | 补齐公开结构体字段表 | 每个公共输入/结果字段均有单位、默认值、索引和语义 |
| P1 | 深化模块专用定量阈值 | 工业指标具有算例、阈值、失败处置与可复现命令 |
| P2 | 控制长文重复 | 同一运行状态只有一个所有者，伴随文档只做链接 |

## 更新检查表

1. 修改覆盖等级前核实公共头、实现、注册测试和 CMake 声明。
2. 当前行为写入契约；方案和假设明确标成研究或规划。
3. 声明结果索引、单位、回退、后端与不支持范围。
4. 同步更新 `docs/README.md`、`document_catalog.md` 与本文件。
5. 执行链接检查、占位符扫描、`git diff --check` 和相关 LaTeX 编译。
