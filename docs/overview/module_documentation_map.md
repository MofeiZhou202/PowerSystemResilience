# 模块文档覆盖地图

最后核实：2026-08-22

本表以当前文件系统为准。`src/<module>/` 与 `docs/modules/<module>/` 的目录名必须完全一致；覆盖等级只表示
文档是否存在并完成源码等价核验，不表示实现已经达到生产认证。

| 源码模块 | 唯一主手册 | 数学实现归口 |
|---|---|---|
| `analysis` | [分析手册](../modules/analysis/analysis_manual.tex) | 理论决策模型、源码等价实现、承载力/薄弱环节/反事实数值证据 |
| `api` | [公共 API 手册](../modules/api/api_manual.tex) | 能力门控、safe API、句柄签名与 OPF 解后审计理论及回归证据 |
| `carbon_analysis` | [碳分析手册](../modules/carbon_analysis/carbon_analysis_manual.tex) | 碳流理论、源码等价追踪/矩阵实现、年度库存与数值证据 |
| `dynamics` | [动力学手册](../modules/dynamics/dynamics_manual.tex) | `source_equivalent_solver.tex` |
| `ev_power_traffic` | [电—交通手册](../modules/ev_power_traffic/ev_power_traffic_manual.tex) | `source_equivalent_model.tex` |
| `graph` | [图模块手册](../modules/graph/graph_manual.tex) | 十一章：接口/索引、通用理论、建图拓扑、收缩、计划、串联/悬垂、稠密/稀疏 Kron、恢复、HTTP、验证审计、数值交叉验证 |
| `harmonics_power_flow` | [谐波专著](../modules/harmonics_power_flow/harmonics_power_flow_manual.tex) | 九章：范围架构、通用理论、单相 AC/DC 源码等价、三相变压器、非线性耦合、API、指标标准、数值交叉验证、审计准入 |
| `integrated_energy` | [综合能源专著](../modules/integrated_energy/integrated_energy_manual.tex) | 十章/23 页：三章理论、源码等价 LP/MILP、完整 I/O、解析最优解、24 h 独立方程重算与准入 |
| `io` | [输入输出手册](../modules/io/io_manual.tex) | 格式字段换算与拒绝条件，不建立独立电气模型 |
| `market` | [市场手册](../modules/market/market_manual.tex) | 市场经济学理论、SCUC/SCED/LODF 源码等价实现与结算数值证据 |
| `model` | [工程模型专著](../modules/model/model_manual.tex) | 十一章/22 页：身份/量纲/商图理论、组件语义、canonical 映射、独立数学 oracle、稳定 ID 缺陷与审计 |
| `network_reconfiguration` | [网络重构手册](../modules/network_reconfiguration/network_reconfiguration_manual.tex) | 九章：接口、通用理论、规范投影/设备、可达 MILP、求解证书、兼容 ONR、HTTP、验证审计、数值交叉验证 |
| `optimal_power_flow` | [最优潮流手册](../modules/optimal_power_flow/opf_manual.tex) | 八个实现章节；不复制 `power_models` 公式 |
| `power_flow` | [潮流计算手册](../modules/power_flow/power_flow_manual.tex) | 十一个实现章节；线性 DC 移相式已按现行源码复核 |
| `power_models` | [AML 建模层手册](../modules/power_models/power_models_manual.tex) | `source_equivalent_builders.tex`，本模块唯一 builder 公式源 |
| `reliability` | [可靠性手册](../modules/reliability/reliability_manual.tex) | `source_equivalent_model.tex` |
| `resilience` | [弹性恢复专著](../modules/resilience/resilience_manual.tex) | 十八个模块专章加统一工业评价基线：理论、灾害/修复、启发式、严格 MIP 逐约束推导、RA MILP、MESS、投影身份、DAE 证书逐公式推导、全字段契约、HTTP、数值验证与准入审计 |
| `scenario_generation` | [场景生成手册](../modules/scenario_generation/scenario_generation_manual.tex) | 16 专章/56 页：条件概率、AR(1)、缩减理论，常规/可靠性/弹性，台风风雨--易损--修复--交通，全字段契约、独立复算与审计准入 |
| `server` | [服务端手册](../modules/server/server_manual.tex) | ETag、revision、缓存、异步作业与版本路由理论及回归证据 |
| `short_circuit` | [短路手册](../modules/short_circuit/short_circuit_manual.tex) | 十五个模块专章加统一工业评价基线：对称分量/IEC/换流器理论、AC 概览与详细序网、DC 故障、投影身份、稀疏批量、完整 I/O、HTTP/GUI、验证架构、数值交叉验证与深度审计 |
| `sppt` | [SPPT 专著](../modules/sppt/sppt_manual.tex) | 十一章/32 页：投影/MR 与证据统计理论、证书/独立残差、守卫/代理、规模/消融/315 样本活动及失败证据 |
| `time_series` | [时序手册](../modules/time_series/time_series_manual.tex) | 三卷专著：SCUC/多时段 AC/DC/储能理论；`time_series_pf.cpp`、`annual_production_sim.cpp`、`lifecycle_simulation.cpp` 各自源码等价实现；HTTP/结果、穷举+SciPy/HiGHS+OpenDSS+GridLAB-D 数值交叉验证及深度审计 |
| `validation` | [静态校验手册](../modules/validation/validation_manual.tex) | 校验谓词理论、源码阈值/四级过滤与规则边界数值证据 |

## 覆盖判据

- 唯一目录：不存在大小写、旧名称或专题名称构成的第二模块目录。
- 唯一主手册：模块 README 明确指定一个主 `.tex`；章节文件只能由该主手册引用。
- 源码等价：实现公式邻近位置必须给出 `文件:函数名` 符号锚点，并记录硬编码阈值、截断和分支；禁止行号锚点。
- 理论分层：通用理论只能进入独立 `theory_*.tex`，并用 `theorynote`、实现对应表和 `gapnote` 与当前行为隔离。
- 诚实边界：未消费字段、回退、近似、缺少测试和已知代码问题必须显式披露。

## 跨目录实现

`projection`、`assembly` 不是 `src/` 顶层目录：前者实现位于 `src/model/`，后者实现位于
`src/power_flow/`，分别归入 `model` 和 `power_flow` 手册。`scenario_generation`、
`short_circuit` 的部分公共头位于 `include/hacdcpf/analysis/`，但其源码所有权仍按 `src/` 模块归档。

更新文档时必须同步检查 [模块索引](../modules/README.md)、[文档分类台账](document_catalog.md) 和本表。
