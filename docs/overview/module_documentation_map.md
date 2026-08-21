# 模块文档覆盖地图

最后核实：2026-08-21

本表以当前文件系统为准。`src/<module>/` 与 `docs/modules/<module>/` 的目录名必须完全一致；覆盖等级只表示
文档是否存在并完成源码等价核验，不表示实现已经达到生产认证。

| 源码模块 | 唯一主手册 | 数学实现归口 |
|---|---|---|
| `analysis` | [分析手册](../modules/analysis/analysis_manual.tex) | `source_equivalent_model.tex` |
| `api` | [公共 API 手册](../modules/api/api_manual.tex) | 能力门控与门面透传，不新增电气方程 |
| `carbon_analysis` | [碳分析手册](../modules/carbon_analysis/carbon_analysis_manual.tex) | `source_equivalent_model.tex` |
| `dynamics` | [动力学手册](../modules/dynamics/dynamics_manual.tex) | `source_equivalent_solver.tex` |
| `ev_power_traffic` | [电—交通手册](../modules/ev_power_traffic/ev_power_traffic_manual.tex) | `source_equivalent_model.tex` |
| `graph` | [图模块手册](../modules/graph/graph_manual.tex) | 十一章：接口/索引、通用理论、建图拓扑、收缩、计划、串联/悬垂、稠密/稀疏 Kron、恢复、HTTP、验证审计、数值交叉验证 |
| `harmonics_power_flow` | [谐波手册](../modules/harmonics_power_flow/harmonics_power_flow_manual.tex) | `source_equivalent_model.tex` |
| `integrated_energy` | [综合能源手册](../modules/integrated_energy/integrated_energy_manual.tex) | `source_equivalent_milp.tex` |
| `io` | [输入输出手册](../modules/io/io_manual.tex) | 格式字段换算与拒绝条件，不建立独立电气模型 |
| `market` | [市场手册](../modules/market/market_manual.tex) | `source_equivalent_contract.tex` |
| `model` | [模型手册](../modules/model/model_manual.tex) | 有效容量、GFM 参数选择、单位换算；专题审计从属保存 |
| `network_reconfiguration` | [网络重构手册](../modules/network_reconfiguration/network_reconfiguration_manual.tex) | 九章：接口、通用理论、规范投影/设备、可达 MILP、求解证书、兼容 ONR、HTTP、验证审计、数值交叉验证 |
| `optimal_power_flow` | [最优潮流手册](../modules/optimal_power_flow/opf_manual.tex) | 八个实现章节；不复制 `power_models` 公式 |
| `power_flow` | [潮流计算手册](../modules/power_flow/power_flow_manual.tex) | 十一个实现章节；线性 DC 移相式已按现行源码复核 |
| `power_models` | [AML 建模层手册](../modules/power_models/power_models_manual.tex) | `source_equivalent_builders.tex`，本模块唯一 builder 公式源 |
| `reliability` | [可靠性手册](../modules/reliability/reliability_manual.tex) | `source_equivalent_model.tex` |
| `resilience` | [弹性手册](../modules/resilience/resilience_manual.tex) | `source_equivalent_model.tex` |
| `scenario_generation` | [场景生成手册](../modules/scenario_generation/scenario_generation_manual.tex) | `source_equivalent_model.tex` |
| `server` | [服务端手册](../modules/server/server_manual.tex) | ETag、revision、缓存和作业状态契约 |
| `short_circuit` | [短路手册](../modules/short_circuit/short_circuit_manual.tex) | 十五个模块专章加统一工业评价基线：对称分量/IEC/换流器理论、AC 概览与详细序网、DC 故障、投影身份、稀疏批量、完整 I/O、HTTP/GUI、验证架构、数值交叉验证与深度审计 |
| `sppt` | [SPPT 手册](../modules/sppt/sppt_manual.tex) | `source_equivalent_relations.tex` |
| `time_series` | [时序手册](../modules/time_series/time_series_manual.tex) | `source_equivalent_model.tex` |
| `validation` | [静态校验手册](../modules/validation/validation_manual.tex) | 实际谓词、阈值和四级过滤 |

## 覆盖判据

- 唯一目录：不存在大小写、旧名称或专题名称构成的第二模块目录。
- 唯一主手册：模块 README 明确指定一个主 `.tex`；章节文件只能由该主手册引用。
- 源码等价：公式邻近位置必须给出实现函数及行号，并记录硬编码阈值、截断和分支。
- 无推测补模：未实现的理论通式、规划约束和教材模型不得写入当前实现手册。
- 诚实边界：未消费字段、回退、近似、缺少测试和已知代码问题必须显式披露。

## 跨目录实现

`projection`、`assembly` 不是 `src/` 顶层目录：前者实现位于 `src/model/`，后者实现位于
`src/power_flow/`，分别归入 `model` 和 `power_flow` 手册。`scenario_generation`、
`short_circuit` 的部分公共头位于 `include/hacdcpf/analysis/`，但其源码所有权仍按 `src/` 模块归档。

更新文档时必须同步检查 [模块索引](../modules/README.md)、[文档分类台账](document_catalog.md) 和本表。
