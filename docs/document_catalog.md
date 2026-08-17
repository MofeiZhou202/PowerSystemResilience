# 文档分类台账

最后核实：2026-08-18

本台账给 `docs/` 下 Markdown 资产指定唯一的**主要归属**。文件保留原路径以维持链接和历史可追溯性；
分类入口形成新的维护层级。一个文档可由其他分类交叉引用，但只在本表登记一个主要归属。

## 根目录 Markdown 归属

| 主要分类 | 文件 |
|---|---|
| 仓库总览 | `README.md`、`document_catalog.md`、`development_status.md`、`module_documentation_map.md`、`case_catalog.md` |
| 用户指南 | `parameter_system.md`、`distribution_parameter_completion_standards.md`、`reliability_calculation_workflow_and_case_guide.md`、`ev_traffic_scenario_format.md` |
| 开发者指南 | `commenting_guide.md`、`data_structure_api_contract.md`、`data_structure_design_review.md`、`module_data_structures.md`、`model_data_semantics_contract.md`、`projection_and_results.md`、`graph_runtime_contract.md`、`gui_canvas_runtime.md` |
| 理论与模型 | `annual_simulation_models.md`、`capacity_analysis_implementation.md`、`carbon_analysis_contract.md`、`certified_restoration_runtime.md`、`integrated_energy_contract.md`、`market_simulation_mathematical_models.md`、`multidimensional_weak_link_identification.md`、`network_reconfiguration_models.md`、`pv_pq_switching_contract.md`、`reliability_assessment_models.md`、`reliability_mathematical_models_and_intelligent_cyber_physical_assessment.md`、`scenario_generation_contract.md`、`sequential_production_simulation_rich_models.md`、`short_circuit_rich_acdc_derivation.md`、`time_series_power_flow_models.md`、`transient_runtime.md`、`vsc_limit_ncp_power_flow_contract.md` |
| API 与数据参考 | `runtime_api.md`、`python_api.md`、`digital_twin_data_io_architecture.md`、`cim_cgmes3_crosswalk.md`、`bpa_dsp_component_mapping.md`、`svg_distribution_import.md`、`market_simulation_runtime.md` |
| 测试与验证 | `module_code_audit.md` |
| 部署与运维 | `cross_platform_build.md`、`trial_edition_design.md` |
| 模块运行契约 | `sppt_runtime_contract.md` |

## 子目录 Markdown 归属规则

| 路径规则 | 主要分类 | 入口/说明 |
|---|---|---|
| `overview/**/*.md` | 仓库总览 | [总览](overview/README.md) |
| `guides/**/*.md` | 用户指南 | [用户指南](guides/README.md) |
| `developer/**/*.md` | 开发者指南 | [开发者指南](developer/README.md) |
| `theory/**/*.md` | 理论与模型 | [理论与模型](theory/README.md) |
| `modules/**/*.md` | 模块文档 | [模块手册](modules/README.md) |
| `reference/**/*.md` | API 与数据参考 | [API 与数据参考](reference/README.md) |
| `tutorials/**/*.md` | 示例与教程 | [示例与教程](tutorials/README.md) |
| `validation/**/*.md`、`harmonics_analysis/**/*.md` | 测试与验证 | [测试与验证](validation/README.md) |
| `operations/**/*.md` | 部署与运维 | [部署与运维](operations/README.md) |
| `research/**/*.md` | 研究资料 | [研究资料](research/README.md) |
| `planning/**/*.md` | 规划与演进 | [规划与演进](planning/README.md) |
| `_manual_common/**/*.md` | LaTeX 手册维护 | [_manual_common 说明](_manual_common/README.md) |
| `Analysis/README.md`、`Api/README.md`、`CarbonAnalysis/README.md`、`Dynamics/README.md`、`EvPowerTraffic/README.md`、`Graph/README.md`、`HarmonicsPowerFlow/README.md`、`IntegratedEnergy/README.md`、`IO/README.md`、`Market/README.md`、`NetworkReconfiguration/README.md`、`PowerModels/README.md`、`Reliability/README.md`、`Resilience/README.md`、`ScenarioGeneration/README.md`、`ShortCircuit/README.md`、`SPPT/README.md`、`TimeSeries/README.md` | 模块文档 | [模块手册索引](modules/README.md) |
| `archive/**/*.md` | 历史归档 | [归档索引](archive/README.md)；不得作为当前行为证据 |
| `latex/paper/**/*.md` | 论文伴随材料 | [论文工作区索引](latex/paper/README.md)；由每篇论文 README 管理 |
| `latex/**/*.md`（不含 `paper/`） | LaTeX 研究资料 | [LaTeX 文档说明](latex/README.md) |

## LaTeX 归属

- 工业手册：`ComponentModels/`、`PowerFlow/`、`OptimalPowerFlow/` 以及
  [18 个模块目录](modules/README.md)。
- 研究论文：`latex/paper/`，由 [论文工作区索引](latex/paper/README.md) 逐篇登记。
- 专题长文、章节片段与生成证据：`latex/`，由 [LaTeX 文档说明](latex/README.md) 分类。
- 历史 LaTeX 审计：`archive/`，由 [归档索引](archive/README.md) 管理。

## 防孤儿规则

新增 Markdown 必须满足下列至少一项：列入根目录归属表；位于已登记路径规则之下；由模块或论文
README 明确引用。新增主 LaTeX 必须登记到模块索引、论文索引或 LaTeX 分类表。文档检查应报告
不满足这些条件的文件。
