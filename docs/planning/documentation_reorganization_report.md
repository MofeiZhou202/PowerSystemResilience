# 文档重组、源码等价审计与问题卡报告

最后核实：2026-08-18

## 文档整理总览

本轮将文档所有权统一为 `src/<module>/` 对应 `docs/modules/<module>/`。当前两侧顶层目录集合均为
23 个且完全相同；旧大驼峰目录、`harmonics_analysis` 及旧 `docs/validation/README.md` 已停止作为
活文档入口。每个模块目录都有一个中文 `README.md` 和一个主 LaTeX 手册，章节只能从属于该主手册。

数学文档执行源码等价规则：只记录当前代码实际执行的变量、赋值、残差、目标、约束、阈值、截断和
条件分支；公式邻近位置绑定实现函数及行号；教材通式、理想化补模和未核验推导不得进入主手册。
`optimal_power_flow` 中重复的 AML builder 章节已删除，全部 builder 公式只在 `power_models` 手册维护。

## 新增目录结构

```text
docs/
├── README.md                 唯一总导航
├── overview/                 仓库总览
├── guides/                   用户指南
├── developer/                开发者指南
├── theory/                   理论与模型入口
├── modules/                  23 个源码模块的唯一手册目录
├── reference/                API 与数据参考
├── tutorials/                示例与教程
├── testing/                  测试与验证入口
├── operations/               部署与运维
├── research/                 研究资料入口
├── planning/                 规划与问题卡
├── archive/                  历史材料，不作为当前行为证据
├── latex/                    研究论文、专题材料与生成证据
└── _manual_common/           模块手册共享样式与工业评价章节
```

## Markdown 归类结果

Markdown 已完成物理归类，`docs/` 根目录仅保留总入口 `docs/README.md`。当前分类文件清单如下：

| 分类目录 | 已归类文件 |
|---|---|
| `overview/` | `README.md`、`case_catalog.md`、`development_status.md`、`document_catalog.md`、`module_documentation_map.md` |
| `guides/` | `README.md`、`distribution_parameter_completion_standards.md`、`ev_traffic_scenario_format.md`、`parameter_system.md`、`reliability_calculation_workflow_and_case_guide.md` |
| `developer/` | `README.md`、`commenting_guide.md`、`data_structure_api_contract.md`、`data_structure_design_review.md`、`graph_runtime_contract.md`、`gui_canvas_runtime.md`、`model_data_semantics_contract.md`、`module_data_structures.md`、`projection_and_results.md` |
| `theory/` | `README.md`、`annual_simulation_models.md`、`capacity_analysis_implementation.md`、`carbon_analysis_contract.md`、`certified_restoration_runtime.md`、`integrated_energy_contract.md`、`market_simulation_mathematical_models.md`、`multidimensional_weak_link_identification.md`、`network_reconfiguration_models.md`、`pv_pq_switching_contract.md`、`reliability_assessment_models.md`、`reliability_mathematical_models_and_intelligent_cyber_physical_assessment.md`、`scenario_generation_contract.md`、`sequential_production_simulation_rich_models.md`、`short_circuit_rich_acdc_derivation.md`、`sppt_runtime_contract.md`、`time_series_power_flow_models.md`、`transient_runtime.md`、`vsc_limit_ncp_power_flow_contract.md` |
| `reference/` | `README.md`、`bpa_dsp_component_mapping.md`、`cim_cgmes3_crosswalk.md`、`digital_twin_data_io_architecture.md`、`market_simulation_runtime.md`、`python_api.md`、`runtime_api.md`、`svg_distribution_import.md` |
| `testing/` | `README.md`、`module_code_audit.md` |
| `operations/` | `README.md`、`cross_platform_build.md`、`trial_edition_design.md` |
| `tutorials/`、`research/`、`planning/` | 各目录保留各自 `README.md` 与已登记的专题材料 |
| `modules/<module>/` | 23 个模块 README；谐波材料统一归入 `modules/harmonics_power_flow/` |
| `archive/`、`latex/`、`_manual_common/` | 历史归档、论文/专题材料和手册维护材料，均有对应入口 |

分类后的本地文档链接已执行语法感知扫描：共检查 291 个 Markdown、LaTeX、图片和 PDF 目标，缺失链接为 0。

模块目录严格为：

```text
analysis api carbon_analysis dynamics ev_power_traffic graph
harmonics_power_flow integrated_energy io market model
network_reconfiguration optimal_power_flow power_flow power_models
reliability resilience scenario_generation server short_circuit sppt
time_series validation
```

## 已归类文件列表

Markdown 的分类归属记录在 [文档分类台账](../overview/document_catalog.md)：

- 仓库总览：`README.md`、`development_status.md`、`case_catalog.md`、模块地图和分类台账；
- 用户指南：参数系统、配网参数补全、可靠性工作流、电动汽车场景格式；
- 开发者指南：数据结构、模型语义、投影恢复、图运行时、GUI Canvas、注释规范；
- 理论与模型：年度仿真、碳、市场、可靠性、弹性、短路、时序、承载力和 VSC 限制契约；
- API 与数据参考：HTTP、Python、CIM、BPA/DSP、SVG、数字孪生和市场运行时；
- 测试与验证：开发状态、模块代码审计、谐波验证、模块手册验证章节；
- 部署与运维：跨平台构建和版本能力；
- 研究与归档：`latex/`、`latex/paper/` 和 `archive/` 各自索引管理；
- 模块文档：`modules/**/*.md` 由 [23 模块总索引](../modules/README.md) 管理。

## LaTeX 模块文档列表

| 模块 | 主手册 | 源码等价数学归口 |
|---|---|---|
| `analysis` | `modules/analysis/analysis_manual.tex` | `source_equivalent_model.tex` |
| `api` | `modules/api/api_manual.tex` | 能力门控与门面透传 |
| `carbon_analysis` | `modules/carbon_analysis/carbon_analysis_manual.tex` | `source_equivalent_model.tex` |
| `dynamics` | `modules/dynamics/dynamics_manual.tex` | `source_equivalent_solver.tex` |
| `ev_power_traffic` | `modules/ev_power_traffic/ev_power_traffic_manual.tex` | `source_equivalent_model.tex` |
| `graph` | `modules/graph/graph_manual.tex` | `source_equivalent_algorithms.tex` |
| `harmonics_power_flow` | `modules/harmonics_power_flow/harmonics_power_flow_manual.tex` | `source_equivalent_model.tex` |
| `integrated_energy` | `modules/integrated_energy/integrated_energy_manual.tex` | `source_equivalent_milp.tex` |
| `io` | `modules/io/io_manual.tex` | 字段换算、拒绝条件与格式约束 |
| `market` | `modules/market/market_manual.tex` | `source_equivalent_contract.tex` |
| `model` | `modules/model/model_manual.tex` | 有效容量、GFM 参数、投影语义 |
| `network_reconfiguration` | `modules/network_reconfiguration/network_reconfiguration_manual.tex` | `source_equivalent_model.tex` |
| `optimal_power_flow` | `modules/optimal_power_flow/opf_manual.tex` | 八个 OPF 实现章节 |
| `power_flow` | `modules/power_flow/power_flow_manual.tex` | 十一个 PF 实现章节 |
| `power_models` | `modules/power_models/power_models_manual.tex` | `source_equivalent_builders.tex` |
| `reliability` | `modules/reliability/reliability_manual.tex` | `source_equivalent_model.tex` |
| `resilience` | `modules/resilience/resilience_manual.tex` | `source_equivalent_model.tex` |
| `scenario_generation` | `modules/scenario_generation/scenario_generation_manual.tex` | `source_equivalent_model.tex` |
| `server` | `modules/server/server_manual.tex` | revision、ETag、缓存、作业状态 |
| `short_circuit` | `modules/short_circuit/short_circuit_manual.tex` | AC 序网、换流器、DC 故障、IEC 60909 分章 |
| `sppt` | `modules/sppt/sppt_manual.tex` | `source_equivalent_relations.tex` |
| `time_series` | `modules/time_series/time_series_manual.tex` | `source_equivalent_model.tex` |
| `validation` | `modules/validation/validation_manual.tex` | 实际谓词、阈值与层级过滤 |

## Markdown 归类结果

采用“一个主要归属、允许跨分类链接”的规则。根目录契约保留稳定路径，分类 README 只做导航，避免复制
易漂移的运行行为。测试分类统一使用 `docs/testing/`，源码校验模块使用
`docs/modules/validation/`，二者不再共用 `validation` 路径。`harmonic_verification.md` 已归入
`docs/modules/harmonics_power_flow/`，不再保留 `harmonics_analysis` 平行目录。

## 与 PowerFlow、OptimalPowerFlow 风格一致性

23 部手册统一使用 `ctexart`、共享 `hysim_manual.sty`、`\fld` 字段宏、`\srcpath` 源码宏、
参数表和工业评价章节。更重要的是统一了证据标准：

1. `power_flow` 的线性 DC 移相方程已对照现行 `ac_linearized_pf.cpp` 修正为
   `Pspec - p_shift`，并记录非参考节点残差的实际计算范围；
2. `optimal_power_flow` 不再包含 `src/power_models` 的重复公式章节；
3. 场景、可靠性、弹性、短路、谐波、时序等章节均记录当前代码特有的阈值、回退和不对称分支；
4. `server` 的 `max_nodes` 已由错误的 50000 修正为代码实际上限 5000；
5. `model`、`server`、`validation` 不强造电力方程，只形式化其代码实际执行的数据和控制规则。

## 未完全解决的问题

- 源码行号会随代码修改漂移；每次实现变更必须进行语义复核，不能只机械更新行号。
- `component_models_math_audit.tex` 作为历史审计证据保留在 `model` 目录，但不属于第二主手册。
- `archive/` 和 `latex/` 中的历史理论文档可能与现行实现不同，已降级为研究或归档材料，不能作为运行契约。
- 部分根目录旧专题 Markdown 含英文历史正文；本轮未修改其内容，因此未将其伪装为新中文文档。
- 模块手册说明实现，不等同于求解器正确性认证；已披露的代码缺陷仍需由代码修复和回归测试闭环。

## 后续改进方向

- 将 23 模块集合相等、唯一 README、唯一主手册、`\input` 存在性和旧路径扫描接入 CI；
- 为每组数学公式建立“源码函数—文档公式—测试断言”三向追踪表；
- 代码修改时自动提示受影响的模块手册，但禁止自动生成未经人工核验的数学公式；
- 为未被注册测试覆盖的实现分支增加最小可复现算例和独立数值 oracle；
- 持续把历史英文契约迁移为中文，并在迁移时重新核对现行实现。

## 问题卡

### DOC-102

- 问题编号：DOC-102
- 问题标题：重复模块目录与数学模型深度不足
- 当前状态：已完成
- 涉及路径：`docs/modules/`、旧大驼峰目录、`docs/harmonics_analysis/`
- 已完成工作：建立 23 个同名唯一目录，合并谐波验证材料，删除平行活目录。
- 存在问题：历史提交记录仍会显示旧路径，这是必要的迁移追踪。
- 改进方向：在 CI 中比较 `src` 与 `docs/modules` 的目录集合。
- 下一步计划：任何新模块随源码提交同时创建唯一文档目录。
- 风险等级：高风险已关闭。

### DOC-103

- 问题编号：DOC-103
- 问题标题：数学章节未完成源码等价核验
- 当前状态：已完成本轮逐式核验
- 涉及路径：23 部模块主手册及其 `chapters/`
- 已完成工作：新增或替换源码等价章，删除未核验章节，修正移相、可靠性、弹性、短路、场景和服务端错误。
- 存在问题：后续源码修改可能重新引入漂移。
- 改进方向：建立公式、实现函数和测试的三向追踪。
- 下一步计划：实现变更必须同步复核相应公式和行号。
- 风险等级：严重风险降为持续监控。

### DOC-104

- 问题编号：DOC-104
- 问题标题：OPF 内重复维护 power_models 数学规范
- 当前状态：已完成
- 涉及路径：`modules/optimal_power_flow/`、`modules/power_models/`
- 已完成工作：删除 OPF 的 `power_models.tex`，全部引用改到独立 `power_models` 主手册。
- 存在问题：跨模块验证文字仍需随测试变化同步。
- 改进方向：模块主手册只描述自身 `src/<module>` 的数学实现。
- 下一步计划：持续扫描跨目录重复公式。
- 风险等级：高风险已关闭。

### DOC-105

- 问题编号：DOC-105
- 问题标题：23 部 LaTeX 编译与交叉引用验证
- 当前状态：已完成
- 涉及路径：`docs/modules/*/*manual.tex`
- 已完成工作：统一样式路径并修复语法；23 部手册在全新隔离目录中全部生成 PDF，致命错误、未定义引用和 overfull 均为 0。
- 存在问题：编译结果仍依赖可用的 TeX Live/XeLaTeX 环境。
- 改进方向：输出隔离到 `/private/tmp`，禁止提交构建产物。
- 下一步计划：将同一全量编译检查接入 CI。
- 风险等级：低。

### DOC-106

- 问题编号：DOC-106
- 问题标题：导航旧路径与 README 契约漂移
- 当前状态：已完成
- 涉及路径：`docs/README.md`、`docs/modules/README.md`、覆盖地图、分类台账、模块 README
- 已完成工作：权威入口改为 23 个同名目录，修复公共样式、总导航和分类入口相对路径；有效 Markdown 本地链接缺失为 0。
- 存在问题：数学区间等文本可能被朴素正则误判为链接，检查器需解析 Markdown 语法。
- 改进方向：将链接存在性检查接入文档 CI。
- 下一步计划：把语法感知的链接检查接入 CI。
- 风险等级：低。

### DOC-107

- 问题编号：DOC-107
- 问题标题：Markdown 文件物理归类与迁移后链接修复
- 当前状态：已完成
- 涉及路径：`docs/overview/`、`docs/guides/`、`docs/developer/`、`docs/theory/`、`docs/reference/`、`docs/testing/`、`docs/operations/`、`docs/planning/`
- 已完成工作：根目录 Markdown 已迁移到维护分类目录，根目录仅保留 `docs/README.md`；修复总览、台账、模块地图、开发者、可靠性、测试、理论和归档文档中的相对路径。
- 存在问题：历史归档中仍保留部分英文正文和英文交叉引用文字，这是历史材料属性，不作为当前实现契约。
- 改进方向：将语法感知链接检查器接入文档 CI，并逐步翻译历史归档正文。
- 下一步计划：文档新增或移动时同步更新分类台账和入口 README。
- 风险等级：低。

## 问题卡汇总表

| 问题编号 | 问题标题 | 当前状态 | 风险等级 |
|---|---|---|---|
| DOC-102 | 重复模块目录与数学模型深度不足 | 已完成 | 高风险已关闭 |
| DOC-103 | 数学章节未完成源码等价核验 | 已完成本轮核验 | 严重风险降为持续监控 |
| DOC-104 | OPF 内重复维护 power_models 数学规范 | 已完成 | 高风险已关闭 |
| DOC-105 | 23 部 LaTeX 编译与交叉引用验证 | 已完成 | 低 |
| DOC-106 | 导航旧路径与 README 契约漂移 | 已完成 | 低 |
| DOC-107 | Markdown 文件物理归类与迁移后链接修复 | 已完成 | 低 |
