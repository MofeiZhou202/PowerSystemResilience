# 文档系统化整理与补全文档报告

最后核实：2026-08-18

## 文档整理总览

本次整理对 `docs/` 的 Markdown、工业手册、研究论文、专题 LaTeX、生成证据和历史归档进行了
系统盘点。为保护现有源码、测试、论文 `\input`、图片和外部链接，采用非破坏式重构：保留原路径，
新增分层入口、唯一归属台账和跨目录导航。运行行为仍以源码与注册测试为准。

完成结果：

1. `docs/README.md` 改为全中文唯一入口，并建立 11 类维护层级。
2. `document_catalog.md` 对根目录 Markdown 逐文件归类，对深层 Markdown 按路径规则全量归类。
3. 18 个指定源码模块均具有独立目录、中文 LaTeX 主文档与中文模块 README。
4. 18 部手册统一接入工业评价基线，补齐复杂度、稳定性、异常、接口、验证和指标等横向要求。
5. `module_documentation_map.md` 更新为当前中文覆盖矩阵，不再将已有完整手册标为“分散”。
6. `latex/README.md` 按工业手册、完整论文、专题长文、章节片段和生成证据分类 LaTeX 资产。

## 新增目录结构

```text
docs/
├── overview/       仓库总览
├── guides/         用户指南
├── developer/      开发者指南
├── theory/         理论与模型
├── modules/        模块手册总索引
├── reference/      API 与数据参考
├── tutorials/      示例与教程
├── validation/     测试与验证
├── operations/     部署与运维
├── research/       研究资料
└── planning/       规划、审计与本整理报告
```

18 个模块手册继续使用与既有 `ComponentModels/`、`PowerFlow/`、`OptimalPowerFlow/` 一致的
大驼峰目录命名，以保持现有链接和手册体系兼容。

## 已归类文件列表

根目录所有 Markdown 的逐文件归属见 [文档分类台账](../document_catalog.md)，主要结果如下：

- 仓库总览：文档中心、开发状态、模块覆盖、算例目录。
- 用户指南：参数系统、参数补全、可靠性工作流、电动汽车场景。
- 开发者指南：数据结构、语义契约、投影恢复、图运行时、GUI 和注释规范。
- 理论与模型：潮流/OPF 专项、可靠性、弹性、市场、短路、时序、承载力等数学说明。
- API 与参考：HTTP、Python、数字孪生、CIM、BPA/DSP、SVG 和市场运行时。
- 测试与验证：当前开发基线、模块审计、谐波交叉校核和手册验证章节。
- 部署与运维：跨平台构建、试用版、服务和运行期能力。
- 研究与归档：`latex/paper/` 论文体系、`latex/` 专题材料和 `archive/` 历史资料。

深层论文伴随 Markdown 由每篇论文 README 管理；历史材料由 `archive/README.md` 管理；新增模块
README 由 `modules/README.md` 统一管理。由此不存在未声明用途的 Markdown 路径类别。

## 新增或补齐的 LaTeX 模块文档

| 编号 | 模块 | 主文档 |
|---|---|---|
| 1 | `analysis` | `Analysis/analysis_manual.tex` |
| 2 | `api` | `Api/api_manual.tex` |
| 3 | `carbon_analysis` | `CarbonAnalysis/carbon_analysis_manual.tex` |
| 4 | `dynamics` | `Dynamics/dynamics_manual.tex` |
| 5 | `ev_power_traffic` | `EvPowerTraffic/ev_power_traffic_manual.tex` |
| 6 | `graph` | `Graph/graph_manual.tex` |
| 7 | `harmonics_power_flow` | `HarmonicsPowerFlow/harmonics_power_flow_manual.tex` |
| 8 | `integrated_energy` | `IntegratedEnergy/integrated_energy_manual.tex` |
| 9 | `io` | `IO/io_manual.tex` |
| 10 | `market` | `Market/market_manual.tex` |
| 11 | `network_reconfiguration` | `NetworkReconfiguration/network_reconfiguration_manual.tex` |
| 12 | `power_models` | `PowerModels/power_models_manual.tex` |
| 13 | `reliability` | `Reliability/reliability_manual.tex` |
| 14 | `resilience` | `Resilience/resilience_manual.tex` |
| 15 | `scenario_generation` | `ScenarioGeneration/scenario_generation_manual.tex` |
| 16 | `short_circuit` | `ShortCircuit/short_circuit_manual.tex` |
| 17 | `sppt` | `SPPT/sppt_manual.tex` |
| 18 | `time_series` | `TimeSeries/time_series_manual.tex` |

可靠性、弹性和短路手册按 `chapters/` 拆章；其余采用可独立编译的单文件结构。每个目录均包含
`hysim_manual.sty` 与 README，主文件统一引入 `_manual_common/industrial_evaluation.tex`。

## Markdown 归类结果

分类采用“一个主要归属、允许跨分类引用”的原则。根目录契约保持稳定路径；新的分类 README 是
面向角色和任务的入口，不复制契约正文。该结构降低重复状态和链接漂移风险，同时满足用户、开发者、
研究者和运维人员的不同阅读路径。

## 与标杆手册的风格一致性

18 部手册沿用 `PowerFlow` 与 `OptimalPowerFlow` 的以下规范：

- `ctexart` 中文文档类、A4 页面和统一版心；
- `amsmath`/`amssymb` 数学排版，`booktabs`/`longtable` 参数表；
- `\fld` 字段宏、`\srcpath` 路径宏和 `\compmeta` 数据结构元信息；
- “源码依据—数学模型—参数/结果—验证—限制”的论证顺序；
- 标幺/有名值、稳定 ID、索引空间和 AC/DC 域的明确声明；
- 近似、回退、时限和模型覆盖不足的诚实结果口径。

共享样式只抽取排版与横向评价要求，模块专用公式、约束、算法和边界仍保留在各自正文中。

## 未完全解决的问题

1. 本次重构未物理移动旧契约和论文文件；这是为避免破坏大量相对链接而保留的兼容策略。
2. 部分单文件模块手册尚未像可靠性、弹性和短路一样细分章节；当前篇幅仍可维护。
3. 工业评价基线给出统一指标族，但各模块的现场验收阈值仍需结合企业标准、算例和硬件标定。
4. 论文工作区存在历史编译产物和含空格入口文件；未经作者确认未执行破坏性清理或重命名。
5. 文档内容会随公开结构体和求解器行为变化，需要持续的字段覆盖与链接检查。
6. `docs/Reliability/` 当前存在被 `.gitignore` 排除的本地 PDF 与辅助文件；规范编译输出已隔离到
   `/private/tmp`，本次未擅自删除可能属于用户的本地产物。

## 后续改进方向

- 建立文档 CI：Markdown 链接、LaTeX `\input`、占位符、中文比例和主文件编译检查。
- 为每个公开输入/结果结构生成字段覆盖清单，检测单位、默认值、索引和限制说明漂移。
- 为 18 部手册分别标定工业验收阈值和标准算例，形成模块专用验证矩阵。
- 在不破坏引用的前提下，于自然大修时逐步把旧根目录文档迁入分类目录并保留重定向说明。
- 经论文作者确认后，统一历史论文入口为 `main.tex` 并清理不应跟踪的构建产物。

## 问题卡汇总表

| 问题编号 | 问题标题 | 当前状态 | 涉及路径 | 已完成工作 | 存在问题 | 改进方向 | 下一步计划 | 风险等级 |
|---|---|---|---|---|---|---|---|---|
| DOC-001 | 文档资产与实现基线盘点 | 已完成 | `docs/`、`src/`、`include/`、`tests/` | 盘点文档类型、模块入口、源码和测试依据 | 文档量大且论文资产混杂 | 以台账和模块地图持续维护 | 纳入文档 CI | 中 |
| DOC-002 | 模块手册覆盖一致性 | 已完成 | 18 个模块目录、`_manual_common/` | 独立主文档、README、共享样式与工业评价基线齐备 | 专用字段表仍可能漂移 | 自动生成字段覆盖矩阵 | 逐模块量化验收阈值 | 中 |
| DOC-003 | Markdown 层级与孤儿治理 | 已完成 | `README.md`、`document_catalog.md`、分类入口 | 建立 11 类层级、主要归属和防孤儿规则 | 旧文件未物理搬移 | 自然大修时渐进迁移 | 增加链接/归属检查 | 低 |
| DOC-004 | 模块覆盖地图漂移 | 已完成 | `module_documentation_map.md` | 更新为中文现状，18 模块均为完整手册 | 状态会随源码演进 | 变更时同步三个索引 | 加入维护检查表 | 中 |
| DOC-005 | LaTeX 编译与排版可验证性 | 已完成 | 18 部新增模块手册及共享样式 | Fandol 字体下 XeLaTeX 18/18 成功；0 致命错误、0 未定义引用、0 页面边界越界；代表性页面视觉检查通过 | `Reliability/` 有被忽略的本地产物 | 继续使用隔离输出目录，按需清理本地产物 | 将 18 部编译检查接入 CI | 低 |
| DOC-006 | 研究论文与生成物混杂 | 受控保留 | `latex/paper/`、`latex/sim_results/` | 按篇幅/用途分类并建立论文索引 | 历史入口含空格，存在构建产物 | 作者确认后规范化和清理 | 暂不执行破坏性动作 | 中 |

## 验收标准

- 18 个指定模块均存在目录、主 `.tex`、README 和统一工业评价引用。
- 3 个标杆手册保持原路径并列入总索引。
- 根目录 Markdown 全部逐文件归属；深层 Markdown 全部匹配已登记规则。
- 所有新建或修改的 Markdown 与 LaTeX 文档均使用简体中文叙述。
- 本地链接、LaTeX 输入路径、占位符、差异空白和编译结果均有机器检查记录。

实际验收结果：18 部新增模块手册全部生成 A4 PDF；日志中无致命错误和未定义引用；PDF 文本边界
坐标均位于页面内；新导航失效链接为 0；全部 Markdown 均匹配分类规则；`git diff --check` 通过。
