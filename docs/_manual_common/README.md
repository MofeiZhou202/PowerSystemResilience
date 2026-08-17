# 模块技术手册共享说明（HySim-XJTU-HRPES）

本目录保存 18 个源码模块技术手册的**共享排版样式** `hysim_manual.sty`，
版式规范来源为既有的两部标杆手册：

- `docs/PowerFlow/power_flow_manual1.tex`（潮流计算技术手册）
- `docs/OptimalPowerFlow/opf_manual.tex`（最优潮流技术手册）

两部标杆手册把导言区（参数表环境 `paramtable`、字段宏 `\fld`、源码路径宏
`\srcpath`、元信息宏 `\compmeta`）内嵌在各自主文件中。为保证 18 个新增模块
手册版式**逐字节一致**，此处将该导言区抽取为唯一规范的样式包
`hysim_manual.sty`，并在每个模块文件夹内保留一份逐字节相同的副本，使每个
文件夹都能“整包复制到纯 ASCII 路径后独立编译”。

## 覆盖的模块手册（与 `src/` 模块一一对应）

| 文件夹 | 源码模块 | 对应 `src/` 目录 |
|---|---|---|
| `docs/Analysis/` | 承载力·薄弱环节·反事实规划 | `src/analysis/` |
| `docs/Api/` | 公共门面与能力查询 | `src/api/` |
| `docs/CarbonAnalysis/` | 碳流追踪·年度碳·GEC | `src/carbon_analysis/` |
| `docs/Dynamics/` | 机电暂态 DAE | `src/dynamics/` |
| `docs/EvPowerTraffic/` | 电-交通耦合 A–H | `src/ev_power_traffic/` |
| `docs/Graph/` | 图建模·拓扑·降阶 | `src/graph/` |
| `docs/HarmonicsPowerFlow/` | 谐波潮流 | `src/harmonics_power_flow/` |
| `docs/IntegratedEnergy/` | 园区综合能源 MILP | `src/integrated_energy/` |
| `docs/IO/` | 数据接口与格式适配 | `src/io/` |
| `docs/Market/` | 电力市场仿真 | `src/market/` |
| `docs/NetworkReconfiguration/` | 网络重构 | `src/network_reconfiguration/` |
| `docs/PowerModels/` | MIPSolvers AML 建模层 | `src/power_models/` |
| `docs/Reliability/` | 可靠性评估 | `src/reliability/` |
| `docs/Resilience/` | 弹性恢复 | `src/resilience/` |
| `docs/ScenarioGeneration/` | 场景生成·台风 | `src/scenario_generation/` |
| `docs/ShortCircuit/` | 短路计算 | `src/short_circuit/` |
| `docs/SPPT/` | 语义保持投影理论验证层 | `src/sppt/` |
| `docs/TimeSeries/` | 时序潮流·年度生产·全生命周期 | `src/time_series/` |

## 每部手册的统一结构与写作规范

与两部标杆手册一致，每节遵循：

> 功能定位 → 数学模型/算法（公式逐条注明源码出处 文件:行号）→
> 参数表（字段名｜符号｜单位｜典型范围｜默认值｜说明，六列）→
> 验证与校核 → 边界与局限（如实标注近似/fallback/未实现项）。

写作硬约束（继承仓库文档政策）：

- 全部模型、参数、结论以源码为准（`include/`、`src/`、`tests/`）；
- 每个公开结果向量必须声明索引空间与单位；
- 每个近似/回退/时限/模型覆盖不足必须写入 `model_scope`／`ValidityFlags`／
  `model_limitations` 的对应说明；
- 不提交渲染 PDF、`.aux`、`.log` 等构建产物。

## 本地编译（路径须为纯 ASCII）

把某个模块文件夹整包复制到纯英文路径（如 `C:\latex_build\`），执行两遍：

```
xelatex <module>_manual.tex
xelatex <module>_manual.tex
```

第一遍排版、第二遍生成目录与交叉引用。生成后进行页面渲染检查；不要提交
PDF、`.xdv` 或辅助文件。

## 当前状态（首版骨架）

18 部手册均已建立**源码支撑的首版骨架**（功能定位、架构与入口族、核心数学
模型概形、关键选项/结果参数表、验证与边界局限）。后续加深方向见仓库
`docs/module_documentation_map.md` 与本次整理的问题卡：逐条补齐“文件:行号”
级公式出处、拆分为 `chapters/` 多章结构、并补全每个公开结构体的全字段参数表。
