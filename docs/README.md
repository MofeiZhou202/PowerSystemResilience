# HySim-XJTU-HRPES 文档中心

最后核实：2026-08-22

本文件是仓库文档的唯一导航入口。运行行为以 `include/`、`src/`、
`tests/run_gui_server.cpp`、`web/` 与已注册测试为准；文档用于解释实现，不覆盖实现。

本次整理采用**非破坏式分类**：现有契约、论文和测试引用仍保留原路径，新的分类目录提供
稳定入口与归属台账。这样既建立层级，又避免搬移文件造成的链接失效。逐文件归属见
[文档分类台账](overview/document_catalog.md)，模块覆盖状态见
[模块文档地图](overview/module_documentation_map.md)。

## 文档层级

| 层级 | 入口 | 用途 |
|---|---|---|
| 仓库总览 | [总览](overview/README.md) | 项目范围、能力边界、当前状态与阅读路径 |
| 用户指南 | [用户指南](guides/README.md) | 算例、参数、工作流、部署与使用说明 |
| 开发者指南 | [开发者指南](developer/README.md) | 架构、数据语义、开发约束与维护流程 |
| 理论与模型 | [理论与模型](theory/README.md) | 数学推导、模型契约与有效边界 |
| 模块文档 | [模块手册](modules/README.md) | 源码模块到中文 LaTeX 技术手册的映射 |
| API 文档 | [API 文档](reference/README.md) | C++ 门面、HTTP、Python 与数据交换契约 |
| 示例与教程 | [示例与教程](tutorials/README.md) | 可复现实例、场景格式与操作路径 |
| 测试与验证 | [测试与验证](testing/README.md) | 测试基线、交叉验证与文档验收 |
| 部署与运维 | [部署与运维](operations/README.md) | 构建、依赖、版本能力与运行边界 |
| 研究资料 | [研究资料](research/README.md) | 论文、理论资料、复现实验与归档 |
| 规划与演进 | [规划与演进](planning/README.md) | 文档缺口、代码审计与受控改进项 |

## 文档等级

| 等级 | 含义 |
|---|---|
| 状态 | 易变的构建、测试、依赖与当前工作证据 |
| 契约 | 当前公共行为、输入输出、限制和失败语义 |
| 实现参考 | 由源码支撑的推导、手册或验证材料；代码变化后需复核 |
| 研究资料 | 论文、理论探索与复现实验，不自动构成运行时承诺 |
| 归档 | 历史审计或旧设计，仅保留决策溯源价值 |

## 首要入口

| 需求 | 等级 | 文档 |
|---|---|---|
| 当前构建、测试、依赖与未闭环工作 | 状态 | [开发状态](overview/development_status.md) |
| 跨平台离线构建与依赖配置 | 契约 | [跨平台构建](operations/cross_platform_build.md) |
| 组件模型与参数 | 实现参考 | [模型手册](modules/model/model_manual.tex) |
| 交直流与三相潮流 | 实现参考 | [潮流计算手册](modules/power_flow/power_flow_manual.tex) |
| AC/DC、混合与三相 OPF | 实现参考 | [最优潮流手册](modules/optimal_power_flow/opf_manual.tex) |
| 灾害恢复、MESS 与动态认证 | 实现参考 | [弹性恢复专著](modules/resilience/resilience_manual.tex) |
| 常规/可靠性/弹性场景与台风交通耦合 | 实现参考 | [场景生成手册](modules/scenario_generation/scenario_generation_manual.tex) |
| 谐波潮流、标准与跨引擎验证 | 实现参考 | [谐波潮流专著](modules/harmonics_power_flow/harmonics_power_flow_manual.tex) |
| 全部源码模块技术手册 | 实现参考 | [模块手册索引](modules/README.md) |
| HTTP 路由与响应边界 | 契约 | [运行时 API](reference/runtime_api.md) |
| 内置算例与能力示例 | 契约 | [算例目录](overview/case_catalog.md) |
| Python 客户端边界 | 契约 | [Python API](reference/python_api.md) |
| 模块文档覆盖与缺口 | 状态 | [模块文档地图](overview/module_documentation_map.md) |
| 代码审计深度与开放问题 | 状态 | [模块代码审计](testing/module_code_audit.md) |

## 工业级 LaTeX 手册

`src/` 的 23 个顶层模块均在 `docs/modules/<module>/` 中具有唯一中文手册目录。原
`ComponentModels`、`PowerFlow`、`OptimalPowerFlow` 的材料已分别归并到 `model`、
`power_flow`、`optimal_power_flow`，不再作为平行活目录。所有数学公式只描述源码实际实现，
并要求实现转写绑定 `文件:函数名` 符号锚点；通用理论必须进入独立理论章，并用实现对应表和
`gapnote` 与当前运行行为隔离。

完整清单、源码映射和编译入口见 [模块手册索引](modules/README.md)，统一编译方法见
[LaTeX 文档说明](latex/README.md)。

## 文档政策

- 更新现有活文档，不新增带日期的审计快照。
- 易变的构建和测试证据只写入 `overview/development_status.md`。
- 路线图、假设、回退和归档材料不得表述为当前实现。
- 所有公共结果向量必须声明索引空间与单位。
- 近似、回退、时限和模型覆盖不足必须声明有效边界及结果标志。
- HTTP 示例必须使用 `tests/run_gui_server.cpp` 中的生产路由。
- LaTeX 只提交可编辑源文件；PDF、`.aux`、`.log` 等构建产物不作为规范源。
- 新增文档必须登记到本索引与 [文档分类台账](overview/document_catalog.md)，不得形成孤儿文件。

历史审计、旧设计与理论资料统一由 [归档索引](archive/README.md) 管理。归档可解释历史决策，
但不得作为当前行为证据。
