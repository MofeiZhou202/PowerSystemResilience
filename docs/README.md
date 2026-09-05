# HySim-XJTU-HRPES 文档中心

最后核实：2026-09-05

市场运行模拟的Gurobi/HiGHS选择、D+1预测联动、末日预测编辑和限额求解质量见
[南方执行契约](modules/market/southern_execution_contract.md)。

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
| 南方区域 2025 V1.0 日前 SCUC/SCED/LMP 规则与实现差异 | 规则参考与实现对照 | [市场规则原文与手册](modules/market/README.md)、[逐条对照](modules/market/southern_rules_comparison.md) |
| 南方日前细则2.3/2.4边界目录、每日设备覆盖、预测联合采样、Canvas与ΔP统计 | 契约 | [南方规则执行契约](modules/market/southern_execution_contract.md) |
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

2026-08-23 的系统审计确认 23/23 个源码模块均已在其声明范围内建立理论--实现--数值证据链；
不同模块分别采用外部公共子集交叉验证、独立方程/变形 oracle 或解析校核加内部回归。
动力学新增 MassMatrixDae 下 IEEE 1547 与直接 API 定时限 UVLS/正序 Zone-1
继电器的状态事件定位、固定前向窗口聚类、本地相量 CT/PT 频率/距离测量及事件后
代数残差审计。EMT 测量在当前相量网络显式拒绝。作者 COSMIC 固定公开提交的自带
示例已复现，但按论文图 2 参数重建不产生论文动作序列；该负结果、自动装配缺口
与形式化认证边界均显式保留。混合 AC/DC 初始化现与 DC/DC 稳态端口方程同源，
DER_A 的 7/10 状态、非抗饱和和 COI 频率输入已按 PSD 源码建立方程级门；PSD Test 42
因外部 SciML 环境冲突尚无轨迹级结论。
修正 case33bw AC/DC 研究算例现以可选的 GFL 直流欠压有功降额和定时闭锁形成
DC 故障到 AC 功率/电压的平衡相量可靠性入口路径，并由 C++ 解析测试、1/2/5 ms
步长检查和独立 Julia oracle 交叉验证；反并联二极管馈流、MMC 阀级动态和 DCCB
电弧仍明确不支持，不得据此宣称 EMT 级双向故障传播。
N-k 研究驱动现把 trajectory class 限定为固定进入事件下 DAE 实际传给恢复
MILP 的不可用 VSC 集；AC/DC 负荷与原有 `sP/sQ` 切负荷仍逐状态求解。case33
正式 N-1/N-2/N-3 设计的 552 个条件全部求解，留一入口类准确率为 100%，
两次端到端投影加速为 5.76--5.99 倍；原预注册 20 倍门槛失败，case123 仍待闭环。
case123 的 1050 个条件中有 158 个在 DAE/事件时代数求解阶段失败；进入恢复阶段的
892 个条件均可由原切负荷 MILP 求解。其 class 准确性与 8.81--8.91 倍诊断投影因未解析
质量非零均不准入。
闭环等级、未覆盖模型和外部认证缺口见 [模块文档地图](overview/module_documentation_map.md)，
不得仅按章节文件名或“测试通过”推断能力完整性。

## 文档政策

- 更新现有活文档，不新增带日期的审计快照。
- 易变的构建和测试证据只写入 `overview/development_status.md`。
- 路线图、假设、回退和归档材料不得表述为当前实现。
- 所有公共结果向量必须声明索引空间与单位。
- 近似、回退、时限和模型覆盖不足必须声明有效边界及结果标志。
- HTTP 示例必须使用 `tests/run_gui_server.cpp` 中的生产路由。
- LaTeX 只提交可编辑源文件；PDF、`.aux`、`.log` 等构建产物不作为规范源。
  市场手册保留的官方规则 PDF 为外部规范源附件，按精确路径例外管理，来源与哈希记录在模块入口。
- 新增文档必须登记到本索引与 [文档分类台账](overview/document_catalog.md)，不得形成孤儿文件。

历史审计、旧设计与理论资料统一由 [归档索引](archive/README.md) 管理。归档可解释历史决策，
但不得作为当前行为证据。
