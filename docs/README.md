# HySim-XJTU-HRPES 文档中心

最后核实：2026-09-10

[市场求解性能与资源对照](modules/market/performance.md)：水量/储能/开停机分项对照、
零成本开停机/小时储能方向等价投影、同初态恢复实验并行、完整周任务复测、实际输入参数对照与分段计时。
新增装配模板与连续系数存储、原矩阵逐元素核对，以及原装配/缓存装配完整七日对照。
SCED到LMP同日有序矩阵派生、首次/重复各五次的浏览器验收协议和稀疏分解诊断见同页；
保留独立原装配比较及3247054行完整对偶核验，不改变储能/水库承接。
原因分析支持异常触发、历史日补算及恢复价格范围；求解器计时细分见同一性能契约。
定价一致性采用固定算法与完整行对偶独立复算，失败阻止价格与结算；百万列Gurobi
定价使用固定8线程障碍法，其余使用单线程对偶单纯形。
新策略的数值门槛、版本范围及完整七日复测记录见上述性能契约。
2000节点新增独立原LP下界、整数候选修复、SCED精确比较/条件派生及索引/输出优化；
SCED→LMP派生后首次/重复各五次完整浏览器最大58.52/58.57 s，十次均低于一分钟，
不作任意边界硬截止保证。原矩阵/完整响应对照、跨进程全对偶、分解诊断和新版8107入口见性能记录。

[电力市场分步操作教程](guides/market_simulation_workflow.zh.md)：单日入门、周月运行、故障/来水对比、
云南调频、实时滚动及通用AC/DC研究；提供按钮顺序、成功标志、模型分工和故障处理。

[周月跨日状态继承](modules/market/southern_execution_contract.md)：
机组连续开停机时长、储能SOC、水位与下泄历史的字段、单位、递推和失败不推进规则；
区分每日SOC终值、次日峰谷参考与完整七日联合优化。上述教程、契约和性能记录已列入
GUI“帮助”目录，可搜索“跨日继承”“SOC”“性能”；在线直接读取本地`docs/`，无需复制正文。

[电力市场系统设计](modules/market/system_design.md)：将业务操作、市场品种、实验方式与
计算阶段分开，明确七工作区、输入版本、执行依赖、图形分析和验收路径。
文档为目标设计；现有GUI/API行为仍以模块执行契约为准。

[智能仿真标签与价格代理](modules/market/intelligent_simulation.md)：完整周Oracle、LMP标签、两轮独立周验证、压力场景代理及分组基线负结果。

[周尺度市场边界优化模型](theory/market_boundary_optimization_model.md)：定义代理模型与
Bayesian Optimization/CMA-ES 使用的随机双层问题、输入输出、风险目标、标签口径和 Oracle 复核边界。

[组内与过渡区域采样理论](theory/market_surrogate_sampling_design.md)：参数规划切换结构、边界动作配对、误差/成本预算、主动学习与独立留出设计；属于下一轮理论方案，尚未执行新采样。

[通用AC/DC混合市场引擎运行时契约](reference/market_simulation_runtime.md)：
报价→SCUC→固定组合SCED/LMP→可选LODF N-1割→非线性AC/DC认证→结算→实时双结算→重复博弈；
DC电压线性化、双向换流器与DC储能跨期SOC，external_grid/energy_router显式退回，
model_scope/model_limitations 随结果返回。

[AEMO官方数据与市场可证伪验证](modules/market/aemo_validation.md)：108项实验检查、
256组合UC穷举、独立水量/SOC/价量账本、实时状态及12类变异；完整功能覆盖矩阵明确
四个AC失败场景、顺序调频容量不足、FCAS字段差异和缺少真实网络/计量的实证边界。

[报价行为文献与实现审查](modules/market/bidding_behavior_review.md)：核验主体报价、随机场景、
水电跨期价值及无真实申报数据时的验证边界；包含2024水电运营实务全文依据。

[实际报价对照实验](modules/market/empirical_bidding_analysis.md)：AEMO七天真实申报、
27组本模块配对出清与独立oracle、9项数据/数学测试、报价及价格曲线；区分跨市场实验与本地校准。

[南方实时市场规则、数学模型与GUI契约](modules/market/southern_real_time.md)
覆盖第3章实时边界、5分钟出清、15分钟定价及滚动状态；规则解释与外部运营限制明确记录。

[云南辅助服务规则与调频衔接](modules/market/yunnan_ancillary_markets.md)
记录两份正式细则原件、日前/日内固定UC与一次调频、静态水电允许区间、独立储能/负荷排他、
安全调整、AGC计量、月分摊及更正账本；含GUI、手算/反例与IEEE118验证范围，区分外部认证事项。

[市场故障 / 来水对比试验](modules/market/southern_execution_contract.md#faultinflow-study)
记录独立日/周/月场景、五环节共享结果、交流事后校核、条件性研究账本与Canvas曲线联动。

[市场全设备图形化周计划](modules/market/southern_execution_contract.md#weekly-plan-results)
记录六类总览、672点热力图/设备曲线、备用与负载率口径、CSV和图形联动验证。

[IEEE118 水火风光储内置案例](modules/market/southern_execution_contract.md#mixed-ieee118)
记录资源配置、7日/28日/联合概率测试及交流安全和通用结算覆盖缺口。

市场运行模拟的四步操作流程、可配置火电数量、逐时PTDF查询、Gurobi/HiGHS/Native
对比、D+1预测联动、2000节点等价紧凑建模与性能实测见
[南方执行契约](modules/market/southern_execution_contract.md)。
IEEE 118多资源系统测试、一周边界压力测试及固定15分钟出清下的预测分辨率影响
也记录在该契约；30/60分钟计算步长尚未实现。
Native固定整数LP修复、GAP退出条件及IEEE118恢复出清的证据见同一契约的
[调试记录](modules/market/southern_execution_contract.md#native-fixed-integer-repair-debug)。

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
| 通用 AC/DC 混合出清、实时双结算与重复博弈 | 契约 | [市场模拟运行时契约](reference/market_simulation_runtime.md) |
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

完整清单、源码映射和统一编译方法见 [模块手册索引](modules/README.md)。

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
