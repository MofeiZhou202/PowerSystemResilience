> 本文档为 [README.md](README.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

# 源码模块技术手册索引

最后核实：2026-08-22

本目录与 `src/` 顶层目录执行严格的一一映射：目录名必须与源码模块名完全相同，每个模块只能有一个
文档目录、一个 `README.md` 和一个主手册。章节文件可以位于该模块的 `chapters/`，但不得另建大小写、
缩写或历史名称不同的平行目录。

| 源码模块 | 唯一文档目录 | 主手册 | 实现范围 |
|---|---|---|---|
| `analysis` | `modules/analysis/` | [分析手册](analysis/analysis_manual.tex) | 理论决策模型、承载力/薄弱环节/反事实源码映射与数值证据 |
| `api` | `modules/api/` | [公共 API 手册](api/api_manual.tex) | 能力门控、safe API、句柄签名、OPF 解后审计与验证边界 |
| `carbon_analysis` | `modules/carbon_analysis/` | [碳分析手册](carbon_analysis/carbon_analysis_manual.tex) | 碳流理论、矩阵/比例追踪、年度储能碳递推与交叉验证 |
| `dynamics` | `modules/dynamics/` | [动力学手册](dynamics/dynamics_manual.tex) | 机电暂态 DAE、设备模型、小信号 |
| `ev_power_traffic` | `modules/ev_power_traffic/` | [电—交通手册](ev_power_traffic/ev_power_traffic_manual.tex) | CTM/LTM、联合优化、MPC |
| `graph` | `modules/graph/` | [图模块手册](graph/graph_manual.tex) | 拓扑、收缩、降阶、结果恢复 |
| `harmonics_power_flow` | `modules/harmonics_power_flow/` | [谐波手册](harmonics_power_flow/harmonics_power_flow_manual.tex) | 频域、三相、AC/DC 耦合、标准校核 |
| `integrated_energy` | `modules/integrated_energy/` | [综合能源专著](integrated_energy/integrated_energy_manual.tex) | 十章/23 页：守恒、库存、LP/MILP 对偶，源码等价模型，解析解与 24 h 独立残差 oracle |
| `io` | `modules/io/` | [输入输出手册](io/io_manual.tex) | 格式映射、导入报告、往返 |
| `market` | `modules/market/` | [市场手册](market/market_manual.tex) | SCUC/SCED/LMP、LODF N-1、混合模型边界、结算与数值证据 |
| `model` | `modules/model/` | [工程模型专著](model/model_manual.tex) | 十一章/22 页：类型化身份、量纲/标幺、商图恢复、组件语义、投影 oracle 与稳定 ID 开放缺陷 |
| `network_reconfiguration` | `modules/network_reconfiguration/` | [网络重构手册](network_reconfiguration/network_reconfiguration_manual.tex) | 混合重构 MILP、ONR |
| `optimal_power_flow` | `modules/optimal_power_flow/` | [最优潮流手册](optimal_power_flow/opf_manual.tex) | Native、Parity、DC、RPO、三相 OPF |
| `power_flow` | `modules/power_flow/` | [潮流计算手册](power_flow/power_flow_manual.tex) | AC/DC、三相、配网、CPF、鲁棒求解 |
| `power_models` | `modules/power_models/` | [AML 建模层手册](power_models/power_models_manual.tex) | ACOPF、ACDCOPF、DCOPF、LinDistFlow、SCUC builder |
| `reliability` | `modules/reliability/` | [可靠性手册](reliability/reliability_manual.tex) | MC、FMEA、三阶段、F\&D |
| `resilience` | `modules/resilience/` | [弹性手册](resilience/resilience_manual.tex) | 启发式恢复、严格 MILP、MESS |
| `scenario_generation` | `modules/scenario_generation/` | [场景生成手册](scenario_generation/scenario_generation_manual.tex) | 常规、可靠性、弹性、台风场景 |
| `server` | `modules/server/` | [服务端手册](server/server_manual.tex) | `RuntimeApiV1` 的 revision/ETag/缓存/作业/路由契约与验证边界 |
| `short_circuit` | `modules/short_circuit/` | [短路手册](short_circuit/short_circuit_manual.tex) | IEC 60909、序网、换流器、直流故障 |
| `sppt` | `modules/sppt/` | [SPPT 专著](sppt/sppt_manual.tex) | 十一章/32 页：投影/MR 理论、独立残差、守卫/代理、证书/规模/消融/故障活动及失败边界 |
| `time_series` | `modules/time_series/` | [时序手册](time_series/time_series_manual.tex) | UC→OPF→PF、年度生产、生命周期 |
| `validation` | `modules/validation/` | [静态校验手册](validation/validation_manual.tex) | 谓词理论、Basic/Electrical/SolverReady/Strict 过滤、规则回归与限制 |

## 数学模型规范

1. 实现转写章的公式只允许转写当前源码实际执行的赋值、残差、目标、约束、截断、阈值和条件分支。
2. 每组实现公式必须给出 `文件:函数名` 符号锚点；禁止使用会随源码漂移的行号。
3. `theory_*.tex` 可给出通用理论、推导和证明，但必须有章首 `theorynote`、章末实现对应表，并以
   `gapnote` 标记所有超出当前实现的内容。
4. 代码未消费的字段、近似路径、回退、硬编码阈值和已知缺陷必须在相邻正文中明确声明。
5. `src/power_models/` 的公式只在 `modules/power_models/` 维护；OPF 手册只描述跨模块接口，禁止复制。

完整模板、深度分级与验收规则见 [理论写作指南](THEORY_WRITING_GUIDE.md)。

## 编译与验收

全部主手册共享 `docs/_manual_common/hysim_manual.sty` 和
`docs/_manual_common/industrial_evaluation.tex`。编译方法见
[LaTeX 文档说明](../latex/README.md)，整理与验收结果见
[文档重组报告](../planning/documentation_reorganization_report.md)。
