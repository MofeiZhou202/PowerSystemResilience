# 源码模块 LaTeX 技术手册索引

最后核实：2026-08-18

本目录索引 18 个此前缺少独立 LaTeX 说明结构的源码模块。每个模块均保留独立文件夹和主
`.tex`，并与 `PowerFlow`、`OptimalPowerFlow` 的 `ctexart`、参数表、符号、源码引用和诚实结果
口径保持一致。

| 源码模块 | 手册目录 | 主文件 | 主要范围 |
|---|---|---|---|
| `analysis` | `Analysis/` | [分析手册](../Analysis/analysis_manual.tex) | 承载力、薄弱环节、反事实规划 |
| `api` | `Api/` | [API 手册](../Api/api_manual.tex) | C++ 门面、能力门控与异常语义 |
| `carbon_analysis` | `CarbonAnalysis/` | [碳分析手册](../CarbonAnalysis/carbon_analysis_manual.tex) | 碳流、年度碳、用户/节点 GEC |
| `dynamics` | `Dynamics/` | [动力学手册](../Dynamics/dynamics_manual.tex) | 机电暂态 DAE、设备与小信号 |
| `ev_power_traffic` | `EvPowerTraffic/` | [电—交通手册](../EvPowerTraffic/ev_power_traffic_manual.tex) | CTM/LTM、联合优化与 MPC |
| `graph` | `Graph/` | [图模块手册](../Graph/graph_manual.tex) | 拓扑、收缩、降阶与恢复 |
| `harmonics_power_flow` | `HarmonicsPowerFlow/` | [谐波手册](../HarmonicsPowerFlow/harmonics_power_flow_manual.tex) | 频域、三相、NIC 与标准校核 |
| `integrated_energy` | `IntegratedEnergy/` | [综合能源手册](../IntegratedEnergy/integrated_energy_manual.tex) | 园区电—热—氢 MILP |
| `io` | `IO/` | [I/O 手册](../IO/io_manual.tex) | 多格式映射、报告与往返 |
| `market` | `Market/` | [市场手册](../Market/market_manual.tex) | 日前、实时、N-1 与结算 |
| `network_reconfiguration` | `NetworkReconfiguration/` | [网络重构手册](../NetworkReconfiguration/network_reconfiguration_manual.tex) | 混合重构 MILP 与 ONR |
| `power_models` | `PowerModels/` | [建模层手册](../PowerModels/power_models_manual.tex) | AML ACOPF、ACDCOPF、DCOPF、SCUC |
| `reliability` | `Reliability/` | [可靠性手册](../Reliability/reliability_manual.tex) | MC、FMEA、三阶段与 F&D |
| `resilience` | `Resilience/` | [弹性手册](../Resilience/resilience_manual.tex) | 启发式、MILP、MESS 与认证恢复 |
| `scenario_generation` | `ScenarioGeneration/` | [场景生成手册](../ScenarioGeneration/scenario_generation_manual.tex) | 常规、可靠性、弹性与台风 |
| `short_circuit` | `ShortCircuit/` | [短路手册](../ShortCircuit/short_circuit_manual.tex) | IEC 60909、序网、换流器与直流故障 |
| `sppt` | `SPPT/` | [SPPT 手册](../SPPT/sppt_manual.tex) | MR1–MR8、证书、守卫与代理循环 |
| `time_series` | `TimeSeries/` | [时序手册](../TimeSeries/time_series_manual.tex) | UC→OPF→PF、年度与生命周期 |

## 保留的标杆手册

| 类别 | 主文件 |
|---|---|
| 元件模型 | [元件模型技术手册](../ComponentModels/component_models_math_audit.tex) |
| 潮流计算 | [潮流计算技术手册](../PowerFlow/power_flow_manual1.tex) |
| 最优潮流 | [最优潮流技术手册](../OptimalPowerFlow/opf_manual.tex) |

## 统一内容要求

每部手册至少覆盖：模块定位和场景、输入输出与边界、符号和数据结构、数学模型和约束、算法流程、
工程假设、数值稳定性、复杂度、异常工况、模块接口、测试建议、工业指标、适用范围、局限与改进方向。
近似、回退、时限与未覆盖模型必须以 `model_scope`、`ValidityFlags`、`model_limitations` 或等价字段
如实说明。

## 编译与验收

从相应模块目录使用 XeLaTeX 编译两遍，或使用仓库 LaTeX 编译工具。统一命令和验收清单见
[LaTeX 文档说明](../latex/README.md)。构建产物不得作为规范源提交。
