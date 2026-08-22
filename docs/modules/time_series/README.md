# time_series 模块技术手册

本目录是 `src/time_series/` 的唯一模块文档目录。

本目录对应源码 `src/time_series/`，说明范围为UC→OPF→PF、年度生产与生命周期。公共头文件通常位于
`include/hacdcpf/time_series/`；若头文件采用仓库的跨模块布局，准确位置以主手册“数据来源”节为准。

- 主文档：[time_series_manual.tex](time_series_manual.tex)
- 编译 PDF：[output/pdf/time_series_manual.pdf](/Users/tianyangzhao/Codes/HybridACDCDistributionSystemsSimulation/output/pdf/time_series_manual.pdf)
- 统一样式：`../../_manual_common/hysim_manual.sty`
- 工业评价基线：`../../_manual_common/industrial_evaluation.tex`
- 总导航：[模块手册索引](../README.md)

主手册分为理论、实现和验证三卷。理论卷包括 `theory_scuc_fundamentals.tex`（ED/UC/SCUC、
启停、爬坡、备用、DC 网络与最优性证书）、`theory_network_storage_flexibility.tex`（多时段
AC/DC、储能、灵活资源和 UC→PF 校核理论）、`theory_decomposition_lifecycle.tex`（年度分解、
抽样、生命周期、碳核算与误差传播）。每章均含推导、定理/证明、适用边界、参考文献和实现对应表。

实现卷按源码责任拆分为：`scope_and_architecture.tex`（范围、数据流与 ID 契约）、
`time_data_and_profiles.tex`（profile、时间步长和单位）、`tspf_pipeline.tex`（UC→OPF→PF）、
`annual_decomposition.tex`（年度方法概览）、`implementation_annual_production.tex`（逐字段、逐分支覆盖
`annual_production_sim.cpp`）、`lifecycle_and_carbon.tex`（生命周期方法概览）、
`implementation_lifecycle_simulation.tex`（逐字段、逐公式覆盖 `lifecycle_simulation.cpp`）、
`api_http_results.tex`（C++/HTTP/结果字段）、`numerical_validation.tex`（Release 数值证据）和
`audit_and_admission.tex`（理论—实现差距与工业准入）。`source_equivalent_model.tex` 保留逐行
MILP/生命周期方程，所有章节均以 `src/`、`include/` 和注册测试为规范源。

本轮 Release 证据还包括 TS-UC-4H 三路独立验证：HySim/HiGHS、SciPy/HiGHS 重建和 256 个承诺
序列穷举的目标均为 9233.35，承诺/出力/SOC 一致；TS-PF-6STEP 对同一 6 步网络快照同时运行
HySim、OpenDSS 和本机 GridLAB-D，OpenDSS 最大电压误差 $6.59\times10^{-10}$ pu，GridLAB-D
最大电压误差 $5.85\times10^{-7}$ pu，6/6 通过。机器可读报告位于
`external_data/time_series_validation/`。既有证据还包括多尺度案例 24/24 与 168/168 时序 PF、
全年度耦合 UC 的 SOC/预算证书、物理 replay 与抽样 PF correction、3 年生命周期 HTTP 结果及
BESS/DC 储能容量扫描。
OpenDSS/GridLAB-D 只验证逐时 PF 快照，不被写成 UC 或生命周期参考解；生命周期 HTTP 默认构造
6 h 输入；schedule-only 现在是显式可选降级，结果声明 asset/grid/storage/DC 碳分项及 fallback 来源；
抽样 bound 仍是确定性分层 proxy 界，不是随机抽样覆盖率或外部生命周期 oracle。

在本目录执行两遍 `xelatex time_series_manual.tex` 生成目录和交叉引用。公式、默认值、结果口径和限制必须随
公开头文件、实现与注册测试同步更新；PDF 与 LaTeX 辅助文件不作为规范源提交。
