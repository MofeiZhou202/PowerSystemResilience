# scenario_generation 模块技术手册

本目录是 `src/scenario_generation/` 的唯一模块文档目录。

本目录对应源码 `src/scenario_generation/`，说明范围为常规、可靠性、弹性与台风场景。公共头文件位于
`include/hacdcpf/analysis/scenario_generation.hpp`、`typhoon_resilience.hpp` 和
`typhoon_traffic_impact.hpp`。

- 主文档：[scenario_generation_manual.tex](scenario_generation_manual.tex)（16 个专章，56 页 PDF）
- 理论层：[概率与条件分布](chapters/theory_probability_scenarios.tex)、[随机过程](chapters/theory_stochastic_processes.tex)、[场景缩减](chapters/theory_scenario_reduction.tex)
- 三族实现：[常规](chapters/regular_profiles_climate.tex)、[可靠性](chapters/reliability_contingencies.tex)、[弹性目录](chapters/resilience_catalog.tex)
- 灾害链：[风雨](chapters/typhoon_track_wind_rain.tex)、[易损/故障/修复](chapters/fragility_fault_repair.tex)、[交通](chapters/traffic_coupling.tex)
- 缩减实现：[混合 k-medoids](chapters/clustering_implementation.tex)
- 公共契约：[全部选项/结果](chapters/options_results.tex)、[JSON/HTTP/工作簿](chapters/json_http_workbook.tex)
- 验证：[验证架构](chapters/validation_architecture.tex)、[数值交叉验证](chapters/numerical_validation.tex)、[审计准入](chapters/audit_admission.tex)
- 统一样式：`../../_manual_common/hysim_manual.sty`
- 工业评价基线：`../../_manual_common/industrial_evaluation.tex`
- 总导航：[模块手册索引](../README.md)

在本目录执行两遍 `xelatex scenario_generation_manual.tex` 生成目录和交叉引用。公式、默认值、结果口径和限制必须随
公开头文件、实现与注册测试同步更新；PDF 与 LaTeX 辅助文件不作为规范源提交。

固定复核案例由 `tools/scenario_generation_review_case.cpp` 驱动，独立 oracle 为
`tests/scenario_generation_cross_validation.py`；证据保存在
`external_data/scenario_generation_validation/`。该证据验证概率守恒、4096 步 AR(1)、
128→12 聚类逐成员复算、Holland/雨量/易损公式和交通递推；同时保留 hybrid 尾部覆盖提升但
transport/regime 指标变差的负面取舍。它不等价于气象预报、二维水动力或外部引擎全模型认证。
