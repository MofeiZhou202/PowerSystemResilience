# scenario_generation 模块技术手册

本目录是 `src/scenario_generation/` 的唯一模块文档目录。

本目录对应源码 `src/scenario_generation/`，说明范围为常规、可靠性、弹性与台风场景。公共头文件位于
`include/hacdcpf/analysis/scenario_generation.hpp`、`typhoon_resilience.hpp` 和
`typhoon_traffic_impact.hpp`。

- 主文档：[scenario_generation_manual.tex](scenario_generation_manual.tex)
- 源码等价数学章：[chapters/source_equivalent_model.tex](chapters/source_equivalent_model.tex)
- 统一样式：`../../_manual_common/hysim_manual.sty`
- 工业评价基线：`../../_manual_common/industrial_evaluation.tex`
- 总导航：[模块手册索引](../README.md)

在本目录执行两遍 `xelatex scenario_generation_manual.tex` 生成目录和交叉引用。公式、默认值、结果口径和限制必须随
公开头文件、实现与注册测试同步更新；PDF 与 LaTeX 辅助文件不作为规范源提交。
