# harmonics_power_flow 模块技术手册

本目录合并原 `HarmonicsPowerFlow` 与 `harmonics_analysis` 内容，是 `src/harmonics_power_flow/` 的唯一模块文档目录。

本目录对应源码 `src/harmonics_power_flow/`，公共头文件实际位于
`include/hacdcpf/analysis/harmonics_power_flow.hpp`。说明范围为频域、三相序网、交直流耦合、
Newton 谐波平衡、HSS、电力电子开关函数、频扫、标准评价和跨引擎数值证据。

- 主文档：[harmonics_power_flow_manual.tex](harmonics_power_flow_manual.tex)
- 十个专章：范围架构、通用理论、源码等价线性模型、三相变压器、非线性耦合、HSS 与电力电子、API、指标标准、数值验证、审计准入
- 数值证据：`../../../external_data/harmonics_validation/`
- 统一样式：`../../_manual_common/hysim_manual.sty`
- 工业评价基线：`../../_manual_common/industrial_evaluation.tex`
- 总导航：[模块手册索引](../README.md)

在本目录执行两遍 `xelatex -output-directory=../../../output/pdf harmonics_power_flow_manual.tex`
生成目录和交叉引用。公式、默认值、结果口径和限制必须随
公开头文件、实现与注册测试同步更新；PDF 与 LaTeX 辅助文件不作为规范源提交。
