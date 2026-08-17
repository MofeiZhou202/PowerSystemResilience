# short_circuit 模块技术手册

本目录对应源码 `src/short_circuit/`，说明范围为IEC 60909、序网、换流器与直流故障。公共头文件通常位于
`include/hacdcpf/short_circuit/`；若头文件采用仓库的跨模块布局，准确位置以主手册“数据来源”节为准。

- 主文档：[short_circuit_manual.tex](short_circuit_manual.tex)
- 统一样式：`hysim_manual.sty`
- 工业评价基线：`../_manual_common/industrial_evaluation.tex`
- 总导航：[模块手册索引](../modules/README.md)

在本目录执行两遍 `xelatex short_circuit_manual.tex` 生成目录和交叉引用。公式、默认值、结果口径和限制必须随
公开头文件、实现与注册测试同步更新；PDF 与 LaTeX 辅助文件不作为规范源提交。
