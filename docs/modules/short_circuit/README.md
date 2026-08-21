# short_circuit 模块技术手册

本目录是 `src/short_circuit/` 的唯一模块文档目录。

本目录对应源码 `src/short_circuit/`，说明范围为 IEC 60909、序网、换流器与直流故障。公共头文件
实际位于 `include/hacdcpf/analysis/short_circuit.hpp` 和
`include/hacdcpf/analysis/dc_short_circuit.hpp`。

- 主文档：[short_circuit_manual.tex](short_circuit_manual.tex)
- 成品 PDF：`../../../output/pdf/short_circuit_manual.pdf`
- 数值证据：IEC TR 60909-4 §6.2 与综合 13 母线网络、50 组 OpenDSS 完整网络、
  IEEE 13/34/123 外部 Thevenin 故障核、35 组 GridLAB-D 5.3.0 平衡分流探针、AC/DC 解析例、
  并联 DCCB 10/5 kA 分流和 1000 母线/64 故障稀疏批量基准；详见
  [数值交叉验证](chapters/numerical_cross_validation.tex)
- 审计范围：公共头、AC/DC 核心实现、canonical 投影、三条生产 HTTP 路由、GUI 消费和注册测试；
  详见 [系统审计](chapters/verification_audit.tex)
- 统一样式：`../../_manual_common/hysim_manual.sty`
- 工业评价基线：`../../_manual_common/industrial_evaluation.tex`
- 总导航：[模块手册索引](../README.md)

在本目录执行两遍 `xelatex short_circuit_manual.tex` 生成目录和交叉引用。公式、默认值、结果口径和限制必须随
公开头文件、实现、生产路由与注册测试同步更新。PDF 是便于交付的构建产物，LaTeX 源仍是规范源。
