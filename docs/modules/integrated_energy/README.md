# integrated_energy 模块技术手册

本目录是 `src/integrated_energy/` 的唯一模块文档目录。

本目录对应 `src/integrated_energy/` 与 `include/hacdcpf/integrated_energy/`。十章主手册覆盖
能源枢纽、守恒/库存、LP/MILP 对偶与互斥理论，源码等价模型、完整数据/结果契约，以及解析解、
24 时段独立方程重算和研究准入。

当前实现是孤立园区的电--热--氢--燃料代数优化；`pcc_ac_bus` 仅用于稳定归因，模型不包含
交流无功、电压、支路潮流或安全约束。`feasible` 只证明所建多载能模型有后端原始解，不能解释为
电网可行性证书。理论章中的通用能源枢纽扩展凡超出实现者均以 `gapnote` 标记。

注册的 `integrated_energy_cross_validation` 包含一时段光伏解析解、两时段储能套利解析解和 24 时段
全载能逐式重算。当前证据不包含第二个外部 MILP 全模型 oracle，也不验证 PCC 后的 AC 网络。

- 主文档：[integrated_energy_manual.tex](integrated_energy_manual.tex)
- 统一样式：`../../_manual_common/hysim_manual.sty`
- 工业评价基线：`../../_manual_common/industrial_evaluation.tex`
- 总导航：[模块手册索引](../README.md)

编译与验收方法见 [LaTeX 文档说明](../../latex/README.md)。公式、默认值、结果口径和限制必须随
公开头文件、实现与注册测试同步更新；`output/pdf/integrated_energy_manual.pdf` 是可阅构建产物，
规范源仍是本目录的 LaTeX 文件。
