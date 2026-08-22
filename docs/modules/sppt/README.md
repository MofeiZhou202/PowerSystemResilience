# sppt 模块技术手册

本目录是 `src/sppt/` 的唯一模块文档目录。

本目录对应 `src/sppt/` 与 `include/hacdcpf/sppt/`。十一章主手册覆盖语义保持投影理论、MR1--MR7
及 MR3d/MR3t 的源码等价关系、证书与独立残差、三门守卫、代理权限、固定种子故障活动、Wilson
统计、基准/消融、完整数值表、失败证据、验证准入和统一工业评价基线。

SPPT 是可执行验证层，不是物理求解器。只有 `observation=Observed` 的 `passed` 才可解释；
`NotObserved` 和 `supported=false` 都表示证据不可得，不是通过或零残差。当前没有公共 `mr8_*` API，
MR8 只由注册 DAE 适定性测试在声明工作点执行。

当前证书语料的独立作者方程覆盖 6/10 案例；其余 4 个 rich-macro 案例为 unsupported，不能按通过计。
固定种子故障活动在 repetitions=5 时完成 315 个基础样本，但 repetitions=10/20 会因未捕获
`std::invalid_argument` 终止，因此统计证据仍为探索性。

- 主文档：[sppt_manual.tex](sppt_manual.tex)
- 统一样式：`../../_manual_common/hysim_manual.sty`
- 工业评价基线：`../../_manual_common/industrial_evaluation.tex`
- 总导航：[模块手册索引](../README.md)

编译与验收方法见 [LaTeX 文档说明](../../latex/README.md)。公式、默认值、结果口径和限制必须随
公开头文件、实现与注册测试同步更新；`output/pdf/sppt_manual.pdf` 是可阅构建产物，规范源仍是
本目录的 LaTeX 文件。
