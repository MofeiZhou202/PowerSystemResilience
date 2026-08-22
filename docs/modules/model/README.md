# model 模块技术手册

本目录是 `src/model/` 的唯一模块文档目录。

| 文档 | 用途 |
|---|---|
| [model_manual.tex](model_manual.tex) | 十一章主手册：类型化身份、量纲/标幺、商图与可恢复性理论，组件族、canonical 映射、独立 oracle、验证准入与工业评价 |
| [component_models_math_audit.tex](component_models_math_audit.tex) | 历史元件模型数学审计；仅作修复证据，不是当前规范 |

主手册覆盖 `include/hacdcpf/model/`、`src/model/` 与公共 `projection/` 头。它按组件族和下游消费语义
组织 POD，而不把“字段存在”误写成“所有工作流均完整消费”。稳定组件 `index`、vector position 与
graph node/edge index 始终分离，AC/DC 同号母线必须使用 domain-qualified 映射。

物理求解方程分别归 `power_flow`、`optimal_power_flow`、`dynamics` 等模块手册。理论章给出工程模型到
规范模型投影的一般框架；不对应当前实现的扩展均显式标记。结果恢复按 Strong、Approximate、
AuditOnly 或 Unsupported 说明，不以零值伪造未知结果。

注册的 `model_projection_cross_validation` 独立检查商类成员、广延量守恒、强度量广播、AC/DC 同号
身份、dead-island 位置恢复和欧姆--标幺公式。该 oracle 同时发现稀疏 DC bus ID `(1,2,5)` 的
`dc_dead_bus_indices` 当前报告位置 token `3` 而非稳定 ID `5`；位置恢复通过，但稳定身份契约未闭环。

编译与验收方法见 [LaTeX 文档说明](../../latex/README.md)。
`output/pdf/model_manual.pdf` 是可阅构建产物，规范源仍是本目录的 LaTeX 文件。
