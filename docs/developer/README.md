# 开发者指南

本分类面向维护者，聚合稳定架构、身份映射、数据语义与开发规范。

| 主题 | 文档 |
|---|---|
| 数据结构与 API 契约 | [数据结构 API 契约](data_structure_api_contract.md) |
| 数据结构设计审查 | [数据结构设计评审](data_structure_design_review.md) |
| 跨模块数据结构 | [模块数据结构](module_data_structures.md) |
| 模型、数据与结果语义 | [跨模块语义契约](model_data_semantics_contract.md) |
| rich 到 canonical 投影 | [投影与结果恢复](projection_and_results.md) |
| 图与降阶运行时 | [图运行时契约](graph_runtime_contract.md) |
| 注释规范 | [注释指南](commenting_guide.md) |
| 文档维护 | [文档中心政策](../README.md#文档政策) |

开发时必须保持 AC/DC 域限定映射、稳定组件 ID 与向量位置的严格区分，并在结果中显式记录
近似、回退和覆盖不足。
