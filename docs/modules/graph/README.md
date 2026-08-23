# graph 模块技术手册

本目录是 `src/graph/` 的唯一模块文档目录，覆盖混合 AC/DC 图投影、拓扑报告、零阻抗收缩、
串联/悬垂/Kron 降阶、结果恢复及生产 HTTP/GUI 图分析链。

## 受控源文件

- [graph_manual.tex](graph_manual.tex)：主手册与章节编排。
- `chapters/overview_contract.tex`：职责边界、公共入口、单位和索引空间。
- `chapters/theory_graph_foundations.tex`：专著级图论、网络矩阵和 Schur 补理论。
- `chapters/construction_topology.tex`：富模型建图覆盖、岛、径向性、桥、割点和基本圈。
- `chapters/contraction_semantics.tex`：闭合开关/零阻抗并查集收缩及富组件回映射。
- `chapters/reduction_planning.tex`：候选分类、选项生效矩阵和计划语义。
- `chapters/series_pendant_reduction.tex`：串联精确条件与悬垂近似。
- `chapters/source_equivalent_algorithms.tex`：稠密/稀疏 Kron 源码等价算法。
- `chapters/recovery_mapping.tex`：映射索引空间与四类电压恢复。
- `chapters/runtime_http_contract.tex`：生产拓扑/网络化简 HTTP 与 GUI 契约。
- `chapters/verification_audit.tex`：测试矩阵、审计关闭证据和使用判据。
- `chapters/numerical_cross_validation.tex`：Kron、收缩、串联及 PF/OPF round-trip 的数值结果、误差门槛与复现条件。

公共头位于 `include/hacdcpf/graph/`，九个实现文件位于 `src/graph/`；生产路由位于
`tests/run_gui_server.cpp`。较短的 Markdown 契约为
[graph_runtime_contract.md](../../developer/graph_runtime_contract.md)，本手册是完整实现参考。

## 编译与验收

在本目录执行：

```bash
xelatex -interaction=nonstopmode graph_manual.tex
xelatex -interaction=nonstopmode graph_manual.tex
```

PDF、`.aux`、`.log`、`.toc`、`.xdv` 与 SyncTeX 文件是构建产物，不作为规范源提交。修改图语义、
映射、默认值或近似边界后，必须同步核对公共头、九个实现文件、生产路由及四个直接 graph 测试目标。
数值结果章节还必须注明构建提交、依赖提交、测试命令、单位、误差门槛和未覆盖范围。
