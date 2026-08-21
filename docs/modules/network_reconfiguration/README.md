# network_reconfiguration 模块技术手册

本目录是 `src/network_reconfiguration/` 的唯一模块文档目录，覆盖混合 AC/DC 单时刻拓扑重构、
设备动作回投影、历史 AC ONR 兼容接口和 `/api/session/run_reconfig` 运行时验证链。

## 受控源文件

- [network_reconfiguration_manual.tex](network_reconfiguration_manual.tex)：主手册与章节编排。
- `chapters/overview_contract.tex`：职责边界、公共 API、单位和字段契约。
- `chapters/theory_reconfiguration.tex`：工程参考级通用理论；未实现扩展均显式标注。
- `chapters/canonical_device_semantics.tex`：规范投影、域限定索引和设备能力语义。
- `chapters/source_equivalent_model.tex`：当前可达 MILP 的源码等价公式。
- `chapters/solver_result_contract.tex`：启发式、后端分派、后验证和结果真值表。
- `chapters/compatibility_api.tex`：拓扑查询与历史 `ONRResult` 包装语义。
- `chapters/runtime_http_contract.tex`：生产 HTTP 路由的输入、后处理和响应边界。
- `chapters/verification_audit.tex`：注册测试矩阵、审计发现和限制。
- `chapters/numerical_cross_validation.tex`：6-bus 穷举对照、IEEE 33-bus BFS/PF 结果、跨模块流水线与数值门槛。

公共头文件位于 `include/hacdcpf/network_reconfiguration/`；生产 HTTP 路由位于
`tests/run_gui_server.cpp`。`docs/theory/network_reconfiguration_models.md` 是较短的模型速查，
本手册是完整实现契约。

## 编译与验收

在本目录执行：

```bash
xelatex -interaction=nonstopmode network_reconfiguration_manual.tex
xelatex -interaction=nonstopmode network_reconfiguration_manual.tex
```

也可使用仓库 LaTeX 编译脚本输出到独立构建目录。PDF、`.aux`、`.log`、`.toc`、`.xdv` 和
SyncTeX 文件均为构建产物，不作为规范源提交。修改公式、默认值、结果口径或限制后，必须同步
核对公共头、两份实现、生产路由和已注册测试。
数值结果章节中的 MILP objective、BFS loss、Newton PF loss 和 HTTP 字段必须保持单位与
model scope 分离；不得把 proxy 或 `optimal=false` 结果写成物理损耗或已证明最优。
