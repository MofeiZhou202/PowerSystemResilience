# 文档分类台账

最后核实：2026-09-05

本台账给 `docs/` 下 Markdown 资产指定唯一的**主要归属**。Markdown 已按分类物理迁移到对应目录；
一个文档可由其他分类交叉引用，但只在本表登记一个主要归属。

## Markdown 主分类

| 主要分类 | 文件 |
|---|---|
| 仓库总览 | `overview/*.md` |
| 用户指南 | `guides/*.md` |
| 开发者指南 | `developer/*.md` |
| 理论与模型 | `theory/*.md` |
| API 与数据参考 | `reference/*.md` |
| 测试与验证 | `testing/*.md` |
| 部署与运维 | `operations/*.md` |
| 模块文档 | `modules/*/*.md` |

## 子目录 Markdown 归属规则

| 路径规则 | 主要分类 | 入口/说明 |
|---|---|---|
| `overview/**/*.md` | 仓库总览 | [总览](README.md) |
| `guides/**/*.md` | 用户指南 | [用户指南](../guides/README.md) |
| `developer/**/*.md` | 开发者指南 | [开发者指南](../developer/README.md) |
| `theory/**/*.md` | 理论与模型 | [理论与模型](../theory/README.md) |
| `modules/**/*.md` | 模块文档 | [模块手册](../modules/README.md) |
| `reference/**/*.md` | API 与数据参考 | [API 与数据参考](../reference/README.md) |
| `tutorials/**/*.md` | 示例与教程 | [示例与教程](../tutorials/README.md) |
| `testing/**/*.md` | 测试与验证 | [测试与验证](../testing/README.md) |
| `operations/**/*.md` | 部署与运维 | [部署与运维](../operations/README.md) |
| `research/**/*.md` | 研究资料 | [研究资料](../research/README.md) |
| `planning/**/*.md` | 规划与演进 | [规划与演进](../planning/README.md) |
| `_manual_common/**/*.md` | LaTeX 手册维护 | [_manual_common 说明](../_manual_common/README.md) |
| `archive/**/*.md` | 历史归档 | [归档索引](../archive/README.md)；不得作为当前行为证据 |
| `latex/paper/**/*.md` | 论文伴随材料 | [论文工作区索引](../latex/paper/README.md)；由每篇论文 README 管理 |
| `latex/**/*.md`（不含 `paper/`） | LaTeX 研究资料 | [LaTeX 文档说明](../latex/README.md) |

## 市场智能仿真资料

市场智能仿真资料归属：理论模型为
[`theory/market_boundary_optimization_model.md`](../theory/market_boundary_optimization_model.md)；
模块实现与实验协议为
[`modules/market/intelligent_simulation.md`](../modules/market/intelligent_simulation.md)。

## LaTeX 归属

- 工业手册：与 `src/` 顶层目录严格同名的
  [23 个 `modules/<module>/` 目录](../modules/README.md)。
- 研究论文：`latex/paper/`，由 [论文工作区索引](../latex/paper/README.md) 逐篇登记。
- 专题长文、章节片段与生成证据：`latex/`，由 [LaTeX 文档说明](../latex/README.md) 分类。
- 历史 LaTeX 审计：`archive/`，由 [归档索引](../archive/README.md) 管理。

## 防孤儿规则

市场模块新增的 [南方区域规则对照](../modules/market/southern_rules_comparison.md) 归属模块文档，
由 [market 入口](../modules/market/README.md) 管理。规则转写章节属于原有市场 LaTeX 主手册；
`modules/market/references/southern_region_spot_energy_rules_2025_v1_0.pdf` 为官方外部规范源附件，
不是手册构建产物，来源、版本、页码和 SHA-256 记录在该模块入口。

新增 Markdown 必须满足下列至少一项：列入根目录归属表；位于已登记路径规则之下；由模块或论文
README 明确引用。新增主 LaTeX 必须登记到模块索引、论文索引或 LaTeX 分类表。文档检查应报告
不满足这些条件的文件。
