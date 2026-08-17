# MIPSolvers 工业应用手册 (Industrial Application Manual)

本手册面向工业现场工程师与集成商，系统介绍 MIPSolvers 数学规划求解器套件
（LP / MILP / QP / NLP / SOCP / SDP）的安装部署、建模、求解器选型、API、
电力行业应用（SCUC/OPF）、测试基准与故障排查。

正文中专业术语、命令、代码与 API 名称保留英文原文（如 dual simplex、
barrier parameter、`cmake --preset`）。

## 章节导航

| 章节 | 内容 | 读者 |
|---|---|---|
| [第 1 章 概述](01-overview.md) | 能力矩阵、应用场景、系统架构、适用边界 | 所有读者 |
| [第 2 章 安装与部署](02-installation-deploy.md) | 工具链、CMake 预设、oneMKL staging、离线构建、SDK 消费 | 集成/部署工程师 |
| [第 3 章 快速上手](03-quickstart.md) | 第一个 LP/MILP、AML 建模、SCUC 入门 | 新用户 |
| [第 4 章 建模与数据 (AML)](04-modeling-aml.md) | AML 代数建模层、SCUC 输入/输出 JSON 规范、算例构造器 | 建模工程师 |
| [第 5 章 求解器与引擎架构](05-solvers-engines.md) | 求解器选型指南、数学原理要点、关键选项与终止判据 | 算法/应用工程师 |
| [第 6 章 数值方法](06-numerical-methods.md) | scaling、KKT 分解后端、迭代改进、惯性校正、锥内点法 | 高级用户 |
| [第 7 章 API 参考](07-api-reference.md) | C++ engine API 与 Python API 对照参考 | 开发人员 |
| [第 8 章 电力行业应用 (SCUC/OPF)](08-industrial-applications.md) | SCUC 建模要素、命令行工具、结果判读、OPF 路径选择 | 电力行业用户 |
| [第 9 章 测试与性能基准](09-testing-benchmarks.md) | 测试体系、NETLIB 基准结论、复现步骤 | QA/验证工程师 |
| [第 10 章 故障排查](10-troubleshooting.md) | 构建、网络、求解、部署常见问题 FAQ | 所有读者 |
| [第 11 章 理论与设计参考](11-theory-references.md) | 推导文档与设计记录索引（docs/archive/） | 算法研究者 |

## 推荐阅读路径

- **第一次使用**：第 1 章 → 第 2 章 → 第 3 章。
- **电力调度 (SCUC/OPF) 集成**：第 1 章 → 第 2 章 → 第 8 章 → 第 4 章。
- **二次开发 / 求解器选型调优**：第 5 章 → 第 6 章 → 第 7 章。
- **出问题的时候**：直接查 [第 10 章 故障排查](10-troubleshooting.md)。

## 相关目录

- `docs/archive/`：手册整合前的原始文档与带日期的推导/设计记录（冻结保存）。
- `docs/tutorial/`：可运行的 Python 演示与教程幻灯片源文件。
- `docs/todo/`：尚未完成的研究性笔记，不属于手册内容。
