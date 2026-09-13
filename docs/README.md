# MIPSolvers 文档总览

文档入口是 **[工业应用手册](manual/README.md)**（docs/manual/），覆盖安装
部署、快速上手、AML 建模、求解器选型、数值方法、API、电力行业应用
（SCUC/OPF）、测试基准与故障排查，共 11 章。

## 目录结构

| 目录 | 内容 |
|---|---|
| [manual/](manual/) | 工业应用手册（当前维护的用户文档，以此为准） |
| [modules/](modules/) | 源码模块技术手册（LaTeX）：与 `src/` 一一映射的理论层 + 实现转写层 + 数值证据，规范见 [modules/README.md](modules/README.md) |
| [_manual_common/](_manual_common/) | 模块手册共享 LaTeX 排版资源（样式、理论环境、工业评价基线） |
| [archive/](archive/) | 手册整合前的原始文档与带日期的推导/设计记录（冻结保存，含 AGENTS.md 要求的算法推导依据） |
| [tutorial/](tutorial/) | 可运行的 Python 演示（demo_01–demo_05）与教程幻灯片源文件 |
| [todo/](todo/) | 未完成的研究性笔记，不代表当前实现状态 |

## 维护规则

- 用户可见的功能、选项、行为变更应同步更新 `docs/manual/` 相应章节。
- 算法/数值改动的理论依据写入 `docs/archive/` 中对应的推导文档（见手册
  [第 11 章](manual/11-theory-references.md) 索引），并在代码处引用。
- 测试结果写明提交、构建目录、命令、平台和日期；没有本机复现的数据不得
  写成当前性能结论。
- 文档引用源码时使用"文件 + 类型/函数名"，不依赖易漂移的固定行号。
- PDF、测试日志、JSON 结果和图表均为构建产物，不提交到 `docs/`。例外：
  `docs/manual/` 下被章节引用的 machine-readable evidence JSON（如
  `windows-integration-evidence.json`）作为测试证据保留。
