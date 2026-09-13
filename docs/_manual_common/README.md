# MIPSolvers 模块手册共享排版资源（_manual_common）

本目录是 `docs/modules/` 各部模块技术手册的唯一共享排版资源，移植自
HybridACDCDistributionSystemsSimulation 项目的 `docs/_manual_common/`（2026-09-13）。

## 文件

| 文件 | 用途 |
|---|---|
| `mipsolvers_manual.sty` | 共享样式：版式、`paramtable` 参数表、`\fld`、`\srcpath`（可断行源码锚点）、`\compmeta`；自动引入 `theory_environments.tex` |
| `theory_environments.tex` | 理论层环境：定理族、`theorynote`/`gapnote`/`boundarynote`、`\implfull`/`\implpart`/`\implnone` |
| `industrial_evaluation.tex` | 共享末章"跨模块工业级技术评价基线"，由各主手册在末尾 `\input` |

## 使用规则

1. 模块手册统一用 `\documentclass{ctexart}` +
   `\usepackage{../../_manual_common/mipsolvers_manual}`；共享末章用
   `\input{../../_manual_common/industrial_evaluation}`。
2. 路径基准：`../../_manual_common/` 相对于 `docs/modules/<module>/` 的主手册。
   **模块目录不得复制同名样式文件**；样式只在本目录维护。
3. 自带导言区（不使用公共样式）的手册，须在导言区手动
   `\input{../../_manual_common/theory_environments.tex}`（只加一次）。

## 编译

在模块目录下执行（需要 XeLaTeX，本机 MiKTeX 已验证）：

```bash
xelatex -interaction=nonstopmode <module>_manual.tex
xelatex -interaction=nonstopmode <module>_manual.tex   # 第二遍解析交叉引用
```

编译产物（PDF、aux、log 等）不提交仓库。
