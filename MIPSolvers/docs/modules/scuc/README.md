# SCUC 模块技术手册（docs/modules/scuc/）

建立日期：2026-09-13

本目录是 `src/scuc/` 与 `include/mipsolvers/scuc/`（安全约束机组组合 SCUC / SCED /
LMP 模块，含算例构造器 case_builder 与命令行工具 scuc_solve、scuc_case_builder）的
**唯一文档目录**。索引与规范见 [docs/modules/README.md](../README.md)；
理论章写作规范见 [THEORY_WRITING_GUIDE.md](../THEORY_WRITING_GUIDE.md)。

## 主手册

- [scuc_manual.tex](scuc_manual.tex) —— 《MIPSolvers SCUC 模块技术手册》（工程参考级）
- 章节文件位于 [chapters/](chapters/)：
  - `theory_unit_commitment.tex` —— 通用理论层（SCUC 标准模型、MILP 松弛与 LMP）
  - `source_equivalent_scuc.tex` —— 源码等价转写层（JSON→MILP 全流程）
  - `numerical_validation.tex` —— 数值验证证据

## 编译

在本目录下执行（需要 XeLaTeX；本机 MiKTeX 已验证，规则见
[docs/_manual_common/README.md](../../_manual_common/README.md)）：

```bash
xelatex -interaction=nonstopmode scuc_manual.tex
xelatex -interaction=nonstopmode scuc_manual.tex   # 第二遍解析交叉引用
```

编译产物（PDF、aux、log 等）不提交仓库。

## 数值证据

手册中的数值结论只引用 `tests/` 中真实注册并可在本仓库复现的测试目标
（`test_scuc_module`、`test_market_simulation`，注册见根 `CMakeLists.txt`）。
覆盖范围、运行命令与已知边界集中在
[chapters/numerical_validation.tex](chapters/numerical_validation.tex)；
面向用户的算例使用说明见主手册 [docs/manual/ 第 8 章](../../manual/08-industrial-applications.md)。
未在本仓库执行的交叉验证（如与外部参考求解器的系统性比对）不写成已完成。

## 锚点回归

文中所有 `文件:函数名` 锚点（不含行号）由仓库根目录执行回归：

```bash
python tools/doc_anchor_check.py
```

源码改动后必须重新核验锚点语义并重新运行该脚本。
