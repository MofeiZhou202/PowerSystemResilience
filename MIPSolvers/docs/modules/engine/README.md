# 求解引擎（engine）模块技术手册（docs/modules/engine/）

建立日期：2026-09-13

本目录是 `src/engine/` 与 `include/mipsolvers/engine/`（统一求解门面 SolverEngine、
原生对偶单纯形 LP 内核、LP/NLP/锥内点、KKT 与稀疏线性代数后端、原生 MILP
Branch-and-Cut、LP 预求解、外部求解器适配器层）的**唯一文档目录**。索引与规范见
[docs/modules/README.md](../README.md)；理论章写作规范见
[THEORY_WRITING_GUIDE.md](../THEORY_WRITING_GUIDE.md)。

## 主手册

- [engine_manual.tex](engine_manual.tex) —— 《MIPSolvers 求解引擎模块技术手册》（专著级）
- 章节文件位于 [chapters/](chapters/)：
  - `theory_linear_programming.tex` —— 通用理论层：LP 对偶理论、单纯形方法与预求解（含证明）
  - `theory_interior_point.tex` —— 通用理论层：原始-对偶内点法、中心路径、牛顿系统与全局化（含证明）
  - `theory_conic.tex` —— 通用理论层：锥规划、自和谐障碍、NT 缩放方向与 chordal 分解
  - `theory_milp.tex` —— 通用理论层：MILP 分支定界、分支策略、割平面、域传播与预求解（含证明）
  - `source_equivalent_lp_kernel.tex` —— 源码等价转写：原生对偶单纯形 LP 内核全流程
  - `source_equivalent_ipm_kkt.tex` —— 源码等价转写：内点法主循环、KKT 系统与稀疏线性代数后端
  - `source_equivalent_milp_bc.tex` —— 源码等价转写：原生 MILP Branch-and-Cut 主流程与树搜索
  - `source_equivalent_api_adapters.tex` —— 源码等价转写：求解器门面、适配器注册与分派
  - `numerical_validation.tex` —— 数值验证证据（测试矩阵、NETLIB/MIPLIB 基准、微基准）

## 编译

在本目录下执行（需要 XeLaTeX；本机 MiKTeX 已验证，规则见
[docs/_manual_common/README.md](../../_manual_common/README.md)）：

```bash
xelatex -interaction=nonstopmode engine_manual.tex
xelatex -interaction=nonstopmode engine_manual.tex   # 第二遍解析交叉引用
```

编译产物（PDF、aux、log 等）不提交仓库。

## 数值证据

手册中的数值结论只引用 `tests/`、`benchmark/`、`reports/` 中真实存在的测试目标、
基线文件与可复现命令，并注明日期、平台、构建配置与验证层级；覆盖范围与判读纪律
集中在 [chapters/numerical_validation.tex](chapters/numerical_validation.tex)；
面向用户的求解器说明见主手册
[docs/manual/ 第 5 章](../../manual/05-solvers-engines.md) 与
[第 9 章](../../manual/09-testing-benchmarks.md)。未在本仓库执行的交叉验证不写成已完成。

## 锚点回归

文中所有 `文件:函数名` 锚点（不含行号）由仓库根目录执行回归：

```bash
python tools/doc_anchor_check.py
```

源码改动后必须重新核验锚点语义并重新运行该脚本。
