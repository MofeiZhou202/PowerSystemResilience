# 理论章写作指南（theory_*.tex）

建立日期：2026-09-13

本指南是模块手册"通用理论层"的唯一写作规范，配合
[模块手册索引](README.md) 的数学模型规范使用。所有 `chapters/theory_*.tex`
必须遵循本指南；写作代理开工前须通读本文件。
（本指南移植自 HybridACDCDistributionSystemsSimulation 项目同名规范，并按 MIPSolvers
模块划分调整。）

## 1. 双层结构

每本模块手册由两层组成：

| 层 | 文件 | 内容 | 锚点要求 |
|---|---|---|---|
| 理论层 | `chapters/theory_*.tex` | 通用数学理论：标准模型、推导、算法原理、适用条件 | 禁止行号；引用实现用 `文件:函数名` |
| 实现层 | 其余章节 | 源码等价转写 | `文件:函数名`，禁止行号 |

理论层的目标是让每本手册在数学上**完整且自洽**：读者不依赖外部教材即可理解
该领域的问题提法、模型体系与方法原理；实现层回答"代码实际做了什么"。

## 2. 深度分级

| 级别 | 适用模块 | 要求 |
|---|---|---|
| 专著级 | engine | 完整推导 + 关键定理的严格证明；篇幅不设硬上限，按主题清单收敛 |
| 工程参考级 | scuc、aml、l2o、python | 标准模型、公式体系、算法原理、适用条件与误差讨论；证明可省略但需给出结论与条件；单模块理论增量 ≤1200 行 LaTeX |

无论哪一级，**正确性优先于完备性**：写不进把握的内容宁可列入"进一步阅读"，
不得编造定理、证明或文献。

## 3. 章节骨架（模板）

每个 `theory_*.tex` 是一个可直接 `\input` 的章节文件，骨架如下：

```latex
% theory_<topic>.tex —— <模块名> 通用理论：<主题>
% 本文件为理论层章节，遵循 docs/modules/THEORY_WRITING_GUIDE.md。

\section{<主题标题>}

\begin{theorynote}
本章为通用数学理论，按领域标准模型与文献体系撰写，<一两句说明覆盖范围>。
本章公式不要求逐式对应代码；与平台实现（\texttt{src/<module>/}）的对应关系
见本章末"与实现的对应"表，超出实现的内容以"未实现扩展说明"框就地标记。
\end{theorynote}

\subsection{记号与约定}
% 本章使用的符号表（用小表格或 itemize）；与手册其余部分冲突时以本章声明为准。

\subsection{<理论小节>}
% 定义/定理/推导正文。使用 theorem 环境族：
%   \begin{definition}...\end{definition}
%   \begin{theorem}...\end{theorem} \begin{proof}...\end{proof}（专著级）
%   \begin{remark}...\end{remark}
% 超出实现的内容：
%   \begin{gapnote} 本节模型在当前代码中尚未实现……\end{gapnote}

\subsection{与实现的对应}
% 必须存在，用 longtable 或 itemize，逐条：
%   理论条目 & \implfull{ipm_solver.cpp:solve} \\
%   理论条目 & \implpart{代码仅实现原始-对偶不可行内点，未含自对偶嵌入} \\
%   理论条目 & \implnone \\

\subsection{参考文献}
% 章末内联 thebibliography（不引入 .bib 文件）：
% \begin{thebibliography}{9}\small
% \bibitem{wright1997} S. J. Wright, Primal-Dual Interior-Point Methods, SIAM, 1997.
% \end{thebibliography}
% 正文引用用 \cite{wright1997}。只列确有把握的真实文献，禁止编造。
```

## 4. 排版环境

由 `docs/_manual_common/theory_environments.tex` 提供：

- 定理族：`\begin{definition/theorem/lemma/proposition/corollary/example/remark}`，
  按 section 编号；`\begin{proof}` 标题自动为"证明"；
- `theorynote`：章首声明框（必须用）；
- `gapnote`：未实现扩展框（超实现内容就地用）；
- `boundarynote`：模型边界说明框（明确拒绝/适用范围）；
- `\implfull{锚点}` / `\implpart{说明}` / `\implnone`：对应表状态标签。

引入方式：使用公共样式 `mipsolvers_manual.sty` 的手册**自动获得**，无需操作；
自带导言区的手册，在主手册导言区加
`\input{../../_manual_common/theory_environments.tex}`（只加一次）。

## 5. 插入位置

主手册中，理论章统一 `\input` 在"阅读指南/概述"章之后、实现转写章之前；
多章理论按主题逻辑排序。主手册除新增 `\input` 行（及自带导言区手册的一行
环境引入）外不得因理论章而改动其他内容。

## 6. 禁区

1. 理论章不得出现行号锚点（如 `solver.cpp:123`）；
2. 不得把未实现的内容写成已实现；对应表中 `\implfull` 的锚点必须真实存在
   （写作者须用 Grep/Read 核实函数名）；
3. 不得编造文献；不确定的引用一律不写；
4. 不得新增平行目录或改动目录结构；理论章一律放既有 `chapters/` 目录；
5. 数学内容必须与代码事实相容：若代码实现了某模型的简化版，理论章给出完整模型时
   必须用 `gapnote` 说明简化差异，不得暗示代码即完整模型；
6. 公式唯一归宿：已在其他模块手册维护的公式组不得复制，只写跨模块接口。

## 7. 验收清单

- [ ] `xelatex -interaction=nonstopmode` 两遍编译通过，无未定义引用；
- [ ] 章首 `theorynote`、章末"与实现的对应"表、超实现处 `gapnote` 三要素齐全；
- [ ] 新增内容零行号锚点（`grep -n ':[0-9]\+' chapters/theory_*.tex` 仅命中参考文献年份等无害处）；
- [ ] 对应表中 `\implfull` 锚点经源码核实，且 `python tools/doc_anchor_check.py` 通过；
- [ ] 工程参考级模块理论增量 ≤1200 行。
