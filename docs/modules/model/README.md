# model 模块技术手册

本目录是 `src/model/` 的唯一模块文档目录。

| 文档 | 用途 |
|---|---|
| [model_manual.tex](model_manual.tex) | 当前模型语义、单位变换和有效容量实现手册 |
| [component_models_math_audit.tex](component_models_math_audit.tex) | 历史元件模型数学审计；属于审计证据，不替代模块手册 |

主手册只转写当前源码中已实现的规则。物理求解方程分别由 `power_flow`、`optimal_power_flow`、
`dynamics` 等模块手册说明。

编译命令：

```bash
python3 /Users/tianyangzhao/.codex/plugins/cache/openai-bundled/latex/0.2.4/scripts/compile_latex.py \
  "$PWD/docs/modules/model/model_manual.tex" --compiler texlive --engine xelatex \
  --output-directory /private/tmp/hysim-docs/model
```
