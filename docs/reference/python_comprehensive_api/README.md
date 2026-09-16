# 综合 Python API 技术手册

本目录保存综合 Python API 的 LaTeX 技术手册，版式与 23 部模块手册一致。

本手册对应源码 `python/src/hysim/`（`analyses.py`、`requests.py`、`client.py`、
`v1.py`、`ai.py`、`models.py`），说明综合 Python API 的分析目录、命名空间家族门面、
类型化请求、诚实结果口径与效果门控。设计契约见
[综合 Python API 设计](../python_comprehensive_api_design.md)。

- 主文档：[python_comprehensive_api_manual.tex](python_comprehensive_api_manual.tex)
- 统一样式：`../../_manual_common/hysim_manual.sty`
- 工业评价基线：`../../_manual_common/industrial_evaluation.tex`
- 总导航：[API 与数据参考](../README.md)

在本目录执行两遍 `xelatex python_comprehensive_api_manual.tex` 生成目录和交叉引用。
公式、字段名、默认值、结果口径和限制必须随 Python 源码、生产路由与注册测试同步更新；
PDF 与 LaTeX 辅助文件不作为规范源提交（已由仓库 `.gitignore` 覆盖）。

> 说明：本手册置于 `docs/reference/` 下的二级子目录，以便共享样式与 `theory_environments`、
> `industrial_evaluation` 的 `../../_manual_common/` 相对路径与模块手册完全一致。
> Python API 并非 `src/` 下的 C++ 模块，故不进入 `docs/modules/`（该目录与 `src/` 严格一一对应）。
