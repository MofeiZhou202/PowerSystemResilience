# LaTeX 文档结构与编译说明

## 文档类型

| 类型 | 位置 | 判定标准 |
|---|---|---|
| 工业技术手册 | `../modules/<module>/` 的 23 个模块目录 | 面向当前实现，覆盖接口、模型、算法、验证与局限 |
| 完整研究论文 | `paper/<论文目录>/` | 有独立主文件、参考文献、图表与复现说明 |
| 专题长文 | `platform_overview.tex`、`sppt_theory.tex`、`time_series_annual_simulation_zh.tex` | 跨模块综述或单专题完整推导 |
| 章节片段 | `dr_effects_zh_part.tex`、`sppt_*_scope.tex` 等 | 由其他主文档 `\input`，不可独立构成契约 |
| 生成证据 | `sppt_*.tex`、`sim_results/`、`figures/` | 表格、图形或实验输出，只支撑主文档论证 |

论文的逐目录入口与规范化状态见 [论文工作区索引](paper/README.md)。文件按篇幅/用途分类后保留
原路径，避免破坏 `\input`、图件和复现脚本的相对引用。

## 模块手册编译

每个模块目录包含一个主 `.tex`，并引用 `../../_manual_common/hysim_manual.sty`。在模块目录执行：

```bash
xelatex <module>_manual.tex
xelatex <module>_manual.tex
```

也可从仓库外调用统一编译器：

```bash
python3 /path/to/compile_latex.py /absolute/path/to/<module>_manual.tex --engine xelatex
```

第一次生成版面，第二次更新目录和交叉引用。23 部模块手册固定使用 TeX Live 随附的
Fandol 中文字体集，避免依赖 Windows 字体；不得为了通过编译而删除中文内容。

## 验收清单

1. 主文件可解析，所有 `\input`、图片和样式路径存在。
2. 两遍编译后不存在未定义引用、致命错误或越出版心的关键表格。
3. 公式、字段默认值和结果语义与当前头文件、实现和测试一致。
4. 结果向量声明单位与索引空间；AC/DC 同号 ID 不使用裸整数跨域传递。
5. 近似、回退、时限、后端缺失和模型覆盖不足均有显式标志或限制说明。
6. PDF、`.aux`、`.log`、`.out`、`.synctex.gz` 等构建产物不作为规范源提交。
