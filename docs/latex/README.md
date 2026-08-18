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

## SPPT Applied Energy 母稿

`sppt_theory.tex` 是 SPPT 唯一主稿，面向 Applied Energy 的当前版本以
“自动化混合 AC/DC 数字孪生的可验证安全边界”为主问题；数值主线由
`sppt_campaign_study.tex` 承载。实验生成器 `sppt_scale_fault_campaign`
使用固定种子 20260818，通过现有 GridLAB-D 接口导入 7 个公开配电 feeder，
并按统一、完全披露的双端 DC/VSC 规则构造 32--6986 AC 母线的混合变种。
每个系统执行 9 类编辑、每类 25 次，输出逐样本 CSV、汇总 CSV、公开案例
manifest 与 LaTeX 表；`tools/sppt_campaign_plots.py` 由这些 CSV 生成三组主文图。
主稿严格区分结构缺陷检出与“结构合法但用户意图错误”的物理影响，不把后者
计作守卫漏检，也不把 Debug timing 当作投稿性能结论。

主稿机制图由 `tools/sppt_mechanism_figures.py` 生成四组矢量 PDF：总体验证
层、语义投影与反向归因、换流器控制角色闭合、以及事务式 Agent 状态隔离。
图中只保留物理对象、控制变量、结构方程和证据回路；换流器图将 DC 缺陷
表述为“角色方程与未知量不闭合/结构秩不足”，不作普遍 DC 电压平移不变性
的过强断言。四图已在 24 页主稿中按 Applied Energy 双栏宽度渲染检查。

复现实验与编译：

```bash
cmake --build <build-dir> --target sppt_scale_fault_campaign
<build-dir>/sppt_scale_fault_campaign <output-dir> 25 20260818
python3 tools/sppt_campaign_plots.py <output-dir> docs/latex/figures
python3 <latex-plugin>/scripts/compile_latex.py \
  "$PWD/docs/latex/sppt_theory.tex" --compiler texlive \
  --output-directory <latex-output-dir>
```

正式投稿前须在干净、固定版本的 MIPSolvers 依赖和优化构建上重跑 timing，并补全
作者、CRediT、基金与公共归档信息；当前 1575 行固定种子结果已通过独立复跑，
除 wall-clock 字段外逐字段一致。结构检出率、Wilson 区间和物理偏差不依赖
Debug/Release timing。
