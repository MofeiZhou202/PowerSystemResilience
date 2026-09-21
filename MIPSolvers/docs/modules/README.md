# 源码模块技术手册索引

建立日期：2026-09-13

本目录与 `src/` 顶层目录执行严格的一一映射：目录名必须与源码模块名完全相同，每个模块只能有一个
文档目录、一个 `README.md` 和一个主手册。章节文件位于该模块的 `chapters/`，不得另建大小写、
缩写或历史名称不同的平行目录。

| 源码模块 | 唯一文档目录 | 主手册 | 状态 | 实现范围 |
|---|---|---|---|---|
| `engine` | `modules/engine/` | [求解引擎手册](engine/engine_manual.tex) | 专著级（本轮） | 对偶单纯形、IPM（LP/NLP/锥）、KKT 与稀疏后端、MILP B&C、预求解、求解器门面与外部适配器 |
| `scuc` | `modules/scuc/` | [SCUC 手册](scuc/scuc_manual.tex) | 工程参考级（本轮） | SCUC MILP 建模、case_builder、CLI 工具 |
| `aml` | `modules/aml/` | — | 第二阶段 | AML 代数建模层与 bridges |
| `l2o` | `modules/l2o/` | — | 第二阶段 | L2O 策略与 trace |
| `python` | `modules/python/` | — | 第二阶段 | pybind11 绑定 |

标注"第二阶段"的模块本轮不建目录、不留占位文件；索引行即其唯一规划记录。

## 数学模型规范

1. **实现转写章**（`chapters/` 下非 `theory_` 前缀的章节）的公式只允许转写当前源码实际执行的
   赋值、残差、目标、约束、截断、阈值和条件分支。
2. 实现转写章的每组公式必须给出实现函数及 `文件:函数名` 锚点（如
   `solver.cpp:SolverEngine::solve_lp`、lambda 记作 `文件名:lambda名（lambda）`）；**禁止写行号**——行号
   随代码演进必然漂移。源码变化后必须重新核验语义，并用 `tools/doc_anchor_check.py` 回归。
3. **理论章**（`chapters/theory_*.tex`）允许且应当包含通用数学理论：标准模型、教材通式、
   推导与（专著级模块的）定理证明，不要求逐式对应代码。但必须同时满足：
   - 独立 `theory_*.tex` 文件，与实现转写章物理分离；
   - 章首使用 `theorynote` 环境声明"本章为通用理论，非代码转写"；
   - 章末必须给出"与实现的对应"表：每个理论条目标注 `\implfull{文件:函数名}` /
     `\implpart{差异说明}` / `\implnone`；
   - 理论内容超出当前实现的部分，必须就地使用 `gapnote` 环境标记；
   - 理论章同样禁止行号锚点；引用实现时使用 `文件:函数名` 符号锚点。
   排版环境由 `docs/_manual_common/theory_environments.tex` 统一提供（使用公共样式
   `mipsolvers_manual.sty` 的手册自动引入）。
   写作模板与深度分级见 [理论写作指南](THEORY_WRITING_GUIDE.md)。
4. 实现转写章中，代码未消费的字段、近似路径、回退、硬编码阈值和已知缺陷必须在相邻正文中明确声明。
5. 同一组公式只在一个模块手册维护；其他手册只描述跨模块接口，禁止复制。

## 数值证据规范

- 数值结论只能引用 `tests/`、`benchmark/`、`reports/` 中真实存在的测试目标、基线文件与
  可复现命令；写明构建配置、平台、容差与日期；未执行的交叉验证不得写成已完成。
- 手册中的性能/精度表述必须与仓库 AGENTS.md 的"理论—实现—数值闭环"规则一致。

## 编译与验收

全部主手册共享 `docs/_manual_common/mipsolvers_manual.sty` 和
`docs/_manual_common/industrial_evaluation.tex`。编译方法见
[LaTeX 文档说明](../_manual_common/README.md)。验收清单：

- [ ] `xelatex` 两遍编译通过，无未定义引用；
- [ ] `python tools/doc_anchor_check.py` 退出码 0；
- [ ] 理论章三要素（theorynote / 与实现的对应表 / gapnote）齐全；
- [ ] 新增内容零行号锚点；
- [ ] 对应表中 `\implfull` 锚点经源码核实。
