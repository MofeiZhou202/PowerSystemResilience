最优潮流（OPF）技术手册 — 说明
==============================

内容
----
opf_manual.pdf     当前版本 PDF（XeLaTeX 编译，143 页）
opf_manual.tex     主文档（导言区、统一参数表环境、阅读指南：数据来源 /
                   单位制约定 / 求解器家族与选择逻辑 / 诚实结果口径 /
                   验证通道总览）
chapters/          九个章节源文件（由主文档 \input 引入）：
  overview.tex           最优潮流总览与公共接口层（求解器家族全景、Native AC 与
                         交直流混合引擎对比、ACOPFOptions/DCOPFOptions/
                         ACOPFResult/DCOPFResult 全字段表、诚实口径、Ipopt 门控、
                         调用方速览、能力查询）
  ac_opf_native.tex      AC 最优潮流：Native IPM 求解器（纯交流紧致路径：极坐标
                         全模型、原生内点循环、求解路径分派、暖启动与同伦延续）
  dc_opf.tex             直流潮流近似最优潮流（DC OPF：LP-PWL/QP、VOLL 切负荷、
                         LMP、回退链；纯交流口径，不建直流电网）
  parity_formulation.tex 交直流混合最优潮流：Parity 建模层（含 DC/VSC/LCC 算例的
                         承载模型：17 块变量布局、VSC 三端口耦合、DC/DC、LCC
                         准稳态、调制窗口、雅可比组装）
  parity_ipm.tex         交直流混合最优潮流：Parity IPM 求解核（障碍问题、KKT
                         求解、滤波线搜索、收敛判据全集、状态串全集、调谐参数）
  rpo.tex                无功优化与 OLTC 离散档位（RPO：离散坐标搜索、候选评估
                         管线、28 个选项字段、专项验证）
  three_phase.tex        三相混合最优潮流（adapter/opf/relaxation 三层、相域
                         模型、VSC 控制律、SOCP 下界证书）
  power_models.tex       求解器无关建模层（MIPSolvers AML：acopf/acdcopf/
                         dc_opf/lindistflow/scuc 五个 builder、后端分发）
  verification.tex       验证与校核、已知局限（12 通道验证矩阵、30 条已知
                         局限汇总、可信边界结论）

每节结构（与《元件模型技术手册》一致）：
  功能定位 → 数学模型/算法（公式逐条注明源码出处 文件:行号）→
  参数表（字段名|符号|单位|典型范围|默认值|说明，六列）→
  验证与校核 → 边界与局限（如实标注近似/fallback/未实现项）。

内容全部对照 HySim-XJTU-HRPES 源码 src/optimal_power_flow/、
include/hacdcpf/optimal_power_flow/、src/power_models/ 及 tests/、docs/
逐一核对；行号以 2026-07-31 检出处为准。撰写中发现的注释/文档与代码不符
之处（悬置声明、死代码、过时行号引用等）均已在正文"边界与局限"中如实登记。

重新编译（重要：路径须为纯 ASCII）
--------------------------------
本机已装 MiKTeX 25.12（xelatex）。注意：MiKTeX 在含中文的路径下编译
会崩溃（xelatex FATAL Invalid argument），请先把整个文件夹复制到
纯英文路径（如 C:\latex_build\），再执行两遍：

  xelatex opf_manual.tex
  xelatex opf_manual.tex

（第一遍排版，第二遍生成目录与交叉引用；宏包缺失时 MiKTeX 会自动下载。）
编译出的 PDF 可拷回本文件夹。本次构建副本位于 C:\latex_build\opf_manual\。

生成日期：2026-07-31（v2：章节按 AC OPF / DC OPF / 交直流混合 OPF 三个主类
重组，交直流混合 OPF（Parity 建模层 + Parity IPM 求解核）独立成章）
