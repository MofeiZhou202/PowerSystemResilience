最优潮流技术手册 — 说明
==========================

内容
----
opf_manual_v3.pdf     当前版本 PDF（XeLaTeX 编译，54 页；opf_manual.pdf 为第二版留存）
opf_manual.tex        主文档（导言区、统一参数表环境、阅读指南：
                      数据来源 / 单位制约定 / 编号与符号约定 /
                      求解器家族总览 / 验证通道总览 / 现状与诚实口径声明）
chapters/             十一个章节源文件（由主文档 \input 引入）：
  overview.tex        第 1 章  架构总览与数据流（两条 OPF 对外面、流水线、
                               后端分发路由、API 门面、缓存与暖启动、GUI 接入）
  ac_opf.tex          第 2 章  AC OPF 顶层求解器（预处理、后端分发与目标联动、
                               目标同伦、经济调度兜底、Ipopt 适配器、暖启动与
                               状态续传、Native AC 简约空间旧路径与全局化常量）
  parity.tex          第 3 章  Parity 全空间 NLP 建模（变量布局、固定注入台账、
                               目标函数细节、等式/不等式、VSC 容量圆半径、准入
                               条件、DC/DC 占空比全拓扑公式、变量界与初值、
                               ZIP 与 Hessian 稀疏结构、ParityOptions）
  ipm.tex             第 4 章  原生原对偶内点法（condensed/augmented KKT 与
                               RHS 装配、δW 暖启动与 KLU/MUMPS 不对称、Ruiz 均衡、
                               Mehrotra/Gondzio、再中心化重试、ABO μ 策略、
                               滤波线搜索、SOC、恢复阶段、可接受收敛、
                               对偶最小二乘抛光、同伦切线、IPMOptions）
  dc_opf.tex          第 5 章  DC OPF（LP/QP 目标、LP 回退 lambda 形式 PWL
                               凸组合成本、预切负荷剪枝、后端链、支撑 LP 取
                               LMP、可行性复核、branch_mu_valid 与 lmp_valid
                               认证声明、DCOPFOptions）
  rpo.tex             第 6 章  无功优化（OLTC/并联 MINLP、网损台账、控制清单、
                               内层目标解耦与求解配置、四阶段搜索、并行评估、
                               终止标志与 model_limitations、无全局证书声明、
                               RPOOptions）
  three_phase.tex     第 7 章  三相混合 OPF（相域直角坐标、GFL/GFM 换流器、
                               VUF、GraphReduced 降阶与证书、对偶初始化、
                               原始恢复、约束 oracle、控制投影与动态平衡恢复、
                               OPF→PF 回放、序列/分支驱动、Options）
  power_models.tex    第 8 章  AML 求解器无关建模层（acopf/acdcopf/dc_opf/
                               lindistflow/scuc builder，DC/DC I²R 已建模，
                               model_scope/model_limitations 诚实口径字段，
                               非生产链路声明）
  options_results.tex 第 9 章  选项、结果与诊断参考（ACOPFOptions 全字段表、
                               ACOPFResult/DCOPFResult/RPOResult/IPMResult/
                               ThreePhaseHybridOPFResult 逐字段表（含
                               lmp_valid/branch_mu_validity_reason/
                               pwl_segments_effective/model_limitations/
                               terminated_by_* 认证与边界字段、OpfAudit 与
                               ACOPFProfiling 子表、va 弧度存储约定）、
                               OPFProblem 与 DefaultOPFSolver、
                               诊断与环境变量全集）
  applications.tex    第 11 章 OPF 在平台中的应用与接入（时序暖启动链、
                               可靠性 DC OPF 配置、EV-交通 LMP 迭代、反事实
                               规划与 SPPT MR3d、市场模块边界、GUI/HTTP
                               参数映射、参数契约端点 /api/opf/
                               parameter_contract 与结果回显、Python 绑定
                               与能力查询）
  validation.tex      第 10 章 验证与校核总览（三层验证体系、85 个 MATPOWER
                               算例分档、扩充测试矩阵（15 目标 + E2E）、
                               交叉验证机制、性能锚点、已知边界与覆盖缺口、
                               如何运行验证）

（正文章节序号按 LaTeX \section 编号：validation 为第 11 章、applications 为第 10 章
——主文档 \input 顺序中 applications 在 validation 之前。）

每节结构：功能说明 → 数学模型与算法（逐条注明源码出处 文件:行号）→
参数表（字段名|符号|单位|典型范围|默认值|说明，六列）→ 验证与校核。
全部公式对照 HySim-XJTU-HRPES 源码 src/optimal_power_flow/、
include/hacdcpf/optimal_power_flow/、src/power_models/ 与
include/hacdcpf/power_models/ 逐一核对；近似、未接线、覆盖薄弱处均已如实标注
（见主文档"现状与诚实口径声明"与末章"已知边界与覆盖缺口"）。

重新编译（重要：路径须为纯 ASCII）
--------------------------------
本机已装 MiKTeX（xelatex）。注意：MiKTeX 在含中文的路径下编译会崩溃
（xelatex FATAL Invalid argument），请先把整个文件夹复制到纯英文路径
（如 C:\latex_build\opf_manual\），再执行两遍：

  xelatex opf_manual.tex
  xelatex opf_manual.tex

（第一遍排版，第二遍生成目录与交叉引用；宏包缺失时 MiKTeX 会自动下载。）
编译出的 PDF 可拷回本文件夹。

生成日期：2026-07-21（初版）；2026-07-22（第二版：覆盖完整性审计后补齐，
修正 compute_jacobian_diagnostics 未实现、ACOPFObjective 四档、
DCOPFSolverBackend 枚举名、MUMPS 后端名 4 处不一致，新增应用与接入章。）；
2026-07-23（第三版：对照 2026-07-22 晚"update the OPF bugs"提交全面更新——
DC OPF 的 LP 回退改为 lambda 形式 PWL 凸组合（pwl_segments 正式接线，
objective_model 增 "LP-PWL"，新增 pwl_segments_effective）；新增 lmp_valid/
lmp_validity_reason、branch_mu_validity_reason、model_limitations、
RPO terminated_by_* 等认证与边界字段；AML 混合 OPF 的 DC/DC I²R 损耗已建模、
SCUC price_valid、LinDistFlow 注释与实现一致化；新增 opf_solver_interface.hpp
（DefaultOPFSolver）；AC OPF 相移变压器支路限值修复；GUI/HTTP 参数契约端点
/api/opf/parameter_contract。同时按潮流手册深度标准补齐第 9 章逐字段表
（修正 va 弧度单位、RPOResult/IPMResult/ThreePhaseHybridOPFResult 字段名、
OpfAudit/ACOPFProfiling 子表），验证章 MATPOWER 算例数 95→85 并新增
算例分档与"如何运行验证"小节。）
