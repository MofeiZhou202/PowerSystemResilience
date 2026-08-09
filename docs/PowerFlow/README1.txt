潮流计算技术手册 — 修订版 1 说明
==================================

受控源文件（最初修订于 2026-07-22）
----------------------------------
power_flow_manual1.tex   修订版 1 主文档（导言区已同步修订，
                         "现状与诚实口径声明"更新为 7 条）
chapters1/               修订版 1 章节源文件（11 章，与 chapters/ 同名对应）

PDF 和 LaTeX 中间文件是本地构建产物，不纳入版本控制。

修订依据
--------
对照代码库 C:\Users\82536\Desktop\tem_write（2026-07-22）与旧库
C:\Users\82536\Desktop\HybridACDCDistributionSystemsSimulation 的潮流模块
差异逐条修订；所有受影响"文件:行号"引用已在新库中重新核实。

主要修订内容
------------
1. CPF（第 9 章，改动最大）：求解器重写为默认弧长延拓——λ 作为增广
   未知量参与校正（有限差分稠密雅可比 + FullPivLU + 回溯线搜索），
   可翻越鼻点并按 lower_branch_steps{2} 保留下支点；CpfOptions 新增
   enable_arc_length{true}（false 保留自然参数化旧模式）；CpfPoint
   新增 vdc 字段、va 单位由度改为弧度；鼻点摘要改取 λ 最大点；
   负荷方向尊重组件聚合负荷（pd_pu/qd_pu）。
2. FDPF（第 4 章）：注入计算稀疏化 S=V·conj(Ybus·V)（O(nnz)，不再
   转稠密）；指定注入在 has_component_loads 时改用逐母线 ZIP 系数。
3. Newton 核心（第 2、3 章）：残差求值器同样稀疏化；SolverWorkspace
   落地（prepare_state/prepare_equations/reset_iteration）并被
   NewtonSolver 按实例复用（带同伦递归守卫）。
4. 混合交直流（第 6 章）：LCC 换流站模型整体移除（lcc_model、
   SolverData::lcc_converters、API lcc_transfers）；事后物理校核
   补充 opt-in 硬约束模式说明。
5. 选项与结果（第 10 章）：PowerFlowOptions 新增
   enforce_converter_physical_limits{false}（越限 Newton 根判物理
   不可行，实施于 src/api/hacdcpf.cpp:957 附近）；新增告警码
   [CONVERTER-PHYS-HARD]；solver_factory 语义收窄为显式方法选择。
6. 架构总览（第 1 章）：新增 solver_interface.cpp（工厂层已接线，
   生产门面仍直连 engine::NewtonSolver）；缓存体系补 SolverWorkspace。
7. 验证总览（第 11 章）：新增 4 个专项回归测试（test_helm_solver、
   test_homotopy_continuation、test_voltage_stability、
   test_newton_krylov，tests/CMakeLists.txt:122-125），"无测试"
   类过时陈述已更正。
8. 第 7、8 章（配电网、三相）：实现无变化，仅个别 tests/CMakeLists.txt
   行号引用更新，实质内容原样保留。

本地生成 PDF（路径须为纯 ASCII）
--------------------------------
把整个文件夹（power_flow_manual1.tex 与 chapters1/）复制到纯英文路径
（如 C:\latex_build\pf_manual1\），执行两遍：

  xelatex power_flow_manual1.tex
  xelatex power_flow_manual1.tex

生成后应进行页面渲染检查；不要提交 PDF、.xdv 或辅助文件。
