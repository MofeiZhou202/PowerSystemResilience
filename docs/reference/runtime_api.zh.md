> 本文档为 [runtime_api.md](runtime_api.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

# 运行时 API 契约

更新日期：2026-08-18（result_window 新增可选的 lod 0/1 聚合结果）

GUI 服务器实现于 `tests/run_gui_server.cpp`。会话端点作用于一个已加载的
`HybridPowerSystem`；改变模型的请求会清除缓存的分析结果。

## 静态文档挂载

`GET /xjtu/docs/<relative path>` 以只读方式提供仓库 `docs/` 目录树中的
markdown 文件（例如 `/xjtu/docs/README.md`、
`/xjtu/docs/overview/case_catalog.md`）。试图越出 `docs/` 的路径遍历将被
以 404 拒绝。GUI 文档帮助中心
（`web/js/core/help_center.js`，清单文件 `web/help_docs.json`，schema 为
`hysim_help_docs_v1`）会在应用内渲染这些文档；前端契约见
[GUI Canvas 运行时](../developer/gui_canvas_runtime.zh.md)。

## 第 1 版多会话 API

新的自动化与 AI 客户端应使用 `/api/v1`。这些资源独立于 GUI 的进程全局
`/api/session/*` 状态。

| 方法与路由 | 契约 |
|---|---|
| `GET /api/v1` | 能力与有界资源声明。 |
| `POST /api/v1/sessions` | 创建空会话，或加载 `case`、`model`、`json_string`；返回 `201`、`Location` 与 `ETag`。 |
| `GET /api/v1/sessions` | 列出内存中的会话。 |
| `GET /api/v1/sessions/{session_id}` | 读取模型摘要与当前修订号。支持 `If-None-Match`。 |
| `GET /api/v1/sessions/{session_id}/model` | 读取作者编辑的 rich 模型及其 ETag。 |
| `GET /api/v1/sessions/{session_id}/topology` | 读取分页的 LOD0/1/2 拓扑分块；接受 `offset`、`limit` 以及完整的 `xmin,ymin,xmax,ymax` 视口。 |
| `GET /api/v1/sessions/{session_id}/subgraph` | 读取围绕必需的 domain 限定 `domain,index` 母线引用的有界 k 跳子图。 |
| `PUT /api/v1/sessions/{session_id}/model` | 原子替换模型并递增修订号。要求携带当前的 `If-Match`。 |
| `DELETE /api/v1/sessions/{session_id}` | 删除无活动任务的会话。要求携带 `If-Match`。 |
| `POST /api/v1/sessions/{session_id}/jobs` | 快照匹配的模型修订并入队潮流或最优潮流任务；返回 `202` 与 `Location`。 |
| `GET /api/v1/sessions/{session_id}/jobs` | 列出该会话保留的任务（不含结果数组）。 |
| `GET /api/v1/jobs/{job_id}` | 轮询状态并获取终态结果。 |
| `GET /api/v1/jobs/{job_id}/frames/{step}` | 读取一个时间步，支持 domain、稳定索引、视口、offset 与 limit 过滤。静态潮流/最优潮流只有 step 0。 |
| `GET /api/v1/jobs/{job_id}/violations` | 读取某一步最严重的电压/负载率越限，无需下载完整帧。 |
| `POST /api/v1/jobs/{job_id}/cancel` | 立即取消排队中的工作，或将运行中的工作标记为 cancelling。 |
| `DELETE /api/v1/jobs/{job_id}` | 移除已保留的终态任务。 |

模型 ETag 是强不透明值，形如
`"hysim-ses-...-r3"`。缺少 `If-Match` 返回 HTTP 428；过期值返回 HTTP 412，
并附带 `current_etag` 与 `current_model_revision`。已提交的任务拥有一份不可变
的模型快照，因此之后的模型替换不会改变求解。轮询会报告
`stale_against_current_model`；嵌套的 `hysim_result_v1` 契约会重复给出任务修订号
与当前修订号。

有界的内存工作线程池默认为两个线程，可用 `--api-job-workers 1..32` 配置。
进程最多保留 128 个会话和 4096 个任务。终态任务应由客户端删除。存储在服务器
重启后不持久。

第 1 版目前接受 `power_flow`（`method=ac_newton`）与
`optimal_power_flow`（`network_model=balanced_aggregate`，后端可选 parity、
Ipopt、auto、经济调度或 DC）。其他生产级分析仍保留在旧路由上，直到其
结果契约完成迁移。

Parity/auto 最优潮流接受 `request.options` 下有界的 Phase-I 控制项：
`enable_phase_one`，时间/迭代/分解/回溯预算，barrier、admission、
primal-corridor 与 centrality 参数。大型纯 AC 调度启动还额外接受
`ac_pf_warm_start`、`ac_pf_dc_phase_one`、最小母线数、端到端的
`ac_pf_dc_phase_one_time_limit_ms` 墙钟预算、DC 迭代/残差限制、必需的对偶
改善量，以及 baseline-dual 触发条件。该预算涵盖投影、建模、结构初始化、
符号分析与数值迭代；一个正在进行中的稀疏分解可能超出该预算。`<= 0` 的值
会禁用墙钟限制。响应返回 `ipm_profiling`，其中包含实际生效的
baseline/candidate 原始-对偶残差、预算使用情况、admission 状态以及
Phase-II 接受情况。`parity_formulation_builds=1` 证明 baseline 与 candidate 评估
复用了同一个不可变建模，而 `dc_phase_one_symbolic_analyze_calls` 报告独立的
DC KKT 分析。默认关闭的研究型控制项 `phase_one_dispatch_dual_predictor` 与
`phase_one_dispatch_dual_min_improvement` 启用一个零分解的组件价格 candidate。
其 attempted/accepted/runtime/status 以及原始与归一化残差对以
`dispatch_dual_predictor_*` 返回；被拒绝的 candidate 不会改变 Phase-II 的起点。
同一 profiling 对象还携带 prepared-session 契约：
`prepared_session_used`、`formulation_reused`、`mapping_reused`、
`symbolic_reused`、`continuation_state_reused`、`numeric_refactor_*` 以及
`prepared_session_invalidation_reason`。当前 HTTP 端点执行的是独立求解，因此
`prepared_session_used=false`；对应的重复求解 API 是 C++ 中的
`opf::PreparedACOPFSession`。这些字段仍会返回，以便未来的会话感知 HTTP
调度器能够在不改变响应 schema 的情况下采用该契约。数值因子复用是失败关闭
（fail-closed）的：当前后端在复用符号排序后会重新计算数值因子，并在启用
实验性请求时报告明确的 `unsupported` 状态。非有限的未激活残差以 JSON
`null` 表示。达到 DC 迭代上限的点仅以 `phase_one_warm_start_only=true` 返回；
它不会设置 `converged=true`，也不宣称 DCOPF 最优性。

排队中的取消是立即生效的。运行中的潮流/最优潮流求解器目前没有协作式取消
令牌；其状态变为 `cancelling`，最终结果会被丢弃，终态变为 `cancelled`。

拓扑 LOD2 节点暴露稳定的 `ref: {domain,index}` 值。边引用还携带
`resource`，使支路、变压器与换流器不会冲突。LOD1 按 domain/area/zone 聚合；
LOD0 按电气域聚合。空间坐标是 API 布局空间，不是图索引，也不是工程身份。
分页作用于节点，且仅包含两个端点都在返回节点分块内的边。

帧分块返回作者空间的结果行。`indices=1,7,20` 始终表示所选 domain 内的
稳定组件索引。只要对应的会话修订仍然存在，就可以使用视口过滤。越限端点
将最大的工程越限排在前面，使概览类客户端能够在请求稠密细节之前先检查
最严重状态。

## 会话生命周期

| 方法与路由 | 用途 |
|---|---|
| `POST /api/session/new_empty` | 创建空系统。 |
| `POST /api/session/load_builtin` | 加载内置算例。 |
| `POST /api/session/load_json_string` | 加载作者编辑的 rich 模型 JSON。 |
| `POST /api/session/load_matpower` | 加载 MATPOWER 算例。 |
| `POST /api/session/update_components` | 应用组件修改。 |
| `GET /api/session/status` | 读取当前会话状态。 |
| `POST /api/session/cancel` | 请求取消长时间运行的分析。 |

会话将当前系统保存为只读共享的不可变快照
（`std::shared_ptr<const HybridPowerSystem>`）；改变模型的路由通过
`session_replace_system` 原子替换它（写时复制），每次写入重建驻留的
`PowerSystemGraph`、母线空间索引与紧凑的 `_raw_json` 序列化缓存各一次，并递增
`system_revision`。`cache_last_power_flow`（在会话锁下调用）仅当请求捕获的不可变
系统快照仍是 `current_system` 时发布缓存并记录
`last_pf_revision`；`/api/session/result_window` 比较两个修订号以声明
`result_matches_current_system`。`last_pf_result`/`last_pf_system` 是共享快照，
因此结果服务路由读取它们时无需复制。
模型替换后才完成的旧 PF 可以向原请求返回旧快照结果，但不能重新写入会话缓存；
此时窗口路由返回 `409/no_cached_power_flow`，直到当前模型的新 PF 成功发布。
加载路径的响应（`load_*`、`update_components`、参数库与设计手册应用）内嵌
`_raw_json`，即缓存的紧凑（无缩进）序列化；前端用 `JSON.parse` 解析它。

45 个使用 `Session::busy` 的分析处理器统一通过 `AtomicFlagLease` 原子获取占用权，
仅成功获取者可以释放。非法 JSON、获取前异常和 `409` 冲突均不能清除另一个任务的
`busy` 或重置其 `cancel`。全局异常处理器只生成错误响应，已获取的占用权由栈展开释放。
`/api/session/status` 中 `busy`/`cancel` 的布尔类型及前端任务管理契约保持不变。
注册测试 `session_integrity_e2e` 验证拒绝请求、取消、PF 期间替换模型和新 PF 发布流程。

## 拓扑窗口

| 方法与路由 | 契约 |
|---|---|
| `POST /api/session/topology_window` | 对驻留的均匀网格空间索引做包围盒查询。请求体：必需的数值 `min_x,min_y,max_x,max_y`（WGS84 度；x=经度，y=纬度；强制 min<=max）和可选的 `lod`（0=按电气域聚合，1=按 domain/area/zone 聚合，2=完整逐母线细节，默认 2；语义与 `web/js/core/network_overview.js` 的 `aggregate()` 一致）。响应 `topology_window_v1`：`nodes` 携带 domain 限定的稳定 `.index`（lod 2）或分组 `key`/质心/`count`（lod 0/1）；`edges` 仅在两个端点都在窗口内时包含，并携带 `category` 与稳定 `index`。`coordinate_coverage` 声明有多少 AC/DC 母线缺少坐标（模型默认 lat/lon 为 0,0）；此类母线被排除在窗口结果之外，且该排除会在 `model_limitations` 中重复声明，而不是静默返回空视图。响应从不内嵌完整的组件参数集。 |
| `POST /api/session/result_window` | 来自缓存的最近一次潮流（`last_pf_result` + `last_pf_system`）的视口范围逐元件潮流结果。请求体与 `topology_window` 的 bbox 相同，外加同样可选的 `lod`（默认 2；0/1 以外的非法值返回 400）。当没有缓存的潮流结果时返回 `409` 且 `error=no_cached_power_flow` —— 绝不静默返回空视图。响应 `result_window_v1`（扩展了 `lod` 与下述聚合节点/支路形态）：lod 2 时，`nodes` 为窗口内有地理参考的母线 `{domain,index,x,y,vm_pu,va_rad?}`（`vm_pu` 为标幺值；`va_rad` 为弧度，仅 AC；DC 母线的 `vm_pu` 为 vdc 标幺值）；`branches` 为两个端点都在窗口内的 AC 支路 `{domain,index,from,to,in_service,pf_mw,qf_mvar,pt_mw,qt_mvar,loss_mw,loading_pct,rate_mva}`（MW/MVAr；`loading_pct = 100 * max(|S_from|,|S_to|)/rate_a_mva`，与完整潮流的 `geo_ac_branches` 相同；DC 支路潮流与换流器传输被排除并予以声明）。lod 0/1 时，分组键、质心坐标与组内边折叠与 `topology_window` 完全相同：`nodes` 为 `{group,domain,x,y,count,vm_avg,vm_min,vm_max}`，其中 vm 字段是成员母线电压的统计量（不是求解出的组电压），`branches` 为组对聚合边 `{key,source,target,domain:"AC",kind:"aggregate",count,loading_pct,in_service}`，其 `loading_pct` 取被折叠成员中的**最大值**（保守口径）；两种语义均在 `units`（`vm_avg`/`vm_min`/`vm_max`/`aggregated_loading_pct`）与 `model_limitations` 中声明。`result_meta` 报告 `source`、`method`、`converged`、`iterations`、`residual` 与 `result_matches_current_system` —— 当求解之后模型被替换或编辑时，该标志为 `false`，且 `model_limitations` 会声明该滞后。`units`、`bbox` 与 `coordinate_coverage` 与 `topology_window_v1` 一致。 |

当作者编辑的画布模型有脏数据时，GUI 会在分析之前调用 `syncToBackend()`。
结果回放永不同步或修改模型。

## 主要分析

| 路由 | 结果族 |
|---|---|
| `POST /api/session/pf` | 作者空间的潮流结果，带 rich 组件归因与 P/Q 诊断。`method=three_phase_hybrid` 选择单体不平衡 abc/DC/VSC Newton 路径。 |
| `POST /api/session/opf` | 最优潮流外加作者空间的后潮流（post-PF）画布载荷。`network_model=three_phase_hybrid` 选择 Full/GraphReduced 相域混合最优潮流与同模型潮流重放。 |
| `POST /api/session/run_ts_pf` | UC/OPF/PF 时间序列摘要与缓存的逐步结果。 |
| `POST /api/session/run_annual_sim` | 年度生产模拟与聚合统计。 |
| `POST /api/session/run_carbon` | 静态碳流。 |
| `POST /api/session/run_dynamic_carbon` | 使用缓存或新求解的 TSPF 状态的动态碳流。 |
| `POST /api/session/run_transient` | 相量域暂态结果与缓存的遥测数据。 |
| `POST /api/session/small_signal` | 在初始化后的动态运行点处做模态分析。 |
| `POST /api/session/sc`、`dc_sc`、`sc_detailed` | AC/DC 短路分析。 |
| `POST /api/session/harmonics*` | 谐波潮流、三相、频率扫描、指标与 Newton 变体。 |
| `POST /api/session/run_reliability*` | 非序贯、序贯、FMEA、馈线与三阶段可靠性。 |
| `POST /api/session/run_reconfig` | 拓扑重构。 |

`POST /api/session/run_transient` 接受 MassMatrixDae 的 IEEE 1547 事件控制：
`enable_der_protection`、`localize_der_protection_events`、
`protection_event_time_tol_s`、`protection_event_max_localization_iters`、
`protection_event_cluster_window_s` 与 `post_event_algebraic_residual_tol`。聚类窗口
固定锚定最早物理动作，不滚动延长。结果返回是否实际定位、试积分次数、事件簇数、
最大时间括号/簇跨度（s）和最大事件后代数电流平衡无穷范数（pu）。保护事件记录
包含零基 `protection_cluster_id`，非保护事件为 `-1`。精确数值范围与不支持
的继电器/步内脉冲见[暂态运行时契约](../theory/transient_runtime.zh.md)。
该路由还接受并在 `options` 回显 `algebraic_network_max_iters`
（默认 10）与 `algebraic_network_tol`（默认 `1e-6`）。一致初始化阶段可按暂态契约
临时使用更严的内部代数容差；回显值仍是时间积分阶段实际采用的请求值。

潮流数值覆盖项是稀疏意图。特别地，省略
`options.robust_nonlinear.enable_auto_fallback_scheduling` 会保留 C++
后端默认值；GUI 的 `后端默认` 选项刻意省略该键。
显式的 `true` 或 `false` 会为该请求覆盖它。每个潮流响应都在
`options_effective.robust_nonlinear.enable_auto_fallback_scheduling` 下返回
解析后的生效值；客户端必须显示该生效值，而不是从未勾选或缺失的控件去推断。

平衡 Newton 潮流接受 `enable_pv_pq_conversion` 与有界的
`pv_pq_max_outer_iterations` 控制项。响应的 `reactive_limits` 对象报告
是否请求并认证了发电机 Q 限值强制执行、是否出现重复的活跃集签名、
外层预算是否耗尽、切换次数、活跃受限母线，以及以标幺值表示的最大越限。
对应的有效性标志为 `generator_reactive_limits_enforced` 与
`generator_reactive_limits_certified`。GUI 默认是禁用转换的快速筛查；
一个收敛的筛查结果不得被解释为经过 Q 限值认证的工程结果。切换与
光滑 NCP 理论见 [PV/PQ 契约](../theory/pv_pq_switching_contract.md)。

平衡 Newton 潮流还在 `/api/session/pf` 与 `/api/v1` 上接受
`options.robust_nonlinear` 下述研究级数值策略：

| 字段 | 类型与生效域 | 含义 |
|---|---|---|
| `enable_smooth_ncp` | boolean | 使用光滑 FB/CHKS 延拓，不改变固定方程布局。 |
| `ncp_mu0`、`ncp_mu_min` | 有限、非负；`mu0 >= mu_min` | 初始与终止光滑水平。 |
| `ncp_mu_factor`、`ncp_mu_factor_coarse` | 严格介于 0 与 1 之间的有限值 | 细阶段与粗阶段的乘性缩减因子。 |
| `ncp_mu_phase_transition` | `(0,1]` 内的有限值 | 粗调度与细调度之间的相对残差阈值。 |
| `enable_vsc_local_schur` | boolean | 允许对所支持的固定六状态 VSC 局部块做精确消元。 |
| `vsc_schur_min_network_dimension` | 整数，归一化为不小于零 | 生产规模分界点，低于该值时保留完整稀疏 LU。 |
| `vsc_schur_local_rcond_tolerance` | `[0,1]` 内的有限值 | 每个局部块可接受的最小倒数条件数估计。 |
| `vsc_schur_backward_error_tolerance` | `[0,1]` 内的有限值 | 约简与重构后全系统按范数向后误差的最大值。 |

非法的无量纲策略值会恢复为对应的具名 C++ 默认值。机器 epsilon 是只读的
表示常量，不是 API 参数。响应在 `options_effective.robust_nonlinear` 下返回
归一化后的策略。`linear_structure` 另行报告 Schur 是否实际被尝试并被接受、
其维度与结构非零元、拒绝/回退次数、局部 `rcond`、向后误差以及采样的
半光滑速率。选项被启用并不证明该路径被采纳。这些运行时数值认证的是
求解期间装配的所选广义雅可比元素，而不是 B-次微分的每一个元素。

弹性、承载力、园区综合能源系统（IES）、EV 交通、生命周期、场景生成与
SPPT 的专用生产路由仍可在服务器源码中发现。旧的嵌入式 UI 路由不属于
本契约。

## 可靠性与保护配置

进程全局的 GUI 会话暴露一个与模型绑定的可靠性配置：

| 方法与路由 | 契约 |
|---|---|
| `GET /api/session/reliability/configuration` | 返回 schema、稀疏的已保存覆盖项、生效故障模式、稳定组件清单、覆盖度、支持诊断与校验结果。 |
| `POST /api/session/reliability/configuration/validate` | 解析并校验候选配置，不改变会话状态。 |
| `POST /api/session/reliability/configuration` | 校验、解析保护区引用、原子保存并清除缓存的分析结果。 |

`POST /api/session/run_reliability` 的顶层字段 `failure_rate_basis` 明确年故障率口径：
`operating_time` 表示设备运行期间的条件故障强度，`calendar_time` 表示含停运暴露的日历年事件频率。
后者按报告年小时数 $H$ 和修复时间 $r$ 精确执行
$U=f_{cal}r/H$、$\lambda_{up}=f_{cal}/(1-U)$；$U\ge1$、未知枚举值或非正 $H$ 均返回中文 400，
不采用稀有事件近似。响应原样返回实际采用的 `failure_rate_basis`。

主路由的 `method=physical_cut_set` 执行物理成功路径的精确容斥与最小击中集枚举。请求示例：

```json
{
  "method": "physical_cut_set",
  "physical_cut_set": {
    "components": [
      {"stable_id": "ac_branch:10", "availability": 0.9},
      {"stable_id": "ac_branch:20", "availability": 0.8},
      {"stable_id": "ac_branch:30", "availability": 0.7}
    ],
    "success_paths": [[0, 1], [0, 2]]
  }
}
```

`success_paths` 使用 `components` 数组的零基序号，稳定身份只由 `stable_id` 对外返回。后端拒绝空数组、
重复/空稳定 ID、越界序号及不在 $[0,1]$ 的可用率。响应给出 `availability`、`loss_probability`、
`reduced_success_path_count`、`minimal_cut_sets`（稳定 ID）、`minimal_cut_set_indices` 和
`exact_independent_path_model=true`。该接口要求输入路径完整描述单调相干网络，并假设元件状态独立；
超过精确枚举规模守卫时明确拒绝，不回退到路径独立近似。

三阶段请求可在 `restoration.apparent_power_polygon_sides` 指定交流支路视在功率内接多边形边数，
必须为不小于 4 的偶数，默认 16。响应返回该边数、`maximum_ac_branch_apparent_power_ratio` 以及
`validity.apparent_power_polygon_enforced`；有效结果的最大比值不得超过 1（仅允许数值容差）。
每个故障还返回 `stage1/2/3_solver_status`、认证后的 `stage1/2/3_mip_gap` 和后端原始
`stage1/2/3_solver_reported_mip_gap`。零或近零目标下后端可能报告相对间隙 1；若终止状态明确为最优，
认证间隙为 0，但原始值不会丢失。变量界、整数性和模型约束后验检查仍可将该阶段判为失败。

故障模式覆盖项以 `mode_id` 为键；保护引用使用
`component_kind + component .index` 形式的稳定 ID，例如 `ac_transformer_2w:7`。
vector position 仅以 `component_position` 返回用于诊断，不是对外身份。同一
组件类别内重复的 `.index` 值会被拒绝，因为这会使稳定映射产生歧义。

模式 schema 暴露可选的 `enabled`、hazard（`failure_rate_per_year`、
`mtbf_hours`、`forced_outage_rate`）、修复/持续时间、条件/按需概率、
信息物理恢复与残余容量字段。hazard 形式最多只能提供一种。生效优先级为：
用户模式覆盖、用户保护配置、作者算例数据、内置组件模板，
然后是所选的缺失数据策略。

每一行 `effective_modes[]` 还返回 `resolved_parameters`，其字段名与可写
schema 使用的 12 个数值字段名相同。这些值是应用上述优先级后的
规范/等效结果：解析出的年频率、等效 MTTF/MTBF、修复持续时间、稳态不可用度、
条件与按需概率、阶段持续时间、信息物理恢复与残余容量。它们是展示值，
不是已保存的覆盖项。GUI 将它们标记为继承值，只提交用户实际修改的字段；
选择不同的 hazard 表示形式会替换先前的 hazard 覆盖项，而不是把互斥的
形式一起提交。
其中 `effective_failure_rate_per_year` 始终表示设备运行期间的条件强度，
`effective_calendar_frequency_per_year` 表示用于年度后果加权的日历频率；保护误动等按日历输入的事件会同时返回二者。

保护行把一个保护装置映射到一个被保护组件、可选的后备装置以及显式的
保护区组件 ID。拒动与拒分闸概率、误动频率、清除时间、自动重合闸与
重合成功概率都通过同一 schema 往返。故障模式 FMEA 消费装置故障概率、
误动与区域变更。三阶段恢复消费保护行、配置的清除时间、自动重合闸概率、
主/后备可靠性（dependability）以及后备区域扩展；它不消费 `mode_overrides`。

对于物理起始频率 `lambda`、重合成功概率 `r` 与主/后备失效概率
`q=1-(1-p_fail_to_trip)*(1-p_fail_to_open)`，三阶段路由生成互斥的
持续场景：

```text
lambda_transient  = lambda*r
lambda_primary    = lambda*(1-r)*(1-q1)
lambda_backup     = lambda*(1-r)*q1*(1-q2)
lambda_unresolved = lambda*(1-r)*q1*q2
```

重合成功以瞬时频率报告，并从持续的 IEEE 1366 SAIFI/SAIDI/EENS 聚合中
排除。主保护与后备场景使用各自配置的清除时间。后备清除会移除
`zone_component_ids` 中每一个受支持的稳定组件；主保护加后备均未解决的
故障会阻塞恢复直到清除。后备装置的匹配保护行提供 `q2`；否则响应会声明
假定的上级后备边界。起始频率在瞬时与持续场景之间守恒。

三阶段恢复路由本身是按概率条件化的保护事件抽象。可靠性主路由另提供
`method=protection_cyber_compare`，并支持两种可执行模式：

- `protection_cyber.online_dae=false`：消费请求给出的主/后备测量轨迹；
- `protection_cyber.online_dae=true`：对每个场景执行主保护、后备保护各自的
  “故障发现 + 动作反馈”两遍 Mass-Matrix DAE，再把实测轨迹、支路跳闸、
  失电岛负荷退出和 DER-FRT 终态送入年度事件树。

两种模式都计算定时限与 IEC 60255 反时限、方向、mho/四边形距离、差动、
主后备配合、断路器动作链、信息共享依赖、QoS、共因、供电与电池依赖，并
并列返回静态 FMEA、仅保护、信息物理联合 EENS/LOLE/LOLF。在线响应额外返回
`dae_trajectories_consumed_by_event_tree` 与 `online_diagnostics`；后者逐场景
给出 DAE 和事件树的主/后备清除时刻、轨迹点数、动作反馈、失电母线稳定 ID
及 DER-FRT 消费标志。两种清除时刻的差不得超过一个 DAE 时间步。

保护误动可在 FMEA 的
`dimensions.information.protection_misoperation` 中按无故障判别窗、误跳概率、
通道和断路器成功率配置，实际进入 EENS、LOLE、LOLF，并返回分解残差。
当前在线入口采用正序网络、单相故障输入和三窗口年度后果；LCC 与显式三相
保护动态不在该结果口径内。

已保存的覆盖项是稀疏的。GUI 不会把每一行生效的内置值都变成显式的用户值。
加载不同的内置或导入模型，或通过 `/api/session/update_components` 替换
组件数组，都会清除该配置。`/api/session/load_json_string` 也会清除它，除非
调用方设置 `preserve_reliability_configuration: true`；GUI 仅在同一画布的
同步中使用该标志，之后在可靠性执行前会再次校验稳定引用。保存或导入
非空的自定义配置会使 GUI 选择 `failure_mode_fmea`，因为它是唯一消费
故障模式 ID 的方法；随后用户可以显式选择三阶段恢复以仅应用保护行。

每个 `POST /api/session/run_reliability` 响应都包含一个
`reliability_configuration` 审计对象。`configured` 报告会话是否有稀疏的
模式/保护行。故障模式 FMEA 消费模式与保护覆盖项。三阶段恢复仅当至少
一条启用的保护行匹配到枚举的预想故障时才报告 `applied: true` 与
`applied_scope: ["protection"]`；否则其 scope 为空，且其 limitation 会解释
未匹配的行或被忽略的模式覆盖项。其他方法在存在已保存配置时返回
`applied: false` 加上非空的 `limitation`；它们绝不静默暗示自定义值影响了
本次运行。

`coverage.modes_unsupported` 派生自与 `effective_modes[].supported` 返回的
同一个后果补丁（consequence-patch）决策；它不是仅基于目录的估计。对于
`dist33_microgrid_der`，当前目录有 228 个模式，其中 54 个没有后果表示：
33 个 AC 母线、2 个 DC 母线、14 个 AC 开关、3 个柔性负荷和 2 个 VSC
量测/控制模式。它们可编辑的可靠性参数仍可用，但响应携带具体的
`unsupported_reason`，而不是把它们的影响当作零。

GUI 工作流区分"已完成的近似"与"求解失败"。非空的 `model_limitations`
使后果映射被可见地标记为受限，即使指标已经算出。名称表达有效性、收敛、
强制执行或求解器准入的、值为 false 的有效性字段，会使求解/恢复被标记为
受限。传输错误或 JSON `error` 则视为失败的阶段。

对于 `dist33_microgrid_der`，普通 FMEA 目前以
`physical_model=hybrid_network_lp` 与 `model_scope=hybrid-acdc-network-lp`
完成。它包含有功 AC/DC 恢复，但不认证非线性 AC 电压/无功可行性，也不搜索
DC 侧修复切换。三阶段评估使用
`model_scope=coupled-acdc-lindistflow-restoration-milp`，并返回 `ok=true`，
且恢复 MILP、支路潮流、电压、辐射状拓扑、DC 潮流与 VSC/SOP 调度的
有效性标志均为 true。故障行为每个阶段返回带符号的 VSC 调度。纯 AC 的
`dist33_tie_demo` 仍是对应的 AC-only 认证算例。两条三阶段 HTTP 路由都在
专用的大栈工作线程上执行求解器，使求解器的栈需求不会终止 HTTP 工作进程。

## 惰性画布帧

大型分析在服务器端缓存求解结果，每次暴露一帧：

```http
GET /api/session/tspf/frame?step=12
GET /api/session/transient/frame?index=120
```

两者都返回 `schema: "canvas_frame_v1"`、`analysis`、`index`、`count`、
`time`、`time_unit`、`converged`、`capabilities` 以及作者空间的组件数据。
缺少缓存返回 HTTP 400。索引越界返回 HTTP 416。

TSPF 帧可包含母线电压、AC/DC 支路潮流、开关与断路器端子 P/Q、
Grid/发电机/储能调度、VSC/DC-DC 传输、SOC、变压器端子潮流以及
母线平衡诊断。

暂态帧只包含已记录的动态量：母线电压、频率、设备指标与已施加的事件
状态。其 `capabilities.branch_power` 为 false，且支路/断路器潮流数组为空，
除非动态求解器日后扩展为记录真实的端子电流或 P/Q。

## 错误与缓存规则

- 当另一个分析占用会话时，重型分析返回 HTTP 409。
- 非法输入与缺失前置条件返回 HTTP 400 并带 `error`。
- 帧响应使用 `Cache-Control: no-store`；服务器端会话缓存是事实来源。
- 加载或修改模型会使潮流、TSPF 与暂态缓存失效。
- 消费方在解释可选结果数组之前，必须检查 `converged`/`success` 与
  capability 标志。

## 验证

```bash
python3 tools/gui_api_e2e.py \
  --server build/tests/run_gui_server \
  --data-dir data --skip-etap
```

该 E2E 套件覆盖潮流/最优潮流画布重投影、Grid 与断路器 P/Q、TSPF 帧
获取、P/Q 诊断以及暂态不伪造潮流（no-fabricated-flow）契约。

v1 的隔离与并发契约由独立套件覆盖：

```bash
python3 tools/runtime_api_v1_e2e.py \
  --server build/macos-release/tests/run_gui_server --data-dir data
```
