> 本文档为 [development_status.md](development_status.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

# 开发状态

更新日期：2026-08-18

本文档是经核实的构建状态与在研工程工作的活交接文档。
请就地更新；不要创建带日期的副本。源码、已注册测试和当前
Git 工作树仍为准绳。

## 技术委托要求文档

已根据当前源码、公共接口、GUI、数据交换能力和已验证测试基线完成
`技术委托要求V2_已填充.docx`。文档明确覆盖交直流及三相潮流、OPF、
RPO、CPF、短路、谐波、动态、小信号、可靠性、弹性、重构、时序、市场、
碳流、HTTP/Python 接口和 GUI，并对可选 ETAP/OpenDSS 依赖、近似模型与
能力边界作出限定；未将 DLL 热插拔、PSCAD/PSASP 原生导入或通用自动参数
辨识表述为现有功能。原始模板保持不变，填充副本经 OOXML 结构检查、
占位符检查和 Microsoft Pages 全 16 页 A4 渲染复核，未发现裁切、重叠或
表格破损。

## 已验证基线

| 范围 | 结果 |
|---|---|
| 必需的 MIPSolvers 源码 | 当前钉定 `3bf1e66749e3b3e0bbd57696a7d4f43ecf09218c`；干净的兄弟工作树与之匹配，并包含 PF KLU 数值重构接口/适配器。下述 PF 回归在该三文件依赖内容提交之前即使用了相同内容。更早的通用全量回归基线建立于 `60f8bc4e4eeb58c239f83b7ff0fde1be75cd05b0`。 |
| 当前干净的 `macos-release` 构建 | 在仓库 `1773aa0e75d7` 与 MIPSolvers `3bf1e66749e3` 下，`cmake --preset macos-release` 后接 `cmake --build --preset macos-release --clean-first` 在 macOS 26.5.2 / Apple M4 Max 上成功完成。预编译的 MIPSolvers 依赖清单与当前 ABI 不匹配，因此配置正确地改用了仓库内置的 HiGHS、SCIP、Ipopt 和 SuiteSparse。 |
| 当前完整 `macos-release` 回归 | 在下述修复之后，`cmake --build --preset macos-release -j8` 完成，`ctest --preset macos-release --output-on-failure` 在 178.49 s 内运行了全部 1549 个已注册测试，0 失败：1546 通过，3 个按条件跳过。跳过项为一个不可用的 formal-SOC JSON 输入和两个不可用的外部 GridLAB-D 对比。 |
| `full-dev` 回归 | 1430/1430 个已注册测试全部无失败完成；3 个条件相关测试跳过 |
| 更早的绿色 `macos-release` 回归 | 1425/1425 个已注册测试全部无失败完成；3 个条件相关测试跳过 |
| 图 ASan/UBSan 子集 | 迭代式 Tarjan 修复后，28 个用例、113 条断言通过 |
| 可靠性 ASan/UBSan | 完整三阶段套件 25 个用例/1339 条断言；`case33mg_acdc` 477 条断言且连续五次并行重复通过；`test_1_no_sop` 79 条断言 |
| 市场 ASan/UBSan | 完整套件 22 个用例/845 条断言；聚焦的初始根割用例 1/42 |
| 其他 sanitizer 子集 | 线程池 4 个用例/6 条断言；`test_hacdcpf` 26/89；`test_acopf_dcopf_crossval` 13/119；`test_power_flow_math_audit` 42/231 |

## GUI 规模化扩展：持久索引、SoA WebGL 总览、会话共享、拓扑窗口

四部分可扩展性增强已落入工作树（未提交）：

1. `web/js/canvas.js` 新增持久索引：`state.componentById`（Map，
   在 add/remove/clear/load 路径中维护）与 `state.connectionsByEndpoint`
   （按 `compId`/`compId:portId` 分桶）。`getComponent` 为 O(1)；
   add-connection 去重、`syncConnectivity`、`buildSystemJson`、
   拖拽重路由以及可视化层中的连接重扫现均使用桶查找（拖拽
   重路由每帧为 O(degree) 而非 O(E)）。`getCompBusMap()` 带缓存，
   在拓扑编辑时惰性失效；`resultRowsByIndexOrOrder`
   预构建键→行队列映射（O(C+R)）。
2. `web/js/core/network_overview.js` 采用 SoA（Structure of Arrays，
   结构体数组）：预分配的 `Float32Array` 节点/线路/变压器缓冲区，
   按 2× 几何增长；选中更新将脏槽位合并为单次 `gl.bufferSubData`
   （点击时不再全量 `rebuildBuffers`）；命中测试使用均匀网格
   （约 4× 平均间距，每轴 ≤256 格），保持仅 LOD2 拾取语义与
   `{domain, index}` 身份契约。
3. `tests/run_gui_server.cpp` 的 Session 现在持有
   `std::shared_ptr<const HybridPowerSystem>`；全部 14 处全系统赋值点
   统一走 `session_replace_system()`，它原子地交换指针、重建常驻的
   `PowerSystemGraph` 与母线均匀网格空间索引，并使 compact-JSON
   序列化缓存失效。`/api/session/pf` 保留请求私有副本（它必须修改），
   但在锁外复制；`parameter_library/apply`、`design_handbook/apply+preview`、
   `update_carbon_factors` 与 `set_ts_config` 采用先物化再替换。全部
   16 处 `_raw_json` 内嵌经核实均位于前端消费路径上并予以保留，现由
   缓存提供；`io::to_json_dom()` 消除了 `system_summary` 及同类函数中
   `to_json`→`json::parse` 的二次转换。
4. 新增 `POST /api/session/topology_window`（schema `topology_window_v1`）：
   对 WGS84 坐标的 bbox 查询，可选 `lod` 0/1/2，与前端 `aggregate()`
   语义逐行对齐；响应携带 `units`、`coordinate_coverage` 与
   `model_limitations`（缺坐标母线会被声明，绝不静默丢弃）。
   `/api/session/status` 的 capabilities 报告 `topology_window_v1: true`。

验证：对所有改动的 JS 执行 `node --check`；基于 vm 的画布索引冒烟
加 500 个随机 `resultRowsByIndexOrOrder` 等价用例；桩 WebGL2 总览冒烟
（选中 = 0 次 `bufferData` + 1 次 `bufferSubData`）；
`cmake --build --preset macos-release --target run_gui_server` 干净；
`ctest -R 'gui_api|topology_window|runtime_api_v1'` 3/3，`ctest -R gui` 9/9，
JSON 回环子集 74/74。新索引/总览路径的浏览器交互 E2E（端到端测试，
chromium）尚未运行。

后续（同一会话）：对其余约 40 个分析端点逐一审计——23 个经核实
为只读（const-ref 求解器签名，编译验证零修改），现在直接绑定共享
快照、无拷贝；18 个确实修改工作副本的端点（`solve_power_flow` 的
就地 canonical 投影、构网型换流器转换、
`assign_available_default_profiles`、模板可靠性数据应用、
opf_ac/parity/dc 的运行点回写等）在释放锁后从快照拷贝；
`run_carbon` 现于锁外计算，`last_pf_system` 本身也是共享快照；
`export_json` 在锁外序列化，响应逐字节一致（indent=2）。
`DCBus.area/zone` 现可通过 `io::to_json`/`from_json` 回环（默认 0），
因此 `topology_window` lod=1 的 DC 分组按真实值键控；E2E 断言已
相应更新。v1 多会话 API（`src/server/runtime_api_v1.cpp`）无法共享
GUI 会话的常驻索引——其 `ApiSession` 存储是独立的、带版本追踪的
模型空间——因此它改为按会话缓存 LOD2 拓扑 DOM，由 `revision` 失效；
`/topology` 与 `/subgraph` 不再每请求重新序列化模型。仍未完成：v1
作业帧视口过滤（`frame_chunk`）在提供视口时仍按请求重建 LOD2 位置
（未加作业级缓存）；新路径的浏览器 E2E（chromium）仍未运行。

后续（结果窗口）：新增 `POST /api/session/result_window`（schema
`result_window_v1`），从缓存的 `last_pf_result` + `last_pf_system`
（两者现均为共享快照，故路由零拷贝）提供 WGS84 bbox 内逐元件的
潮流结果。会话新增 `system_revision`/`last_pf_revision`；路由报告
`result_matches_current_system`，当模型在求解后发生变化时在
`model_limitations` 中声明滞后（此时窗口退化为对已求解系统自身
坐标的线性扫描，而非常驻空间索引）。无潮流结果时返回
409 `no_cached_power_flow`。单位/限制在 `units`/`model_limitations`
中声明（vm/vdc pu，va rad，MW/MVAr，仅 AC 支路）。在带合成坐标的
ACTIVSg25k 上实测：世界窗口（25,000 节点 + 32,230 支路）
78.6 ms/10.7 MB——对比完整潮流响应的 3.4 s/106 MB——约 2% 视口
为 1.8 ms/65 KB。E2E：`tools/gui_result_window_e2e.py`（23 条断言：
409、bbox 过滤、覆盖声明、与完整潮流响应的抽样数值一致、过期
标记、混合 DC 节点）。

后续（result_window 聚合 + 图例）：`result_window` 接受可选 `lod`
（默认 2）；lod 0/1 复用与 `topology_window` 完全相同的分组键/质心，
返回聚合节点 `{group, count, vm_avg, vm_min, vm_max}` 与聚合边，
`loading_pct` 取折叠成员的最大值（保守口径；在
`units`/`model_limitations` 中声明）。总览现在为聚合 LOD0/1 视图
着色（每组偏差最大的 vm）并显示结果图例；色标已提升为共享的
单一来源常量。`gui_result_window_e2e` 增至 42 条断言。

## GUI 可用性三轮迭代（P0/P1/P2）

P0（编辑安全网）：`canvas.js` 新增撤销/重做命令栈（6 个原语
+ 复合命令，容量 200，仅结构性编辑——属性面板编辑尚不可撤销）、
组件复制/粘贴/再制（Ctrl+C/V/D，内部剪贴板，+20px 网格对齐粘贴
偏移）与多选成组拖拽（每次手势一条复合撤销）。`app.js` 新增 toast
系统（`App.toast`）、问题面板（`App.reportProblem`，经既有
`panToComponent` 设施支持点击定位）、`_canvasDirty` 上的
beforeunload 保护，以及 30 s localStorage 画布草稿
（`hysim.canvasDraft.v1`，headless 模式及约 4 MB 以上时跳过）。

P1（数据浏览）：`enhanceResultTable` 渐进升级结果表格（潮流、
OPF、短路、市场、可靠性），提供三态数值感知表头排序、逐列过滤，
以及过滤后行的 CSV 导出（RFC-4180 + BOM）；编辑器表格与虚拟化
表格按规则跳过。属性面板新增 on-input 数值校验（范围仅来自后端
schema/目录，绝不硬编码）、由既有标签白名单派生的单位后缀 span，
以及 blur 时的 JSON 预校验。新的 `core/help_panel.js` 提供 `?` 打开的
快捷键速查表（已与实际画布 keydown 处理交叉核对）并接入工具栏
撤销/重做按钮。

P2（引导/无障碍/结果可视化）：首次访问 7 步引导导览
（`hysim.tourDone.v1`，聚光遮罩，跳过/Esc，缺目标步骤自动跳过）、
顶栏帮助菜单（重放导览、快捷键面板、示例模板），以及两个示例模板
（`web/examples/ac_radial_feeder_example.json`、
`hybrid_acdc_microgrid_example.json`，字段集逐字拷贝自
`data/simple_case.json` / `data/dsp/cigre.json`，并已验证可经
`/api/session/load_json_string` 加载）。聚焦画布上的方向键微移
（20 px 网格，Shift = 1 px，连发合并为一条撤销命令）、
`core/accessibility.js` 中的模态焦点圈闭，以及实时缩放百分比指示器。

回归教训：导览的首次访问自启动在全新 Playwright 上下文中截获了
ArrowRight 与指针事件，破坏了 4 个浏览器 E2E（`pf_ncp_schur_gui_e2e`、
`gui_scale_features_e2e`、`market_gui_e2e`、`rpo_gui_e2e`）。全部 15 个
浏览器 E2E 文件现均在 `newPage` 后立即通过 `addInitScript` 钉定
`localStorage['hysim.tourDone.v1']='1'`；未来任何首次访问 UX 必须以
同样方式禁用。修复后 `ctest -R e2e`：17/17。

验证（三轮全部）：对每个改动的 JS 文件执行 `node --check`；
`tmp/` 下 8 个 node vm+DOM 冒烟框架（撤销、索引、反馈、帮助面板、
结果表格、导览、焦点圈闭、WebGL SoA）全绿；`ctest -R e2e` 17/17；
`ctest -R 'gui|topology_window|result_window'` 10/10。

## 平衡 VSC 限流 NCP

普通 `case2000_acdc` 内置算例现在按四组 0.5 pu 固定 P 传输的总量与
集中式 5% Vdc 下垂带来确定其四个 Vdc-Q 站的容量，而不再使用不可行的
原设定 `k_vdc=0.1`。在显式禁用自动回退的情况下，已注册的生产混合
Newton 回归经 37 次迭代收敛，残差 `7.263e-9`，DC 电压
`0.9491--0.9499 pu`；纯 AC 收敛不被接受。重建后的浏览器测试同样加载
普通内置算例并点击常规 PF 按钮。其精简的 `后端默认` 请求省略了回退
覆盖，后端解析为启用，观测到的求解经 5 次迭代收敛，残差
`7.633e-10`，电压合格率 100%。该测试继续覆盖 GFM NCP/Schur 专家
控件。聚焦的 Release 目标现通过 47 个用例、563 条断言，已注册的
浏览器 E2E 通过且无页面溢出。

生产统一 Newton 路径现在为每个受支持的 `PQ_MODE`、`VDC_Q` 或并网
`AC_GRID_FORMING` 换流器提供一个可选启用的固定六状态局部块。共享的
`(Pac,Qac,Pdc,Er,Ei,lambda)` 布局对功率端口模式使用恒等内电势槽位，
对 GFM（grid-forming，构网型）使用虚拟阻抗后的显式内电势。它强制
AC 电流圆盘、幅值/P 优先/Q 优先策略、能量平衡与饱和 Vdc 下垂指令。
网络与局部广义雅可比（含端口角导数）均为解析式，并在激活前后保持
同一固定稀疏布局。

完整 Release `test_vsc_limit_ncp` 目标通过 46 个用例、550 条断言。
覆盖三种优先级、完整的局部/端口雅可比有限差分校验、两个同时生效的
GFM 限值、零半径优先级平局、弱网与不可行点、三步准稳态回放、平衡
OPF 调度回放、两级 OpenDSS 验证、旧式功率端口基准、非约束等价性、
Vdc 饱和、结构准入、standard/JPC 回环、稳定身份、共享参数优先级、
生产演示、单一/多重无平衡节点 GFM 孤岛、自适应孤岛提取，以及
case300/ACTIVSg2000 算例族。精确 FB 现满足双激活原点而非添加隐藏
epsilon。可选的光滑 FB/CHKS 延拓覆盖全部三种功率端口优先级与全部
三种限流 GFM 优先级；实测求解耗时 16--25 次迭代、11 次延拓更新，
仅构建一次雅可比模式，并返回 `1.0e-12` 至 `3.6e-10` 之间的精确残差
证书。三相混合 OPF 目标通过 12 个用例/152 条断言，含 rich-to-phase
GFM 参数映射。

聚焦的暂态初始化契约通过 4 个用例、51 条断言。其主要的 15 断言用例
在网络修剪前认证稳定 ID 的 PF 到动态 Norton 种子：内电势、电流与
功率误差各不超过 `1e-8`。一个配套的可执行守卫证明旧式原设定回退在
PF 与动态中解析一致，而显式暂态 profile 覆盖会使该连续性证书失效并
发出警告。该种子证书不同于之后的全动态平衡点——当动态设备等值不
复现 PF 平衡节点调度时，平衡点可能移动。新增用例以无端子 SLACK
启动，并通过动态构建认证稳定的 GFM Norton 种子。

完整的 `tools/gui_api_e2e.py` 运行通过了全部生产 GFM/NCP HTTP 检查，
包括 Schur 策略请求/生效选项回环。`gfm_norton_limit_demo` 内置算例
以 2 条 AC 母线、2 条 DC 母线和 2 个 VSC 加载；它在 GUI 编辑中保留
全部四个原设定 Norton 字段，并返回稳定的 Canvas 引用、局部证书与
GFM 有效性标志。当前已注册的 GUI API E2E 测试通过，包括混合 Auto
OPF 恢复、GFM 演示与 case300 NCP 求解。四个换流器限值同时生效，
报告的最大 NCP 残差为 `1.5543e-15`。

平衡 PF GUI 现于专家高级组中暴露光滑 NCP 延拓与局部 VSC Schur 准入。
留空控件保留后端持有的默认值；完成的求解渲染归一化生效值与独立的
`linear_structure` 证书，含准入/回退状态、维度与结构非零元缩减、
`rcond`、后向误差与局部速率样本。版本化 `/api/v1` PF 路径接受并回显
同一策略。加入合法/非法光滑策略归一化后，聚焦的 Release
`test_vsc_limit_ncp` 重建通过 46 个用例、550 条断言。已注册的
Playwright `pf_ncp_schur_gui_e2e` 对重建的 Release 服务器通过：它经
GUI 驱动真实 GFM 算例，验证精简留空字段请求语义、生效策略回显、
运行时证书与 VSC NCP 表，并在 1440x1000 与 390x844 下均观测到零
页面级溢出。它现还在求解完成后编辑专家策略，验证先前证书被显式
标记为过期，然后分别以 Schur 开/关重跑，观测到新的 `7 / 7 / 0` 与
`0 / 0 / 0` 证书。在普通 `case2000_acdc` WebGL 路径上，它点击一行
AC 拓扑行以及 AC 母线、DC 母线和 VSC 结果行，并验证域限定的总览
选中。聚焦重建的 Release 浏览器集
`pf_ncp_schur_gui_e2e|topology_transformer_link_e2e` 通过 2/2。重建的
版本化 Runtime API v1 E2E 也以相同生效策略回环通过其完整
会话/作业/拓扑工作流。

两个版本化 MTDC/PQ-Vdc-Q 基准构建器防止纯 AC 证据被误报为换流器
限值可扩展性。已注册的 case300 求解有 300 条 AC 母线、6 条 DC 母线
和 6 个显式 VSC 块；ACTIVSg2000 有 2000 条 AC 母线、8 条 DC 母线和
8 个显式块。每个都有四个同时生效的功率端口限值。独立的 GFM 构建器
先求解该真实 MTDC 基准，保留其电压状态作为显式延拓种子，并添加一个
非约束 Norton GFM 块；其聚焦测试各通过 13 条断言。它们证明真实混合
网络上的固定稀疏结构，而非 GFM 平启动鲁棒性、大规模同时 GFM 约束
生效或 GPU 加速。

固定的两预热/五重复 Release/KLU Schur 基准在最终全套修复后重跑。
强制 case300 Schur 将维度 `640 -> 604`、结构非零元 `4820 -> 4502`
缩减，但中位线性时间从 `0.524958` 增至 `0.689582 ms`（`31.36%`），
墙钟时间从 `2.522875` 增至 `3.353792 ms`（`32.94%`）；因此生产默认
正确地让该算例保持完整 LU。相对上一轮，线性损失小 1.64 个百分点，
墙钟损失大 15.48 个百分点。ACTIVSg2000 缩减 `4054 -> 4006` 与
`29806 -> 29382`；中位线性时间从 `31.979500` 降至 `27.581291 ms`
（`13.75%`），墙钟时间从 `47.372583` 降至 `42.373500 ms`
（`10.55%`）。相对上一轮，这些加速分别低 0.10 与 1.44 个百分点。
大算例收益保持预期符号，且无测量偏离预定义的 50% 重推导阈值；
小算例墙钟开销仍是显式排除的消融项而非生产回归。Schur 启用标志、
网络维度准入、局部 `rcond` 与后向误差容差现可经生产 PF HTTP 配置
回环。非法无量纲容差恢复具名默认值；全局机器 epsilon 保持只读。

获准的 GFM 范围是平衡正序稳态/准稳态运行。无平衡节点孤岛保留全部
端子 `Vm/Va` 变量，并由一个或多个原设定固定 GFM Norton 内电势相量
锚定；不制造端子 SLACK。严格协调仍需要物理 DC 侧电压/功率支撑，
局部 Newton 法不声称跨高低压盆地的唯一性。平衡 OPF 不把 GFM
内电势/优先级 NCP 放入其 KKT 系统：生产 PF `post_pf` 回放保留 GFM
模式并提供证书。OpenDSS 独立复现非约束 Thevenin 根；在约束生效点，
它冻结 HySim 内电势并以原生端子功率 KCL 残差至多 `1e-6 pu` 重解
电路，这不作为独立的模式选择基准呈现。GPU 装配/分解仍未实现。
具体的 authored/canonical/solver/result/replay 归属规则见
`docs/model_data_semantics_contract.md`；方程与数值边界见
`docs/vsc_limit_ncp_power_flow_contract.md`。

全仓库语义守卫 `test_model_semantics_contract` 通过 5 个用例、1531 条
断言。它将全部 43 个组件 I/O 集合与运行时身份/单位/符号/模型保真度
契约配对，覆盖全部 22 个顶层生产源码模块，并锁定共享的
`DCStorage`/`StaticGeneratorDC` 执行视图与累加式母线/组件需求。本次
审计发现并修复了两条 SolverData 构建路径中 `StaticGeneratorDC.cost_c1`
的丢失。既有组件 I/O 注册表同样通过 14 个用例、373 条断言。

## 试用版集成

当前 `main` 在更新的可靠性与 GUI 工作之上重新实现了两个
`trial_design` 提交。后端持有内嵌的版本画像、指标到分析计划和
fail-closed 方法/路径策略。已知禁用路由返回 `TRIAL_FEATURE_DISABLED`；
未分类的未来 API 路由同样被拒绝。GUI 消费画像与计划，移除禁用控件，
避免急切的 dynamics-schema 请求，并保持每次求解器执行显式。Windows
打包要求干净的钉定 MIPSolvers 检出、Trial CTest 成功、必需运行时/数据
齐备，以及解包启动冒烟测试，然后才对清单和 ZIP 做哈希。

下述 Trial 验证使用更早的 MIPSolvers `7b4cba8` 钉定。该提交守护可选的
CHOLMOD 增广因子调用并恢复无 SuiteSparse 构建。其在禁用 SuiteSparse、
SuperLU、MKL、Ipopt、Gurobi 与 PaPILO 的干净 Release 构建通过了
`test_ipm_solver`：30 个用例、188 条断言。当前依赖升级与 OPF 验证
在下方单独记录。尚不对 Windows 包作任何声明。

针对该检出的干净 Debug Trial 构建重建了 `test_edition_profile` 与
`run_gui_server`。全部五个已注册 Trial 测试在 8.19 秒内通过：三个
C++ 画像/策略/计划用例、fail-closed HTTP E2E 与 Playwright GUI E2E。
在 1440x1000 与 390x844 的人工浏览器检查未发现页面级横向溢出、
工具栏重叠或控件裁切；移动端指标-计划带已加宽，使全部九个后端排序
链接终止于工作区上方。另一次干净的完整版 Debug 构建在 57.93 秒内
通过完整画像用例、`runtime_api_v1_e2e`、`reliability_workflow_e2e` 与
`reliability_dimension_validation_e2e`。JavaScript 语法、Python AST
解析、preset JSON 解析、必需打包数据齐备性与 `git diff --check` 均通过。
本 macOS 主机无 PowerShell，因此 Windows preset、暂存 DLL 启动、清单
与 ZIP 创建仍未验证。

两次完整 CTest 结果确立常规构建基线。它们不声称每个 sanitizer 入口
均为绿色。

## OPF Phase I/II 集成与性能

已检入的依赖钉定为 MIPSolvers `3bf1e66`；其干净 `main` 包含本 HySim
变更所用的经审计的中心暖启动与 NativeLCQP 协作式截止契约，以及 PF 所用
的受守护 KLU 数值重构适配器。有界 Phase I 面向平衡 Parity Native IPM
（内点法，纯 AC 与混合 AC/DC）和整体式三相混合 NativeIPM。纯 AC 保留
state/basic PF-Newton 基。混合模型先消去换流器 `Pdc` 方向并求解
DC 电导/换流器 Schur 块，然后在全局准入测试通过时使用相同的
state/basic 回退。它在装配后的 OPF 坐标中施加稀疏约束 Newton 步，
随后执行一次等式对偶与正不等式对偶/松弛拟合。声明的 state/basic
平方雅可比使用 SparseLU；非常规组件混合或激活的非线性不等式回退到
直接 SparseQR。稠密雅可比、`J J^T`、障碍参数调度、过滤器、Hessian
求解、二阶校正、弹性变量或 Ipopt 调用均不属于有限预算 Phase I。

协调后的默认值为 `mu0 = 0.1`、准入比 `eta_a = 1.0`、原始交接比
`eta_p = 0.1`、中心性容差 `eta_c = 0.5`。因此仅当原坐标违反量至多为
`eta_a * mu0 = 1e-1` 时才尝试 Phase I；其有限精度交接目标为
`max(final_primal_tolerance, eta_p * mu0) = 1e-2`。Phase II 的最终原始、
对偶与互补容差不变。因此交接点可能位于 Phase I 走廊内而不满足最终
解容差；`phase_one_in_handoff_corridor` 与 `phase_one_primal_feasible`
分别报告这些事实。

MIPSolvers 仅在以下条件下接受交接：过滤器全局化，完整且有限的
原始/等式对偶/不等式对偶/松弛向量，正的不等式对偶与松弛，原始与
扰动原始残差均在走廊内，`mu0 <= 0.1`，中心性
`max_i |s_i z_i / mu0 - 1| <= eta_c`，且最大不等式对偶至多 `1e4`。
完整对偶残差是诊断而非准入门，因为 Phase I 有意把约简梯度留给
Phase II。拒绝是全有或全无：清空每个暖启动向量并使用常规内点化。
诊断包括审计残差与原因，以及首个接受步长和障碍目标变化。固定变量
消减保持非线性、然后非固定下界、再非固定上界的行序。

迭代、原始加对偶总分解次数与每迭代回溯上限均为硬性工作量界。墙钟
截止是协作式的，因为进行中的稀疏分解无法中断。走廊感知的松弛下界
`max(2e-10, (epsilon_p - violation) / 2)` 保持 `s_i z_i = mu0`，而不会
产生统一 `2e-10` 下界所观测到的极端乘子。原构造使 case30 的 Phase II
从 17 增至 35 次迭代、混合微网从 13 增至 35 次；修订后的缩放消除了
该回归。

可复现的 `phase_one_restoration_benchmark` 协议采用两次预热加五次计时
重复。在 AppleClang 21、arm64 macOS、Release 上，case30 AC 从关闭时
`4.458 ms` 改善到开启时 `4.225 ms`（`1.055x`），同时 Phase II 从 17 降至
16 次迭代、从 16 降至 15 次分解；Phase I 将违反量
`3.106e-2 -> 4.030e-4`，中心性 `2.22e-16`。360 母线三相算例有 1080 个
相节点、2175 个变量：其 `1e-3` 点本已在走廊内，故 Phase I 未做原始
恢复迭代、只做一次对偶拟合分解，达到对偶拟合残差 `8.33e-15`，中位
`5.438 ms`。

准入防止在局部盆地之外浪费全模型稀疏工作。混合 Schur 步由其自身的
DC 平衡/换流器/参考范数审计，但不得绕过全局准入阈值。在生产默认下，
微网（`0.144 > 0.1`）与 case300 ACDC（`1.129 > 0.1`）因此在做完有界
结构探测后仍保留其精确的冷 Phase II 启动。在研究设置
`phase_one_admission_mu_factor = 2` 下，微网 Schur 块被接受，
state/basic Newton 达到 `1.689e-5`；中心交接得到认证。对 case300 有意
拒绝了同样的旁路：它花费约 160 ms 全模型 QR 却仅达到 `1.100`，因此
全局门保持权威。

DCOPF 现为 NativeLCQP 提供独立的仿射结构启动：每个带电组件在
发电机/切负荷界内做功率平衡，选择一个角度参考，约简 Laplacian 在
QP IPM 前投影角度/潮流。MIPSolvers `QPModel::x0` 审计尺寸/有限性并
报告实际使用与初始原始残差；拒绝即确定性冷初始化。在同一两预热/
五重复 Release 协议下，case57 从 29 降至 18 次迭代、`1.934` 降至
`1.284` ms（`1.506x`），case118 从 43 降至 18 次迭代、`5.841` 降至
`2.766` ms（`2.112x`），case300 从 68 降至 19 次迭代、`33.360` 降至
`10.636` ms（`3.137x`）。原等式残差分别为 `6.11e-15`、`1.53e-14` 与
`9.13e-14`；正界松弛初始化后的求解器初始残差为 `0.01`，对比冷启动的
`2.60`、`4.03` 与 `12.00`。因此残差、迭代与端到端预测在三个算例上
全部成立。

大型纯 AC Parity 求解还可以把这些结构组合为可选的调度 Phase I：
先求值常规 AC PF，仅当全模型初始对偶残差较差时才触发五次迭代的
紧凑 NativeLCQP DCOPF。DC 调度与约简 Laplacian 角度经 AC PF 回放；
仅在求值实际 Parity 初始原始/对偶指标后才接纳候选，每次失败都保留
原公式与 PF 点。这是调度暖启动而非中心状态证书：在
`case9241pegase` 上，候选对偶残差改善 `838.166 -> 236.753`，而最大
全模型原始指标恶化 `1.601e-3 -> 1.506e-1`。因此最终 Native IPM 仍负责
不变的 KKT 证书。

固定 Release/KLU 协议在 `case9241pegase` 上采用一次预热加三次计时
重复。常规 AC-PF 启动测得中位 `12.657 s`、77 次迭代、76 次 Phase II
分解；结构化调度启动测得 `11.088 s`、56 次迭代、55 次分解。端到端
中位下降 `12.4%`，Phase II 分解下降 `27.6%`，超过固定的 10%/15%
验收阈值。目标值为 `315911.571480` 与 `315911.571406`；最终原始/对偶
残差分别为 `6.49e-8/4.66e-7` 与 `5.78e-8/3.05e-7`。在 `case13659pegase`
上，基线对偶残差 `1.970` 通过预过滤，故未运行 DC 工作；守卫保留 138
次迭代、137 次分解、目标 `386116.837520`、原始 `2.05e-7`、对偶
`5.89e-9`，用时 `21.506 s`。

DC Phase I 默认墙钟预算端到端为 `2000 ms`：canonical 投影、紧凑建模、
连通分量/约简 Laplacian 种子、符号分析与数值迭代全部计费。稀疏分解
是协作式不可分单元，因此诊断会报告任何超支。在 case9241 上，DC 工作
用时 `1638 ms`，未耗尽预算，执行一次 NativeLCQP 符号分析。基线与候选
共享一个 Parity 公式（`parity_formulation_builds=1`）；仅可变的原设定
运行点种子在候选松弛构建前刷新。在 case13659 上，预过滤执行零次 DC
符号分析。

20k 级验证使用 25,000 母线的 ACTIVSg25k 算例（4,834 台发电机、32,230
条 AC 支路），Release/KLU，同一 AppleClang 21 arm64 主机。固定协议
解析一次，然后一次预热加三次计时求解。常规 AC-PF 启动样本为
`183.967`、`182.383`、`183.808 s`（中位 `183.808 s`）；每次求解经 66 次
迭代、65 次 Phase II 分解收敛到目标 `6252070.20694`、原始 `3.239e-7`、
对偶 `1.004e-7`。进程峰值 RSS（常驻内存集）为 `5.13 GiB`。

仅将默认 2 秒 DC Phase I 预算改动后，样本为 `186.588`、`184.993`、
`187.476 s`（中位 `186.588 s`，慢 1.51%）。DC 路径执行一次符号分析与
两次迭代，在 `2064 ms` 耗尽预算并超支 `64 ms`（协作式），未产生可接纳
迭代点，保留精确基线启动。因此 Phase II 仍为 66 次迭代/65 次分解，
目标与 KKT 指标逐位一致；`parity_formulation_builds` 保持为 1。完整求解
峰值 RSS 为 `4.48 GiB`；与基线峰值的差异是分配器/运行波动，不声明为
内存缩减。

预先声明的预算扫描解释了为何提高 DC-IPM 额度不是该算例的解药。5 秒
内嵌运行完成五次 DC 迭代，但仍未在 `1e-2` 交接容差内产生迭代点。
仅 DC 残差在 `10.55 s/10` 次迭代后为 `3.639e-2`，`20.27 s/17` 后为
`1.146e-2`，`31.05 s/25` 后为 `8.767e-4`；峰值 RSS 从 `2.12` 升至 `3.84`
与 `5.49 GiB`。30 秒内嵌运行达到最后一个 DC 残差，但其 AC 回放把实际
Parity 原始指标从 `3.929e-4` 变为 `9.786e-2`，对偶仅改善
`468.107 -> 448.664`（4.15%，低于固定的 20% 门）。它被正确拒绝，以
`215.012 s` 完成，比基线慢 16.98%，Phase II 工作量不变。

最初的 `O(n^1.5)` 嵌套剖分估计由 case9241 预测 45--90 秒，已被证伪。
重新推导发现是机器/成本模型与输入假设错误，而非实现失真：仅凭母线数
不能决定 KLU 填充、消去树分隔符、发电机/界行密度或跨 PEGASE 与
ACTIVS 算例族的内存流量。基准现报告母线、发电机、支路、可控 DC
预算/迭代、全部交接诊断与 POSIX 峰值 RSS。经核实的结论是：当前求解器
能以最终 KKT 证书完成该 25k ACOPF，但当前紧凑 DC Phase I 不能加速
ACTIVSg25k。未来的 20k 工作应瞄准更便宜的调度/对偶预测器或跨重复
运行点的符号复用，而不是更精确的辅助 DC IPM。

首个零分解对偶预测器探测现已实现为实验性、默认关闭选项。它从内部
发电机的驻值中位数为每个导电 AC 组件形成一个有功/无功平衡价格。
候选复用 Phase II 已装配的雅可比，不做符号或数值分解，仅当原始与
乘子归一化的全驻值残差均至少改善
`phase_one_dispatch_dual_min_improvement`（默认 5%）时才被接受。拒绝
则逐位保留常规的零等式对偶启动。JSON 与 `/api/v1` 诊断报告
尝试/接受、运行时间、两对残差与状态。

在 ACTIVSg25k 上，单次 Release/KLU 研究探测耗时 `1.978 ms`，但原始驻值
仅变化 `47278.840 -> 47276.850`，归一化驻值仅变化
`468.10733 -> 468.08762`，改善 `0.0042%`。候选因此被拒绝；求解以
`184.344 s` 完成，66 次迭代、65 次 Phase II 分解、目标与最终 KKT 残差
均不变。case30 同样恶化并被拒绝；case118 改善约 2%，但仍低于固定的
5% 生产门。这证伪了"孤岛公共部分对偶可在 25k 算例上消除一次 KKT
分解"的预测。主导驻值项位于自由发电机价格子空间之外，尤其是在
界/不等式与未覆盖变量族中。

当前研究证据支持以下优先级顺序：

| 方向 | 额外分解 | 内存生命周期 | 数据/拓扑变更后是否有效 | 实测/预期价值 |
|---|---:|---|---|---|
| 组件调度/对偶预测器 | 0 | 一次初始化 `O(nb + ng)` | 任一变更后重算 | 仅对偶探测在 ACTIVSg25k 上可忽略；原始调度重塑仍需 AC-PF 回放与端到端验证 |
| AC-PF 伴随对偶种子 | 仅当保留最终 PF 分解时一次转置求解；否则一次新分解 | PF 约简雅可比与因子必须存活到 OPF | 数值复用仅限值不变；符号复用要求 PV/PQ 与拓扑模式不变 | 有意排除：约简 PV/PQ PF 雅可比不直接覆盖完整 OPF 等式/界乘子布局 |
| 重复求解延拓加预备符号缓存 | 兼容首解后 0 次额外符号分析；数值 KKT 分解仍按 IPM 迭代进行 | 预备公式、映射、稀疏模式/排序与最近完整 `(x,lambda,z,mu)` 状态跨求解持久 | 完整延拓要求布局一致；符号排序在模式固定时耐受参数变化；拓扑/布局变更使其失效 | 时间序列/预想故障批量的最高优先级，因为 HySim 已在一个兼容公式内验证并复用完整延拓状态 |

该最高优先级路径现已实现为不可拷贝、可移动的
`opf::PreparedACOPFSession`。其精确兼容性键覆盖变量与约束块、界行
有限性/顺序、组件映射、参考行、限值设备行成员、能量路由器端口顺序，
以及 Ybus/Gdc 稀疏模式。LCC 稳定身份、AC/DC 端子、解析后的换相母线
与能量路由器端口稳定身份也被纳入，因为它们决定 KKT 行、列或公共
映射语义。相同输入复用持有的 `parity::Problem`；兼容的负荷、成本、
设定点与限值变化刷新 canonical 数值 `SolverData` 与派生系数而不重建
OPF 映射。结构审计失败会重建公式、清空 KKT 符号缓存，并阻止该次求解
上的不透明状态复用。四个延拓块全有或全无，且仍通过原生正性、中心性
与残差准入检查。每个稀疏后端还按对每个压缩 KKT 坐标的 FNV-1a 扫描
键控其符号分析，而不仅按维度与非零元数；因此等尺寸、等 nnz 但坐标
不同的图会被重新分析。

在固定 case118 Release/KLU 协议上，三次独立 Phase I 求解测得
`22.199/16.407/14.786 ms`（中位 `16.407 ms`）、27 次迭代、26 次分解、
2 次符号分析。一个预备会话测得 `18.524/7.163/6.760 ms`（中位
`7.163 ms`），重复求解为 2 次迭代、1 次分解、0 次分析。预备的
`0,+0.5,-0.5,+2,-2%` 负荷序列测得
`24.907/8.833/7.859/8.374/8.434 ms`（中位 `8.434 ms`）；最后的 -2% 点
用 7 次迭代、6 次分解、0 次分析。因此"Phase II 迭代至少减少 25%"的
固定预测在该基准上达成。最终目标与 KKT 残差保持在既有求解器容差内。
在精确 KKT 坐标缓存审计之后，一次两预热/十重复 case118 预备会话检查
测得中位 `6.121 ms`、2 次迭代、1 次新数值分解、0 次符号分析。这修正
了最初三样本计时的疑虑（中位 `8.501 ms`）：更长序列相对早前
`7.163 ms` 结果未显示稳定回归。

20k 级验收运行使用 ACTIVSg25k（25,000 母线、4,834 发电机、32,230 支路），
`prepared-power-flow`，无预热，在一个 Release/KLU 会话中两次求解。首次
求解用 `184645.248 ms`、66 次迭代、65 次分解。兼容的重复求解用
`8666.249 ms`、4 次迭代、3 次新数值分解、零次符号分析：墙钟时间减少
`95.31%`，且最终原始/对偶残差改善至 `3.389e-8/2.092e-8`。其目标
`6252069.93711` 在首解/参考目标的既定大算例容差内。进程峰值 RSS 为
`4709.6 MiB`；因此该会话在 25k 上闭合了重复求解时间目标，但未对更大
批量闭合内存效率目标。

数值因子复用仍有意不支持；普通 `factorize()` 不标注为复用。当前 Eigen
SuiteSparse 适配器与 MIPSolvers MUMPS 封装暴露持久符号分析后接新数值
分解，但没有带使用前矩阵漂移与使用后向残差/精化审计的独立跨求解因子
对象。因此启用 `prepared_numeric_refactor` 返回显式 `unsupported` 状态并
执行普通数值分解。AC-PF 伴随对偶种子也仍被排除：其约简 PV/PQ 雅可比
不匹配完整 OPF 等式/界乘子布局，且没有保留的 PF 因子时它会增加而非
减少一次分解。

文献核查与这些测量一致。Baker《Learning Warm-Start Points for AC
Optimal Power Flow》（2019，doi:10.1109/MLSP.2019.8918690）与 Cao 等
《Fast and explainable warm-start point learning for AC Optimal Power Flow
using decision tree》（2023，doi:10.1016/j.ijepes.2023.109369）预测原始
运行点，但需要离线数据与分布偏移控制。Park 等《Compact Optimization
Learning for AC Optimal Power Flow》（arXiv:2301.08840）报告高达 30,000
母线的学习暖启动，同样依赖离线模型。更直接相关的是 Taheri 与 Molzahn
《Not All Warm Starts Help: Benchmarking Primal-Dual Initializations for
ACOPF Algorithms》（arXiv:2606.08984），测试至 30,000 母线的系统，发现
多数部分原始加对偶重启更慢或更不可靠；完整覆盖才是稳健情形，且 DC
种子在计入预求解成本后失去统计显著性。WARP（arXiv:2605.05728）同样
认定完整 `(x,lambda,z,mu)` 状态是有用的 IPM 目标。Gondosiswanto 与
Pulsipher（arXiv:2606.04725）展示了跨重复求解保留结构相关符号工作的
类似参数化 NLP 收益。这些结果不足以为今日生产路径中的学习模型辩护；
它们支持的是在尝试数值重构复用之前，先做带精确兼容性键、完整状态
审计与符号缓存失效的预备 OPF 会话。

正式运行可用以下命令复现：

```bash
cmake --build build/macos-release --target opf_numerical_benchmark -j4
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode power-flow \
  --warmups 1 --repeats 3 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode structured-power-flow \
  --warmups 1 --repeats 3 --max-iterations 300 \
  --dc-max-iterations 5 --dc-time-limit-ms 2000
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode power-flow-phase-one \
  --warmups 0 --repeats 1 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case118.m --mode prepared-session \
  --warmups 0 --repeats 3 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case118.m --mode prepared-sweep \
  --warmups 0 --repeats 5 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode prepared-power-flow \
  --warmups 0 --repeats 2 --max-iterations 300
```

增强诊断目标在 `macos-release` 与本地 ASan/UBSan 配置中均已重建。聚焦
Release 与 sanitizer 运行各通过四个预备会话用例（46 条断言）与八个组合
暖启动/Phase I 用例（120 条断言）。其中包括混合 AC/DC 数值刷新与 LCC
数值对结构映射覆盖。25k 计时协议本身仅 Release，且早于最终 KKT 坐标
指纹加固；该 `O(nnz)` 安全扫描之后未重跑。不作 25k sanitizer 或完整
CTest 声明。

库 JSON、GUI 响应与 `/api/v1` 现保留 Phase I 基线/候选残差、预算、准入
状态与仅暖启动 DC 契约。AC 请求面接受有界中心 Phase I 与大型纯 AC
DC 调度控件；调用方仍须显式启用 `ac_pf_warm_start`，DC 路径默认为至少
5000 母线的纯 AC 网络。

障碍参数扫描使 case30 在 `mu0 = 0.03` 与 `0.01` 时位于准入之外，支持
保留的 `mu0 = 0.1` 默认。当前验证通过 MIPSolvers Native IPM 34 用例/246
断言、HySim 三相混合 OPF 11/145、ACOPF/DCOPF 交叉验证 13/119。完整
OPF 后端二进制通过 22/24 用例、639/644 断言；两个失败用例均为 Ipopt 或
Auto-to-Ipopt 迭代上限停滞，绕过 Native Phase I。聚焦 `/api/v1` 运行时
E2E 通过。早前重建的 GUI/API E2E 通过 68/69 检查；唯一失败是同一个
Auto-to-Ipopt 展示停滞。DCOPF 保持仿射，仅在选择 NativeLCQP 时支付
连通分量/约简 Laplacian 投影。聚焦 ASan/UBSan 验证通过平衡 Phase I
3 用例/45 断言、三相 Native 覆盖 5 用例/54 断言（该构建跳过 6 个
Ipopt 依赖用例）、ACOPF/DCOPF 交叉验证 13 用例/119 断言。本次增量不作
完整 CTest、Windows 或浏览器布局 E2E 声明。

## DER 控制与可靠性方法对比

统一可靠性请求现可应用一个可选的计算副本 DER（分布式能源）控制
场景：原设定角色、强制跟网型，或将合格可控资源提升为构网型。每个
结果暴露稳定 ID 设备审计，含原设定/生效角色、参考能力、防孤岛与
黑启动认定。这是稳态可靠性灵敏度模型；动态同步、限流与保护 FRT
轨迹仍是显式限制。禁用黑启动会移除显式的 FMEA 储能认定，而仍使用
复合 GFM/黑启动标志的方法会披露该边界。

GUI 新增确定性、蒙特卡洛与完整六方法对比集。每个生产结果携带系统
指纹与归一化计算基准。对比后端拒绝不匹配的基准，省略不可用指标，
报告最小/中位/最大、变异系数与归一化极差，并按稳定 ID 的 Spearman
相关、top-5 Jaccard 重叠与共识倒数秩比较薄弱元件排名。默认报告
跨度为对比方法间统一的 8760 小时。

聚焦 `macos-release` 验证重建了 `run_gui_server`。三个已注册
GFL/GFM 物理用例通过，`reliability_control_state_test` 通过，扩展的
`reliability_workflow_e2e` 在 8.90 秒内通过。后者使用内置信息物理
算例验证逐设备控制审计与同基准的组件 FMEA 对故障模式 FMEA 对比，
包括统计界与稳定共识身份。JavaScript 语法与 `git diff --check` 通过。
本次增量未执行完整 CTest 或 sanitizer 运行。

可靠性计算指南刷新复用了既有 `macos-release/tests/run_gui_server`
二进制，未重新构建。直接运行 `reliability_workflow_e2e.mjs` 成功完成，
并复现了文档记录的信息物理 EENS 序列 `1.0000005 -> 4.30000075`、
`0.10000005 -> 4.400001` 与 `10.000005 -> 20.000005 MWh/year`。对
`dist33_microgrid_der` 的直接生产 API 抓取返回耦合模型基线
EENS/LOLE/SAIDI 为 `4.276339 MWh/year`、`1.261167 h/year` 与
`72.861707 min/customer-year`；将 11 个合格资源提升为稳态 GFM 后，
这些值降至 `0.010627`、`0.027833` 与 `0.172683`。这些 Dist33 数值是
演算算例证据，不是已注册的跨版本数值阈值。有效性标志与方向性检查
仍是强制契约。本次文档刷新未执行完整 CTest 或 sanitizer 运行。

## 可靠性配置增强

脏工作树现注册 `reliability_dimension_validation_e2e`，它对
`dist33_microgrid_der` 与 `comprehensive_hybrid_acdc` 两个算例的七个
物理、信息与智能场景使用生产统一可靠性路由。已注册的
`macos-release` CTest 在 0.79 秒内通过 1/1。负荷倍率 1.3 使 EENS 从
`0.149400` 变为 `0.194220`、从 `15.162140` 变为
`35.964740 MWh/year`；禁用六类恢复资源使其变为 `1.656000` 与
`21.312060`。在自动与手动端点为 `0.014940/0.334800` 与
`15.449294/267.580000` 时，分解后的智能概率恰为 `0.6500736`，有效
自动化概率为 `0.58506624`，实测联合 EENS 为 `0.1476607125` 与
`120.0668358520`，在预声明的 `1e-8` 容差内匹配仿射全期望恒等式。

综合算例保留了 `-0.251539 MWh/year` 的带符号时长归因与
`+104.869081 MWh/year` 的控制归因。这不作截断：推导显式允许非单调
反事实归因——当更长的切换窗口缩短固定 MTTR（平均修复时间）修复
窗口或改变拓扑时。响应将负时长项定位到 `HV-Line-110kV` 与
`F3-Line-15-16`；端点总排序与精确分解仍通过。两个算例均声明
Level-1 独立性边界、无联合类别概率、无保护/FRT 耦合。测试复用了
既有 `macos-release/tests/run_gui_server` 二进制；CMake 重新配置披露
本地 MIPSolvers HEAD `82f4583d4f9070549c75a1ffa5d8d3bac8d9dc6f`
与记录的钉定 `60f8bc4e4eeb58c239f83b7ff0fde1be75cd05b0` 不同，
因此本次增量不作可复现重建或全套件声明。

当前脏 HySim 工作树在 `failure_mode`、GUI 服务器与 `/xjtu/` 上新增
模型界可靠性/保护配置。目录现包含此前缺失的 AC/DC/三相母线、变压器、
母线负荷、调压器、专用 DC 储能、LCC 与三相组件族。用户覆盖是稀疏的，
使用稳定组件 `.index` 身份；显式保护区域与全部后端 schema 字段可经
GUI 回环。不支持的稳态后果与保护区域目标仍是诊断，而非伪造的零影响
支持。

聚焦验证使用既有 Debug 构建目录与本地依赖脏检查覆盖；不声明
Release 结果：

```bash
cmake -S . -B /private/tmp/hysim_reliability_config_debug \
  -DCMAKE_BUILD_TYPE=Debug \
  -DHACDCDSS_SKIP_MIPSOLVERS_DIRTY_CHECK=ON \
  -DHACDCPF_ENABLE_ETAP=OFF \
  -DHACDCPF_ENABLE_IPOPT=OFF \
  -DHACDCPF_ENABLE_OPENDSS=OFF
cmake --build /private/tmp/hysim_reliability_config_debug \
  --target test_reliability_resolver run_gui_server -j4
```

`test_reliability_resolver` 通过 53 个用例、347 条断言。已注册 CTest
`reliability_configuration_e2e` 在 40.09 秒内通过，并验证了变压器与
AC 支路覆盖、每个返回模式及全部 12 个数值字段的解析值显示、稀疏
GUI 保存、全部模式/保护字段、非法输入拒绝、重名身份、同模型保持、
组件数组与替换模型重置、其他方法的显式不应用、自定义保护概率、跨
27 个已映射 Canvas 组件种类的语义属性分组、一个非发电机属性面板
编辑，以及覆盖持久化 1180 px 面板宽度后的 390 px 布局。聚焦的
`topology_transformer_link_e2e` 在 3.71 秒内通过，并验证独立与支路
承载的变压器行均选中正确的 Canvas 图元。自动化综合算例覆盖核心
AC/DC 设备族；该算例的人工 GUI 检查显示 121 个组件、266 个模式，
全部 3192 个数值单元均填入有限解析值。改动的 JavaScript 文件
`node --check` 通过。1440x1000 与 390x844 的浏览器检查未发现页面级
横向溢出；大模式表使用有界独立滚动；计算原理、全部五个执行阶段与
保护编辑器均可达。仅重建了下述聚焦 `macos-release` 目标；本次增强
未执行完整 Release/full-dev 构建、完整 CTest 或 sanitizer 运行。

三阶段恢复现消费相同的会话保护行，但不声称故障模式覆盖适用。对每个
始发故障，它将频率拆分为重合闸成功、主保护清除、后备清除与未解决
场景；配置的清除时间进入 Stage 1，后备区域扩大停电集，主加后备均
失败则阻断恢复。重合闸成功不计入持续 IEEE 1366 指标，而结果审计
始发、暂时与持续频率及其精确守恒。GUI 从响应渲染该保护/恢复审计
与控制公式。

聚焦 `macos-release` 验证现通过 35 个常规
`test_three_stage_reliability` 用例、1454 条断言。除持续 EENS 按
`r=0.8` 精确缩放到基线 20%、主/后备频率守恒、后备区域扩展与配置
清除时间外，套件现还验证耦合 AC/DC 节点平衡与电压/辐射约束、双向
VSC 与 DC-DC 效率、DC 线路限值、四类 DC DER 故障族、AC/DC/移动储能
阶段时序、移动储能转运状态、VPP PCC 边界停电，以及同号 AC/DC 母线
ID 隔离。直接 `build_dist33_microgrid_der()` 回归返回 `ok=true`，耦合
范围与全部支路潮流/电压/辐射/DC/VSC 有效性标志为真。已注册的
`reliability_configuration_e2e` 与 `reliability_default_policy_e2e` 在
检查了统一与旧式 API 保护字段、严格策略拒绝、统一回退执行与空配置
JSON 契约后，分别以 39.39 与 7.16 秒通过。在同一 arm64 macOS 26.5.2
主机、Apple clang 21、`-O3 -DNDEBUG`、提交 `493f6351` 上的直接 C++
计时探测对每个变体采用两次预热加五次计时重复。三次独立运行中，无
配置/主保护/主加后备的中位时间分别为
`0.473--0.488 / 0.477--0.480 / 0.630--0.648 ms`；双场景/单场景比为
`1.320--1.353x`，在预声明的约 `2x` 上阶预测之内，因为解析、装配与
健康状态求解是共享固定成本。隐藏诊断可用
`./build/macos-release/tests/test_three_stage_reliability
'Protection scenario runtime probe'` 复现；它排除在常规 CTest 之外。
重建 GUI 的人工检查加载了 `dist33_tie_demo`，选择三阶段后果模型，
并显示其方程与五阶段工作流。后续浏览器回归闭合了一个发现的策略
不匹配：`missing_only` 现以后端声明的统一回退值运行，而单独的
`case_data_only` 阻断零覆盖输入并提供显式的"应用参数库并运行"路径。
选择三阶段恢复现默认 GUI 故障循环为串行执行，避免对进程串行化
求解器的队列放大，同时保留显式并行启用项。本段更早的 Dist33 GUI
证据早于耦合 DC 模型，已被下方当前源码级与工作流契约取代。声称
可靠性与弹性后端接口待定的过时空结果文本已移除。

同一脏工作树现向 GUI 新增可靠性计算工作流：分方法的原理与方程、
五个持久执行阶段、目录/保护覆盖，以及由响应支撑的限制状态。由于
统一端点没有阶段进度流，GUI 把枚举、后果映射与求解标注为一个后端
区间，而非编造完成百分比。已注册的 `reliability_workflow_e2e` 使用
内置 `cyber_physical_reliability_demo` 算例并通过，其方向性 EENS 检查
（MWh/year）为：物理负荷倍率 `1.0 -> 1.5`，
`1.0000005 -> 4.30000075`；信息可用性 `1 -> 0`，
`0.10000005 -> 4.400001`；主导被动模式故障频率加倍，
`10.000005 -> 20.000005`。其完成的 GUI 阶段状态为
`complete/complete/limited/complete/complete`；受限的后果状态由运行时
模型声明支撑，390 px 视口溢出为零。重建聚焦 `macos-release` 目标后，
已注册的 `reliability_workflow_e2e` 在 11.14 秒内通过，并对两个
Dist33 变体执行真实三阶段请求：`dist33_microgrid_der` 使用
`coupled-acdc-lindistflow-restoration-milp`，返回 `ok=true`、带符号
VSC 调度，以及为真的支路潮流/电压/辐射/DC/恢复有效性标志；
`dist33_tie_demo` 仍是精确 AC 算例。重建的已注册
`reliability_configuration_e2e` 与 `reliability_default_policy_e2e` 也
分别以 50.17 与 9.78 秒通过。统一与旧式三阶段路由均在专用 4 MiB 栈
工作线程上执行核心求解器；工作流墙钟时间为此前记录的 9.20 秒运行的
`1.21x`，在预声明的 `3x` AC/DC 模型扩展预算内。这是端到端运行时间
对比，不是仅求解器微基准。另有一个响应构建失败在 LLDB 下追踪到
成功求解后传给 nlohmann JSON 的可空 `const char*`。两条路由现均构造
显式 JSON 字符串或 JSON null，闭合了空自定义配置下观测到的
`strlen(nullptr)` 进程终止。重建的 `test_reliability_resolver` 保持
53 个用例、347 条断言。更宽的 `gui_scale_features_e2e` 变压器链接
断言也通过，但该运行整体不为绿，因为后面两个无关的综合 OPF 检查
未收敛；不对该套件声明绿色结果。

## 模型参数浏览器

脏工作树扩展了 `/api/session/parameter_library` 与模型参数 GUI，加入
后端持有的展示元数据。全部 52 条底层规则返回符号、物理量、模型角色、
方程、典型范围出处与等值电路族。典型筛选范围与可编辑硬校验界显式
分离。后端现持有一个 44 条目目录，覆盖 `HybridPowerSystem` 序列化的
每个物理/系统族；选择器不再依赖恰好有标准补全规则的族。

当前系统快照展平 `hacdcpf::io::to_json` 的全部字段，使用域限定稳定
身份，并按工程语义分组。解析后的故障模式通过
`reliability_kind + component .index` 挂到其物理组件；旧式可靠性规则
组保留在 52 规则 JSON 契约中，但作为独立模型隐藏。因此 GUI 在总览中
显示 24 个物理标准规则、完整只读实例字段，以及每个匹配实例内的
可靠性参数。未知的典型/硬范围报告为未发布/未注册，而非合成。既有
保存/导入/导出/校验/应用行为保持不变；原设定实例值仍可通过 Canvas
属性编辑。

聚焦 Debug 验证重建了 `run_gui_server`。已注册的
`parameter_contract_e2e` 在 8.99 秒内通过，含 44 个目录模型、无独立
可靠性伪模型、完整字段契约检查、稳定 Canvas 选中、画像保存回环与
移动端溢出检查。已注册的 `reliability_configuration_e2e` 在 38.63 秒内
通过。对 `comprehensive_hybrid_acdc` 的真实浏览器检查发现 108 个稳定
实例；选中的发电机暴露 76 个分组行，含 41 个解析可靠性行，无可见
重叠。`web/js/app.js` 与 E2E 脚本 `node --check` 通过。本次 GUI 增强
未执行 Release/full-dev 构建、完整 CTest 或 sanitizer 运行。

同一回归还演练了配电算例中每个可映射实例族、同号 AC/DC 母线、连续
AC 支路、VSC/储能、变压器别名、当前值刷新、未保存画像编辑保持与
模块/标签稳定性。在模型参数激活时选中 Canvas 图元会跟随精确的域
限定实例，而不重建或丢失未保存的画像编辑。

## GUI 工作区密度

脏工作树现提供以工作区为先的 GUI 布局，仿照常见工程仿真工具：标准
与紧凑密度、独立的组件库/上下文 ribbon/检查器/控制台 dock、保留
活动模块与检查器标签的专注模式，以及持久化布局状态。桌面首次加载
使用紧凑密度并收起控制台。在 720 px 及以下，组件库、上下文 ribbon、
控制台与仅桌面的依赖 chip 初始收起，而全局元件定位与活动的右侧
参数/结果视图保持可达。

已注册 Debug CTest 选择通过 5/5：`top_toolbar_semantics_test`、
`workspace_layout_e2e`、`topology_transformer_link_e2e`、
`parameter_contract_e2e` 与 `reliability_configuration_e2e`（共 54.21
秒）。在 1440x1000 下，专注模式把实测 Canvas 区域从 959x649 增至
1115x919，同时保留短路模块、拓扑标签与控制台内容。布局回环经受住
重载；真实 390x844 Chromium 检查报告零页面级横向溢出且无重叠控件。
JavaScript 语法与 `git diff --check` 通过。本次纯前端变更未执行
Release/full-dev 构建、完整 CTest 或 sanitizer 运行。

## 活跃模块审计

活的[模块代码审计](../testing/module_code_audit.md)记录当前有源码支撑
的发现与审计深度。AUD-001 至 AUD-011 已闭合。聚焦运行时契约现覆盖
`graph/`、`scenario_generation/`、`carbon_analysis/`、
`integrated_energy/` 与 `sppt/`；规范链接见 [docs/README.md](README.md)。

当前源码 Debug 验证重建了全部受影响目标。聚焦与契约套件通过：台风
交通/目录 5 用例/35 断言、园区 IES 6/106、谐波 52/359、EV Formulation D
15/196、图/Kron/回环 55/461、场景生成/schema 12/105、碳快照/年度/GEC
41/574、SPPT 33/213，以及共享的弹性/可靠性可执行 39/360。
`run_gui_server` 也重建并链接成功。

AUD-011 通过在新建的 joined 线程上运行 StrictHiGHS 恢复 B&C 调用、同时
保持 Native 在调用方线程而闭合——递归子 MIP 需要调用方线程更大的栈。
同进程回归完成五次 Native 到 StrictHiGHS 循环。一个 1000 次循环的
packaged-task/jthread 基准测得每循环 0.0145--0.0187 ms 固定开销，低于
10 ms 阈值。本次闭合未专门运行完整 CTest、完整 sanitizer 套件或外部
引擎交叉验证。

## 已闭合调查

HySim 工作树当前包含未提交的依赖、文档、图、可靠性与图测试变更。
依赖此列表前请重新运行 `git status`。

- `src/graph/topology_analysis.cpp` 修复了迭代式 Tarjan 遍历中一个失效
  的栈帧引用，并为平行边正确性保留精确的父边。聚焦 ASan/UBSan 测试
  通过。
- `src/reliability/three_stage_reliability.cpp` 跨预想故障工作线程串行化
  进程级 HiGHS 与原生 B&C 求解状态；原生回退使用一个内部线程。携带
  求解器的可靠性工作线程通过可选 `ThreadPool` 栈大小参数请求 4 MiB
  POSIX 栈。
- 在该闭合点，兄弟 MIPSolvers 工作树干净，位于 `5a5fad0`。更早的本地
  HiGHS 实验不是待处理的工作树状态。

`case33mg_acdc` 中表面上的 `HPresolve::changeImplColUpper` 容器损坏是
工作线程栈溢出的下游症状，而非预求解数据结构缺陷。在 Darwin 上，默认
pthread 栈约 512 KiB，而 sanitizer 插桩把观测到的最大求解器栈帧扩大到
约 340 KiB。同一算例在主线程上完成，并行算例在配备 4 MiB 工作线程栈后
稳定。连续五次 sanitizer 重复在 414.63 秒内完成。所得 case33mg 指标为
SAIFI 0.7833、SAIDI 10.87、EENS 588.5；case33bw 交叉核对产生
SAIFI 0.7787、SAIDI 10.60、EENS 570.3。

## 性能闭合

一次更早的清理前 MIPSolvers Release 实验在同一 arm64 主机上用 A/B/C
三明治设计对比了修改后二进制与钉定提交的干净导出：24 个 NETLIB 算例、
Native 与 HiGHS、3 次重复、单线程 HiGHS、30 秒求解上限。每次运行均
精确（每个求解器在每段均为 72/72）。此结果保留为历史性能证据，而非
声称当前兄弟工作树为脏。

| 段 | Native 几何均值 |
|---|---:|
| 干净 A | 3.761956 ms |
| 脏 B | 3.761257 ms |
| 干净 C | 3.763764 ms |
| `sqrt(A * C)` 对照 | 3.762860 ms |

脏对三明治对照为 -0.0426%，在预声明的 1% 绝对验收阈值内。移除计时
字段后，JSON 运行记录在 A/B/C 间完全一致，包括状态、目标、可行性、
迭代与对偶主元计数。原始报告为
`/private/tmp/hysim_native_dual_{control_a,experiment_b,control_c}_repeat3.{csv,json}`。

## PV/PQ 无功限值切换

平衡 Newton 潮流现仅在固定活跃集收敛点附近执行发电机 Q 限值转换。
全部越限 PV 母线在一批内进入其 Q 界。PQ 到 PV 恢复至多审计一次，
且要求配置的保持计数加既有电压方向裕度。返回收敛的活跃集签名、
外层预算耗尽、最终 Q 违反量与显式证书。GUI 默认禁用转换的快速筛选，
并单独标注 Q 限值认证。

光滑 NCP 对 PV/PQ 与优先级投影使用 CHKS 中位数光滑化，对幅值优先
VSC 块使用 Kanzow 光滑 FB。延拓持有局部 `SolverData` 副本，在首次
求值前初始化 `mu`，每次缩减后重新装配残差与雅可比，且在达到
`ncp_mu_min` 前不能终止。VSC 延拓可在不启用发电机 PV/PQ NCP 选项的
情况下启用。公共换流器证书按精确 `mu=0` 方程重新求值。

Release 验证重建了 `test_power_flow_math_audit`、`matpower_pf_compare`
与 `run_gui_server`。完整数学审计通过 50 个用例、305 条断言。
ASan/UBSan 构建通过其配置的 42 个用例、260 条断言；LeakSanitizer 在
macOS 运行时不可用。JavaScript 语法与 `git diff --check` 通过。

在 `case6515rte` 上，Q 限值强制经 9 次 Newton 迭代、8 次分解收敛，
74 次 PV 到 PQ 切换、无恢复、无重复活跃集，认证最大 Q 违反量
`4.22e-11 pu`。普通布局的五重复 Release 运行测得 `56.4--61.2 ms`，
中位 `58.7 ms`；固定布局对比记录于下方平启动后续。快速筛选此前为
4 次迭代、3 次分解，中位约 21.2 ms。

原始 `data/云南案例.json` 暴露的是一个独特的输入/投影缺陷，而非活跃
集振荡。它包含 587 条旧式 BPA 零数据 L 卡联络线，编码为物理
`x=1e-4 pu` 支路。这些行上冲突的 PV 设定点产生 `O(Delta V/x)` 无功
环流；旧求解达到约 `410 pu` 残差与 `8736 pu` 最大 Q 违反量，且未重复
活跃集。BPA 导入现将精确零 L 卡标记为理想连接。JSON 回环保留该出处，
严格的 BPA-schema 迁移识别旧式编码，同时排除普通短物理线路。
Canonical 投影执行 585 次有效合并，在 Ybus 与 PV/PQ 构建前将 4512 条
原设定母线缩减为 3927 条 canonical 母线。

未修改的云南 JSON 经重建的 Release GUI 服务器加载后，Q 限值强制在
19 次总 Newton 迭代、5 次外层求解内收敛并认证：218 个 PV 到 PQ 条目、
3 次恢复、215 条最终受限母线、无重复活跃集或循环、`7.28e-12` 电气
残差、`2.27e-13 pu` 最大 Q 违反量。冷求解/展示/序列化前总计时为
`71.9/224.2/296.8 ms`；五次热紧凑 Q 限值求解中位 `47.3 ms`。禁用 Q
的筛选测得 `18.3 ms`。

Release 验证另外通过 `test_io_json`（41 用例、398 断言，一个数据相关
跳过）、`test_bpa_io`（25/885）、`test_component_models_math_audit`
（15/111）、`test_power_flow_math_audit`（49/304）与 `test_advanced_pf`
（30/218）。重建的 `run_gui_server` 即用于完整云南请求的二进制。

生产 HTTP 云南能力审计可用 `tools/validate_yunnan_capabilities.py`
复现；其合并的机器可读与人读报告为
`output/yunnan_capability_audit.json` 与 `.md`。它覆盖 46 个探测且不
编造缺失算例数据：14 个通过，6 个带显式输入限制运行，20 个被正确
分类为不适用，3 个返回数值不收敛，3 个耗尽声明的墙钟预算。

PF 对比确认固定活跃集 Newton 路径是该算例正确的生产默认。筛选用
`16.0 ms` 求解时间、7 次迭代。Q 认证 Newton 用 `42.6 ms`、19 次总
迭代、218 个 PV 到 PQ 条目、3 次恢复、无循环。分布式平衡节点
（`1081 ms`）、HELM（`443 ms`）与 Newton-Krylov（`378 ms`）收敛但明显
更慢。FDPF 在 1000 次迭代后未收敛停止；显式同伦在 110 次迭代后停在
残差 `1.35` 与未认证 Q 违反。DC 与混合线性化 PF 仍是有用的筛选模型，
不能替代非线性 Q 证书。

带 AC-PF 暖启动的 Parity OPF 在有无中心 Phase I 两种情况下均经 37 次
迭代、36 次分解收敛。Phase I 测得 `0.66 ms`，观测初始约束违反
`0.2199 > 0.1`，以 `outside-phase-one-admission` 终止，并正确地保持
常规 Phase II 启动不变。核心求解时间带 Phase I 为 `676.7 ms`、不带为
`675.3 ms`；差异是噪声。DCOPF 核心时间 `217 ms` 完成。由于该算例无
原设定发电机成本曲线，这些运行认证的是可行性与求解器工作量，而非
经济保真度。

拓扑、投影守卫、网络降阶、谐波零注入、承载力、碳流管线与场景生成
均正常返回。该系统为单连通 AC 孤岛，有 673 个环。通用图降阶在
`159 ms` 内消除 143 条母线/支路；SPPT 准入通过。承载力在 `27.7 ms`
内求值 2428 条 BPA 变压器支路。碳追踪在 `11.35 s` 内求解秩 3927 的
canonical 矩阵，报告相对残差为零，但因无原设定碳因子，结果使用回退
因子。场景生成耗时 `2.36 s`，产生 6344 个默认可靠性预想故障；这些
默认值是工作流证据，而非云南随机数据证据。

云南短路与碳流规模失败已在脏工作树中闭合。详细序矩阵为稀疏，KLU
因子跨批量故障位置共享，选定的逆列替代完整 `Zbus` 矩阵，逆对角元
使用有界 RHS 块与协作式取消。GUI 全母线路由现使用其文档化的正序
总览模型；选定故障路由计算完整故障点 duties 与全部剩余电压分布，
不含未用的非故障电流对角元。Release HTTP 墙钟时间为 4512 行总览
`74.2 ms`、单个详细故障 `29.7 ms`，而此前两条路径均超过 90 秒。碳
追踪与节点强度现全程保持稀疏；同一算例从 `11.35 s` 降至 `1.469 s`，
秩 3927，相对残差为零，条件数估计有限值 `5.173`，全部有效性标志为
真。聚焦 Release 测试通过 523 条短路断言、8 条 IEC 断言与 594 条碳
断言。

两个规模失败仍未闭合。默认反事实规划超过 180 秒且忽略取消，因为它
嵌套基线、五个度量以及跨经济、碳、可靠性与弹性模型的至多十对评估。
它需要共享的预备评估会话、有界维度选择，以及每个克隆/度量/成对边界
上的取消。最后，四步默认时间序列 profile 返回了电压数组但零收敛步；
全部四个最小值触到 `0.05 pu`。该算例无原设定负荷或发电机 profile，
因此默认 `0.4--0.5` 负荷倍率不是可采纳的云南运行轨迹，不得呈现为
成功的生产仿真。

GUI/API E2E 通过全部 PF Q 证书与快速筛选断言，但完整运行为 70/71，
因为独立的混合 Auto OPF 检查触到 Ipopt 迭代上限。浏览器 E2E 通过新
默认/控件断言与全部大系统 PF 检查；仍不为绿是因为两个既有多尺度
综合 OPF 断言失败。不对这些 OPF 路径作全套件绿色声明。

82,000 母线的 `case_SyntheticUSA.m` 暴露了一个独立的 GUI 编排回归。
`ac_newton` 路由检测到三个可解 AC 孤岛，运行自适应孤岛求解器，然后
仅为填充展示/缓存数据又跑了第二次全网络 Newton。第二次求解已移除。
自适应结果现聚合逐孤岛 Q 限值证书与剖析，并直接用于支路潮流重建、
紧凑展示与 PF 缓存状态。启用 PV/PQ 转换的精确 Release GUI 请求在
2.58 秒内收敛，4,820 次 PV 到 PQ 切换、76 次恢复、无重复活跃集、
最大 Q 违反量 `1.34e-11 pu`。此前的双重求解在同一服务器上约需
5.80 秒。聚焦 Release 自适应测试通过 30 用例/218 断言；ASan/UBSan
集成与自适应选择通过 4 用例/23 断言。

## 潮流平启动鲁棒性后续

已提交的 PF 后续闭合了大算例 PF 评估中的两个正确性缺陷。无功限值
认证现使用 `max(1e-10, PowerFlowOptions::tol)`：受限母线 Q 违反量是
剩余 Q 行失配，不能认证得比产生它的根更紧。聚焦回归有意以高于
`1e-10`、低于 `1e-3` 的违反量收敛，且仍被认证。可选的
Levenberg-Marquardt 恢复不再通过 `J` 的符号分析分解
`J^T J + lambda I`；它持有独立稀疏求解器，对正规方程模式分析一次、
跨阻尼尝试复用该模式，并报告其分析、分解、求解与线性耗时。

默认线搜索 Newton 现只检测高残差停滞：最佳残差必须在 15 次迭代内
未能改善 10% 且保持高于 `1.0`。近根阻尼进展保留调用方的完整迭代
预算。对结构性闭合的数值失败，公共默认升级阶梯现是确定性的：直接
固定布局 Newton、半光滑 NCP、线性化 DC 角度种子加固定布局活跃集、
DC 种子 NCP，最后同伦兜底。内部重试持有自己的 `SolverData` 与
`NewtonSolver`，不经门面递归，保留调用方的 Q 强制语义，并在
`SolverProfiling` 中聚合其工作量/出处。LCC 变压器分接头控制保留其
专用外层循环，不被直接重试绕过。

`enable_fixed_pv_pq_layout` 现为 C++ 默认。它把每条非平衡 AC 母线的
`Vm/Q` 坐标嵌入一个稳定的 canonical 母线位置顺序。PV 行变为 `Vm`
恒等行，Q 受限行保留物理 Q 平衡；外层活跃集计时与证书语义不变。
半光滑 NCP 保持互斥，在两个选项同时请求时胜出并给出显式警告。行
非零下标预计算一次，使活跃行替换为 `O(nnz(row))` 而非完整压缩矩阵
扫描。

在 AppleClang 21 arm64 macOS、Release/KLU、工作树 `3cc92658373a` 上，
`case6515rte` 把雅可比模式重建/分析从 `3/3` 降至 `1/1`，9 次 Newton
迭代、8 次分解、74 次 PV 到 PQ 切换与 Q 证书（`4.22e-11 pu`）不变；
五重复中位求解时间 `58.7 -> 40.9 ms`（`30%`）。`case_ACTIVSg10k`
从 `10/10` 降至 `1/1`，26 次迭代、25 次分解、1151 个条目、40 次恢复
与证书不变；其中位 `261.1 -> 152.0 ms`（`42%`）。默认固定布局不声称
减少外层活跃集求解次数。

硬平启动验收集在不调同伦的情况下闭合。Release/KLU 默认求解收敛并
Q 认证如下：`case1888rte` 用 DC 角度/固定活跃集阶段 21 次迭代，
`case3375wp` 用半光滑 NCP 9 次，`case6468rte` 用 DC 角度/固定 21 次，
`case6515rte` 用 DC 角度/固定 30 次，`case_ACTIVSg10k` 用 DC 角度/
固定 29 次。均未到达同伦。平启动 `case118` 保持直接 Newton，有无
外层阶梯均 18 次迭代收敛；热 `case3375wp` 保持 7 次迭代，热
`ACTIVSg10k` 保持 26 次，零次升级尝试。

`PreparedPowerFlowSession` 现跨兼容求解缓存 canonical 投影、
`SolverData`、固定雅可比模式与符号分析。同布局的原设定负荷/设定点
变化刷新数值而不投影或装配；拓扑/网络参数变更保守重建。在
`case6515rte` 上，五次 Release 重复经普通门面的中位为 `28.962 ms`，
经一个预备会话为 `16.463 ms`（低 `43.2%`）；预备重复报告零投影、
装配与符号分析工作。

PF 线性路径现仅在既有分解且模式不变时请求 KLU 数值重分解。每个候选
求解由范数型后向误差审计守护，失败或误差过大时回退全新选主元。在热
`case_ACTIVSg10k` 上，五重复 Release 协议将中位总时间从 `154.032`
降至 `104.024 ms`（`32.5%`），末次求解线性时间从 `98.549` 降至
`54.699 ms`（`44.5%`）；全部 24 次重分解被接受，最大后向误差
`2.29e-17`。一个零容差回归强制每个候选走完整分解回退且仍收敛。

剖析现覆盖残差求值、雅可比缩放、活跃集扫描、投影、装配、结果推导、
求解器核心/门面总计、未分类余项、预备会话复用与升级出处。W4 清理
也已闭合：FDPF 电压除法使用配置的下限，GFM 导数使用 `min_vm_pu`，
Newton-Krylov Schur 预条件子复用符号分析，独立 DC 求解器使用解析稀疏
雅可比/SparseLU 而非稠密有限差分与 `FullPivLU`。

Release/KLU 聚焦验证通过 `test_power_flow_math_audit` 50 用例/305
断言、`test_advanced_pf` 30/218、`test_homotopy_continuation` 5/36、
`test_newton_krylov` 5/30、`test_hacdcpf` 27/117，以及平启动/KLU
MATPOWER 过滤 3/45。禁用 SuiteSparse 与泄漏检测的 ASan/UBSan 通过
43/261、30/218、5/36、5/30、预备会话过滤 1/28 与 MATPOWER 过滤
3/41。随后观测到的八个 Release 失败现已闭合。混合 OPF 测试 34、62、
73 及其 GUI/RPO 后果 1531、1538、1548 在存在 DC/VSC 变量时使用 Ipopt
有限内存 Hessian 近似；纯 AC 问题保留经审计的精确 Lagrangian Hessian。
这避免消费一个不完整的混合二阶块，而不声称该块已认证。测试 904 现
在计及 `r=0.01 pu` 支路的正损耗的同时检查其实际的原设定顺序契约。
动态初始化测试 1127 现将 rich 系统路由经生产 PF 投影门面，而非把
稳定母线 ID 解释为 canonical 位置。首次完整重跑暴露了相关的
canonical 输入测试 1148；显式 `projection_certificate` 拆分现防止对已
canonical 系统重复投影，聚焦测试 1127/1148 均通过。最终
`macos-release` CTest 在 178.49 s 内运行全部 1549 个已注册测试，0
失败：1546 通过，3 个条件相关测试跳过。跳过项为一个不可用的 formal
SOC JSON 输入与两个不可用的外部 GridLAB-D 对比。聚焦 NCP/Schur GUI
测试、Canvas/WebGL 链接、混合 Auto OPF 与 RPO 交叉验证均通过。主仓库
位于 `1773aa0e75d7`；干净的 MIPSolvers 依赖位于钉定的 `3bf1e66749e3`。

## 快速定位

```bash
git status --short --branch
git -C ../MIPSolvers status --short --branch
cmake --build --preset full-dev
ctest --preset full-dev
cmake --build --preset macos-release
ctest --preset macos-release
```

用户向能力基线见 [README.md](../README.md)，架构与不变量见
[AGENTS.md](../../AGENTS.md)，主题文档见 [docs/README.md](README.md)。
