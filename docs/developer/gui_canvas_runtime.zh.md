> 本文档为 [gui_canvas_runtime.md](gui_canvas_runtime.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

# 画布运行时与结果回放

南方市场模块通过 `web/js/core/market_canvas.js` 在主视口显示独立市场拓扑，
保留原工程 Canvas；按市场稳定 ID 联动场景/日期/时段、点击、播放和局部邻域。
有效性、单位和回归证据见[市场 Canvas 契约](../modules/market/southern_execution_contract.md#市场-canvas-身份与时段契约)。
下述工程 WebGL 总览使用另一套视图和身份空间。

对于大模型，主视口不再是一块空的 headless 表面。
`web/js/core/network_overview.js` 使用 WebGL2 与 LOD0/1/2 聚合渲染完整的
母线/一次边图，且不创建任何逐母线的 SVG DOM。渲染状态存放在预分配的
类型化数组顶点存储中：选中变化通过合并的 `bufferSubData` 脏区间修补节点
颜色，而不是重建缓冲区；LOD2 拾取使用建立在节点位置上的均匀网格空间索引，
而非线性扫描。

当足够多的母线带有真实坐标（≥80% 非默认经纬度）时，总览切换为视口驱动
的拉取：它针对当前视口（每侧向外扩展 40% 预取边距）请求
`POST /api/session/topology_window`，在平移/缩放后防抖 250 ms 发出，并且
当视口仍落在同一 LOD 下已拉取的窗口内时完全跳过该请求。LOD 选择根据估计
的可见节点数自动进行（≤2,500 → LOD2，≤30,000 → LOD1，否则 LOD0），除非
用户在 LOD 下拉框中固定了某个级别（`自动` 选项会重新启用自动）。无坐标的
母线由后端排除；其 `coordinate_coverage` 与 `model_limitations` 会原样呈现
在总览状态行和 `stats()` 中。坐标覆盖不足的系统——或首次窗口拉取失败的
系统——回退到历史沿用的本地全系统渲染，使用手动 12,000 母线的 LOD 启发式
规则。

窗口化模式还会叠加缓存的最近一次潮流结果：在请求每个拓扑窗口的同时，它以
相同 bbox、相同 LOD 请求 `POST /api/session/result_window`，并且当一次新的
潮流完成时，`Canvas.showPowerFlowResults` 触发一次仅结果的重新拉取。LOD2
节点按 `vm_pu` 着色，采用与 SVG 结果叠加层相同的分档（<0.95 pu 红色，
>1.05 pu 橙色，带内绿色/青色），支路按 `loading_pct` 着色，采用与热力图
相同的绿→黄→红渐变（截断于 150%）。在 LOD0/1 下，后端返回组级聚合：
组节点按其成员中最差的电压带偏差着色（`vm_min`/`vm_max` 超出
[0.95, 1.05]），否则按 `vm_avg` 着色；聚合边取其折叠支路中最大的
`loading_pct`——这两种语义都由后端在 `units`/`model_limitations` 中声明，
并呈现在状态行和 `stats()` 中。结果颜色写入 SoA 颜色槽位，通过合并的
`bufferSubData` 脏区间上传；被选中的母线保留其选中色，取消选中时回退到
其结果色。`409 no_cached_power_flow` 响应会静默地将着色清除回结构性域
颜色。`result_meta.converged = false` 或
`result_matches_current_system = false` 会在状态行和 `stats()` 中声明。
结果着色激活期间，总览显示一个小型图例叠加层（由 JS 以内联样式创建，
`aria-label="结果着色图例"`）：三个电压带加上带数值标注的负载率渐变；
无激活结果时隐藏。电压带与负载率渐变在 `network_overview.js` 中作为共享
常量定义一次，同时供 WebGL 着色与图例使用。本地回退路径从不拉取或应用
结果颜色。

既有 SVG 画布对于常规规模的手绘模型图仍然是权威载体。大模型的选中操作
在 WebGL、虚拟化拓扑表、结果导航和有界本地 SVG 子图之间统一使用一个
`{domain,index}` 母线引用。对于常规 SVG 模型，拓扑表的组件行使用稳定的
组件 `.index` 映射。从 AC 支路提取出的双绕组变压器会同时注册在其支路索引
和变压器索引之下，因此任一表格都能选中同一个变压器图形，而不会丢失到
富设备的链接。

组件属性编辑器按语义分组手写字段：标识与连接、电气/额定、运行/控制、
保护/安全、成本/碳、可靠性、分析专用、数据/来源，以及动态。可靠性分组
对每个被选中的画布组件都存在，并从后端的
`component_kind + component .index` 清单解析其失效模式。能量路由器的端口
模式显示在其父路由器上，支路承载的变压器同时保留支路与变压器两个别名。
全部 12 个后端数值模式字段都会显示；继承得到的有效值使用虚线样式，而
Apply 只提交用户实际编辑过的字段。

模型参数（Model Parameters）模块对其只读的当前值快照遵循同一标识规则：
`domain:component_kind:component .index`。其 profile 表格编辑的是已注册
默认值与校验策略；手写实例值仍归画布属性编辑器所有。该模块激活期间，
选中一个已映射的画布图形会同时更新组件模型过滤器和精确的当前实例。AC/DC
母线保持域限定，支路承载的变压器在存在专用变压器别名时优先使用变压器
标识，否则使用其 AC 支路标识。选中刷新会保留未保存的 profile 表格编辑。
因此，显示的典型范围或 profile 默认值不会静默地变成实例覆盖值。

组件模型选择器由后端 `model_catalog` 填充，当前为 44 个可序列化的物理/
系统族，而非更小的标准规则集。当前实例行是展开后的权威系统 JSON 字段，
按标识/连接、电气/物理、额定/限值、运行/控制、成本/碳、可靠性和动态分组。
解析出的失效模式被整合进所选物理实例；遗留的 `Reliability - ...` 注册表
分组不作为独立模型显示。未知的典型范围或校验范围会明确标记为未发布或
未注册。

## 工作区布局

GUI 采用工程工作站式布局，而不是让每个工具面常驻打开。首次桌面会话使用
紧凑密度和折叠的控制台；首次访问且视口宽度不超过 720 px 时还会折叠元件库
和上下文功能区。用户可以独立显示或隐藏元件库、上下文功能区、右侧检查器和
控制台。标准密度始终保留全部原始工具栏命令，而紧凑密度只暴露主要画布命令，
次要命令可通过返回标准密度获得。

专注模式隐藏工作流指引、依赖状态、上下文功能区、元件库和控制台，同时保留
当前模块、检查器标签页、结果、手写模型以及控制台内容。退出专注模式会恢复
此前的停靠偏好。密度、停靠可见性和专注状态通过 `hysimWorkspaceLayoutV1`
往返持久化；快捷键为 `Ctrl/Cmd+Shift+F`。布局变化会触发图表和视口的尺寸
重算处理，而不会重建画布模型或改变结果标识。

## 用户反馈与数据保护

瞬态通知使用堆叠式 toast 容器（`#toastContainer`，`aria-live="polite"`，
错误级 toast 带 `role="alert"`），对外暴露为
`App.toast(msg, level, {sticky, actions})`。`log()` 将 warn/error 控制台
条目镜像为 toast（info 仅留在控制台）；`setStatus(text, 'error')` 报告具体
的失败文本，并跳过那些细节已通过 `log('error')` 报告过的笼统单词状态。
两个通道都会在 2 s 窗口内对相同消息去重。

常驻的问题（Problems）标签页（`#problemList`）与日志共享底部控制台视口：
每条 error 级控制台条目都会被自动收集，而
`App.reportProblem({level, message, compId?, bus?})` 还会附加一个可选的画布
定位目标，渲染为键盘可达的 定位 按钮，复用
`panToComponent`/`panToBusId`。条目携带时间和级别，3 s 内连续相同消息去重，
上限 100 条。属性面板校验失败（JSON 字段解析、可靠性配置保存）随被编辑
组件 id 一起上报；单母线短路失败随故障母线一起上报。headless WebGL 总览
模式下定位被禁用。

属性编辑立即校验，而不是仅在 Apply 时校验：数值输入在 `input` 事件上标记
红色边框 + `title` 提示，并在 `change` 事件上（带组件 id）上报到问题面板；
取值范围来自元素自身的 `min`/`max` 属性——由后端可靠性 schema 或动态模型
目录填充——未声明范围的通用字段只做合法性检查（无任何硬编码）。JSON 文本
域（`dynamic_model`、`*_profile_values`）在 `blur` 时按 Apply 所强制的同一
规则预校验语法与形状。`COMP.fieldLabels` 尾部括号中被识别的单位（白名单见
`splitPropertyLabelUnit`）会从标签移入输入框后缀 span；未识别的括号提示
（`0=HV,1=LV`、`JSON数组`、……）保留在标签中——不凭空发明任何单位。

只要 `_canvasDirty` 为 true，`beforeunload` 守卫就会弹出提示；该标志在
后端同步成功或加载全新模型时清除，因此提示只会在确有未保存工作时出现。
画布处于脏状态且页面可见期间，自动草稿计时器（30 s）将
`Canvas.buildSystemJson()` 以 `{saved_at, json}` 形式存入 localStorage 的
`hysim.canvasDraft.v1` 键下。headless 大系统模式下以及序列化系统超过 4 MB
时跳过草稿（每会话通知一次）；启动时一个粘性 toast 提供明确的 恢复草稿/丢弃
操作——画布永远不会被静默覆盖。恢复会把存储的字符串 POST 到
`/api/session/load_json_string`，然后对本地副本运行
`Canvas.loadFromSystemJson`，本地副本仍携带着后端会剥离的 `_canvas` 布局块。
加载任何模型都会将已存草稿作为过期数据清除。

预期的分工是：

- WebGL：平移、缩放、命中测试并检视全网络；有坐标覆盖时仅从后端拉取
  可见的拓扑窗口。
- 本地 SVG：检视/选中一个有界的 k 跳邻域。
- 虚拟表格：编辑完整的手写模型而不物化所有行。
- 图表：浏览降采样时间窗口，同时保留完整导出数据。
- 后端分块：提供拓扑空间窗口（`/api/session/topology_window`，由总览消费），
  以及通过 `/api/v1` 提供回放时间帧和分析范围限定的结果子集。

更新：2026-08-18（result_window LOD0/1 聚合结果色 + 结果着色图例；色阶抽为
共享常量）
此前：2026-08-17（feedback/toast/problems/draft 小节；属性面板即时校验 +
单位后缀；结果表排序/筛选/CSV 层）

## 新手引导、帮助菜单、文档中心与示例模板

首次会话（localStorage 中没有 `hysim.tourDone.v1`）会在短暂稳定延迟后自动
启动七步新手引导：元件库 → 画布拖放与连线 → 属性面板 → 运行潮流 → 结果浏览
与定位 → 全局定位 → 帮助入口。引导是纯 DOM 实现（聚光环使用超大调光
`box-shadow`，外加一个位置受约束的提示气泡，样式在 `style.css` 的
"Onboarding Tour" 段下）；文案只通过 `textContent` 赋值。目标选择器解析到
缺失或无布局（折叠/隐藏）元素的步骤会在启动前被过滤掉，并在每一步重新检查，
因此未激活模块的运行按钮或已折叠面板会直接丢弃其对应步骤。Esc 随时退出，
ArrowRight/ArrowLeft 切换步骤，上一步/下一步/跳过/完成 按钮均键盘可达
（每步主操作获得焦点）；完成、跳过或 Esc 都会写入
`hysim.tourDone.v1=1`，因此引导在每个浏览器只运行一次。控件暴露为
`App.tour` 供冒烟测试使用（`tmp/tour_smoke.mjs`）。

顶部工具栏辅助簇中的 帮助 按钮打开一个下拉菜单：文档中心（在 typeof 守卫
之后调用 `HySimCore.HelpCenter.open()`；同时绑定 F1，会抑制浏览器默认行为，
且在可编辑控件持有焦点时被忽略）、新手引导（无视完成标志重播引导）、快捷键
面板（在 typeof 守卫之后调用 `HySimCore.HelpPanel.open()`），以及示例模板
（打开案例加载弹窗）。菜单在外部点击或 Esc 时关闭，并把焦点交还按钮。

文档中心（`web/js/core/help_center.js`，schema `hysim_help_center_v1`）是一
个 `.modal`，包含分节的导航树、带防抖的标题/标签搜索框和 markdown 内容
面板。条目来自清单 `web/help_docs.json`（schema `hysim_help_docs_v1`），分组
为 用户指南 / 示例教程 / 模块手册 / API 契约 / 理论模型 / 测试验证 各节；
每个条目可携带一个 GUI 模块 id 的 `modules` 列表（`.module-btn` 的
`data-module` 值），匹配条目会在对应模块标签页激活时被固定到 当前模块相关
区块。文档从服务器挂载点 `/xjtu/docs/<path>` 只读拉取（见
[运行时 API](../reference/runtime_api.zh.md)），用 vendored marked 渲染
（`web/vendor/marked.min.js`；原始 HTML token 会被转义，并有一个极简的内建
渲染器作为回退），相对 `.md` 链接被改写为中心内导航，外部/非 markdown 链接
在新标签页打开，相对图片源被重指到 `/xjtu/docs/` 之下。720px 以下导航折叠
为抽屉，由 目录 按钮切换。Esc 关闭；ArrowUp/ArrowDown 在可见导航内移动；
共享的 `.modal` 焦点陷阱适用。清单或文档拉取失败会渲染明确的错误面板（绝不
是空白面板），且失败的文档拉取不会被缓存，重新打开会重试。

更新：2026-08-18（文档帮助中心；新手引导、帮助菜单、示例模板）

案例加载弹窗额外提供随附的 JSON 示例模板，来自 `web/examples/`
（`app.js` 中的 `EXAMPLE_TEMPLATES` 清单；C++ 服务器将 `web/` 挂载在
`/xjtu/`，因此模板通过相对 `fetch` 加载，然后沿着与 `importJson` 完全一致
的路径经 `/api/session/load_json_string` 导入，共享逻辑被抽取为
`importSystemJson`）。当前模板：`ac_radial_feeder_example.json`（5 母线
10 kV 辐射状 AC 馈线，字段集复制自 `data/simple_case.json` +
`data/dsp/cigre.json`）和 `hybrid_acdc_microgrid_example.json`（两母线直流侧，
含一条 DC 支路和 VSC 后方的两个光伏阵列，派生自 `data/simple_case.json`）。
`ev_traffic_scenario_template.json` 是独立的 EV-交通场景 schema，不是网络
模型，不在案例加载弹窗中列出。

手写模型存放于 `Canvas.state`，分析结果存放于 SVG 结果叠加层。这两类状态
必须保持分离。

## 编辑安全网（SVG 画布）

SVG 编辑器维护一个会话本地的撤销/重做栈（容量 200 条命令），挂钩在
`web/js/canvas.js` 中的结构性变更收口点
（`addComponent` / `removeComponent` / `addConnection` / `removeConnection`）。
一条命令覆盖：组件添加；组件删除（其附带的连线被打包进同一条复合命令）；
连线添加/删除；一次完成的拖拽（在 mouse-up 时记录一次，绝不按 mousemove
记录）；旋转；粘贴；以及多选移动/删除。撤销会按原始 id 恢复被删除的组件与
连线，包括用户编辑过的连线拐点。`clearAll()` 和 `loadFromSystemJson()` 会
清空该栈——刚加载的模型不能被撤销回上一个模型。快捷键：`Ctrl/Cmd+Z` 撤销，
`Ctrl/Cmd+Shift+Z` 或 `Ctrl+Y` 重做。`Canvas.undo()` / `Canvas.redo()` /
`Canvas.canUndo()` / `Canvas.canRedo()` 支撑工具栏 撤销/重做 按钮
（`#btnUndo` / `#btnRedo`，由 `web/js/core/help_panel.js` 接线；禁用状态在
每次 keyup/mouseup 时刷新，外加 300 ms 轮询）。属性面板的参数编辑（归
`app.js` 所有）目前刻意不支持撤销：它们在收口点之外就地修改 `comp.params`。

`Ctrl+C` 把选中的组件连同两端点都被选中的所有连线，以深拷贝参数的方式复制
到一个内部的、会话本地的剪贴板（不是操作系统剪贴板）。`Ctrl+V` 以全新 id
粘贴，连续粘贴时每次累加 +20 px 的网格对齐偏移；`Ctrl+D` 为复制（copy +
立即粘贴）。粘贴落地为单条复合撤销命令并选中新组件。拖动多选中的任一成员
会以相同位移移动整个选区；网格与对齐吸附只作用于被按住的组件，其余跟随
位移。当 input/textarea/select 或 contentEditable 元素持有焦点时，所有编辑
快捷键都被忽略。在 headless 大系统模式（>400 母线，无 SVG 图）下，
复制/粘贴/复制副本为空操作，撤销栈保持为空。

当 SVG 画布本身持有焦点时（`tabindex="0"`，由 `canvas.js` 初始化和
`core/accessibility.js` 共同设置；焦点环来自全局 `[tabindex]:focus-visible`
规则），方向键把每个选中组件微移一个网格步长（20 px；按住 Shift 为 1 px，
用于精细定位）。该分支仅在 keydown 目标正是画布元素时生效，因此面板、
标签列表和对话框中的方向键导航永不被劫持；在 select 模式之外和 headless
模式下为空操作。一连串按键——键盘自动重复或快速连点——会合并为一条复合
`move` 撤销命令，与拖拽的 mouse-up 记录方式一致：起始位置在第一次 keydown
时捕获，只要 keydown 在 500 ms 内持续到达连发就继续延伸，并在第一次方向键
keyup、500 ms 暂停计时器到期、或任何其他编辑开始时（mouse down、任意非方向
键如 Ctrl+Z 或 Delete、`clearUndoStacks`）立即提交。实时微移使用轻量正交
布线器重布连线并跳过对齐参考线；提交时执行一次完整的避障重布线与可视化
刷新。

缩放百分比指示器（`#zoomIndicator`，例如 "125%"）位于画布右上角。它由
`canvas.js` 以内联样式惰性创建（无样式表依赖），`aria-live="off"`，并从
`updateViewBox()`——唯一的平移/缩放收口点——刷新，因此滚轮、工具栏按钮、
全图适配和小地图平移保持同步。headless 大系统模式下从不创建它，若会话切换
进入该模式则会被移除。

`web/js/core/help_panel.js` 拥有键盘快捷键速查表（`#helpModal`）：`?`
（Shift+/）切换它，Esc 或点击背板关闭它，并适用与画布处理程序相同的表单
字段守卫（INPUT/SELECT/TEXTAREA/contentEditable 焦点会抑制 `?`）。内容由
静态数据数组（分组 选择 / 编辑 / 视图 / 分析）经 DOM 构建并用 `textContent`
生成，并重申 headless 大系统限制（编辑快捷键不可用，撤销栈保持为空）。该
弹窗遵循 `.modal` + `h3` 约定，因此 `core/accessibility.js` 会自动附加
`role="dialog"` / `aria-modal` / `aria-labelledby`。同一模块还为每个可见的
`.modal` 运行焦点陷阱：弹窗打开期间，Tab / Shift+Tab 在其可聚焦元素间循环，
无法离开对话框（Esc 仍归各弹窗自身逻辑处理）；关闭时，仅当打开前的焦点元素
仍滞留在正在关闭的弹窗内或落在 `<body>` 上时才把焦点交还它，因此像快捷键
面板这样自行恢复焦点的弹窗不会被覆盖。附加/分离由既有的 MutationObserver →
调度同步流水线驱动（逐弹窗的属性观察器外加一个 `body` childList 观察器，
用于后添加的弹窗），打开前的焦点目标保存在 WeakMap 中。该陷阱暴露为
`HySimCore.Accessibility.syncFocusTraps`。画布的 `aria-label`（在
`core/accessibility.js` 初始化时设置）会播报方向键微移。快捷键面板本身暴露为
`HySimCore.HelpPanel`（`open` / `close` / `toggle` / `refreshUndoRedo` /
`shortcutGroups`，schema `hysim_help_panel_v1`）。

## 静态结果

`Canvas.showPowerFlowResults(result)` 是静态潮流/最优潮流的呈现路径。它
可以刷新已求解组件的显示和可视化叠加层。潮流（PF）和最优潮流（OPF）的
post-PF 载荷共享手写空间的组件键与端口潮流字段。

平衡潮流高级面板把光滑 NCP 延拓和局部 VSC Schur 准入暴露为专家控件。其
三态开关和留空数值字段保留稀疏意图：留空意味着浏览器省略该字段，由后端
解析其权威默认值。求解后，面板显示 `options_effective.robust_nonlinear`；
它绝不从 JavaScript 默认值推导有效容差。

潮流结果分组把 `linear_structure` 渲染为求解器证书。它区分禁用/未准入、
已接受、已拒绝并全量 LU 回退三种状态，并显示维度/结构非零元缩减、局部块
正则性、后向误差、光滑延拓更新和局部收敛速率样本。设备行继续拥有物理电流
限值动作、裕度、优先级和精确 NCP 残差。高级数值控件仅适用于平衡正序稳态/
准稳态潮流：它们不声称 GFM 优先级 NCP 位于 OPF KKT 系统之内，也不覆盖被
暂态初始化消费的共享 GFM 参数。

修改任何 NCP/Schur 专家控件会立即使所显示的有效策略和求解器证书失效。GUI
把旧证书标记为过期并要求重新运行潮流；它绝不把先前的广义雅可比证书当作新
编辑数值控件的证据。

拓扑与结果表导航有两条显式标识路径。常规 SVG 系统仅把内部画布组件标识符
用于即时图形选中。headless/WebGL 系统从 `canvas_type` 加稳定 `.index`
（或以显式命名的组件位置作为兼容性回退）恢复手写组件，解析其相连母线，
并调用域限定的 `{domain, index}` 总览选择器。因此整数索引相同的 AC 与 DC
母线仍然保持区分。拓扑行还保留其搜索注册表来源，因此 WebGL 选中可以可见地
高亮匹配的虚拟化行。

结果 标签页中的结果表（潮流、OPF、短路、可靠性，以及日前 / 实时 / 重复博弈
市场视图）在每个模块渲染完成后由 `web/js/app.js` 中的
`enhanceResultTablesIn(root, {module})` 渐进增强：表头点击三态排序（数值
感知，活动 `<th>` 上带 `aria-sort`，第三次点击恢复原始行序）、紧邻表头下方
的逐列包含匹配筛选行（200 ms 防抖，大小写不敏感），以及只导出当前可见行的
导出 CSV 工具栏按钮（RFC-4180 转义，UTF-8 BOM 以便 Excel 正确读取中文表头，
文件名 `<module>_results_<timestamp>.csv`）。不符合预期形状的表——缺少
thead/tbody、多行表头、colspan/不规则行、内嵌表单控件（成本编辑器、市场
参与者编辑器）或虚拟化表格——会被跳过并记录一条 `console.debug`，渲染绝不
被破坏。行级 `data-comp-id` 画布定位在排序下仍正常工作，因为点击委托使用
`closest('[data-comp-id]')`，而排序只重排 DOM 行。由
`tmp/result_table_smoke.mjs` 覆盖。

## 时间回放

`web/js/app.js` 中的 `CanvasPlaybackController` 提供：

- 时间滑块；
- 播放/暂停和上一帧/下一帧；
- 1x、2x 和 5x 速度；
- 当前帧时间与收敛状态；
- 电压、潮流、频率、P/Q 和 SOC 模式；
- 惰性帧拉取，带过期请求取消和有界帧缓存。

`web/js/canvas.js` 中的 `Canvas.renderFrame(frame)` 接受 `canvas_frame_v1`。
它只移除和更新 `.frame-overlay` 与 `.viz-overlay` 元素。它绝不能：

- 写 `component.params`；
- 重建组件或连接；
- 把模型同步到后端；
- 为暂态支路箭头复用静态潮流方程。

TSPF 的 flow/PQ 模式可以复用已求解的静态支路几何，因为帧中包含实际的逐步
潮流端口功率。暂态渲染仅限于记录的电压、频率、设备遥测、SOC/状态和激活
事件标记。当 `capabilities.branch_power` 为 false 时 flow 模式被禁用。

## 标识与单位

帧组件查找使用 `canvas_type`、`canvas_index` 和 `position`，以及显式别名，
例如 `switch -> switch_comp` 和 `renewable_generator -> renewable_gen`。
功率值以 MW/MVar 存储；当切换 MW/kW/W 时，画布显示对 P 和 Q 做一致换算。

## 性能

后端不会随主分析响应发送数千个完整画布帧。回放按索引一次请求一帧，并在
浏览器中缓存一个小窗口。这让长时间运行保持响应，并避免污染下一次手写模型
同步。

至少有 5,000 母线的大型 headless 系统以 `response_detail=compact` 请求
`POST /api/session/pf`。该响应保留完整的已求解向量：

- `vm` 和 `va`：AC 母线向量位置，pu 和弧度；
- `vdc`：DC 母线向量位置，pu；
- `branch_abs`：手写 AC 支路潮流位置，绝对值 MW。

它省略 `geo_*`、`component_results` 和构建成本随每个手写组件增长的
rich-to-canonical 归因行。响应通过 `model_scope`、`model_limitations` 和
`presentation_omitted` 声明这一边界；GUI 渲染向量统计量，而不是为每条母线
或支路物化一行 HTML。常规规模系统继续使用默认的 `response_detail=full`
契约。

潮流响应通过 JSON `timing` 对象和 HTTP `Server-Timing` 头暴露快照、装配、
求解、呈现、收尾和序列化各阶段耗时。大型 MATPOWER 加载使用单个结构化
`_system_json` 模型载荷，避免第二份转义副本和一次额外的浏览器
`JSON.parse`；较小的加载保留旧版响应以保持 API 兼容性。
