# Web 前端技术文档 — Hybrid AC/DC Power System Simulator

> 适用版本：2026-05  
> 适用范围：`web/` 目录下的浏览器端 SPA（HTML / CSS / 原生 JS），以及与之直接对接的 GUI 后端 `tests/run_gui_server.cpp`。

本文档记录当前 ETAP 风格交直流混合系统单线图编辑器的 Web 端设计：目录结构、UI 分层、模块职责、命名约定、API 契约、关键事件流与扩展约束。读者对象为后续维护者与新功能贡献者。

---

## 1. 目录结构

```
web/
├── index.html              # SPA 单页面骨架：三层工具栏 + 主区 + 弹窗
├── css/
│   └── style.css           # 全局样式（含三层工具栏、暗色主题变量、模态框、画布、表格）
└── js/
    ├── components.js       # 元件库：每种元件的 SVG 图形、默认参数、端口定义、分类
    ├── canvas.js           # 画布编辑器核心（约 3700 行）：SVG 渲染、拖拽、连线、缩放/平移、自动布局、可视化、JSON 互转
    └── app.js              # 业务逻辑层：API 调用、按钮绑定、属性面板、拓扑/结果表、各计算流程
```

后端入口与静态资源服务：

- `tests/run_gui_server.cpp` — 基于 `httplib` 的 HTTP 服务，监听 `--host/--port`（默认 `127.0.0.1:8088`）。
- 启动时同时打印两个目录：`Data directory`（内置算例 `.m` 等数据，由 `--data-dir` 覆盖）与 `MATPOWER directory`（MATPOWER `.m` 文件库，由 `--matpower-dir` 覆盖，默认 `../external_data/matpower`）。
- `web/` 目录通过 `/xjtu/` 路径前缀作为静态资源挂载（参见服务端日志：`XJTU frontend: http://127.0.0.1:<port>/xjtu/`）。

启动示例（PowerShell + MSYS2 UCRT64 工具链）：

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
.\build\run_gui_server.exe --port 8088
# 可选参数：
#   --data-dir       <path>   覆盖内置算例数据目录（默认 ../data）
#   --matpower-dir   <path>   覆盖 MATPOWER 文件目录（默认 ../external_data/matpower）
```

---

## 2. 三层工具栏（核心 UI 分层）

`index.html` 顶部由 3 条独立工具栏组成，自上而下：

### 2.1 Bar 1 — `#topToolbar`（画布操作工具栏）

DOM 容器：`<div id="topToolbar" class="top-toolbar">`

| ID | 文本 | 行为（绑定函数） |
|---|---|---|
| `btnLoadCase` | 加载算例 | 打开 `#caseLoadModal`（`showCaseLoadModal()`） |
| `btnSelect` ⭐默认激活 | 选择 | `Canvas.setMode('select')` + `setActiveCanvasTool('select')` |
| `btnConnect` | 连线 | `Canvas.setMode('connect')` |
| `btnBoxSelect` | 框选 | `Canvas.setMode('boxselect')` |
| `btnDelete` | 删除 | `Canvas.removeSelected()`（单个） |
| `btnDeleteSelected` | 删除选中（批量） | `Canvas.removeSelected()`（全部已选） |
| `btnZoomIn` / `btnZoomOut` / `btnZoomFit` | 放大 / 缩小 / 适应画面 | `Canvas.zoomIn/Out/Fit()` |
| `btnAutoLayout` | 自动布局 | `Canvas.autoLayout()` |
| `btnRotateCW` / `btnRotateCCW` | 顺/逆时针旋转 | `Canvas.rotateSelected(±90)` |
| `btnNewSystem` / `btnExportJson` / `btnImportJson` | 新建/导出/导入 | `createNewSystem()` / `exportJson()` / `importJson(file)` |
| `btnExportEtap` | 导出ETAP | `exportEtap()` —— `syncToBackend()` 后 `POST /api/session/export_etap`，把返回的二进制 `.xlsx` 触发浏览器下载 |
| `canvasInfo` | （文本） | 实时显示 `N 元件 ｜ M 连接` |
| `statusBadge` | （徽章） | 状态：就绪 / busy / error |

样式约定：
- 静态：深灰 `--tb-bg #2c313a` + `--tb-text #dcdfe4`，与全站暗色主题统一。
- 悬停：`--tb-bg-hover #353b46` + 青色 1px 描边 `--tb-border-hover #56b6c2`。
- 激活：青蓝渐变 `--tb-active-bg1/2`，描边 `--tb-active-border #7cc7ee`，字体白色。
- “运行类”按钮 `.run-btn`：暖橙 `--tb-run-bg #d19a66`，与激活态在视觉上明确区分。
- ⚠️ 一次仅有一个 `.canvas-tool` 处于 `active`；切换由 `setActiveCanvasTool()` 维护互斥。

### 2.2 Bar 2 — `#moduleBar`（功能模块栏）

DOM 容器：`<div id="moduleBar" class="module-bar">`，深色渐变背景 `--module-bar-bg/bg2`。

按钮统一 `class="module-btn"`，通过 `data-module` 标识当前选择的功能模块：

| ID | `data-module` | 子工具栏标签 | 用途 |
|---|---|---|---|
| `modulePowerFlow`    | `powerFlow`     | 潮流计算 | 显示算法/单位/可视化/运行 |
| `moduleShortCircuit` | `shortCircuit`  | 短路计算 | 故障点/类型/电压系数/运行 |
| `moduleTopology`     | `topology`      | 拓扑重构 | 仅一个“运行”按钮 |
| `moduleTimeSeries`   | `timeSeries`    | 时序潮流 | 辐照/电价/负荷/小时数/运行/碳流 |
| `moduleHosting`      | `hosting`       | 承载力分析 | 仅一个“运行”按钮 |
| `moduleReliability`  | `reliability`   | 可靠性分析 | 占位 |
| `moduleResilience`   | `resilience`    | 弹性评估 | 占位 |

切换逻辑由 `setActiveModule(name)` 统一维护：
- 取消其他 `.module-btn.active`，将本按钮置为激活；
- 调用 `renderSubToolbar(name)` 切换 Bar 3 中对应 `data-sub` 区段的可见性；
- 显示 / 隐藏 `#subToolbar` 整体（无激活模块时整条隐藏 `.hidden`）。

### 2.3 Bar 3 — `#subToolbar`（上下文子工具栏）

DOM 容器：`<div id="subToolbar" class="sub-toolbar">`，每个模块对应一个 `<div class="sub-section" data-sub="...">` 子段，互斥 `hidden`。

| `data-sub` | 关键控件（ID） | 运行 / 导出按钮 |
|---|---|---|
| `powerFlow` | `pfMethod`（9 种算法）、`pfDisplayUnit`（MW/kW/W）、`vizMode`（off/flow/heatmap/both）、`btnImportGreenCert`（隐藏 `fileImportGreenCert`） | `btnPowerFlow` → `runPowerFlow()`；`btnExportPfResults` → `downloadJsonFile(_lastPfData)` |
| `shortCircuit` | `btnFaultLocation`（仅设置故障母线 ID）、`faultTypeSelect`、`voltageCorrectionFactor` | `btnRunShortCircuit` → `runShortCircuit()` |
| `topology` | — | `btnRunTopology` → `runTopologyReconfig()` |
| `timeSeries` | `btnImportIrradiance/Price/LoadProfile`（隐藏 file input）、`simulationHours`、`tspfSkipUC`、`tspfRunOPF` | `btnRunTimeSeriesPF` → `runTimeSeriesPF()`；`btnExportTspfResults` → `downloadJsonFile(_lastTspfData)`；`btnRunCarbonFlow` |
| `hosting` | — | `btnRunBearingCap` → `runBearingCapacity()` |
| `reliability` | — | `btnRunReliability`（TODO）、`btnExportReliabilityResults` |
| `resilience` | `resFaultCount` / `resFaultLocations` / `resFaultStartHour` / `resFaultDuration` / `resRepairDuration` / `resMobileSpeed`；`btnGenExtremeScenario`（极端场景生成 TODO） | `btnRunResilience`（TODO）、`btnExportResilienceResults` |

> **故障点位设置（2026-05）**：原对话框同时包含“故障类型/电压调节系数/母线ID”；现在后两项已上提到子工具栏（`faultTypeSelect` / `voltageCorrectionFactor`），“故障点位设置”对话框 `#scDialog` 仅保留故障母线 ID 输入，避免重复。

样式：与主面板同色系（`--sub-bar-bg #21252b`），select/input 也使用暗色主题（深灰底+亮文字+青色聚焦光晕），不再使用浅色背景以避免视觉跳变。

---

## 3. 加载算例模态框 `#caseLoadModal`

由 `btnLoadCase` 触发，覆盖整个视口的暗色卡片：

| 行 ID | 行为 |
|---|---|
| 内置算例：`btnImportLibraryCase` + `caseSelect` | `loadCaseList()` 在初始化时填充下拉框；点击按钮调用 `loadBuiltinCase(caseSelect.value)` → POST `/api/session/load_builtin` |
| MATPOWER 文件：`btnImportMatpowerCase` + `matpowerSelect` | `loadMatpowerFileList()` 调用 `GET /api/matpower_files`（**指向 `external_data/matpower/`**，列出 85 个 MATPOWER 案例 `.m` 文件）；点击按钮调用 `loadMatpowerCase(filename)` → POST `/api/session/load_matpower` |
| JSON 文件：`btnImportJsonCase` | 复用 `fileImportJson` 的 file picker → `importJson(file)`（前端解析后调用 `Canvas.loadFromSystemJson`） |
| ETAP 工作簿：`btnImportEtapXlsxCase` + `fileImportEtapXlsx` | `loadEtapXlsx(file)` —— 读 `file.arrayBuffer()` 以 `application/octet-stream` 二进制 `POST /api/session/load_etap_xlsx`，后端 `load_etap()` 解析后回画布 |
| ETAP 工程：`btnImportEtapXmlCase` + `fileImportEtapXml` | `loadEtapXml(file)` —— 读 `file.text()` 后 `POST /api/session/load_etap_xml {xml_string}`，后端 `load_etap_xml()` 解析后回画布 |

模态关闭：`btnCaseModalClose` 或点击 `.modal-backdrop`，统一调用 `hideCaseLoadModal()`。

> **MATPOWER 目录变更（2026-05 修复）**：原服务端将 `--data-dir` 同时用作 MATPOWER 目录，导致 `external_data/matpower` 下的算例无法被列出。当前实现新增 `--matpower-dir` 命令行参数（默认 `../external_data/matpower`），并自动在常见相对位置回退（cwd / cwd/.. / cwd/../..），从而兼容 `build/`、仓库根目录等不同启动位置。两个 endpoint `/api/matpower_files` 与 `/api/session/load_matpower` 均改为引用 `matpower_dir`。

> **内置算例库路径修复（2026-05）**：`src/io/case_builders.cpp` 中的 `data_file_path()` 仅在 `<root>/data/<name>` 查找文件，导致内置算例（如 IEEE118/case300/case2000 等通过 `parse_matpower("case118.m")` 加载的 builder）报错 `Failed to open MATPOWER file: ...\data\case118.m`。现在 `data_file_path()` 会按以下顺序搜索：`<root>/external_data/matpower/`、`<root>/data/`、`cwd/external_data/matpower/`、`cwd/../external_data/matpower/`、`cwd/data/`、`cwd/../data/`，找到任一存在的路径即返回；全部缺失时回退到旧的 `<root>/data/<name>` 以保留原始错误信息。15 个内置算例已全部验证可正常加载。

---

## 4. 主区布局

```
┌────────────────── #topToolbar (Bar 1) ──────────────────┐
├────────────────── #moduleBar  (Bar 2) ──────────────────┤
├────────────────── #subToolbar (Bar 3) ──────────────────┤
│                                                          │
│  #componentLib  │   #canvasContainer    │  #rightPanel   │
│  (元件库左栏)   │   (SVG 画布 + 可视化)  │  (属性/拓扑/结果) │
│                 │   #vizLegend          │                │
└──────────────────────────────────────────────────────────┘
                       #console (底部日志)
```

### 4.1 左侧元件库 `#componentLib`

由 `initComponentLibrary()` 根据 `components.js` 动态生成，分为：交流元件 / 直流元件 / 换流器 / 储能-新能源 / 充电设施 / 聚合资源。`.lib-item` 支持拖拽到画布。

### 4.2 中央画布 `#canvasContainer`

- `<svg id="canvas">` 内含 4 个 `<g>` 渲染层（顺序决定 z-order）：
  - `connectionsLayer` — 连线（普通线 + 潮流方向动画）
  - `componentsLayer` — 元件主体（rect/line/path 组成的图标 + 标签）
  - `resultsLayer` — 结果叠加（电压热力图、潮流箭头）
  - `tempLayer` — 临时图形（橡皮筋连线、框选矩形）
- 网格背景由 `#gridSmall` / `#gridLarge` 两层 `<pattern>` 组成。
- `#vizLegend` 显示热力图色带（0%–100% 负载率），由 `Canvas.refreshVisualization()` 控制可见性。

`Canvas` 对外 API（在 `canvas.js` 末尾以全局对象 `window.Canvas` 暴露）：

| 方法 | 说明 |
|---|---|
| `setMode('select'\|'connect'\|'boxselect')` | 切换交互模式 |
| `zoomIn() / zoomOut() / zoomFit()` | 缩放控制 |
| `autoLayout()` | 自动布局（基于电气拓扑） |
| `rotateSelected(deg)` | 旋转选中元件 |
| `removeSelected()` | 删除选中（单个或批量） |
| `loadFromSystemJson(sys)` | 从后端 JSON 重建画布 |
| `serializeToSystemJson()` | 导出当前画布为 `HybridPowerSystem` JSON |
| `showPowerFlowResults(data)` | 在线路上叠加潮流箭头/热力图 |
| `setVisualizationMode('off'\|'flow'\|'heatmap'\|'both')` | 切换可视化模式 |
| `refreshVisualization()` | 重新绘制结果叠加层 |
| `getPowerUnit()` | 读取 `#pfDisplayUnit` 当前单位（MW/kW/W） |

### 4.3 右侧面板 `#rightPanel`

带 3 个标签页（`.panel-tab`）：

- **属性 `#tabProperties`** — `onSelectionChanged(compId)` 根据 `components.js` 中元件的 `params` schema 动态生成表单；`applyProperties()` 写回画布并触发 `_canvasDirty=true`。
- **拓扑 `#tabTopology`** — `updateTopologyTables()` 渲染节点表/支路表/发电机/负荷/变压器/外网/储能/换流器等表格（`<thead>` 已在 HTML 中静态定义，行体 `<tbody>` 由 JS 填充）。
- **结果 `#tabResults`** — 根据当前功能模块只展示其计算输出。`#resultsContent` 下按模块拆分为多个 `<div class="result-group" data-result-group="<key>">`块（`powerFlow` / `shortCircuit` / `topology` / `hosting` / `timeSeries` / `reliability` / `resilience`）；`#resultsContent[data-active-group=...]` 属性控制当前可见块，同一时间仅显示一个分组。`setActiveResultGroup(name)` 由两处调用：各计算结果展示函数（`showPowerFlowResultsTables` / `showShortCircuitResults` / `showTopologyResults` / `showBearingCapResults` / `showTimeSeriesResults` 等）以及模块切换 `setActiveModule()`。

---

## 5. 后端 API 契约（前端依赖部分）

| Method & Path | Body / Query | 用途 |
|---|---|---|
| `GET  /api/cases` | — | 内置算例名列表 |
| `GET  /api/matpower_files` | — | `external_data/matpower` 下的 `.m` 文件列表 |
| `POST /api/session/load_builtin` | `{case}` | 加载内置算例 |
| `POST /api/session/load_matpower` | `{filename}` | 加载 MATPOWER 文件（仅 basename，禁止 `..` 与路径分隔符） |
| `POST /api/session/load_json_string` | `{json_string}` | 加载画布序列化的 JSON（伴随 `debug_json_dump_*.json` 落盘以便回放） |
| `POST /api/session/new_empty` | — | 新建空白系统 |
| `POST /api/session/export_json` | — | 导出当前会话为 JSON |
| `POST /api/session/export_etap` | `{}`（须有请求体 / Content-Length） | 导出当前系统为 ETAP `.xlsx`，返回二进制工作簿（`Content-Disposition: attachment`） |
| `POST /api/session/load_etap_xlsx` | 原始 `.xlsx` 字节（`application/octet-stream`） | 上传并导入 ETAP 工作簿 |
| `POST /api/session/load_etap_xml` | `{xml_string}` | 导入原生 ETAP 工程 XML |
| `POST /api/session/update_components` | `{components}` | 增量同步画布 |
| `POST /api/session/pf` | `{method, options...}` | 潮流计算（9 种算法之一） |
| `POST /api/session/sc` / `/sc_detailed` | 故障参数 | 短路计算 |
| `POST /api/session/run_reconfig` | — | 拓扑重构 |
| `POST /api/session/run_ts_pf` / `/set_ts_config` | 时序参数 | 时序潮流 |
| `POST /api/session/run_carbon` | — | 碳流分析 |
| `POST /api/session/run_bearing_capacity` | — | DG 承载力 |
| `POST /api/session/run_reliability_*` | — | 可靠性（NSQ/SEQ/FMEA/FD） |
| `POST /api/session/run_distribution_resilience` | — | 弹性评估 |
| `POST /api/session/cancel`、`GET /api/session/status` | — | 任务取消 / 忙闲状态 |

所有 endpoints 返回 JSON。错误以 HTTP 4xx/5xx + `{error: "..."}` 返回；`apiPost/apiGet` 在 `app.js` 中统一捕获并 `log()` 到底部控制台。

> 后端使用全局 `Session g_session`（`std::mutex` 保护）持有 `current_system / last_pf_result / ts_data`；所有计算 endpoint 都从其读取当前系统而非随请求传入，以避免大体量数据反复传输。

---

## 6. 关键事件流

### 6.1 加载 MATPOWER 算例 → 自动潮流

```
用户点击 btnLoadCase
   → showCaseLoadModal()
   → 用户选择 matpowerSelect 后点击 btnImportMatpowerCase
   → loadMatpowerCase(filename)
        → POST /api/session/load_matpower {filename}
        → 后端 fs::path(matpower_dir) / filename → parse_matpower(...)
        → 返回 {_raw_json, summary...}
        → Canvas.loadFromSystemJson(JSON.parse(_raw_json))
        → log("画布已更新...")
        → await runPowerFlow()      // 自动触发一次潮流
   → hideCaseLoadModal()
```

### 6.2 画布编辑 → 后端同步

`canvas.js` 在元件/连线发生变化时设置 `_canvasDirty = true`。运行任何计算前由 `syncToBackend()` 自动调用 `POST /api/session/load_json_string`（参数为 `Canvas.serializeToSystemJson()` 的输出）。

### 6.3 模块切换

```
点击 .module-btn
   → setActiveModule(data-module)
        → 互斥更新 active class
        → renderSubToolbar(name): 显示对应 .sub-section[data-sub=name]，隐藏其它
        → 整条 #subToolbar 添加/移除 .hidden
```

### 6.4 时序潮流（仿真时间 + UC/OPF 选项）

时序潮流不再依赖独立对话框，所有运行参数都内联在 Bar 3 子工具栏：

```
[辐照导入][电价导入][负荷导入]  仿真时间(小时)=[8760]
   [☑ 跳过机组组合]  [☐ 启用OPF优化调度]   [运行时序潮流]
```

点击 `btnRunTimeSeriesPF` 直接调用 `runTimeSeriesPF()`：

```js
const numSteps = parseInt(simulationHours.value) || 24;  // 仿真时间(h) = 步数(假定 step_duration_hr=1)
const skipUC   = tspfSkipUC.checked;                      // 默认勾选
const runOPF   = tspfRunOPF.checked;                      // 默认不勾选
POST /api/session/run_ts_pf  { num_steps, skip_uc, run_opf }
→ showTimeSeriesResults(data) → switchTab('results')
```

CSV 时序数据由三个独立按钮（辐照/电价/负荷）各自上传，不再合并到运行流程中；后端导入接口待补全（当前仅前端记录文件名，TODO）。

---

## 6.5 模块化结果展示与导出（2026-05）

为避免不同功能模块的输出在右侧 `结果` 标签页中混在一起，`#resultsContent` 内拆分为多个 `<div class="result-group" data-result-group="<key>">` 块，CSS 通过 `#resultsContent[data-active-group=...]` 控制单一可见块。`<key>` 取值与 `data-module` 完全一致：`powerFlow / shortCircuit / topology / hosting / timeSeries / reliability / resilience`。

切换时机：
- 切换功能模块（点击 `.module-btn`）时由 `setActiveModule(name)` 调用 `setActiveResultGroup(name)`；
- 各计算结果展示函数（`showPowerFlowResultsTables` / `showShortCircuitResults` / `showTopologyResults` / `showBearingCapResults` / `showTimeSeriesResults` 等）首行调用 `setActiveResultGroup(<key>)`，确保跑哪个就显示哪个。

结果导出按钮统一调用 `downloadJsonFile(filename, obj)`（`app.js` 顶部，使用 `Blob + URL.createObjectURL` 在浏览器侧生成下载，无需后端落盘）。文件名形如 `pf_results_<YYYYMMDD_HHMMSS>.json`。当前已落地：

| 子工具栏按钮 | 缓存变量 | 备注 |
|---|---|---|
| `btnExportPfResults`（潮流） | `_lastPfData`（在 `runPowerFlow()` 中赋值） | 已可用 |
| `btnExportTspfResults`（时序潮流） | `_lastTspfData`（在 `runTimeSeriesPF()` 中赋值） | 已可用 |
| `btnExportReliabilityResults`（可靠性） | `_lastReliabilityData` | 仅占位，等待后端结果赋值 |
| `btnExportResilienceResults`（弹性） | `_lastResilienceData` | 仅占位，等待后端结果赋值 |

---

## 6.6 时序潮流模块：绿证数据导入（2026-05，2026-06 修订）

> **2026-06 修订**：绿证数据导入按钮已从「潮流计算」子工具栏移动到「时序潮流」子工具栏（`data-sub="timeSeries"`），与辐照强度 / 电价 / 负荷波动数据导入按钮并排。

时序潮流子工具栏现有 4 个数据导入按钮：

| 按钮 ID | 文件输入 | 用途 | 接受格式 |
|---|---|---|---|
| `btnImportIrradiance`  | `fileImportIrradiance`  | 辐照强度时序 | `.json,.xlsx,.xls,.csv,.txt` |
| `btnImportPrice`       | `fileImportPrice`       | 电价时序 | 同上 |
| `btnImportLoadProfile` | `fileImportLoadProfile` | 负荷波动时序（多曲线） | 同上 |
| `btnImportGreenCert`   | `fileImportGreenCert`   | 绿证数据 | 同上（**后端 TODO**） |

JSON 导入已对接后端，Excel/CSV 当前仅占位（控制台日志），后端端点 TODO。详见 §6.8。

---

## 6.8 时序潮流：JSON 数据导入与每负荷曲线映射（2026-06）

### 6.8.1 JSON 文件结构

样例文件位于 `external_data/profiles/case33bw/`，由 `scripts/generate_case33bw_profiles.py` 生成（8760 步，1 小时步长）。

**单曲线（辐照强度 / 电价）**

```jsonc
{
  "type": "irradiance",                 // 或 "price"
  "name": "case33bw_irradiance_8760",
  "num_steps": 8760,
  "step_duration_hr": 1.0,
  "unit": "pu_of_stc_1000Wm2",          // 或 "RMB/MWh"
  "values": [/* 8760 个 double */]
}
```

**多曲线（负荷波动，每个负荷一条 — 2026-07 改为绝对功率值）**

```jsonc
{
  "type": "load_profiles_mw",
  "name": "case33bw_loads_8760",
  "num_steps": 8760,
  "step_duration_hr": 1.0,
  "unit": "MW",                      // 重要：每个 8760 步序列以 MW 表示，而非无量纲倍数
  "case": "case33bw",
  "num_loads": 32,
  "load_profiles": [
    { "load_index": 0, "bus": 2, "name": "case33bw_load_bus2",
      "category": "residential",
      "p_mw_nominal": 0.10, "q_mvar_nominal": 0.06,
      "p_mw_values":   [/* 8760 个 double, MW */],
      "q_mvar_values": [/* 8760 个 double, MVAr */],
      "values":        [/* alias of p_mw_values，向后兼容 */] },
    /* ... 共 32 条，每个负荷有自己独立的 8760 序列 ... */
  ]
}
```

> **设计动机**：用户希望文件直接用功率（MW）描述每条负荷曲线（而非系统级 scaling）。前端 `pushTsConfigToServer()` 在上传前会用 `p_mw_nominal` 把每条 `p_mw_values[t]` 换算成无量纲 `scaling[t] = mw[t] / p_mw_nominal`，以保留后端 `P[t] = p_mw * scaling * profile[t]` 的乘法约定。

### 6.8.2 前端缓存与多文件合并上传

`web/js/app.js` 内 `_tsProfileCache` 暂存 `irradiance / price / load_profiles / num_steps`，每次成功导入后调用 `pushTsConfigToServer()`，把已缓存的全部曲线合并成一次 `POST /api/session/set_ts_config`。`num_steps` 不一致会自动重置缓存。

约定的 `profile_id` 编号：
- 辐照强度：`2`（与后端默认 PV / 太阳能可再生能源的 profile_id 相同）
- 电价：`50`
- 各负荷曲线：`100, 101, …, 100 + N - 1`

### 6.8.3 后端端点扩展：`POST /api/session/set_ts_config`

新增三个可选字段（向后兼容）：

```jsonc
{
  "num_steps": 8760,
  "step_duration_hr": 1.0,
  "profiles": [ /* 现有 */ ],

  // ① 每负荷映射：把指定 profile_id 绑定到指定 AC 负荷
  "load_profile_map": [
    { "load_index": 0, "profile_id": 100 },   // 优先按 load_index
    { "bus": 5,        "profile_id": 104 }    // 也可按 bus 索引
  ],

  // ② 一次性把所有 ac.loads 绑定到同一个 profile（可选，便于简单场景）
  "assign_all_loads_to": -1,

  // ③ 一次性把所有 PV 系统 / DC PV 阵列绑定到 irradiance profile（可选）
  "assign_all_pv_to": 2
}
```

响应新增字段：
- `num_loads_mapped`：实际成功绑定的负荷数量。
- `num_loads_materialized`（**2026-07 新增**）：当 case 是 MATPOWER 解析得到（`ac.loads.empty()`）时，后端会基于 `bus.pd_mw / qd_mvar` 自动物化 Load 实体，并清零 bus 上的 demand 以避免双重计数。该字段返回物化的负荷数。**这修复了一个关键 bug**：在不物化的情况下，`load_profile_map` 会写入空的 `ac.loads` 数组，导致时序潮流走"bus.pd_mw 兜底"路径并使用不存在的 `profile_id=0`，结果是 8760 步全部使用恒定负荷与恒定电压。

### 6.8.4 后端响应：`POST /api/session/run_ts_pf` 完整字段

为了导出系统全部节点 8760 小时的电压/相角与全部边的潮流，后端时序潮流响应除已有的 `vm_mean / vm_min / vm_max / total_load / losses_mw` 外，新增以下完整二维矩阵：

| 字段 | 形状 | 单位 | 含义 |
|---|---|---|---|
| `vm_matrix`     | `[num_buses][num_steps]` | pu   | 各母线各时刻电压幅值 |
| `va_matrix`     | `[num_buses][num_steps]` | rad  | 各母线各时刻电压相角 |
| `bus_labels`    | `[num_buses]`            | —    | 母线名称（无名时回退为 `Bus <index>`）|
| `branch_pf_mw`  | `[num_branches][num_steps]` | MW   | 各分支 from 端有功潮流 |
| `branch_pt_mw`  | `[num_branches][num_steps]` | MW   | 各分支 to 端有功潮流 |
| `branch_qf_mvar`| `[num_branches][num_steps]` | MVAr | 各分支 from 端无功潮流 |
| `branch_qt_mvar`| `[num_branches][num_steps]` | MVAr | 各分支 to 端无功潮流 |
| `branch_labels` | `[num_branches]`         | —    | 分支名称 |
| `branch_from_bus` / `branch_to_bus` | `[num_branches]` | — | 分支端点 bus index |

前端"结果导出"按钮把这些字段打包到 `tspf_results_<tag>.json`：
```jsonc
{
  "meta": { "exported_at": "...", "num_steps": 8760, "num_converged": 8760 },
  "bus_voltages":  { "bus_labels": [...], "vm_matrix": [[...]], "va_matrix": [[...]] },
  "branch_flows":  { "branch_labels": [...], "branch_from_bus": [...], "branch_to_bus": [...],
                     "pf_mw": [[...]], "pt_mw": [[...]], "qf_mvar": [[...]], "qt_mvar": [[...]] },
  "per_step_summary": { "vm_mean": [...], "vm_min": [...], "vm_max": [...],
                        "losses_mw": [...], "total_load_mw": [...] },
  "raw": { /* 原始后端响应，便于回溯 */ }
}
```

### 6.8.5 验证：8760 小时 case33bw PF 测试

`tests/test_case33bw_8760h_pf.cpp`（CMake target `test_case33bw_8760h_pf`，CTest tier `slow`）执行：

1. 加载 `external_data/profiles/case33bw/` 下三份 JSON。
2. 对 `case33bw.m` 解析后，把 `bus.pd_mw / qd_mvar` 物化为 32 条 `Load`（解析器只填 bus 字段，没有创建 Load 实体）。
3. 校验 JSON `unit == "MW"` 且每条曲线提供 `p_mw_values`；按 `scaling[t] = p_mw_values[t] / p_mw_nominal` 转换为后端可用的无量纲序列，并按 `load_index → 100 + i` 给每个负荷写入唯一 `profile_id`。
4. 检查映射：32/32 全部映射、`profile_id` 无重复、无未映射负荷。
5. **每负荷独立性校验**：32 条 8760-h MW 序列两两不同（独立的负荷波动，而非系统级缩放）。
6. **精度校验**：对 `t = 0..8759` 计算 `Σ_i p_mw[i] * scaling[i] * profile[load.profile_id][t]` 与 JSON 原始 `p_mw_values[i][t]` 的差异，要求最大绝对误差 ≤ 1e-9 MW。
7. **总负荷波动校验**：8760 步内总负荷 spread ≥ 0.1 MW。
8. 调用 `solve_time_series_pf` 跑 8760 步（skip_uc=true, run_opf=false），要求 ≥ 99% 收敛。
9. **电压随时间变化校验**（关键修复验证）：取最远端负荷所在母线，要求其 vm 时序 stdev ≥ 1e-4 pu —— 防止再次出现"映射失败导致 8760 步电压完全不变"的回归。

实测（2026-07）：32/32 映射、32/32 序列两两不同、最大重构误差 1.78e-15 MW（数值零）、总负荷范围 2.32–5.23 MW（spread 2.92 MW）、8760/8760 PF 收敛、末端 33 母线 vm stdev = 0.014 pu（spread 0.064 pu），耗时约 0.81 s。

---

## 6.7 可靠性 / 弹性评估子工具栏（2026-05 占位）

可靠性模块（`data-sub="reliability"`）新增：
- `btnRunReliability`（运行可靠性分析）
- `btnExportReliabilityResults`（结果导出）

弹性评估模块（`data-sub="resilience"`）新增以下控件，方便后续后端接入：

| 控件 ID | 含义 | 缺省 |
|---|---|---|
| `resFaultCount` | 模拟故障数量 | 1 |
| `resFaultLocations` | 模拟故障位置（元件 ID 或母线 ID，逗号分隔） | 空 |
| `resFaultStartHour` | 故障开始时间（小时） | 0 |
| `resFaultDuration` | 故障持续时长（小时） | 4 |
| `btnGenExtremeScenario` | 极端场景生成（按钮） | — |
| `resRepairDuration` | 修复时长（小时） | 6 |
| `resMobileSpeed` | 移动储能行驶速度（km/h） | 40 |
| `btnRunResilience` | 运行弹性评估 | — |
| `btnExportResilienceResults` | 评估结果导出 | — |

参数收集：`collectResilienceParams()` 把上述输入打包为 `{fault_count, fault_locations[], fault_start_hr, fault_duration_hr, repair_duration_hr, mobile_storage_speed_kmh}`。当前 `btnGenExtremeScenario` / `btnRunResilience` 仅向控制台打印参数，**后端 endpoint 待对接（约定：`POST /api/session/run_distribution_resilience`、`POST /api/session/gen_extreme_scenario`）**。

---

## 7. 主题与设计令牌

CSS 自定义属性集中在 `:root` 中，便于换肤：

```css
/* 全站基础 */
--bg / --bg2 / --bg3 / --paper / --ink / --ink2
--accent  /* #61afef 主蓝 */
--accent2 /* #56b6c2 青色，用于聚焦/悬停描边 */

/* 三层工具栏 */
--tb-bg / --tb-bg-hover         /* 静态底色 */
--tb-border / --tb-border-hover /* 描边（悬停为青色） */
--tb-active-bg1 / --tb-active-bg2 / --tb-active-border  /* 激活蓝色渐变 */
--tb-run-bg / --tb-run-bg-hover                         /* 运行按钮暖橙 */

--module-bar-bg / --module-bar-bg2  /* Bar 2 渐变 */
--sub-bar-bg / --sub-bar-text / --sub-bar-border  /* Bar 3 暗色 */
```

> **2026-05 调色重做**：旧版采用浅绿 `#8fd14f` + 鲜橙 `#ffb000` + 浅灰子工具栏 + 浅灰模态框，与暗色主面板割裂。当前版本统一改为暗色（深灰底 / 青色描边 / 蓝色激活 / 暖橙仅保留给“运行”按钮），与左侧元件库、画布、右侧属性面板一致。

---

## 8. 编码与扩展约定

1. **不要破坏现有 ID**：`btnSelect`、`btnConnect`、`pfDisplayUnit`、`vizMode`、`canvasInfo`、`vizLegend`、SVG 各 `<g>` 层 ID（`resultsLayer / componentsLayer / connectionsLayer / tempLayer`）受多处引用，重命名需全局更新。
2. **Canvas 模块只通过其公开 API 调用**：勿直接操作其内部 SVG 节点，以免破坏选择/撤销/序列化状态。
3. **新增功能模块** 三步走：
   - `index.html`：在 `#moduleBar` 加 `<button class="module-btn" data-module="<key>">…</button>`，并在 `#subToolbar` 加对应 `<div class="sub-section" data-sub="<key>">…</div>`。
   - `app.js`：在 `renderSubToolbar` 中无需改动（按 `data-sub` 自动匹配），在 `init()` 中绑定运行按钮 → 自定义 `runXxx()`。
   - `run_gui_server.cpp`：新增 `svr.Post("/api/session/run_xxx", ...)` endpoint，入参从 `g_session.current_system` 读取。
4. **新增 MATPOWER 数据源**：把 `.m` 文件放到 `external_data/matpower/` 目录即可（无需重启）。下次刷新模态时由 `loadMatpowerFileList()` 重新拉取。
5. **静态资源缓存**：浏览器开发时建议 `Ctrl+Shift+R` 硬刷新，`run_gui_server` 不发送 `Cache-Control` 头。
6. **调试 JSON 落盘**：每次 `POST /api/session/load_json_string` 会写 `debug_json_dump_<n>.json`（递增计数），可用于离线复现，不要把它们提交到仓库。

---

## 8.1 VirtualPowerPlant 模型迁移（2024 重构）

为对齐 `vpp_correct_version` 参考实现，VPP 结构已重构为以 PCC 为接入点的聚合元件，并增加资源 ID 列表字段，便于 OPF 阶段做能量平衡校核。

### 8.1.1 字段重命名

| 旧字段（`mtbf_hours/mttr_hours/aggregation_bus`） | 新字段                                  | 说明                                |
| -------------------------------------------------- | --------------------------------------- | ----------------------------------- |
| `aggregation_bus`                                  | `pcc_bus`                               | 与 `Microgrid.pcc_bus` 命名一致     |
| `mtbf_hours`                                       | `mtbf_hr`                               | 与发电机/可再生/变压器协议保持一致  |
| `mttr_hours`                                       | `mttr_hr`                               | 同上                                |
| —                                                  | `t_scheduled_hr`                        | 计划检修时间（小时/年）             |
| —                                                  | `aggregated_gen_ids` (`std::vector<int>`)    | 该 VPP 聚合的 Generator 索引列表     |
| —                                                  | `aggregated_storage_ids`                | 聚合的 Storage 索引列表             |
| —                                                  | `aggregated_load_ids`                   | 聚合的 Load 索引列表                |

### 8.1.2 JSON 兼容

`src/io/json_io.cpp::vpp_from_json` 通过新增的 `jget_alias` 助手同时识别新旧键：
- `pcc_bus` ↔ `aggregation_bus`
- `mtbf_hr` ↔ `mtbf_hours`
- `mttr_hr` ↔ `mttr_hours`

旧 JSON 文件可以无缝加载。`vpp_to_json` 写出新键。

### 8.1.3 前端映射

- `web/js/components.js` 中 `defaults.vpp` 默认字段已更新；`fieldLabels` 同时保留 `pcc_bus: 'PCC母线'` 与 `aggregation_bus: 'PCC母线'` 别名以便老画布数据导入显示标签。
- `web/js/canvas.js` 的 `case 'vpp':` 序列化分支：写出 `pcc_bus`、`aggregated_gen_ids/_storage_ids/_load_ids`；反序列化时优先读 `pcc_bus`，否则回退 `aggregation_bus`。资源 ID 列表支持字符串（逗号分隔）或数组形式输入。
- `web/js/app.js` 的 VPP 拓扑表格改读 `v.pcc_bus ?? v.aggregation_bus`。
- `web/index.html` 的 `#vppTableInner` 表头由 `Bus` 改为 `PCC Bus`。

### 8.1.4 验证

新增 `tests/test_vpp_pf_verify.cpp`，构建一个 3 总线 AC 系统，VPP 接到 PCC（Bus 3）：
1. JSON 写出/读回保留 `pcc_bus`、`aggregated_*_ids`、`mtbf_hr/mttr_hr/t_scheduled_hr`；
2. 旧 JSON（含 `aggregation_bus`/`mtbf_hours`）通过 alias 路径正确解析；
3. 设 `p_output_mw=6` 与 `p_output_mw=0` 各跑一次潮流，VPP 注入使主网供电下降 ≈ 6 MW（误差 < 0.6 MW）、PCC 节点电压上升 ≈ +0.001 pu。

可执行：`build/test_vpp_pf_verify.exe`。

### 8.1.5 涉及文件

- 模型：`include/hacdcpf/model/aggregation.hpp`
- 序列化：`src/io/json_io.cpp`、`src/io/excel_io.cpp`（Excel 列名 `aggregation_bus` 保留以维持模板兼容）
- 潮流：`src/powerflow/solver_data.cpp`、`src/powerflow/island_detector.cpp`
- 分析/可视化：`src/analysis/distribution_power_flow.cpp`、`src/visualization/html_visualizer.cpp`、`src/model/network_utils.cpp`、`src/opf/parity_formulation.cpp`、`src/api/hacdcpf.cpp`
- 前端：`web/js/components.js`、`web/js/canvas.js`、`web/js/app.js`、`web/index.html`

### 8.1.6 DC/DC 一致性

经与 `vpp_correct_version` 比对，`DCDCConverter` 字段集合一致：`bus_in/bus_out`、`p_ref_mw`、`v_ref_pu`、`sn_mva`、`vn_in_kv/vn_out_kv`、`eta`、`r_eq_pu`、`pmax_mw/pmin_mw`、`k_droop`、`controllable`、`f_switching_hz`、`control_mode (Voltage/Power/Droop)`，仅可靠性字段保留本仓库的 `mtbf_hours/mttr_hours` 命名（与本仓库其他元件一致），不影响潮流行为。`vpp_correct_version` 把整套可靠性字段重命名为 `_hr`，本次未做项目级别重命名以避免大规模回归。

---

## 9. 已知限制 / TODO

- 可靠性、弹性两个模块仅保留占位子工具栏，尚未接入 endpoint UI。
- 时序潮流的 CSV 解析 (`parseCsvProfiles`) 假定单列、无表头；多列辐照/价格/负荷需扩展。
- `Canvas.autoLayout()` 对超大算例（>3000 节点）耗时较高，建议导入大网时先关闭可视化。
- 元件库目前以拖放方式新建；尚未实现“按拓扑批量克隆”。

---

文档维护：与代码同步更新；若 UI 重大变更（新增工具栏层 / 调整主题令牌 / 修改 API 路径），请同步修改本文件。
