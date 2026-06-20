# 基于网页的配电系统建模仿真工具界面与自动布局优化设计方案

## 1. 文档目标

本文档用于指导对现有基于网页的配电系统建模仿真工具进行界面优化与自动布局改造。

当前系统已经具备以下能力：

- SVG 单线图画布
- 元件拖拽放置
- 元件连接
- 潮流计算结果叠加
- 电压、功率流、负载率可视化
- JSON 系统导入与导出
- 简单自动布局

但当前存在以下问题：

- 元件自动分布效果不理想
- 母线、支路、负荷、光伏、储能等设备容易堆叠
- 连接线容易交叉，视觉较乱
- 页面缺少专业工程软件的视觉层次
- 缺少黑色 / 白色背景主题切换能力
- 自动布局算法没有充分利用配电网拓扑特征

本方案目标是：

1. 设计更专业、美观、清晰的界面布局。
2. 优化自动布局逻辑，使不同元件能够自动合理分布。
3. 减少线路交叉和凌乱拖拽。
4. 增加黑色 / 白色背景版面切换属性。
5. 为后续功能扩展预留结构。

---

# 2. 总体界面设计方案

## 2.1 推荐页面结构

推荐采用专业仿真软件常用的“三栏式 + 顶部工具栏 + 底部状态栏”布局。

```text
┌────────────────────────────────────────────────────────────┐
│ 顶部工具栏：文件 / 编辑 / 视图 / 自动布局 / 潮流 / 主题切换 │
├──────────────┬──────────────────────────────┬──────────────┤
│ 左侧元件库     │          SVG 主画布             │ 右侧属性面板   │
│ Palette      │          Canvas              │ Inspector    │
│              │                              │ Results      │
├──────────────┴──────────────────────────────┴──────────────┤
│ 底部状态栏：元件数 / 连接数 / 缩放比例 / 当前模式 / 仿真状态 │
└────────────────────────────────────────────────────────────┘
```

## 2.2 区域功能定义

| 区域 | 功能 |
|---|---|
| 顶部工具栏 | 文件操作、撤销重做、运行潮流、自动布局、缩放、主题切换 |
| 左侧元件库 | AC/DC 母线、线路、变压器、负荷、发电机、储能、光伏等元件 |
| 中央画布 | SVG 单线图编辑、拖拽、连线、结果显示 |
| 右侧属性面板 | 当前选中元件参数编辑、潮流结果详情 |
| 底部状态栏 | 显示系统状态、选择数量、连接数量、错误提示 |

---

# 3. 视觉风格设计

## 3.1 设计原则

界面应遵循以下原则：

- **清晰**：母线、设备、连接线、结果叠加互不干扰。
- **层次分明**：画布、工具栏、面板、弹窗有明确视觉边界。
- **工程化**：采用专业电力仿真软件风格，避免过度装饰。
- **低干扰**：默认显示简洁，选中或悬浮时才强调。
- **可主题切换**：支持白色背景和黑色背景两种版面。

## 3.2 白色主题风格

白色主题适合日常编辑、打印、报告截图。

```css
:root[data-theme="light"] {
  --app-bg: #f3f6fa;
  --panel-bg: #ffffff;
  --canvas-bg: #f8fafc;
  --canvas-grid: #e2e8f0;
  --border-color: #d8dee9;
  --text-main: #0f172a;
  --text-muted: #64748b;
  --primary: #2563eb;
  --primary-soft: #dbeafe;
  --connection: #64748b;
  --connection-hover: #2563eb;
  --component-bg: #ffffff;
  --component-stroke: #334155;
  --selected: #2563eb;
  --danger: #ef4444;
  --success: #22c55e;
  --warning: #f59e0b;
}
```

## 3.3 黑色主题风格

黑色主题适合大屏展示、夜间使用、实时监控界面。

```css
:root[data-theme="dark"] {
  --app-bg: #0f172a;
  --panel-bg: #111827;
  --canvas-bg: #020617;
  --canvas-grid: #1e293b;
  --border-color: #334155;
  --text-main: #e5e7eb;
  --text-muted: #94a3b8;
  --primary: #38bdf8;
  --primary-soft: rgba(56, 189, 248, 0.16);
  --connection: #94a3b8;
  --connection-hover: #38bdf8;
  --component-bg: #111827;
  --component-stroke: #cbd5e1;
  --selected: #38bdf8;
  --danger: #f87171;
  --success: #4ade80;
  --warning: #fbbf24;
}
```

## 3.4 画布背景设计

白色背景：

```css
#canvas {
  background-color: var(--canvas-bg);
  background-image:
    linear-gradient(var(--canvas-grid) 1px, transparent 1px),
    linear-gradient(90deg, var(--canvas-grid) 1px, transparent 1px);
  background-size: 20px 20px;
}
```

黑色背景：

```css
:root[data-theme="dark"] #canvas {
  background-color: var(--canvas-bg);
  background-image:
    linear-gradient(var(--canvas-grid) 1px, transparent 1px),
    linear-gradient(90deg, var(--canvas-grid) 1px, transparent 1px);
  background-size: 20px 20px;
}
```

## 3.5 元件视觉样式

```css
.component {
  filter: drop-shadow(0 2px 3px rgba(0, 0, 0, 0.18));
}

.component .comp-outline {
  transition: stroke 0.15s ease, fill 0.15s ease;
}

.component:hover .comp-outline {
  stroke: var(--primary);
  fill: var(--primary-soft);
}

.component.selected .comp-outline {
  stroke: var(--selected);
  stroke-width: 2.5;
  fill: var(--primary-soft);
}
```

## 3.6 连接线视觉样式

```css
.connection .conn-line {
  stroke: var(--connection);
  stroke-width: 2;
  stroke-linecap: round;
  stroke-linejoin: round;
  transition: stroke 0.15s ease, stroke-width 0.15s ease;
}

.connection:hover .conn-line {
  stroke: var(--connection-hover);
  stroke-width: 3;
}

.connection.conn-selected .conn-line {
  stroke: var(--warning);
  stroke-width: 4;
}
```

---

# 4. 新增“背景版面主题”属性设计

## 4.1 功能目标

界面中新增一个属性，允许用户在以下两种版面中切换：

- 白色背景版面
- 黑色背景版面

推荐名称：

```text
主题模式 / Theme Mode
```

取值：

```text
light：白色背景
dark：黑色背景
```

## 4.2 UI 控件设计

建议在顶部工具栏加入：

```html
<label class="toolbar-label">主题</label>
<select id="themeModeSelect" class="toolbar-select">
  <option value="light">白色背景</option>
  <option value="dark">黑色背景</option>
</select>
```

或者使用按钮形式：

```html
<button id="btnToggleTheme" class="toolbar-btn">
  切换黑/白背景
</button>
```

## 4.3 状态设计

可以在全局 App 状态中增加：

```js
const appSettings = {
  themeMode: 'light'
};
```

或者存入 Canvas 内部状态：

```js
const state = {
  ...
  themeMode: 'light'
};
```

## 4.4 主题切换函数

```js
function setThemeMode(mode) {
  const finalMode = mode === 'dark' ? 'dark' : 'light';
  document.documentElement.setAttribute('data-theme', finalMode);

  try {
    localStorage.setItem('themeMode', finalMode);
  } catch (e) {
    console.warn('Failed to save theme mode:', e);
  }

  if (Canvas && Canvas.state) {
    Canvas.state.themeMode = finalMode;
  }
}

function toggleThemeMode() {
  const current = document.documentElement.getAttribute('data-theme') || 'light';
  setThemeMode(current === 'dark' ? 'light' : 'dark');
}

function initThemeMode() {
  let saved = 'light';

  try {
    saved = localStorage.getItem('themeMode') || 'light';
  } catch (e) {}

  setThemeMode(saved);

  const select = document.getElementById('themeModeSelect');
  if (select) {
    select.value = saved;
    select.addEventListener('change', e => {
      setThemeMode(e.target.value);
    });
  }

  const btn = document.getElementById('btnToggleTheme');
  if (btn) {
    btn.addEventListener('click', toggleThemeMode);
  }
}
```

## 4.5 与 SVG 结果叠加兼容

结果文本颜色建议不要写死，应根据主题调整。

示例：

```js
function getThemeAwareTextColor() {
  const theme = document.documentElement.getAttribute('data-theme') || 'light';
  return theme === 'dark' ? '#e5e7eb' : '#0f172a';
}
```

对普通说明文本、标签、连接标签，可使用主题色。

电压越限、潮流方向、负载率热力图仍保留专业颜色：

- 正常：绿色
- 低压：红色
- 高压：橙色
- 潮流箭头：蓝色或热力颜色

---

# 5. 自动布局总体流程

## 5.1 自动布局目标

自动布局应满足以下目标：

1. 母线作为主拓扑节点。
2. 线路和变压器放置在两端母线之间。
3. 负荷、光伏、储能、发电机等设备围绕所属母线分区排列。
4. 以 Slack 母线或外部电网连接母线作为根节点。
5. 配电网主馈线尽量自上而下或自左向右展开。
6. 减少连接线交叉。
7. 避免元件重叠。
8. 布局完成后自动适配视图。

## 5.2 推荐布局方向

默认采用：

```text
TB：Top to Bottom，自上而下
```

对配电系统而言，推荐：

```text
外部电网 / 主变 / Slack 母线
            ↓
         主干馈线
            ↓
         分支馈线
            ↓
          负荷侧
```

可选方向：

| 模式 | 含义 | 适用场景 |
|---|---|---|
| TB | 自上而下 | 配电网、径向网络 |
| LR | 自左向右 | 输电网、报告展示 |
| RADIAL | 径向 | 环网、多分支系统 |
| COMPACT | 紧凑 | 小型测试系统 |

## 5.3 布局流程总览

```text
1. 收集画布元件
   ├── 母线：ac_bus, dc_bus
   ├── 支路：ac_branch, dc_branch, transformer_2w, transformer_3w
   └── 设备：load, generator, pv_system, storage 等

2. 构建母线拓扑图
   ├── 以支路 / 变压器作为边
   └── 以母线作为节点

3. 识别根节点
   ├── 优先选择 SLACK 母线
   ├── 其次选择连接 external_grid 的母线
   └── 否则选择度数最高的母线

4. BFS/DFS 建立层级树
   ├── 计算 parent
   ├── 计算 children
   ├── 计算 depth
   └── 处理孤岛网络

5. 计算子树宽度
   ├── 叶子母线宽度为 1
   └── 父节点宽度为所有子节点宽度之和

6. 分配母线坐标
   ├── 子节点优先分布
   ├── 父节点居中于子节点
   └── 全局居中

7. 放置支路 / 变压器
   ├── 位于两端母线中点
   └── 根据方向轻微偏移

8. 放置挂接设备
   ├── external_grid 放上方
   ├── generator 放左侧
   ├── pv / renewable 放右侧
   ├── load / motor 放下方
   ├── storage 放右下
   └── shunt 放左下

9. 重绘连接线

10. 适配视图 zoomFit()

11. 刷新潮流可视化叠加层
```

---

# 6. 元件分类规则

## 6.1 主节点：母线

```js
const busTypes = new Set([
  'ac_bus',
  'dc_bus'
]);
```

## 6.2 拓扑边：线路和变压器

```js
const branchTypes = new Set([
  'ac_branch',
  'dc_branch',
  'transformer_2w',
  'transformer_3w'
]);
```

## 6.3 挂接设备

除母线与支路外，其余大多数元件都作为挂接设备。

包括：

```text
generator
load
external_grid
storage
pv_system
renewable_gen
static_generator
motor
flexible_load
asymmetric_load
shunt
charger
charging_station
mobile_storage
vpp
microgrid
```

## 6.4 转换设备

下面这些设备具有多端口，布局时建议单独处理：

```text
vsc_converter
dcdc_converter
energy_router
```

---

# 7. 母线拓扑布局算法设计

## 7.1 构建母线图

母线图以母线为节点，以线路 / 变压器为边。

逻辑：

```text
对每一个支路元件：
  找到它连接的母线
  如果连接了两个或多个母线：
    在母线图中添加边
```

示例：

```text
Bus 1 --- Line 1 --- Bus 2
```

转换为：

```text
graph[Bus1].add(Bus2)
graph[Bus2].add(Bus1)
edgeBranchMap["Bus1-Bus2"] = Line1
```

## 7.2 根母线选择规则

优先级：

1. `bus_type === SLACK`
2. 与 `external_grid` 相连的母线
3. 度数最高的母线
4. 第一条母线

## 7.3 层级计算

使用 BFS 从根母线出发：

```text
root depth = 0
root 的邻居 depth = 1
继续向下扩展
```

生成：

```js
parent: Map<busId, parentBusId>
children: Map<busId, childBusId[]>
depth: Map<busId, number>
```

## 7.4 子树宽度计算

```text
叶子母线宽度 = 1
非叶子母线宽度 = 所有子树宽度之和
```

这个步骤用于避免同一层的节点重叠。

## 7.5 坐标分配

推荐自上而下布局：

```text
x = 子树横向位置
y = depth * levelGap
```

推荐参数：

```js
{
  levelGap: 260,
  nodeGap: 220,
  deviceGap: 110
}
```

---

# 8. 支路和变压器布局设计

## 8.1 线路放置规则

对于连接两个母线的线路：

```text
支路位置 = 两个母线的中点
```

同时增加轻微法向偏移，避免线路图标与连线完全重叠。

```text
mx = (busA.x + busB.x) / 2
my = (busA.y + busB.y) / 2

dx = busB.x - busA.x
dy = busB.y - busA.y

法向量:
nx = -dy / length
ny = dx / length

branch.x = mx + nx * offset
branch.y = my + ny * offset
```

## 8.2 变压器放置规则

变压器与线路类似，但偏移略大：

| 元件 | 偏移量 |
|---|---|
| ac_branch | 20 |
| dc_branch | 20 |
| transformer_2w | 30 |
| transformer_3w | 40 |

## 8.3 支路方向建议

如果支路两端母线主要上下分布：

- 支路图标可以保持默认方向
- 连接线从母线 bottom/top 连接

如果支路两端母线主要左右分布：

- 可以考虑旋转支路图标 90 度
- 连接线从 left/right 连接

初期可以不做自动旋转，优先保证位置合理。

---

# 9. 挂接设备布局设计

## 9.1 分区原则

每个母线周围划分为六个区域：

```text
           top
            |
     left - bus - right
            |
          bottom

     bottomLeft     bottomRight
```

## 9.2 元件类型与区域映射

| 元件类型 | 推荐区域 | 原因 |
|---|---|---|
| external_grid | top | 电源入口，放上方直观 |
| generator | left | 发电设备，靠左侧 |
| pv_system | right | 分布式电源，靠右侧 |
| renewable_gen | right | 分布式电源，靠右侧 |
| static_generator | right | 分布式电源，靠右侧 |
| load | bottom | 负荷一般位于馈线末端或下方 |
| flexible_load | bottom | 负荷类 |
| asymmetric_load | bottom | 负荷类 |
| motor | bottom | 负荷类 |
| charging_station | bottom | 负荷类 |
| storage | bottomRight | 可充可放，靠右下 |
| mobile_storage | bottomRight | 储能类 |
| shunt | bottomLeft | 无功补偿设备 |
| vpp | right | 聚合资源 |
| microgrid | bottomRight | 局部系统 |
| charger | bottom | 充电设备 |

## 9.3 多设备排列

同一区域内如果有多个设备：

- top / bottom 区域：水平排列
- left / right 区域：垂直排列
- bottomLeft / bottomRight：垂直或网格排列

示例：

```text
top 区域:
   Dev1  Dev2  Dev3
        Bus

right 区域:
        Bus - Dev1
              Dev2
              Dev3
```

## 9.4 设备间距

推荐：

```js
const deviceSpacing = 80;
const deviceGap = 110;
```

---

# 10. 连线优化设计

## 10.1 短期方案：保留直线

由于当前潮流箭头、热力图、功率标签大量依赖：

```js
conn.el.querySelector('line')
```

因此短期内建议先保留 SVG `line`。

优先通过优化元件位置来减少线条交叉。

优点：

- 改动小
- 不破坏当前潮流可视化
- 容易回归测试

## 10.2 中期方案：正交折线

后续可将连接线改为 `path`：

```text
A ───┐
     │
     └── B
```

推荐路径函数：

```js
function makeOrthogonalPath(p1, p2) {
  const dx = Math.abs(p2.x - p1.x);
  const dy = Math.abs(p2.y - p1.y);

  if (dx >= dy) {
    const midX = (p1.x + p2.x) / 2;
    return `M ${p1.x} ${p1.y} L ${midX} ${p1.y} L ${midX} ${p2.y} L ${p2.x} ${p2.y}`;
  }

  const midY = (p1.y + p2.y) / 2;
  return `M ${p1.x} ${p1.y} L ${p1.x} ${midY} L ${p2.x} ${midY} L ${p2.x} ${p2.y}`;
}
```

## 10.3 长期方案：自动避让

长期可实现：

- 多线平行偏移
- 避免穿越母线
- 避免穿越重要设备
- 端口方向自动选择
- 曼哈顿路径搜索

---

# 11. 建议新增的自动布局函数结构

建议在 `canvas.js` 中新增以下函数：

```js
function autoLayoutSmart(options = {}) {}

function findRootBus(buses, graph) {}

function buildBusGraph(buses, branches) {}

function computeTreeLayout(root, graph, buses, options) {}

function layoutBranches(branches, busIdSet) {}

function layoutDevicesAroundBuses(devices, busIdSet, gap) {}

function findAttachedBusComponentId(compId, busIdSet) {}

function placeGroup(items, cx, cy, mode) {}

function applyComponentTransform(comp) {}
```

## 11.1 `autoLayoutSmart()` 总控函数

推荐流程：

```js
function autoLayoutSmart(options = {}) {
  const direction = options.direction || 'TB';
  const levelGap = options.levelGap || 260;
  const nodeGap = options.nodeGap || 220;
  const deviceGap = options.deviceGap || 110;

  const buses = state.components.filter(c =>
    c.type === 'ac_bus' || c.type === 'dc_bus'
  );

  const branchTypes = new Set([
    'ac_branch',
    'dc_branch',
    'transformer_2w',
    'transformer_3w'
  ]);

  const branches = state.components.filter(c => branchTypes.has(c.type));

  const devices = state.components.filter(c =>
    c.type !== 'ac_bus' &&
    c.type !== 'dc_bus' &&
    !branchTypes.has(c.type)
  );

  if (buses.length === 0) {
    zoomFit();
    return;
  }

  const busIdSet = new Set(buses.map(b => b.id));

  const graphInfo = buildBusGraph(buses, branches);
  const root = findRootBus(buses, graphInfo.graph);

  computeTreeLayout(root, graphInfo.graph, buses, {
    direction,
    levelGap,
    nodeGap
  });

  layoutBranches(branches, busIdSet);
  layoutDevicesAroundBuses(devices, busIdSet, deviceGap);

  state.connections.forEach(rerenderConnection);

  zoomFit();

  if (_vizMode !== 'off' && _lastPfResult) {
    applyVisualizationOverlay();
  }

  updateInfo();
}
```

## 11.2 替换原有 `autoLayout()`

建议将原有 `autoLayout()` 改为：

```js
function autoLayout() {
  autoLayoutSmart({
    direction: 'TB',
    levelGap: 260,
    nodeGap: 220,
    deviceGap: 110
  });
}
```

## 11.3 导出接口

在 `Canvas` 的 public API 中增加：

```js
return {
  ...
  autoLayout,
  autoLayoutSmart,
  ...
};
```

---

# 12. 主题属性持久化到 JSON

## 12.1 是否保存主题到系统 JSON

推荐将主题配置保存到 `_canvas` 字段中。

当前系统已经在导出 JSON 时保存：

```js
sys._canvas = {
  version: 1,
  viewBox: ...,
  components: ...,
  connections: ...
};
```

建议扩展为：

```js
sys._canvas = {
  version: 2,
  themeMode: state.themeMode || 'light',
  layoutMode: state.layoutMode || 'smart',
  viewBox: { ... },
  components: [ ... ],
  connections: [ ... ]
};
```

## 12.2 导入时恢复主题

在 `applyCanvasLayout(layout)` 中增加：

```js
if (layout.themeMode) {
  setThemeMode(layout.themeMode);
}
```

或在 `loadFromSystemJson(jsonSys)` 中：

```js
if (jsonSys._canvas?.themeMode) {
  setThemeMode(jsonSys._canvas.themeMode);
}
```

---

# 13. 顶部工具栏设计建议

## 13.1 工具栏按钮

推荐按钮：

```text
选择
连线
撤销
重做
自动布局
适配视图
放大
缩小
运行潮流
清除结果
主题：白色 / 黑色
```

## 13.2 HTML 示例

```html
<div class="toolbar">
  <button id="btnSelect" class="toolbar-btn canvas-tool">选择</button>
  <button id="btnConnect" class="toolbar-btn canvas-tool">连线</button>

  <span class="toolbar-separator"></span>

  <button id="btnAutoLayoutSmart" class="toolbar-btn">自动美化布局</button>
  <button id="btnZoomFit" class="toolbar-btn">适配视图</button>

  <span class="toolbar-separator"></span>

  <button id="btnRunPF" class="toolbar-btn primary">运行潮流</button>
  <button id="btnClearResults" class="toolbar-btn">清除结果</button>

  <span class="toolbar-spacer"></span>

  <label class="toolbar-label">背景</label>
  <select id="themeModeSelect" class="toolbar-select">
    <option value="light">白色</option>
    <option value="dark">黑色</option>
  </select>
</div>
```

## 13.3 CSS 示例

```css
.toolbar {
  height: 48px;
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 0 12px;
  background: var(--panel-bg);
  border-bottom: 1px solid var(--border-color);
  color: var(--text-main);
}

.toolbar-btn {
  height: 32px;
  padding: 0 12px;
  border-radius: 8px;
  border: 1px solid var(--border-color);
  background: var(--panel-bg);
  color: var(--text-main);
  cursor: pointer;
  font-size: 13px;
}

.toolbar-btn:hover {
  background: var(--primary-soft);
  border-color: var(--primary);
  color: var(--primary);
}

.toolbar-btn.primary {
  background: var(--primary);
  color: white;
  border-color: var(--primary);
}

.toolbar-select {
  height: 32px;
  border-radius: 8px;
  border: 1px solid var(--border-color);
  background: var(--panel-bg);
  color: var(--text-main);
  padding: 0 8px;
}

.toolbar-spacer {
  flex: 1;
}

.toolbar-separator {
  width: 1px;
  height: 24px;
  background: var(--border-color);
  margin: 0 4px;
}
```

---

# 14. 左侧元件库设计

## 14.1 分组方式

推荐分组：

```text
基础元件
  - AC 母线
  - DC 母线
  - AC 线路
  - DC 线路
  - 二绕组变压器
  - 三绕组变压器

电源
  - 外部电网
  - 发电机
  - 光伏
  - 风电 / 可再生能源
  - 静态发电机

负荷
  - 普通负荷
  - 灵活负荷
  - 不平衡负荷
  - 电机
  - 充电站

储能与电力电子
  - 储能
  - 移动储能
  - VSC
  - DC/DC
  - 能量路由器

控制与保护
  - 开关
  - 断路器
  - 并联补偿
```

## 14.2 卡片样式

```css
.component-palette-item {
  display: flex;
  align-items: center;
  gap: 8px;
  height: 36px;
  padding: 0 10px;
  border-radius: 8px;
  color: var(--text-main);
  cursor: grab;
}

.component-palette-item:hover {
  background: var(--primary-soft);
  color: var(--primary);
}
```

---

# 15. 右侧属性面板设计

## 15.1 面板组成

推荐使用 Tab：

```text
属性 | 结果 | 校验 | 图层
```

## 15.2 属性编辑布局

每个属性使用两列：

```text
名称       [输入框]
类型       [下拉框]
额定电压   [输入框]
有功功率   [输入框]
无功功率   [输入框]
是否投运   [开关]
```

## 15.3 结果面板

可显示：

```text
母线电压
线路功率
线路负载率
设备有功 / 无功
越限告警
孤岛状态
```

---

# 16. 状态栏设计

## 16.1 显示内容

```text
元件数：35 | 连接数：42 | 已选：2 | 缩放：100% | 模式：选择 | 状态：潮流计算成功
```

## 16.2 样式

```css
.statusbar {
  height: 28px;
  display: flex;
  align-items: center;
  padding: 0 12px;
  background: var(--panel-bg);
  border-top: 1px solid var(--border-color);
  color: var(--text-muted);
  font-size: 12px;
}
```

---

# 17. 自动布局质量评估指标

为了判断布局是否变好，可以定义以下指标：

## 17.1 元件重叠数量

```text
越少越好，目标为 0。
```

## 17.2 连接线交叉数量

```text
越少越好。
```

## 17.3 平均连接线长度

```text
不宜过长，也不宜过短。
```

## 17.4 同类型设备一致性

```text
同类型设备应尽量出现在母线的相同方位。
```

## 17.5 主拓扑方向一致性

```text
配电网馈线应尽量从上到下或从左到右展开。
```

---

# 18. 推荐实施路线图

## 第一阶段：界面主题与基础美化

目标：不改变核心逻辑，快速改善观感。

任务：

- 增加 CSS 变量主题
- 增加白色 / 黑色背景切换
- 工具栏按钮美化
- 面板边框、阴影、圆角
- SVG 网格背景
- 选中、悬浮样式优化

预期效果：

- 页面更现代
- 白色和黑色主题均可使用
- 编辑体验更清晰

## 第二阶段：智能自动布局

目标：解决元件乱分布问题。

任务：

- 新增 `autoLayoutSmart()`
- 构建母线拓扑图
- 自动选择根母线
- 子树宽度分配
- 支路居中放置
- 设备按类型分区挂接
- 自动 `zoomFit()`

预期效果：

- 拓扑结构更清晰
- 大多数配电系统可以自动生成美观单线图
- 减少人工拖拽

## 第三阶段：连线优化

目标：减少线路交叉和凌乱视觉。

任务：

- 初期保留直线
- 中期支持正交折线
- 增加路径中点计算
- 更新潮流箭头显示逻辑
- 支持多线偏移

预期效果：

- 单线图更接近专业软件
- 连接线更规整

## 第四阶段：高级布局

目标：处理复杂网络。

任务：

- 环网检测
- 多馈线自动分组
- 交叉最小化
- 局部重新布局
- 拖动时局部吸附
- 自动避让标签

预期效果：

- 支持更大规模网络
- 支持复杂配电网和微电网结构

---

# 19. 推荐优先修改的文件和函数

## 19.1 `canvas.js`

重点修改：

```text
autoLayout()
renderConnection()
rerenderConnection()
buildSystemJson()
applyCanvasLayout()
loadFromSystemJson()
clearAll()
updateInfo()
```

新增：

```text
autoLayoutSmart()
buildBusGraph()
findRootBus()
computeTreeLayout()
layoutBranches()
layoutDevicesAroundBuses()
findAttachedBusComponentId()
placeGroup()
applyComponentTransform()
```

## 19.2 CSS 文件

新增：

```text
theme variables
toolbar styles
panel styles
canvas grid styles
component hover/selected styles
connection styles
dark theme overrides
```

## 19.3 HTML 文件

新增：

```text
theme selector
auto layout button
toolbar separator
layout mode selector
statusbar fields
```

---

# 20. 关键代码片段汇总

## 20.1 主题初始化

```js
function initThemeMode() {
  let saved = 'light';

  try {
    saved = localStorage.getItem('themeMode') || 'light';
  } catch (e) {}

  setThemeMode(saved);

  const select = document.getElementById('themeModeSelect');
  if (select) {
    select.value = saved;
    select.addEventListener('change', e => {
      setThemeMode(e.target.value);
    });
  }
}
```

## 20.2 主题切换

```js
function setThemeMode(mode) {
  const finalMode = mode === 'dark' ? 'dark' : 'light';
  document.documentElement.setAttribute('data-theme', finalMode);

  try {
    localStorage.setItem('themeMode', finalMode);
  } catch (e) {}

  if (Canvas && Canvas.state) {
    Canvas.state.themeMode = finalMode;
  }
}
```

## 20.3 自动布局入口

```js
function autoLayout() {
  autoLayoutSmart({
    direction: 'TB',
    levelGap: 260,
    nodeGap: 220,
    deviceGap: 110
  });
}
```

## 20.4 自动布局按钮

```js
document.getElementById('btnAutoLayoutSmart')?.addEventListener('click', () => {
  Canvas.autoLayoutSmart?.();
});
```

---

# 21. 最终效果目标

优化完成后，系统应达到以下效果：

## 21.1 界面效果

- 页面整体清爽、专业
- 支持黑色和白色两种背景版面
- 画布具有网格和层次感
- 元件选中、悬浮效果清晰
- 工具栏、属性栏、状态栏结构明确

## 21.2 布局效果

- 点击一次“自动美化布局”即可形成清晰单线图
- 母线层级清楚
- 线路和变压器位于合理中间位置
- 负荷、电源、储能等设备按类型整齐挂接
- 连线交叉显著减少
- 导入系统后可以自动整理布局

## 21.3 扩展效果

- 后续可增加正交连线
- 可实现环网布局
- 可实现拓扑重构前后对比
- 可实现运行状态大屏展示
- 可支持不同主题导出截图

---

# 22. 建议的最终改造顺序

推荐按以下顺序实施：

```text
1. 增加主题变量和黑/白背景切换
2. 美化工具栏、侧边栏、状态栏
3. 美化 SVG 画布背景和元件选中效果
4. 新增 autoLayoutSmart()
5. 替换原 autoLayout()
6. 增加自动美化布局按钮
7. 保存 themeMode 到 _canvas
8. 导入 JSON 时恢复 themeMode
9. 测试 IEEE 9、IEEE 14、IEEE 33 节点等系统
10. 再考虑将直线连接改为正交折线
```

---

# 23. 结论

本方案的核心思想是：

> 不只是美化 CSS，而是将“配电网拓扑结构”作为布局算法的基础。

对于配电系统建模仿真工具而言，最重要的是：

- 以母线为核心组织拓扑
- 以线路和变压器表达网络骨架
- 以负荷、电源、储能等设备表达挂接关系
- 以 Slack / 外部电网作为视觉起点
- 以自上而下的馈线结构呈现系统

同时，黑色 / 白色背景主题切换可以让系统兼顾：

- 日常建模
- 报告截图
- 夜间使用
- 监控大屏展示

按本方案实施后，你的网页配电系统建模仿真工具将从“可用的编辑器”提升为“更专业、更清晰、更接近工程软件体验的建模平台”。