> 本文档为 [svg_distribution_import.md](svg_distribution_import.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

# IEC-CGE 配电 SVG 导入与导出

## 已实现的 profile

`hacdcpf::io::from_svg_distribution` 导入 `data/` 中馈线图纸所使用的
带标注单线图 SVG 方言。该 profile 通过以下特征识别：

- 带有 `xmlns:cge="http://iec.ch/TC57/2005/SVG-schema#"` 的 SVG XML；
- `cge:psr_ref` 中的设备元数据（`objectid`、`psrtype`、`classname`）；
- 设备图层，如 `ACLineSegment_Layer`、`Breaker_Layer`、
  `PowerTransformer_Layer` 和 `BusbarSection_Layer`；
- 电气连接关系由折线顶点和经坐标变换的设备端子表示，
  而不是 CIM 的 `Terminal` / `ConnectivityNode` 引用。

这不是 IEC 61970/61968 CIM 网络文档。不含 `cge:psr_ref` 设备的
纯装饰性 SVG 会被拒绝。

## 映射契约

| SVG 类 | HACDCPF 模型 | 绑定方式 |
| --- | --- | --- |
| `PWConductorSecPSR` / `PWCableSecPSR` | `ACBranch` | 保留对象 ID/名称；折线段转为支路。 |
| `PWConnectLine` / `PWInnerLinkLine` | 连接节点合并 | 理想连接件；不虚构支路阻抗。 |
| `PWBusbarPSR` | 连接节点合并 | 位于母线上的端点和顶点被合并。 |
| 断路器、隔离开关、负荷开关、熔断器类 | `Switch` | 保留类型；符号变体 `@1` 映射为断开状态。 |
| `PWBusCouplePSR` | `Switch` | 折线两端点即母联的两个端子。 |
| `PWOPTransformerPSR` | `Transformer2W` + 终端 `Load` | 有标注容量时使用标注容量；否则使用可配置的默认值。 |
| `Substation` | 合成的 `ExternalGrid` 位置 | 每个连接到变电站的组件对应一个电源。 |

名为 `TXT-PD_<object-id>` 的文本组在归一化分隔符后与设备关联。
源 `objectid` 仍作为兜底名称，因此对外稳定标识不会被
vector position 或 graph index 替代。

## 参数与有效性边界

源图纸并不包含完整的电气资产台账。因此导入报告始终使用
`UnitAssertion::BestEffort`，并对以下内容记录强制转换（coercion）：

- 图纸距离到线路长度的换算；
- 架空/电缆线路的电阻与电抗默认值；
- 变压器阻抗默认值；
- 变压器终端负荷的容量系数与功率因数；
- 合成的外部电网等值。

所有取值均可通过 `SvgDistributionImportOptions` 配置。默认情况下，
导入器随后仅对标记为 SVG 估计值的字段应用共享的
标准感知参数补全（standards-aware parameter completion）。
报告包含线路级建议、置信度、变更字段计数以及
国家/行业/IEC 标准引用。将 `auto_complete_parameters=false`
可保留初始的 SVG 估计值。严格模式（Strict mode）会拒绝导入，
因为这些推断不可避免。未通过当前闭合开关连接到变电站的组件
保持 `BusType::ISOLATED`；导入器不会仅为了让死孤岛看起来有供电
而添加电源。因此潮流输出会对这些孤立母线报告零电压。

设置 `auto_add_external_grids=false` 可完全禁用电源合成，
并将每个恢复的组件标记为孤立，直到调用方提供显式电源。

## 导出契约

`hacdcpf::io::to_svg_distribution` 和 `save_svg_distribution`
将交流配电子集导出为分层 IEC-CGE SVG。文档包含
`cge:psr_ref` 稳定设备标识，并且默认包含一个
`hacdcpf:model` 扩展，携带通过本适配器进行无损往返（round-trip）
所需的电气参数。导出器覆盖交流母线、支路、开关、
双绕组变压器、外部电网以及变压器终端负荷。
无法内嵌的独立负荷和非交流资产不会被静默近似：
其计数和警告会在 `SvgDistributionExportResult` 中返回。

生成的布局由交流图确定性地生成，并可通过
`horizontal_spacing`、`vertical_spacing` 和 `margin` 配置。
这是工程单线图布局，而不是保留所导入图纸的原始视觉坐标。

## 运行时

GUI 的 Model IO 工具栏提供 **导入配电SVG**。它向
`POST /api/session/load_svg_distribution` 发送：

```json
{
  "svg_string": "<svg ...>",
  "name": "feeder name"
}
```

响应包含常规的系统摘要，外加 `_svg_*` 字段：源对象计数、
恢复的节点、未解析对象、合成电源、孤立组件/母线、警告
以及模型范围。`_svg_parameter_completion` 包含完整的
标准感知补全报告。导入后，常规的会话潮流/最优潮流/分析路由
即可作用于生成的 `HybridPowerSystem`。

同一工具栏还提供 **导出配电SVG**。它先同步画布，然后向
`POST /api/session/export_svg_distribution` 发送请求；
响应包含 SVG 文本、已导出/已省略计数、警告以及
`model_scope=ac_distribution_iec_cge_svg`。浏览器将结果下载为
`*_distribution.svg`。服务器端布局与序列化在获取模型快照后、
于会话互斥锁之外运行。

标准与假设的对照表维护在
`distribution_parameter_completion_standards.zh.md` 中。

## 回归测试基件

`tests/test_io_svg_distribution.cpp` 导入并求解两个已入库的文件：

| 基件 | 恢复的源线路数 | 变压器 / 估计负荷数 | 恢复的母线数 | 潮流结果 |
| --- | ---: | ---: | ---: | --- |
| `张庄C503线-架空-丽水市.svg` | 30 | 11 / 11 | 68 | 收敛 |
| `新区B259线-混合.svg` | 69 | 14 / 14 | 224 | 收敛 |

测试还要求：已识别对象零未解析、电缆/架空类型分离、
死孤岛电压显式为零、稳定 ID 与电气参数往返一致、
分层 SVG 导出，以及拒绝纯 SVG 美术图。GUI API 冒烟测试
覆盖了导出后再导入的流程。

## 安全

解析器拒绝 `DOCTYPE` 和 `ENTITY` 声明，不加载外部资源，
并忽略已识别元数据与几何子集之外的视觉定义。
针对不可信公开上传的文件大小与元素数量上限，
仍是未来的加固项。
