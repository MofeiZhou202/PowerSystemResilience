> 本文档为 [cim_cgmes3_crosswalk.md](cim_cgmes3_crosswalk.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

> 文档同步（2026-07-12）
> 范围：已对照当前仓库结构、CMake 预设/选项与已注册测试目标审查。
> 状态：有实现支撑的参考文档。
> 事实来源：当文本与实现不一致时，以 src/、include/、tests/ 及 CMake 文件为准。

# CIM / CGMES 3.0 ↔ HACDCPF 对照表

本文档是有界 CGMES 3.0 适配器（`include/hacdcpf/io/cim_io.hpp`、`src/io/cim_io.cpp`）的规范性字段级映射。按照 `digital_twin_data_io_architecture.md` 第 5 节，标准适配器在没有提交对照表（crosswalk）的情况下不算"完整"；本文即为该对照表，外加明确记录在案的能力上限。

## 范围

- **配置文件（Profiles）：** CGMES 3.0 **EQ**（设备）+ **SSH**（稳态假设）。
- **绑定层级：** Rich（第 3.2 节）。导入会重构 rich AC 组件。
- **命名空间：** `cim` = `http://iec.ch/TC57/CIM100#`，`rdf` = RDF 语法命名空间。
- **单位：** 断言式（SI / 按 CGMES 类型标注的标幺值）；不做启发式推断。

## 类 / 属性映射

| CGMES 类 | CGMES 属性 | HACDCPF 目标 | 备注 |
| --- | --- | --- | --- |
| `BaseVoltage` | `nominalVoltage` | `ACBus.base_kv` | 由 `TopologicalNode` 引用。 |
| `TopologicalNode`（或 `ConnectivityNode`） | `IdentifiedObject.name` | `ACBus.name` | 母线索引取自 `rdf:ID` 后缀 `_busN`，否则按顺序分配。 |
| `TopologicalNode` | `.BaseVoltage` → `BaseVoltage` | `ACBus.base_kv` | 通过 `rdf:resource` 解析。 |
| `ACLineSegment` | `.r` / `.x` / `.bch` | `ACBranch.r_pu` / `.x_pu` / `.b_pu` | 两个 `Terminal` 给出 `from_bus`/`to_bus`。 |
| `SynchronousMachine` | `.minP` / `.maxP` / `.minQ` / `.maxQ` | `Generator.pmin_mw` / `.pmax_mw` / `.qmin_mvar` / `.qmax_mvar` | 母线经由其 `Terminal` 确定。 |
| `RotatingMachine` | `.ratedS` / `.p`（SSH） | `Generator.mbase_mva` / `.pg_mw` | `.p` 仅存在于 SSH。 |
| `EnergyConsumer` | `.p` / `.q`（SSH） | `Load.p_mw` / `.q_mvar` | 母线经由其 `Terminal` 确定。 |
| `Terminal` | `.ConductingEquipment`、`.TopologicalNode` | （连接关系） | 将设备连接到母线。 |

## 范围之外（如实报告，不静默丢弃）

导入器对上表之外的每一个 CGMES 类都会发出一条 `ImportRecord{Skipped, StructuralLoss, Warning}`，例如：

- `PowerTransformer` / `PowerTransformerEnd`（变压器）
- DC 设备、换流器、`EnergyRouter` 等价物
- `TP`（拓扑）与 `SV`（状态变量）配置文件
- 动态 / 保护 / 量测配置文件

## 往返（round-trip）契约

对有界子集（母线、线路、同步机、负荷、连接关系及逐母线基准电压）成立 `import(export(M)) ≈ M`。由 `tests/test_io_cim.cpp` 验证。母线/支路/发电机/负荷索引通过 `rdf:ID` 后缀约定（`_busN`、`_lineN`、`_genN`、`_loadN`）保留；任意手工编写的 id 回退为按顺序分配。

## 安全

RDF/XML 读取器是自包含的，且**已做 XXE 加固**：`<!DOCTYPE>` 与 `<!ENTITY>` 声明会被拒绝而非展开，从而封闭外部实体攻击面（第 12 节）。

## 上限之外的路线图

- 变压器（`PowerTransformer` + ends → `Transformer2W`/`3W`）。
- 用于已解状态交换的 `TP`/`SV` 配置文件。
- 运行限值（`OperationalLimitSet` → 额定值 / 电压限值）。
- 完整的 ENTSO-E 边界集处理与多文件 EQ/SSH/TP/SV 包。
