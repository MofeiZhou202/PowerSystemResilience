> 本文档为 [parameter_system.md](parameter_system.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

# 注册参数系统

更新日期：2026-08-10

可编辑工程参数注册表由
`include/hacdcpf/model/standard_parameter_library.hpp` 中的
`StandardParameterLibrary` 定义，并在
`src/model/standard_parameter_library.cpp` 中实现。

## 契约

受支持的取值路径为：

```text
compiled profile defaults
  -> external parameter-library JSON
  -> session API selection/update
  -> GUI editing
  -> explicit validation/apply
  -> effective_parameters echo
  -> numerical analysis
```

通用导入器保持对数据源的忠实还原。缺失的工程数据不会通过通用注册表被静默填充。IEC-CGE SVG 适配器是一个显式的 `BestEffort`（尽力而为）例外，因为其数据源中没有可计算的电气资产记录：它会按来源（provenance）标记估计值，并默认调用同一套遵循标准的设计手册补全工作流。该行为可通过 `auto_complete_parameters` 控制，并会在导入报告中返回。

## Profile 与规则字段

当前的 profile 为 `distribution_50hz` 和
`lv_distribution_50hz`。每条 `StandardParameterRule` 声明：

- 稳定的 `id`、组件类型和参数名；
- 可编辑的默认值和单位；
- 可选的下界和上界；
- 警告/错误严重级别；
- 来源和工程描述。

会话响应为每条规则附加只读的展示元数据：
`symbol`、`quantity`、`model_role`、`equation`、`typical_min`、`typical_max`、
`typical_range_kind`、`typical_range_source` 和
`equivalent_circuit_family`。典型范围（typical range）是工程或铭牌层面的筛查参考。它们不是 API 校验边界，永远不会写入模型，也不能替代制造商数据、项目设计输入、测试报告或电网公司统计。`min_value` / `max_value` 仍然是后端校验实际使用的硬边界。典型边界为 null 表示注册表不发布一个负责任的通用范围。

参数库本身支持外部序列化。一次更新必须包含当前 profile 中的全部规则；未知 ID 和非法边界会被拒绝。

## API 与 GUI

| 路由 | 行为 |
|---|---|
| `GET /api/session/parameter_library` | 返回 profile、规则、校验元数据、`effective_parameters`、`model_catalog` 以及当前的 `parameter_instances` 快照。 |
| `POST /api/session/parameter_library/select` | 按 ID 替换当前激活的 profile。 |
| `POST /api/session/parameter_library/update` | 应用一份完整的外部 JSON 覆盖。 |
| `POST /api/session/parameter_library/validate` | 在不产生变更的前提下诊断当前系统。 |
| `POST /api/session/parameter_library/apply` | 显式填充符合条件的缺失/非法值，并报告发生变化的字段。 |

后端拥有的 `model_catalog` 目前枚举了 `HybridPowerSystem` 可序列化的 44 个物理或系统模型族。GUI 选择器由该目录构建，而不是从注册表规则推断，因此即使某个模型没有独立的标准补全规则，它仍然可以被选中。GUI 选择一个组件模型和当前实例，渲染其求解器适用范围和等效电路，并对比当前实例值、profile 默认值、典型筛查值和硬校验值。profile 行可编辑默认值、单位、硬边界、严重级别和来源，支持导入/导出 JSON、运行校验以及应用所选 profile。API 响应暴露 `effective_parameters`，包含取值、来源（origin）、单位、出处（source）、可编辑性、边界以及是否应用了某条规则。

`parameter_instances` 是由权威的 `hacdcpf::io::to_json` 系统序列化输出的全部字段的只读快照。嵌套对象和数组被展开为字段路径，且不替换为前端默认值。每一行使用稳定标识 `domain:component_kind:component .index`；vector position 不对外公开。专用变压器和类变压器的 AC 支路保留各自独立的 kind，AC/DC 储能标识按域限定（domain-qualified）。作者录入的实例值仍可通过画布（Canvas）属性编辑器编辑；模型浏览器仅编辑 profile 默认值和校验策略。

标准库为兼容性和 JSON 往返（round trip）仍保留 52 条底层规则。以可靠性（reliability）为前缀的规则组不作为对等的组件模型暴露：GUI 概览当前显示 24 条物理标准规则，而已解析的失效模式及其参数模式（schema）附加在每个匹配的物理实例的 `Reliability` 分组之下。这使用了 `reliability_kind + component .index` 和会话的生效失效模式目录。因此，没有独立标准规则的组件仍会显示其完整的作者录入字段和已解析的可靠性参数。缺失的典型范围或硬范围渲染为未发布/未注册；GUI 不会臆造工程限值。

已实现的图示涵盖 AC 线路 pi 模型、双绕组变压器 T 型等效、DC 电阻支路、VSC 的 AC 阻抗与换流边界、DC/DC 双端口、储能 PCS/能量平衡、AC/DC 母线状态以及系统标幺基值。可靠性部分在所选物理模型旁使用状态转移框图，因为失效和修复转移并非电气等效。

标准表格、标准计算方法与工程假设之间在字段层面的区分，记录在
`distribution_parameter_completion_standards.zh.md` 中。

## 数值责任

注册表的覆盖并不意味着每个求解器选项都是模型参数。求解器容差、迭代上限、并行设置、动态积分控制、谐波选项和最优潮流（OPF）算法设置仍然是请求选项，必须由其分析结果以 `options_effective`、`effective_parameters` 或等效的模块专属块回显。

要认定一个参数可外部控制，需验证全部六个阶段：

1. 默认 profile 值；
2. 外部 JSON 覆盖；
3. API 接受；
4. GUI 可编辑；
5. 生效值回显；
6. 在聚焦测试中的数值敏感性。

注册表 API 的 E2E 测试验证了 profile 枚举、非法参数诊断、显式应用、变更字段报告和生效边界。
