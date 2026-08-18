> 本文档为 [ev_traffic_scenario_format.md](ev_traffic_scenario_format.md) 的中文译文，供 GUI 帮助中心使用；两者冲突时以原文与代码为准。

> 文档同步（2026-07-12）
> 状态：与实现一致的 schema 参考。
> 事实来源：`web/schemas/ev_traffic_scenario.schema.json`、导入代码
> 以及已注册的测试。

# EV 电力-交通场景格式

GUI 导入一份版本化的 JSON 文档，该文档需符合
`web/schemas/ev_traffic_scenario.schema.json`。可直接编辑的示例见
`web/examples/ev_traffic_scenario_template.json`。

## 顶层结构

```json
{
  "$schema": "/xjtu/schemas/ev_traffic_scenario.schema.json",
  "schema_version": "1.0",
  "name": "Study name",
  "traffic": { "nodes": [], "links": [] },
  "routes": [],
  "demands": [],
  "station_prices": []
}
```

所有 ID 均为用户自定义的正整数。ID 无需连续，但在其所属集合内必须唯一。

## 各集合说明

| 集合 | 必填字段 | 含义与单位 |
|---|---|---|
| `traffic.nodes[]` | `index` | 交通顶点。可选的 `name`、`x`、`y` 控制 GUI 布局。 |
| `traffic.links[]` | `index`、`from_node`、`to_node`、`length_km`、`free_flow_time_hr`、`capacity_veh_per_hr` | 有向路段。`jam_vehicles` 是 CTM（元胞传输模型）使用的路段最大容纳车辆数。可选的 `free_flow_time_profile_hr`、`capacity_profile_veh_per_hr` 与 `availability_profile` 数组按仿真步索引；CTM 子步继承其父仿真步的取值。 |
| `routes[]` | `index`、`origin_node`、`destination_node`、`link_indices` | 有序有向路径。相邻路段必须构成一条从起点到终点的连续链。 |
| `routes[].charging_stops[]` | `station_id`、`requested_energy_kwh_per_vehicle` | 路径沿途的充电动作。`station_id` 必须与当前电力系统模型中的某个交流充电站匹配。 |
| `demands[]` | `index`、`origin_node`、`destination_node`、`departure_step`、`vehicles` | EV OD（起讫）需求。`candidate_route_indices` 选择可用路径；空列表表示使用所有匹配该 OD 的路径。 |
| `station_prices[]` | `station_id`、`price_per_kwh` | 逐仿真步电价。单元素数组表示恒定电价序列。 |

## 引用规则

1. 每条路段的端点必须引用已存在的交通节点。
2. 每条路径的路段必须引用已存在的路段，且按序排列的路段必须
   构成一条从 `origin_node` 到 `destination_node` 的连续链。
3. 每个需求的候选路径必须存在，且必须具有相同的 OD 对。
4. 每个路径的 `station_id` 必须存在于当前电力系统模型的交流
   充电站集合中。
5. `departure_step` 以及所有价格/序列索引均从 0 开始。
6. 功率单位：充电用 kW，电力系统模型用 MW；能量单位为每辆车
   kWh；交通通行能力单位为辆/小时。

GUI 的**交通网络设计**对话框在应用场景之前强制执行规则 1–3。后端执行最终的电力系统耦合校验。
