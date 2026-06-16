# 08 - PnC Map 与 Routing

> 来源：`apollo_docs_md/框架设计/软件核心/包管理工具/软件包文档/Apollo_Core/planning/planning-lane-follow-map.md` 和 `routing.md`

---

## PnC Map（参考线生成）

PnC Map 是 Planning 模块中负责生成**参考线（Reference Line）**的核心组件。参考线是 Planning 局部路径规划的路线基准。

### 作用

- 根据 Routing 模块的全局导航路径和地图信息
- 生成平滑的参考线数据
- 作为 Planning 中 Scenario/Task 的路线参考

### 关键概念

| 概念 | 说明 |
|------|------|
| ReferenceLine | 参考线，车辆规划轨迹的基准线 |
| ReferenceLineInfo | 包含一条参考线及相关规划信息 |
| LaneFollowMap | LaneFollow 场景使用的 PnC Map |
| Smoothing | 参考线平滑算法 |

### 比赛关系

- **通常不需要修改 PnC Map**，除非需要自定义地图逻辑
- 但如果赛题涉及特定地图元素识别（如自定义区域），可能需要读取 PnC Map 数据

---

## Routing（路由/全局路径）

Routing 模块负责根据地图和起止点生成全局导航路径。

### 作用

- 接收起点、终点信息
- 基于高精地图生成全局路由
- 输出 RoutingResponse 给 Planning 模块

### RoutingResponse 结构

```
RoutingResponse
├── 路径段列表（LaneSegment）
├── 车道变更信息
├── 停车信息（parking_id）
└── 导航命令
```

### 与 Planning 的关系

- Planning **订阅** RoutingResponse
- 根据 RoutingResponse 中的路径信息进行局部轨迹规划
- ValetParking 场景依赖 RoutingResponse 中的 `parking_id`

### 比赛关系

- **不需要修改 Routing**，只需了解其输出格式
- Planning 可通过 RoutingResponse 判断是否到达目的地、是否需要变道等

---

## Planning 数据流总结

```
┌──────────┐
│  地图    │──→  Routing  ──→  RoutingResponse
└──────────┘                        │
                                    ↓
┌──────────┐    ┌──────────┐   ┌──────────┐   ┌──────────┐
│ Perception│──→ │Prediction│──→│ Planning │──→│ Control  │
└──────────┘    └──────────┘   └──────────┘   └──────────┘
                                    ↑
                              PnC Map (ReferenceLine)
```

---

## 参考

- PnC Map：`apollo_docs_md/框架设计/软件核心/包管理工具/.../planning/planning-lane-follow-map.md`
- Routing：`apollo_docs_md/框架设计/软件核心/包管理工具/.../planning/routing.md`
