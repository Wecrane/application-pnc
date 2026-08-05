# pnc_map 模块分析报告

> 3 文件 / ~1,381 行 | PnC 专用地图接口

## 功能概述

`pnc_map` 是 Planning 模块的 **地图适配层**，封装对高精地图（HDMap）的查询，为上层的 scenarios 和 tasks 提供统一的地图数据接口。

---

## 核心类：LaneFollowMap

| 文件 | 行数 | 职责 |
|------|------|------|
| `lane_follow_map.h` | 236 | 接口定义 |
| `lane_follow_map.cc` | 904 | 核心实现 |
| `lane_follow_map_test.cc` | 241 | 单元测试 |

### 继承关系
```
PncMapBase
└── LaneFollowMap
```

### 核心接口

| 方法 | 说明 |
|------|------|
| `GetRouteSegments(double backward_length, double forward_length)` | 获取路由段 |
| `ExtendSegments(...)` | 扩展路由段 |
| `FutureRouteWaypoints()` | 获取前方路点 |
| `GetLaneById(Id)` | 按 ID 查询车道 |
| `GetNearestPointFromRouting()` | 路由上最近点 |
| `GetEndLaneWayPoint()` | 终点车道点 |
| `GetDistanceToDestination()` | 到目的距离 |
| `GetAdcWaypoint()` | 当前车辆路点 |

### 核心逻辑

```
LaneFollowMap
├── 封装 HDMap 查询 → 提供语义化接口
├── 路由段管理 (前向/后向扩展)
├── 车道通过性检查 (drive_on_lane_ / turn_on_lane_)
├── 邻近通道 (passage) 管理
└── 路由索引维护 (IndexLaneSegments, IndexPassages)
```

## 依赖关系
- **上游**：HDMap（高精地图模块）
- **下游**：Scenarios（通过 Frame），Tasks（通过 ReferenceLineInfo）
- **同级**：ReferenceLineProvider（使用 LaneFollowMap 获取路由信息）
