# 2026 星火自动驾驶大赛 — U 型弯场景完整分析报告

> 分析日期: 2026-05-21  
> 数据来源: 场景 JSON、base_map.bin、routing_map.bin、仿真日志

---

## 一、2026 赛事全貌

### 1.1 场景集信息

| 字段 | 值 |
|------|-----|
| 名称 | **2026星火自动驾驶大赛省级选拔赛** |
| 场景集 ID | `69d52dd10e41e17bc62fda1a` |
| 地图 | Xh_2026_contest (`69d51ffd0e41e1217f2fda19`, 560 lanes) |
| 场景数 | **12 个** (含 2 个 U 型弯) |

### 1.2 全部 12 个场景一览

| 场景 | 描述 | 起点 | 障碍物 | 标签 |
|------|------|------|--------|------|
| 07b | **xh_2026_U形弯道场景_1** 🔴 | (424040,4438480) h=180° | 3 (1大货+2锥桶) | Curve, U-turn, Static |
| 07c | **xh_2026_U形弯道场景_2** 🔴 | (424040,4438480) h=180° | 3 (1大货+2锥桶) | Curve, U-turn, Static |
| 079 | xh_2026_S弯场景_1 | (424042,4437923) h=0° | 14 锥桶 | Curve, Left turn, Static |
| 07a | xh_2026_S弯场景_2 | (424042,4437923) h=0° | 24 锥桶 | Curve, Left turn, Static |
| 07e | xh_2026_施工区域通行_1 | (423972,4437618) h≈0° | 36 锥桶 | Straight, Static |
| 07f | xh_2026_施工区域通行_2 | (424017,4437610) h≈0° | 36 锥桶 | Straight, Static |
| 084 | xh_2026_红绿灯场景_1 | (423255,4438082) h=89° | 0 | Intersection, No obstacle |
| 084(2) | xh_2026_红绿灯场景_2 | (423259,4438082) h=89° | 0 | Intersection, No obstacle |
| 07d | xh_2026_变道场景 | (423493,4437627) h=179° | 0 | Straight, No obstacle |
| 080 | xh_2026_环岛让行场景_1 | (423378,4438036) h=0° | 0 | Custom |
| 081 | xh_2026_站点接驳场景_1 | (423985,4438464) h≈0° | 2 大车(5×2×2) | Straight, Lane change |
| 082 | xh_2026_站点接驳场景_2 | (423985,4438464) h≈0° | 3 (2大车+1锥桶) | Straight, Lane change |

---

## 二、U 型弯场景源文件拆解

### 2.1 场景定义文件

```
~/.apollo/resources/scenario_sets/69d52dd10e41e17bc62fda1a/scenarios/
  ├── 69d520a3b58e96002b6da07b.json   ← U型弯场景_1
  └── 69d520a3b58e96002b6da07c.json   ← U型弯场景_2 (数据相同，仅感知参数略异)
```

### 2.2 场景元数据

```json
{
    "descriptionEnTokens": ["xh_2026_U形弯道场景_1"],
    "type": "worldsim",
    "mapId": "69d51ffd0e41e1217f2fda19",
    "tags": ["Curve", "U-turn at an intersection", "Static obstacle"]
}
```

### 2.3 车辆起终点

```json
"autoCarInfo": {
    "start": { "x": 424040.27, "y": 4438479.61, "heading": 3.14159 },
    "end":   { "x": 424041.15, "y": 4438467.46 }
}
```

| 参数 | 值 | 说明 |
|------|-----|------|
| 出生坐标 | (424040.27, 4438479.61) | Lane_1498 @ s=357.5m |
| 初始 heading | 3.1416 rad (**180°**, 向西) | 车头朝西 |
| 终点坐标 | (424041.15, 4438467.46) | Lane_1952 @ s=123.9m |
| 起终点 x 差 | 0.9m | **几乎同 x，调头返回** |
| 起终点 y 差 | -12.1m | **隔壁车道** |

### 2.4 障碍物布局

```json
"entities": { "scenarioObjects": [
    { "name": "1602", "entityObject": { "unknownUnmovableObject": {
        "boundingBox": { "dimensions": { "length": 0.2, "width": 0.2, "height": 2 }}}}},
    { "name": "6607", "entityObject": { "unknownUnmovableObject": {
        "boundingBox": { "dimensions": { "length": 0.2, "width": 0.2, "height": 2 }}}}},
    { "name": "7571", "entityObject": { "unknownUnmovableObject": {
        "boundingBox": { "dimensions": { "length": 10, "width": 4, "height": 2 }}}}}
]}
```

| 名称 | 尺寸 | 坐标 (x,y) | heading | 位置 |
|------|------|-----------|---------|------|
| 🟠 1602 | 锥桶 0.2×0.2×2 | (423904.74, 4438473.65) | 3.074 rad (176°) | U弯入口右侧 |
| 🟠 6607 | 锥桶 0.2×0.2×2 | (423904.87, 4438469.60) | -3.111 rad (-178°) | U弯出口左侧 |
| 🚛 7571 | **大货车 10×4×2** | (423910.17, 4438471.36) | -3.139 rad (-180°) | **堵死直行路线！** |

### 2.5 障碍物布局图

```
    车道方向 →→→→→→→→→→→→→→→ (向西, heading=180°)
    ╔══════════════════════════════════════════════╗
    ║               Lane_885 (493m 直道)            ║
    ║  (424398,4438477) ──────────────────→ (423905,4438476)
    ║                                              ║
    ║                      🚛 7571 (10m×4m)        ║
    ║                      堵在直行路上！           ║
    ║                      (423910,4438471)         ║
    ║                          ║                   ║
    ║              🟠 1602     ║                   ║
    ║              (锥桶)      ║    U弯入口        ║
    ║         (423905,4438474) ║                   ║
    ║                          ▼                   ║
    ║                     Lane_1955 (13m, 179°)    ║
    ║                          │                   ║
    ║                          ▼                   ║
    ║                     Lane_1953 (7m)           ║
    ║              🟠 6607     │                   ║
    ║              (锥桶)      │                   ║
    ║         (423905,4438470) │                   ║
    ║                          ▼                   ║
    ║              终点方向 (向东返回)              ║
    ╚══════════════════════════════════════════════╝
```

### 2.6 场景设计意图

1. 🚛 **大货车 7571 (10m×4m)** 横在直行路线上，堵死 `Lane_885` 直行通道
2. 🟠 两个锥桶标记 U 型弯入口/出口边界
3. 车辆**必须通过 `Lane_1955` (179° U弯) 调头**才能到达隔壁车道的终点
4. 终点与起点同一 x 坐标、y 差 12m（相邻车道）

### 2.7 场景参数配置

| 配置项 | 场景_1 | 场景_2 |
|--------|--------|--------|
| 智能障碍物巡航速度 | 10-15 m/s | 10-15 m/s |
| 最大减速度 | -3 m/s² | -3 m/s² |
| 安全距离 | 7-10 m | 7-10 m |
| 检测距离 | 50 m | 50 m |
| 感知漏检率 | **5%** | **5%** |
| 感知 ID 跳变率 | **10%** | **2%** |
| 评分配置 | grading_metrics_default.conf | 同 |
| 排除评分项 | Checkpoint | Checkpoint |

---

## 三、红色参考线的完整来源链

### 3.1 参考线生成流程

```
场景 JSON (routingRequest)
    │   waypoint 1: (424040.27, 4438479.61)
    │   waypoint 2: (424041.15, 4438467.46)
    ▼
DreamView → /apollo/routing_request
    ▼
old_routing_adapter → routing 引擎
    ▼
A* 搜索 routing_map.bin (558 nodes)
    │
    │  输入: 起止 waypoints
    │  输出: lane-by-lane 最优路径
    │
    │  ┌───────────┬──────────┬──────────┐
    │  │ Lane ID   │ start_s  │ end_s    │
    │  ├───────────┼──────────┼──────────┤
    │  │ Lane_1498 │ 357.548  │ 492.895  │
    │  │ Lane_885  │ 357.527  │ 492.880  │
    │  │ Lane_1955 │ 0        │ 13.059   │
    │  │ Lane_1953 │ 0        │ 7.334    │
    │  │ Lane_1949 │ 0        │ 5.034    │
    │  │ Lane_1952 │ 0        │ 123.932  │
    │  └───────────┴──────────┴──────────┘
    ▼
lane_follow_command (protobuf)
    │  每条 lane segment + change_lane_type
    ▼
Planning 模块
    │  从 base_map.bin 取每条 lane 中心曲线几何
    │  拼接 → 平滑 → 生成 Frenet 参考线
    ▼
DreamView 渲染 → 🟥 红色参考线
```

### 3.2 关键文件

| 文件 | 路径 | 作用 |
|------|------|------|
| 场景 JSON | `~/.apollo/resources/scenario_sets/.../69d520a3b58e96002b6da07b.json` | 起止 waypoints + 障碍物 |
| routing_map.bin | `data/map_data/Xh_2026_contest/routing_map.bin` | A* 搜索拓扑图 |
| base_map.bin | `data/map_data/Xh_2026_contest/base_map.bin` | lane 中心曲线几何 |
| 场景集定义 | `~/.apollo/resources/scenario_sets/.../scenario_set.json` | 赛事元数据 |

### 3.3 最终 lane_follow_command

```
lane_follow_command {
  road { id: "Road_885"
    passage { segment { id: "Lane_1498" start_s: 357.548 end_s: 492.895 }
              can_exit: false  change_lane_type: LEFT }
    passage { segment { id: "Lane_885"  start_s: 357.537 end_s: 492.880 }
              can_exit: true   change_lane_type: FORWARD }
  }
  road { id: "Road_1955"
    passage { segment { id: "Lane_1955" start_s: 0 end_s: 13.059 }
              can_exit: true   change_lane_type: FORWARD }  ← U弯
  }
  road { id: "Road_1953" ... }  → Lane_1953
  road { id: "Road_1949" ... }  → Lane_1949
  road { id: "Road_1952" ... }  → Lane_1952 (终点)
}
```

---

## 四、地图层面：为什么 GetPathTurnType 永远检测不到 U 弯

### 4.1 致命数据

| 指标 | 2025 | 2026 |
|------|------|------|
| 物理 U 弯 (Δh>115°) | 18 | 16 |
| 标注为 U_TURN 的 | **2 条** | **2 条** |
| U_TURN 且在 routing 中 | **0 条** ❌ | **0 条** ❌ |
| 标注 NO_TURN 的物理 U 弯 | 16 条 ✅ | 14 条 ✅ |

**Lane_1955 (13m, 179° U弯) 的 `turn` 字段 = `NO_TURN` (=1)**。`GetPathTurnType()` 遍历 reference line 的 lane segments 时不会匹配到它。

### 4.2 原因链

1. `graph_creator.cc` 对 `turn==U_TURN` 的 lane 做 `IsValidUTurn()` 半径检查 → 14m 做 180° 转弯的 lane 半径太小 → 被排除
2. 未标注 `U_TURN` 的 lane 不经过检查 → 直接加入 routing 图
3. `GetPathTurnType()` 只检查 `turn == U_TURN` → 永远找不到

### 4.3 结论

**曲率锚定 heading 检测是唯一可行方案。** 我们的修复（方法B）完全对症。

---

## 五、2025 vs 2026 场景对比

### 5.1 场景集对比

| | 2025 | 2026 |
|---|------|------|
| 名称 | 2025星火自动驾驶大赛省级选拔赛 | 2026星火自动驾驶大赛省级选拔赛 |
| 场景数 | 16 | 12 |
| U 弯场景 | **无** ❌ | **2 个** ✅ |
| 地图 | xh_2025_contest (435 lanes) | Xh_2026_contest (560 lanes) |

### 5.2 2026 新增场景

| 场景 | 2025 有? | 2026 有? |
|------|---------|---------|
| U 型弯 | ❌ | ✅ **新增** |
| S 弯 | ❌ | ✅ **新增** (2个) |
| 施工区域通行 | ✅ | ✅ (参数调整) |
| 红绿灯 | ✅ | ✅ |
| 变道 | ❌ | ✅ **新增** |
| 环岛让行 | ❌ | ✅ **新增** |
| 站点接驳 | ❌ | ✅ **新增** (2个) |
| 停车标志 | ✅ | ❌ 删除 |
| 动态障碍物 | ✅ | ❌ 删除 |
| 自主泊车 | ✅ | ❌ 删除 |
| 狭窄路通行 | ✅ | ❌ 删除 |

---

## 六、代码修复回顾

### 6.1 已修改的文件

| 文件 | 修改内容 |
|------|---------|
| `contest_scenario_util.cc` | 重写 `IsContestUTurn()` — 方法B改为曲率锚定 |
| `contest.proto` | `u_turn_look_forward_distance`: 60→35, `heading_change_threshold`: 2.7→2.0 |
| `stage_contest_lane_follow.cc` | 完成判定与检测阈值解耦 (2.7 rad) |
| `contest_scenario_features.h` | 同步 `kDefaultUTurn*` 常量 |

### 6.2 检测验证

```python
# 场景实测参数 vs 检测阈值
u_turn_look_forward_distance  = 35m    # Lane_885 末端 35m 内 → Lane_1955 高曲率区
u_turn_kappa_threshold        = 0.12   # Lane_1955: 13m转179° → kappa≈0.30 ✅
u_turn_heading_change_threshold = 2.0 rad  # 实测 Δheading=3.14 rad ✅
```

### 6.3 🔴 S 弯误判修复（单向性检查）

**问题**: S 弯场景 `Lane_1744(89°) → Lane_1745(78°)` 两个反向转弯叠加，35m 窗口内 Δheading 可达 150°，超过 114.6° 阈值导致误判。

**根因**: S 弯 heading 变化是双向的（左转完右转，net≈0），U 弯是单向的（net≈sum_abs）。旧算法只看 net heading change，无法区分。

**修复**: 新增 `monotonic_ratio = net_heading_change / sum_abs_heading_change`:
- U 弯: net≈180°, sum_abs≈180° → ratio≈1.0
- S 弯: net≈10°, sum_abs≈160° → ratio≈0.06

同时满足 `net > 2.0 rad` **且** `ratio > 0.6` 才判定为 U 弯。

**修改文件**: `contest_scenario_util.cc` — `IsContestUTurn()` 方法B新增 20 点采样计算累计绝对 heading 变化和单向性比率。

---

## 七、完整文件索引

### 场景定义
```
~/.apollo/resources/scenario_sets/
  ├── 69d52dd10e41e17bc62fda1a/        ← 2026 省级选拔赛
  │   ├── scenario_set.json
  │   └── scenarios/
  │       ├── 69d520a3b58e96002b6da07b.json  ← U弯场景_1
  │       └── 69d520a3b58e96002b6da07c.json  ← U弯场景_2
  ├── 67f33e951abf4bbcf2d417f0/        ← 2025 省级选拔赛
  └── 6880c29dacb66478b25609c4/        ← 2025 国赛
```

### 地图数据
```
data/map_data/Xh_2026_contest/
  ├── base_map.bin         ← lane 几何 (560 lanes)
  ├── routing_map.bin      ← routing 拓扑 (558 nodes)
  ├── sim_map.bin          ← 仿真地图
  ├── current_start_point.txt  ← (424016,4437618) 默认起点
  └── metaInfo.json
```

### 代码
```
modules/planning/scenarios/contest/
  ├── contest_scenario_util.cc    ← IsContestUTurn() 判定
  ├── contest_scenarios.cc        ← ContestUTurnScenario
  ├── proto/contest.proto         ← 配置参数
  └── stage_contest_lane_follow.cc ← 场景退出判定

modules/planning/tasks/contest_lane_borrow_path/
  ├── contest_lane_borrow_path.cc  ← U弯场景 skip 借道
  └── u_turn_cone_nudge_helper.cc  ← 锥桶 Nudge

modules/routing/topo_creator/
  └── graph_creator.cc             ← IsValidUTurn() 排除逻辑
```

### 日志
```
data/log/
  ├── routing.log.INFO.*           ← A* 搜索结果
  ├── external_command.log.INFO.*  ← lane_follow_command
  └── planning.log.INFO.*          ← 场景切换确认
```

---

## 八、🔴 最优方案：U 型弯路径规划策略（已实现）

### 8.1 核心问题诊断

```
┌──────────────────────────────────────────────────────────────────┐
│                    为什么"原地转圈"？                              │
│                                                                  │
│  Lane_1955 参考线: 13m 内转 179° → 曲率 ≈ 0.24 rad/m             │
│  车辆 NEOLIX: min_turn_radius = 2.5m → 最大曲率 ≈ 0.40 rad/m     │
│                                                                  │
│  车辆物理上能转 (0.40 > 0.24)，但问题在路径规划层：               │
│                                                                  │
│  ① ContestLaneBorrowPath 在 U 弯场景直接 return OK               │
│     → 跳过路径生成，只靠 ContestLaneFollowPath                    │
│                                                                  │
│  ② ContestLaneFollowPath 强制 path_reference_l_weight=10000       │
│     → 路径被死死拉向 l=0 的参考线                                 │
│     → 参考线在 13m 内急转 180°，路径优化器无法同时满足：           │
│        - 紧贴参考线 (高 weight)                                   │
│        - 曲率/ jerk 约束                                         │
│     → 路径振荡、发散、甚至 infeasible                             │
│                                                                  │
│  ③ 最终 controller 拿到的路径要么不可行，要么跟踪误差放大         │
│     → 车跑出参考线 → planning 纠偏 → 更激进 → 振荡/转圈          │
└──────────────────────────────────────────────────────────────────┘
```

### 8.2 已实现的方案

**修改 3 个文件，新增 2 个方法**：

| 文件 | 修改 |
|------|------|
| `contest_lane_borrow_path.h` | 新增 `DecideUTurnPathBoundary()` 和 `AddUTurnSpeedLimit()` 声明 |
| `contest_lane_borrow_path.cc` Process() | U 弯不再 return OK，改为生成宽走廊路径 |
| `contest_lane_borrow_path.cc` OptimizePath() | U 弯 label `uturn_wide` 使用 corridor center 作为参考目标 |
| `contest_lane_borrow_path.cc` 新增 | `DecideUTurnPathBoundary()` — 同时向左右借道，生成全路宽 corridor |
| `contest_lane_borrow_path.cc` 新增 | `AddUTurnSpeedLimit()` — U 弯区域限速 5m/s |

**核心改动 Process()**:

```cpp
// OLD: 直接跳过
if (IsContestUTurnScenario()) {
    clear all borrow state...;
    return Status::OK();  // ← 不生成路径！
}

// NEW: 生成宽走廊路径
if (IsContestUTurnScenario()) {
    u_turn_construct_ = true;
    // path_reference_l_weight = 0  → 不强拉向 l=0
    config_.set_path_reference_l_weight(0.0);
    config_.set_l_weight(0.0);
    
    // 向左+右同时借道 → 全路宽 corridor
    DecideUTurnPathBoundary(&boundaries);
    OptimizePath(boundaries, &paths);
    
    // 直接用优化路径，跳过 AssessPath
    *path_data = paths.front();
    
    AddUTurnSpeedLimit();       // 限速 5m/s
    IgnoreAllObstacles();       // 忽略大货车和锥桶
    return Status::OK();
}
```

**DecideUTurnPathBoundary()** 策略：
1. `InitPathBoundary()` — 初始化基础边界
2. `GetBoundaryFromNeighborLane(LEFT)` — 向左扩展至 Lane_1498
3. `GetBoundaryFromNeighborLane(RIGHT)` — 向右扩展至 Lane_1956
4. `ApplyUTurnConeNudge()` — 锥桶 nudging
5. `GetBoundaryFromStaticObstacles()` — 障碍物避让
6. Label 设为 `"regular/uturn_wide"` → OptimizePath 识别并使用 corridor 中心

**OptimizePath()** 识别 `uturn_wide` label：
- `ref_l[i] = (boundary_lower + boundary_upper) / 2` — 以 corridor 中点为目标
- `weight_ref_l[i] = 0` — 零权重，优化器自由选择路径

### 8.3 期望效果

```
宽走廊 (Lane_1498 + Lane_885 + Lane_1956 全路宽)
    ╔════════════════════════════════╗
    ║                                ║
    ║    参考线 (l=0, 13m急弯)       ║
    ║    ╭──────────╮                ║
    ║   ╱            ╲               ║
    ║  ╱  优化器自由   ╲  新路径      ║
    ║ ╱   选择大半径    ╲ (更平滑)    ║
    ║╱   弧线穿过       ╲             ║
    ║                    ╲            ║
    ║                     ╲           ║
    ╚════════════════════════════════╝
```

### 8.4 修改清单

```
modules/planning/tasks/contest_lane_borrow_path/
├── contest_lane_borrow_path.h   ← +2 方法声明
└── contest_lane_borrow_path.cc  ← Process()重写 + OptimizePath()扩展
                                   + DecideUTurnPathBoundary() (新增)
                                   + AddUTurnSpeedLimit() (新增)
