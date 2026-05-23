# 2026 星火自动驾驶大赛 — 环岛场景完整分析报告

> 分析日期: 2026-05-23  
> 数据来源: 场景 JSON、base_map.bin、交通流数据、源码分析

---

## 一、场景概览

### 1.1 场景元数据

| 字段 | 值 |
|------|-----|
| 场景名称 | **xh_2026_环岛让行场景_1** |
| 场景 ID | `69d520a3b58e96002b6da080` |
| 作者 | xiaoxinyu |
| 类型 | worldsim |
| 地图 | Xh_2026_contest (`69d51ffd0e41e1217f2fda19`, 560 lanes) |
| 标签 | `Custom` |
| 场景超时 | 100s |

### 1.2 场景设计意图

此场景为**环岛让行场景**，核心考点：
1. **无静态障碍物** — 场景 entities 为空，挑战在于动态交通流
2. **动态交通流密集** — 2 条交通流在环岛内逆时针绕行，ADC 必须**判断让行时机**
3. **让行规则** — ADC 进入环岛前需避让环岛内车辆（类似真实环岛交通规则）

### 1.3 场景在全部赛题中的位置

从 MAP_ANALYSIS_REPORT 可知，赛事 12 个场景中本场景为**赛题七（环岛）**。比赛路线推测为：
> 起点 → S弯 → 变道 → 施工区域 → 站点接驳 → 交通灯/U-Turn → **环岛**

---

## 二、车辆起终点与路由

### 2.1 起终点

```json
"autoCarInfo": {
    "start": { "x": 423377.61, "y": 4438035.97, "heading": 0 },
    "end":   { "x": 423544.07, "y": 4438146.10 }
}
```

| 参数 | 值 | 说明 |
|------|-----|------|
| 出生坐标 | (423377.61, 4438035.97) | 环岛西侧入口外 |
| 初始 heading | 0 rad (**0°**, 正东) | 车头朝东，朝向环岛方向 |
| 终点坐标 | (423544.07, 4438146.10) | 环岛东北侧出口外 |
| dx (起→终) | +166.5m | 向东行进 |
| dy (起→终) | +110.1m | 向北行进 |

### 2.2 路由途经点（3 waypoints）

| 序号 | 坐标 | 意义 |
|------|------|------|
| Waypoint 1 | (423377.61, 4438035.97) | 起点 — 环岛西侧入口前 |
| Waypoint 2 | (423521.08, 4438016.06) | 途经点 — 环岛南侧底部 |
| Waypoint 3 | (423544.07, 4438146.10) | 终点 — 环岛东北侧出口后 |

**路由路径**: 起点(西) → 向东 → 环岛南侧 → 逆时针绕行 → 环岛北侧出口 → 终点(东北)

---

## 三、交通流分析（核心难点）

### 3.1 交通流总览

| 交通流 | 生成点 | 初始 heading | 间距 | 最大车辆 | 目的 |
|--------|--------|-------------|------|---------|------|
| **BatchRoute1** | (423532, 4438129) | -1.81 rad (-104°) | 10m | 100 辆 | **密集环岛车流** |
| **BatchRoute2** | (423510, 4438052) | -1.75 rad (-100°) | 30m | 10 辆 | 稀疏补充车流 |

- 动态车辆 ID 范围: `30000 ~ 49999`
- 巡航速度: 10 ~ 11.18 m/s (36~40 km/h)
- lateralThreshold: 2.5m

### 3.2 环岛道路结构（从交通流路径反推）

```
                    ← ← ← ← ← ← ← ← ← ← ←
               ┌──────────────────────────────┐
               │         环岛北侧              │
               │   (423524,4438087) → (423539,4438071) → (423559,4438063)
               │                                   │              │
               │                                   │              │
  环岛西侧     │         环岛中心                  │   环岛东侧   │
  (下行)       │       (423534,4438047)            │   (上行)     │
               │         ~63m × 77m               │              │
               │                                   │              │
               │   (423504,4438042) → (423513,4438016) → (423551,4438013)
               │         环岛南侧 (入口)           │
               └──────────────────────────────┘
                    → → → → → → → → → → →
                    
   ADC 路由: 起点(西) → 进入南侧 → 逆时针绕行 → 北侧出口 → 终点(东北)
```

| 属性 | 值 |
|------|-----|
| 方向 | **逆时针** |
| X 范围 | 423504 ~ 423567 (跨度 **63m**) |
| Y 范围 | 4438011 ~ 4438087 (跨度 **77m**) |
| 近似中心 | (423534, 4438047) |
| 形状 | 矩形/椭圆形环路 |

### 3.3 BatchRoute1 路径详情（密集车流）

| 路段 | 距离 | 方向角 | 说明 |
|------|------|--------|------|
| [0]→[2] 西侧下行 | 43m | -112° (西南) | 环岛西侧南行 |
| [2]→[4] 南侧转弯 | 39m | -49° (东南) | 环岛南侧东转 |
| [4]→[6] 东侧上行 | 33m | +25° (东北) | 环岛东侧北行 |
| [6]→[8] 北侧转弯 | 39m | +97° (东转西北) | 环岛北侧西转 |
| [8]→[10] 西侧回环 | 44m | +137° (西北) | 回到起点方向 |

### 3.4 ADC 与交通流的交互关系

```
             时间线
    ┌──────────────────────────────────────┐
    │                                      │
    │  ADC 到达环岛南侧入口                 │
    │  │                                   │
    │  ├─ 观察环岛内是否有来车?             │
    │  │   ├─ BatchRoute1 车流 (密集, 10m间距) │
    │  │   └─ BatchRoute2 车流 (稀疏, 30m间距) │
    │  │                                   │
    │  ├─ 有足够间隙 → 进入环岛             │
    │  └─ 无足够间隙 → 停车让行             │
    │                                      │
    └──────────────────────────────────────┘
```

**核心挑战**: BatchRoute1 车辆间距仅 10m，速度 10m/s，留给 ADC 的切入间隙可能只有 ~1 秒。ADC 必须精准判断让行时机。

---

## 四、场景检测算法 — `IsContestRoundaboutEntry()`

### 4.1 检测流程

```
         ┌──────────────┐
         │  当前帧 RLI   │
         └──────┬───────┘
                │
         ┌──────▼───────┐    命中
         │ ① 排除 U 弯  │─────────→ return false
         │ IsContestUTurn│
         └──────┬───────┘
                │ 未命中
         ┌──────▼───────┐
         │ ② PNC 路口   │
         │ 前方 80m 内   │
         │ 有路口重叠?   │
         │ (-25m 缓冲区) │
         └──────┬───────┘
                │
         ┌──────▼───────┐
         │ ③ 曲率+转角  │
         │ 前方 55m 扫描 │
         │ Δs=2m 采样    │
         │ max_kappa     │
         │ Δheading      │
         └──────┬───────┘
                │
     ┌──────────┼──────────┐
     ▼          ▼          ▼
  kappa      heading    非U_TURN
  >0.025    ∈(0.65,1.8)
     │          │          │
     └──────────┼──────────┘
                │ ALL TRUE
         ┌──────▼───────┐
         │  环岛入口!    │
         │  return true  │
         └──────────────┘
```

### 4.2 三步检测详解

**步骤① — 排除 U 型弯（优先级最高）**

```cpp
if (IsContestUTurn(reference_line_info, config)) {
    return false;  // U 弯优先级高于环岛
}
```

**步骤② — PNC 路口近场检测**

```cpp
bool near_pnc_junction = false;
for (const auto& overlap : reference_line_info.reference_line()
        .map_path().pnc_junction_overlaps()) {
    if (overlap.end_s < adc_end_s - config.roundabout_inside_junction_buffer())
        continue;  // 已过路口 + 25m 缓冲
    if (overlap.start_s > adc_end_s + config.roundabout_entry_look_forward_distance())
        continue;  // 路口太远 (>80m)
    near_pnc_junction = true;
    break;
}
```

> `roundabout_inside_junction_buffer = 25m` 的作用：ADC 在路口内部行驶时不判定为环岛入口（避免重复触发）。

**步骤③ — 曲率与转向角验证**

```cpp
for (double s = start_s; s <= end_s; s += 2.0) {
    if (GetPathTurnType(s) == hdmap::Lane::U_TURN)
        return false;  // 二次确认非 U_TURN
    max_abs_kappa = std::max(max_abs_kappa,
        std::fabs(reference_line.GetReferencePoint(s).kappa()));
}
const double heading_change = std::fabs(NormalizeAngle(
    heading_at_end_s - heading_at_start_s));
```

### 4.3 命中阈值

| 参数 | 阈值 | 含义 |
|------|------|------|
| `roundabout_entry_look_forward_distance` | 80.0m | 检测前方路口的最大距离 |
| `roundabout_inside_junction_buffer` | 25.0m | 路口内部缓冲区 |
| `roundabout_curve_look_forward_distance` | 55.0m | 曲率检测前视距离 |
| `roundabout_min_abs_kappa` | **0.025** | 最小曲率 → 对应转弯半径 ~40m |
| `roundabout_min_heading_change` | **0.65 rad (37°)** | 最小转向角度 |
| `roundabout_max_heading_change` | **1.8 rad (103°)** | 最大转向角度 |

**命中条件（ALL required）**:
1. ✅ 不是 U 型弯
2. ✅ 前方 80m 内有 PNC 路口
3. ✅ `max_abs_kappa > 0.025`
4. ✅ `0.65 < heading_change < 1.8` rad

### 4.4 环岛 vs U 型弯检测区分

| | U 型弯 | 环岛 |
|---|--------|------|
| heading_change 范围 | **>2.0 rad** (114°) | **0.65~1.8 rad** (37°~103°) |
| kappa 阈值 | >0.12 (急转弯) | >0.025 (缓和弯) |
| 前视距离 | 35m | 55m |
| 路口检测 | 不需要 PNC junction | 需要 PNC junction |

> 环岛的 heading_change 上限 1.8 rad 与 U 型弯下限 2.0 rad 之间存在 0.2 rad 死区，防止两类场景误判。

### 4.5 探测日志

**命中时**:
```
[ROUNDABOUT][Scenario] detected entry, adc_s=xxx, pnc_count=N,
    near_pnc=1, max_abs_kappa=0.xxx, heading_change=x.xx
```

**失准探测 (near miss)** — 当部分条件满足但未命中时输出详细信息:
```
[ROUNDABOUT][Scenario] probe miss, adc_s=xxx, pnc_count=N,
    near_pnc=0/1, max_abs_kappa=0.xxx, heading_change=x.xx, ref_len=xxx
```

---

## 五、场景生命周期

### 5.1 状态机

```
LANE_FOLLOW (默认场景)
    │
    │ IsContestRoundaboutEntry() = true
    │ + IsReferenceLineReady()
    ▼
CONTEST_ROUNDABOUT
    │
    │ ContestLaneFollowStage::Process()
    │ (复用 LaneFollow 的 task 流水线)
    │
    │ 每帧检查 StillInScenario()
    │
    │ IsContestRoundaboutEntry() = false
    ▼
LANE_FOLLOW (自动退出)
```

### 5.2 入场 — `IsTransferable()`

```cpp
// contest_scenarios.cc:115-125
bool ContestRoundaboutScenario::IsTransferable(
        const Scenario* other_scenario, const Frame& frame) {
    if (other_scenario == nullptr || !IsReferenceLineReady(frame))
        return false;
    const bool found = IsContestRoundaboutEntry(frame, GetContext()->scenario_config);
    if (found) {
        AINFO << "[ROUNDABOUT][Scenario] transfer to CONTEST_ROUNDABOUT from "
              << other_scenario->Name();
    }
    return found;
}
```

### 5.3 出场 — `StillInScenario()`

```cpp
// stage_contest_lane_follow.cc:219-220
case ContestScenarioKind::ROUNDABOUT:
    return contest::IsContestRoundaboutEntry(frame, context->scenario_config);
```

**退出条件**: 当检测函数返回 false（驶离环岛区域，不再满足曲率+路口条件）时自动退出。

> ⚠️ 与 U_TURN 场景不同，ROUNDABOUT 没有 `completed` 状态追踪和 heading 反转确认。退出只依赖检测条件自然消失。

### 5.4 阻止车道变更

```cpp
// contest_scenarios.cc:54-56
if (IsContestRoundaboutEntry(frame, GetContext()->scenario_config)) {
    AINFO << "[ROUNDABOUT][Scenario] block CONTEST_LANE_CHANGE transfer";
    return false;  // 环岛入口处禁止触发车道变更场景
}
```

---

## 六、SpeedDecider 环岛集成

### 6.1 场景识别

```cpp
bool IsContestRoundaboutScenario(const DependencyInjector& injector) {
    return injector.planning_context()
        .planning_status().scenario().scenario_type() == "CONTEST_ROUNDABOUT";
}
```

### 6.2 紧窄窗口模式（与 U 型弯共享）

```cpp
bool IsTightContestWindowArea(...) {
    return IsContestUTurnScenario(injector)
        || IsContestRoundaboutScenario(injector);
}
```

环岛与 U 型弯共享此模式，触发特殊的停止距离覆盖逻辑。

### 6.3 虚拟停止墙识别

```cpp
bool IsRoundaboutStopWallLike(const Obstacle& obstacle) {
    if (!obstacle.IsVirtual()) return false;
    const std::string& id = obstacle.Id();
    return HasPrefix(id, "PNC_JUNCTION_")   // PNC 路口虚拟障碍物
        || HasPrefix(id, "YS_")             // 让行标志 (Yield Sign)
        || HasPrefix(id, "SS_")             // 停车标志 (Stop Sign)
        || HasPrefix(id, "KC_JC_")          // Keep Clear 路口
        || HasPrefix(id, "PATH_END_");      // 路径终点
}
```

### 6.4 非目标车道车辆忽略（核心让行逻辑）

```cpp
bool IsRoundaboutNonTargetLaneVehicle(
        const Obstacle& obstacle,
        const ReferenceLine* reference_line)
```

**三步判断**:
1. 通过 `HDMapUtil::BaseMap().GetNearestLaneWithHeading()` 查找障碍物所在车道
2. 通过 `reference_line->GetLaneFromS()` 获取 ADC 当前所在车道列表
3. 如果障碍物车道 ≠ ADC 车道 → **忽略该障碍物**（不让行）

```cpp
// speed_decider.cc:308-311
if (IsContestRoundaboutScenario(injector_)
    && IsRoundaboutNonTargetLaneVehicle(*mutable_obstacle, reference_line_)) {
    AppendIgnoreDecision(mutable_obstacle);  // 不给 stop/yield 决策
    continue;
}
```

| 阈值 | 值 | 含义 |
|------|-----|------|
| `kTargetLaneLateralRange` | 2.2m | 障碍物距 reference line 横向距离阈值 |

> **设计意图**: 环岛是多车道环路，ADC 只需让行**同车道**的车辆。非目标车道的车辆（如环岛对侧车道的车）不应影响 ADC 的通行决策。

### 6.5 停止距离覆盖

```cpp
if (IsTightContestWindowArea(...) && stop_distance < 0.0
    && (dynamic_vehicle || roundabout_stop_wall)) {
    const double target_distance = roundabout_stop_wall
            ? kRoundaboutVirtualStopWallDistance    // 0.6m
            : kRoundaboutDynamicStopDistance;       // 1.2m
    stop_distance = std::max(stop_distance, -target_distance);
}
```

| 常量 | 值 | 适用对象 |
|------|-----|---------|
| `kRoundaboutDynamicStopDistance` | 1.2m | 动态车辆 |
| `kRoundaboutDynamicYieldDistance` | 1.2m | 动态车辆 (让行) |
| `kRoundaboutVirtualStopWallDistance` | 0.6m | 虚拟停止墙 |

---

## 七、KeepClear 交通规则集成

### 7.1 独立环岛检测 — `IsRoundaboutEntryLike()`

`keep_clear.cc` 中有独立于场景系统的环岛入口检测函数：

```cpp
bool IsRoundaboutEntryLike(const ReferenceLineInfo* rli) {
    // 内联参数（不依赖 ContestConfig，更保守）
    constexpr double kEntryLookForward = 35.0;      // vs Contest 80m
    constexpr double kInsideJunctionBuffer = 8.0;    // vs Contest 25m
    constexpr double kCurveLookForward = 55.0;       // 与 Contest 一致
    constexpr double kMinMaxKappa = 0.025;           // 与 Contest 一致
    constexpr double kMinHeadingChange = 0.65;       // 与 Contest 一致
    constexpr double kUTurnHeadingChange = 1.8;      // 与 Contest 一致
    // ... 同构检测逻辑
}
```

| 参数 | KeepClear 值 | Contest 值 | 差异 |
|------|-------------|-----------|------|
| entry_look_forward | 35m | 80m | KeepClear 更保守 |
| inside_junction_buffer | 8m | 25m | KeepClear 触发更早 |
| curve_look_forward | 55m | 55m | 一致 |
| min_abs_kappa | 0.025 | 0.025 | 一致 |
| min_heading_change | 0.65 | 0.65 | 一致 |
| max_heading_change | 1.8 | 1.8 | 一致 |

### 7.2 Keep Clear 起点后移

```cpp
if (IsRoundaboutEntryLike(reference_line_info)) {
    constexpr double kRoundaboutKeepClearStartShift = 8.0;
    const double shifted_start_s =
        std::min(pnc_junction_overlap->end_s - 2.0,
                 pnc_junction_start_s + kRoundaboutKeepClearStartShift);
    AINFO << "[ROUNDABOUT][KeepClear] shift junction keep-clear start from "
          << pnc_junction_start_s << " to " << shifted_start_s
          << ", junction_end=" << pnc_junction_overlap->end_s;
    pnc_junction_start_s = shifted_start_s;
}
```

**目的**: 路口 Keep Clear 虚拟障碍物起点后移 8m，让 ADC 能更靠近环岛入口，有更好的视野观察交通流后再决策。

---

## 八、配置文件汇总

### 8.1 Protobuf 定义

```protobuf
// contest.proto:26-31
message ScenarioContestConfig {
  optional double roundabout_entry_look_forward_distance = 17 [default = 80.0];
  optional double roundabout_inside_junction_buffer = 18 [default = 25.0];
  optional double roundabout_curve_look_forward_distance = 19 [default = 55.0];
  optional double roundabout_min_abs_kappa = 20 [default = 0.025];
  optional double roundabout_min_heading_change = 21 [default = 0.65];
  optional double roundabout_max_heading_change = 22 [default = 1.8];
}
```

### 8.2 全局场景注册

```
# public_road_planner_config.pb.txt
scenario {
  name: "CONTEST_ROUNDABOUT"
  type: "ContestRoundaboutScenario"
}
```

### 8.3 流水线配置

```
# pipeline.pb.txt
stage {
  name: "ContestLaneFollowStage"
  type: "ContestLaneFollowStage"
}
```

环岛场景仅使用 `ContestLaneFollowStage`，复用 LaneFollow 的全部 task 流水线（包括 speed_decider、keep_clear 等）。

---

## 九、完整文件索引

### 场景定义与检测

| 文件 | 关键行 | 用途 |
|------|--------|------|
| `modules/planning/scenarios/contest/contest_scenarios.h` | L83-L85 | `ContestRoundaboutScenario` 类定义 |
| `modules/planning/scenarios/contest/contest_scenarios.cc` | L115-L125 | `IsTransferable()` 入场判定 |
| `modules/planning/scenarios/contest/contest_scenario_util.h` | L49-L52 | `IsContestRoundaboutEntry()` 声明 |
| `modules/planning/scenarios/contest/contest_scenario_util.cc` | L242-L320 | 检测算法核心实现 |
| `modules/planning/scenarios/contest/context.h` | L28-L29 | `ROUNDABOUT` 枚举 |
| `modules/planning/planning_base/common/contest_scenario_features.h` | L32 | `kFeatureSampleStep = 2.0` |

### 配置

| 文件 | 用途 |
|------|------|
| `modules/planning/scenarios/contest/proto/contest.proto` | Protobuf 参数定义 |
| `modules/planning/scenarios/contest/conf/scenario_conf.pb.txt` | 运行时配置 |
| `modules/planning/planning_component/conf/public_road_planner_config.pb.txt` | 全局场景注册 |

### 场景执行

| 文件 | 关键行 | 用途 |
|------|--------|------|
| `modules/planning/scenarios/contest/stage_contest_lane_follow.cc` | L35-L36 | 环岛场景阶段执行 |
| `modules/planning/scenarios/contest/stage_contest_lane_follow.cc` | L143-L220 | `StillInScenario()` 出场判定 |
| `modules/planning/scenarios/contest/stage_contest_lane_follow.cc` | L150-L152 | LANE_CHANGE 退出 |

### 速度决策

| 文件 | 关键行 | 用途 |
|------|--------|------|
| `modules/planning/tasks/speed_decider/speed_decider.cc` | L49 | `kContestRoundaboutScenarioName` |
| `modules/planning/tasks/speed_decider/speed_decider.cc` | L52-L54 | 停止/让行距离常量 |
| `modules/planning/tasks/speed_decider/speed_decider.cc` | L65-L68 | `IsContestRoundaboutScenario()` |
| `modules/planning/tasks/speed_decider/speed_decider.cc` | L74 | `IsTightContestWindowArea()` |
| `modules/planning/tasks/speed_decider/speed_decider.cc` | L77-L85 | `IsRoundaboutStopWallLike()` |
| `modules/planning/tasks/speed_decider/speed_decider.cc` | L87-L128 | `IsRoundaboutNonTargetLaneVehicle()` |
| `modules/planning/tasks/speed_decider/speed_decider.cc` | L308-L311 | 非目标车道忽略逻辑 |
| `modules/planning/tasks/speed_decider/speed_decider.cc` | L499-L512 | 停止距离覆盖 |

### 交通规则

| 文件 | 关键行 | 用途 |
|------|--------|------|
| `modules/planning/traffic_rules/keepclear/keep_clear.cc` | L42-L88 | `IsRoundaboutEntryLike()` |
| `modules/planning/traffic_rules/keepclear/keep_clear.cc` | L199-L209 | Keep Clear 起点后移 8m |

### 场景数据

| 文件 | 用途 |
|------|------|
| `~/.apollo/resources/scenario_sets/69d52dd10e41e17bc62fda1a/scenario_set.json` | 赛事场景集元数据 |
| `~/.apollo/resources/scenario_sets/69d52dd10e41e17bc62fda1a/scenarios/69d520a3b58e96002b6da080.json` | 环岛场景 JSON |
| `data/map_data/Xh_2026_contest/base_map.bin` | Lane 几何数据 |
| `data/map_data/Xh_2026_contest/routing_map.bin` | 路由拓扑图 |

---

## 十、关键数值速查表

### 场景参数

| 参数 | 值 |
|------|-----|
| 智能障碍物巡航速度 | 10-15 m/s |
| 最大减速度 | -3 m/s² |
| 安全距离 | 7-10 m |
| 检测距离 | 50 m |
| 车道宽度 | 3.75 m |
| 交通流车辆速度 | 10-11.18 m/s |
| 场景超时 | 100s |

### 检测参数

| 参数 | 值 | 含义 |
|------|-----|------|
| `roundabout_entry_look_forward_distance` | 80m | 路口检测前视 |
| `roundabout_inside_junction_buffer` | 25m | 路口内部缓冲 |
| `roundabout_curve_look_forward_distance` | 55m | 曲率检测前视 |
| `roundabout_min_abs_kappa` | 0.025 | 最小曲率 |
| `roundabout_min_heading_change` | 0.65 rad (37°) | 最小转向 |
| `roundabout_max_heading_change` | 1.8 rad (103°) | 最大转向 |

### 速度决策参数

| 参数 | 值 | 含义 |
|------|-----|------|
| `kRoundaboutDynamicStopDistance` | 1.2m | 动态车停止距离 |
| `kRoundaboutDynamicYieldDistance` | 1.2m | 动态车让行距离 |
| `kRoundaboutVirtualStopWallDistance` | 0.6m | 虚拟墙停止距离 |
| `kTargetLaneLateralRange` | 2.2m | 非目标车道横向阈值 |
| `kRoundaboutKeepClearStartShift` | 8.0m | KC 起点后移量 |

---

## 十一、潜在问题与建议

### 11.1 检测相关

| 问题 | 严重程度 | 说明 |
|------|----------|------|
| 环岛 vs S 弯误判风险 | ⚠️ 中等 | 若 S 弯前段曲率满足 `kappa > 0.025` 且 `heading_change ∈ (0.65, 1.8)`，需依赖 PNC junction 检测保底区分 |
| 路口标注依赖 | ⚠️ 中等 | 若地图 PNC junction 标注缺失，步骤②失败会导致环岛漏检 |
| 环岛内部重复触发 | ℹ️ 低 | 25m junction buffer 可防止环岛内部重复触发 |

### 11.2 交通流相关

| 问题 | 严重程度 | 说明 |
|------|----------|------|
| 密集车流切入难 | 🔴 **高** | BatchRoute1 间距 10m/速度 10m/s → 间隙约 1s，ADC 可能长时间等待 |
| 非目标车道忽略过激 | ⚠️ 中等 | `IsRoundaboutNonTargetLaneVehicle()` 可能忽略本应让行的相邻车道车辆 |
| 停止距离偏保守 | ⚠️ 中等 | 动态车辆停止距离仅 1.2m，真实让行场景通常需要更大的安全余量 |

### 11.3 场景管理

| 问题 | 严重程度 | 说明 |
|------|----------|------|
| 无完成确认 | ℹ️ 低 | 不像 U_TURN 有 heading 反转完成检测，环岛仅靠检测条件自然消失退出 |
| 无进度追踪状态机 | ℹ️ 低 | 无 "entering" / "inside" / "exiting" 等中间状态 |

### 11.4 调优建议

1. **切入间隙优化**: 考虑在环岛场景中动态调整安全距离（当前位置 7-10m 可能不足以找到切入间隙时降低）
2. **让行决策可视化**: 建议在 DreamView 中标记被 `IsRoundaboutNonTargetLaneVehicle` 忽略的车辆（有助于调试）
3. **完成检测增强**: 可考虑基于 waypoint 到达的环岛完成确认（检查 ADC 是否已通过 waypoint 2 和 waypoint 3）

---

## 十二、调试指南

### 12.1 日志标签速查

| 日志标签 | 含义 |
|---------|------|
| `[ROUNDABOUT][Scenario] detected entry` | 环岛入口检测成功 |
| `[ROUNDABOUT][Scenario] probe miss` | 检测失败 (near miss) |
| `[ROUNDABOUT][Scenario] transfer to CONTEST_ROUNDABOUT` | 场景切换 |
| `[ROUNDABOUT][Scenario] running CONTEST_ROUNDABOUT stage` | 场景运行中 |
| `[ROUNDABOUT][Scenario] block CONTEST_LANE_CHANGE` | 阻止变道 |
| `[ROUNDABOUT][Scenario] exit CONTEST_LANE_CHANGE` | 因环岛退出变道 |
| `[ROUNDABOUT][KeepClear] shift junction keep-clear` | KC 调整 |
| `[ROUNDABOUT][SpeedDecider] ignore non-target-lane vehicle` | 忽略非目标车道车辆 |
| `[WINDOW][SpeedDecider] stop distance override` | 停止距离覆盖 |

### 12.2 排查流程

1. **场景未触发** → 检查 `probe miss` 日志，确认 `max_abs_kappa` / `heading_change` / `near_pnc` 值
2. **让行决策异常** → 检查 `[ROUNDABOUT][SpeedDecider] ignore` 日志，确认哪些车辆被忽略
3. **Keep Clear 异常** → 检查 `[ROUNDABOUT][KeepClear]` 日志中的起点后移量
4. **场景无法退出** → 检查 ADC 是否已驶离环岛，`IsContestRoundaboutEntry()` 返回值

### 12.3 关键文件快速跳转

```
检测入口:  contest_scenario_util.cc:242  IsContestRoundaboutEntry()
场景入场:  contest_scenarios.cc:115      ContestRoundaboutScenario::IsTransferable()
场景出场:  stage_contest_lane_follow.cc:219  StillInScenario() ROUNDABOUT case
速度决策:  speed_decider.cc:308          IsRoundaboutNonTargetLaneVehicle()
KC 规则:   keep_clear.cc:199             IsRoundaboutEntryLike() shift
```

---

*本报告参照 U_TURN_ANALYSIS_REPORT.md 结构，完整分析环岛场景的场景定义、交通流结构、检测算法、生命周期、SpeedDecider/KeepClear 集成、配置文件、潜在问题及调试方法。*
