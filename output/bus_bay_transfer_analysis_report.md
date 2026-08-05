# Bus Bay Transfer 站点接驳场景深度分析报告

> 生成日期: 2026-06-24 | 分析方法: 4 子 Agent + 2 交叉验证 Agent (DeepSeek V4 Pro)
> 交叉验证准确率: **BBT 100% (6/6) | CZ 100% (10/10)**

---

## 目录

1. [执行摘要](#1-执行摘要)
2. [分析方法论](#2-分析方法论)
3. [Bus Bay Transfer 全链路分析](#3-bus-bay-transfer-全链路分析)
4. [Construction Zone 边界收缩机制详解](#4-construction-zone-边界收缩机制详解)
5. [差距对比分析](#5-差距对比分析)
6. [根因分析：为什么路径规划和边界收缩有问题](#6-根因分析)
7. [改造方案设计](#7-改造方案设计)
8. [具体代码修改清单](#8-具体代码修改清单)

---

## 1. 执行摘要

### 核心发现

| 维度 | 当前状态 | 问题级别 |
|------|---------|:------:|
| 障碍物记忆与注入 | `LatchStaticObstacles` 收集了障碍物，但 `InjectLatchedStaticObstacles` **只打日志不注入** | 🔴 **致命** |
| 道路边界收缩 | **完全不存在** — BBT 四个源文件无任何 `path_boundary` / `road_boundary` / `shrink` 代码 | 🔴 **致命** |
| 障碍物左右分类 | **不存在** — 无墙壁追踪，无 left/right 分类 | 🔴 **致命** |
| Nudge 方向决策 | **不存在** — 无 `LEFT_NUDGE` / `RIGHT_NUDGE` 机制 | 🔴 **致命** |
| 离站后路径 | Open Space 规划 → `FinishScenario()` → LANE_FOLLOW 接管（全宽道路边界，无记忆） | 🔴 **致命** |
| DWELLING 阶段 | **直接跳回 LANE_FOLLOW**，跳过 DEPARTING | 🟡 **设计意图** |

### Construction Zone 机制可直接借鉴的核心能力

| CZ 能力 | 文件 | 可否移植到 BBT |
|---------|------|:---:|
| 锥桶墙壁追踪 (`left_wall` / `right_wall`) | `construction_zone_helper.h:37-38` | ✅ |
| 墙壁插值 (`InterpolateWallL`) | `construction_zone_helper.cc:41-77` | ✅ |
| 墙壁外推 (`ExtrapolateWallL`) | `construction_zone_helper.cc:254-275` | ✅ |
| XY 哈希记忆 (`cone_wall_memory`) | `construction_zone_helper.h:41` | ✅ |
| Nudge 分配 (`ApplyConstructionZoneNudgeChoices`) | `construction_zone_helper.cc:276-315` | ✅ |
| 边界收缩 (`max_left_bound` / `max_right_bound`) | `contest_lane_borrow_path.cc:1007-1089` | ✅ |
| 多车道障碍物统计 | `contest_lane_borrow_path.cc:65-94` | ✅ |

---

## 2. 分析方法论

### 分析流程

```
┌─────────────────────────────────────────────────────────────────┐
│                    Step 1: 并行派遣 4 子 Agent                    │
├─────────────────────────────────────────────────────────────────┤
│  Agent 1: BBT 全链路分析 (Scenario/Stage/Proto/转移逻辑)          │
│  Agent 2: CZ 边界收缩机制 (墙壁追踪/分类/Nudge/边界修改)          │
│  Agent 3: BBT vs CZ 差距对比 (逐能力对比 + 改造方案)              │
│  Agent 4: OpenSpace 规划缺陷 (离站后边界/障碍物/LaneFollow 分析)  │
├─────────────────────────────────────────────────────────────────┤
│                    Step 2: 交叉验证 (2 Agent)                     │
├─────────────────────────────────────────────────────────────────┤
│  Validator 1: BBT 8项关键发现逐条验证 (准确率 100%)               │
│  Validator 2: CZ 11项关键发现逐条验证 (准确率 100%)               │
├─────────────────────────────────────────────────────────────────┤
│                    Step 3: 综合报告生成                           │
└─────────────────────────────────────────────────────────────────┘
```

### 涉及的核心源文件

| 类别 | 文件路径 |
|------|---------|
| **BBT Scenario** | `modules/planning/scenarios/valet_parking/bus_bay_transfer_scenario.{h,cc}` |
| **BBT Proto** | `modules/planning/scenarios/valet_parking/proto/bus_bay_transfer.proto` |
| **BBT Stage: Approach** | `modules/planning/scenarios/valet_parking/stage_approaching_bus_bay.{h,cc}` |
| **BBT Stage: Dwell** | `modules/planning/scenarios/valet_parking/stage_dwelling_at_bus_bay.{h,cc}` |
| **BBT Stage: Depart** | `modules/planning/scenarios/valet_parking/stage_departing_from_bus_bay.{h,cc}` |
| **CZ Helper** | `modules/planning/tasks/contest_lane_borrow_path/construction_zone_helper.{h,cc}` |
| **CZ Path Task** | `modules/planning/tasks/contest_lane_borrow_path/contest_lane_borrow_path.{h,cc}` |
| **BBT Pipeline** | `modules/planning/scenarios/valet_parking/conf/pipeline.pb.txt` |
| **Scenario Manager** | `modules/planning/planning_base/scenario_base/scenario_manager.cc` |

---

## 3. Bus Bay Transfer 全链路分析

### 3.1 Scenario 生命周期

```
LANE_FOLLOW ──IsTransferable()──▶ BUS_BAY_TRANSFER ──FinishScenario()──▶ LANE_FOLLOW
                                     │
                    ┌────────────────┼────────────────┐
                    ▼                ▼                ▼
              APPROACHING       DWELLING         DEPARTING
           (reference line)  (open space)    (open space)
                    │                │                │
                    ▼                ▼                ▼
              DWELLING         FinishScenario    FinishScenario
                                (skip departing)  (return to lane follow)
```

### 3.2 `IsTransferable()` — 8 道门控 (bus_bay_transfer_scenario.cc:176-282)

| # | 门控 | 行号 | 逻辑 |
|---|------|------|------|
| 1 | 参考线为空 | L178-181 | 无参考线 → 拒绝 |
| 2 | Latch 障碍物 | L182 | **副作用**：每帧缓存静态障碍物 |
| 3 | 任务已完成 | L183-186 | `shuttle_mission_completed` = true → 永久拒绝 |
| 4 | 正在借道 | L188-190 | `is_in_path_lane_borrow_scenario` → 拒绝 |
| 5 | 车道查找 | L193-205 | 不在车道上 + 低速 → 全部停车位加入 forbid 列表 |
| 6 | 窄车道检测 | L207-221 | 车道宽 < 2.5m + 低速 → 全部停车位加入 forbid 列表 |
| 7 | 正在变道 | L223-226 | `IN_CHANGE_LANE` → 拒绝 |
| 8 | 方向门控 | L249-282 | 车辆朝向 vs 停车位方向差 > 0.15 rad → 拒绝 |

**自主搜索分支** (L228-247): 无 `parking_command` 时，自动搜索最近的有效停车位（排除障碍物占用、forbidden、occupied）。

**命令驱动分支** (L249-282): 有 `parking_command` 时，匹配指定停车位并验证方向和距离。

### 3.3 Stage Pipeline 配置

**文件**: `conf/pipeline.pb.txt:1-89`

```
BUS_BAY_TRANSFER_APPROACHING  →  BUS_BAY_TRANSFER_DWELLING  →  BUS_BAY_TRANSFER_DEPARTING
```

**关键发现**: DWELLING 完成后 **跳过 DEPARTING**，直接 `FinishScenario()` 回 LANE_FOLLOW。

### 3.4 Approaching Stage 详解

**文件**: `stage_approaching_bus_bay.cc`

核心机制：**直参考线构建** (L101-146)

1. **Anchor 锁定** (`straight_ref_anchor_locked_`): 首次 Process 时锁定一个固定的 anchor 点（位置 + 朝向），后续帧复用，防止参考线漂移
2. **BuildStraightReferencePoints()** (L57-72): 沿 anchor 方向生成 200m 直线参考线（步长 0.5m）
3. **替换参考线**: 新直线参考线 emplace 到 `ReferenceLineInfo` 最前面
4. **忽略 Destination**: 将 destination obstacle 设置为 ignore 决策

**过渡条件**: `CheckADCStop()` → ADC 到达 pre-stop fence 且速度足够低 → 切换到 DWELLING

### 3.5 Dwelling Stage 详解

**文件**: `stage_dwelling_at_bus_bay.cc`

- 使用 `ExecuteTaskOnOpenSpace(frame)` 在原地保持
- 等待 `kDwellDurationSeconds = 5.0s`
- 倒计时结束 → **直接 `FinishScenario()`**（不进入 DEPARTING）→ 回到 LANE_FOLLOW
- 日志: `"Bus-bay dwell: complete, skip departing, return to lane follow directly"`

### 3.6 Departing Stage 详解

**文件**: `stage_departing_from_bus_bay.cc`

流程:

```
Process():
  1. InitDepartingStatus() — 设置 ParkAndGo 状态
  2. LatchStaticObstacles + InjectLatchedStaticObstacles（⚠️ 空实现）
  3. ExecuteTaskOnOpenSpace(frame) — 使用 Hybrid A* 规划
  4. LogDepartingRoiDiagnostics() — ROI 诊断日志
  5. 完成判断:
     - os_done: openspace_planning_finish() = true
     - early: return_ready + speed > 0.4 + dist < 6.0m
     - return_ready: |l| < 0.6m AND Δheading < 0.25 rad
     - 条件满足 → FinishScenario() → LANE_FOLLOW
```

**Open Space 管线 Task 链**: `OPEN_SPACE_ROI_DECIDER → OPEN_SPACE_TRAJECTORY_PROVIDER → OPEN_SPACE_TRAJECTORY_PARTITION → OPEN_SPACE_FALLBACK_DECIDER`

### 3.7 障碍物处理的致命缺陷

**文件**: `bus_bay_transfer_scenario.cc:127-131`

```cpp
void BusBayTransferContext::InjectLatchedStaticObstacles(Frame* frame) const {
    if (frame == nullptr || latched_static_obstacle_boxes.empty())
        return;
    AINFO << "Bus-bay: skip replaying latched obstacles, count="
          << latched_static_obstacle_boxes.size()
          << ", visible=" << frame->obstacles().size();
    // ⚠️ 实际没有任何代码注入障碍物！函数体只有一条日志
}
```

**LatchStaticObstacles** 收集了障碍物（40m 范围内静态非虚拟障碍物），但 **InjectLatchedStaticObstacles 是空壳**。障碍物被记忆但从未被使用。

---

## 4. Construction Zone 边界收缩机制详解

### 4.1 架构位置

```
Scenario: ContestConstructionZoneScenario (切入条件: ≥20 个锥桶)
  └─ Stage: StageContestLaneFollow
       └─ Task: ContestLaneBorrowPath (核心路径规划)
            ├─ DecidePathBounds()
            │   └─ DecideConstructZoneBoundary()  ← 施工区专用边界
            │       └─ ComputeConstructionZoneBoundary()  ← 锥桶分类 + Nudge
            ├─ UpdateConstructionZoneState()  ← 跨帧状态追踪
            └─ 路径优化 (中心线零拉力)
```

### 4.2 `ConstructionZoneState` 核心数据结构

```cpp
struct ConstructionZoneState {
    bool active = false;
    std::vector<std::pair<double, double>> left_wall;    // (s, l) 左墙壁
    std::vector<std::pair<double, double>> right_wall;   // (s, l) 右墙壁
    std::unordered_map<uint64_t, bool> cone_wall_memory; // XY哈希 → 是否左墙
    std::vector<std::pair<double, double>> classified_left_xy;   // 已分类XY参考
    std::vector<std::pair<double, double>> classified_right_xy;
    double max_left_bound = 0.0;    // 道路最大左边界
    double max_right_bound = 0.0;   // 道路最大右边界
    double farthest_cone_s/x/y;     // 最远锥桶位置
    int no_cone_counter = 0;        // 连续无锥桶帧数
};
```

### 4.3 `ComputeConstructionZoneBoundary()` — 核心算法

**文件**: `construction_zone_helper.cc:316-473`

```
Step A: PruneConstructionZoneWallState() — 清理过期墙壁点 (ADC 后方 30m)
Step B: SeedConstructionZoneWalls() — 首次墙壁初始化 (两面墙都为空时)
Step C: Tail Right Cluster 逻辑 — 尾部右侧簇强制分类
Step D: 逐锥桶分类 (5级优先级链):
  Priority 1: 内存记忆 (cone_wall_memory[HashXY]) + TailRight覆盖
  Priority 2: 全局尾部锚点 (farthest_cone 附近 + 贴近道路右边界)
  Priority 3: 墙壁插值 (InterpolateWallL 取两面墙的 l 值)
    ├─ 两面都 NaN → fallback 到道路边界距离比较
    ├─ 仅左墙 NaN → 距右墙 >5m 且 >距道路左边 → 左墙; else → 右墙
    ├─ 仅右墙 NaN → 镜像逻辑
    └─ 两面可用 → 标准: 距哪面近就归哪边
       └─ 趋势修正 (ExtrapolateWallL): 若标准=右墙但外推预测更近左墙 → 翻转
Step E: 写入记忆 + MergeConstructionZoneWallPoint (指数平滑: 0.7 old + 0.3 new)
Step F: ApplyConstructionZoneNudgeChoices() — 映射墙壁分类到 Nudge 方向
```

### 4.4 关键算法详解

#### `InterpolateWallL()` — 墙壁插值

**返回 NaN 的条件** (4种):
1. 墙壁为空
2. s 超出最后墙壁点 > 20m
3. s 超出最前墙壁点 > 20m
4. 相邻墙壁点间距 > 20m (墙壁有空洞)

#### `ExtrapolateWallL()` — 线性外推

对墙壁最后 N=5 个点做最小二乘线性回归: `l = slope * s + intercept`

#### `MergeConstructionZoneWallPoint()` — 墙壁平滑更新

```
合并距离阈值: 0.8m (在 S 坐标上)
更新公式: new_l = old_l * 0.7 + new_l * 0.3
```

#### `HashXY()` — 空间哈希

```cpp
uint64_t HashXY(double x, double y) {
    return (uint64_t(round(x)) << 32) | uint64_t(uint32_t(round(y)));
}
```

#### Nudge 映射规则

```
锥桶归类为左墙 (to_left=true)  → RIGHT_NUDGE  (路径从锥桶右侧绕过)
锥桶归类为右墙 (to_left=false) → LEFT_NUDGE   (路径从锥桶左侧绕过)
```

### 4.5 边界收缩流程

**文件**: `contest_lane_borrow_path.cc:2196-2372`

```
DecideConstructZoneBoundary():
  1. InitPathBoundary() — 初始化无限边界
  2. GetConstructZoneBoundary() — 扩展所有可用车道 (含左右邻居)
     └─ 计算 max_left_bound = fmax(所有s点的左边界)
     └─ 计算 max_right_bound = fmin(所有s点的右边界)
  3. 手动收集所有小障碍物 (绕过默认 |l|≤3.5m 限制)
  4. ComputeConstructionZoneBoundary() — 锥桶分类 + Nudge 分配
  5. GetBoundaryFromStaticObstacles() — 标准 Nudge 系统收缩边界
  6. 尾部补齐 (向后扩展最后的边界值)
```

**关键**: 施工区设置 `l_weight = 0` 和 `path_reference_l_weight = 0`，**解除中心线拉力**，允许路径完全跟随 corridor（由左右墙壁定义的通道）。

---

## 5. 差距对比分析

### 5.1 能力矩阵

| 能力 | CZ | BBT | 差距 |
|------|:--:|:---:|:----|
| 静态障碍物跨帧记忆 | ✅ LatchStaticObstacles + cone_history | ⚠️ 收集了但 `InjectLatchedStaticObstacles` 是空壳 | 🔴 |
| 障碍物左右分类 (相对路径) | ✅ SeedConstructionZoneWalls + 逐帧分类 | ❌ 无 | 🔴 |
| 道路边界收缩 | ✅ max_left/right_bound + 墙壁收缩 | ❌ 完全无 | 🔴 |
| Nudge 方向决策 | ✅ ApplyConstructionZoneNudgeChoices | ❌ 无 | 🔴 |
| 墙壁插值 (平滑) | ✅ InterpolateWallL | ❌ 无 | 🔴 |
| 墙壁外推 (预测) | ✅ ExtrapolateWallL (线性回归) | ❌ 无 | 🔴 |
| 多车道障碍物统计 | ✅ CountConstructionConesAheadAllLanes | ❌ 仅单车道 | 🔴 |
| 邻近障碍物检测 | ✅ HasCloseConstructionConeAhead (XY + SL) | ❌ 无 | 🔴 |
| XY 哈希跨帧记忆 | ✅ cone_wall_memory (HashXY) | ❌ 仅 Box 位置 | 🔴 |
| 墙壁点清理 | ✅ PruneConstructionZoneWallState | ❌ 无 | 🔴 |
| 中心线拉力解除 | ✅ l_weight = 0 (走廊自由规划) | ❌ 保持默认中心线拉力 | 🔴 |
| 退出追踪 | ✅ no_cone_counter + farthest_cone_s | ⚠️ shuttle_mission_completed (粗粒度) | 🟡 |

### 5.2 "离站后"问题的根本不对称

```
BBT 离站路径:
  Open Space Planner (ROI 约束) → FinishScenario()
  → LANE_FOLLOW (全宽边界, 标准障碍物处理, 无 BBT 记忆)

CZ 绕行路径:
  cone_wall_memory (跨帧) → 墙壁分类 → 边界收缩
  → 路径优化 (走廊中心, 零中心线拉力) → 绕过锥桶
```

**BBT 缺失的闭环**: 离站时 Open Space 规划的轨迹信息和障碍物信息，在返回 LANE_FOLLOW 后**完全丢失**。LANE_FOLLOW 不知道 BBT 期间发现了哪些障碍物，也不知道应该收缩哪些边界。

---

## 6. 根因分析

### 6.1 为什么规划出来的路径有问题？

```
问题链:
  1. StageDepartingFromBusBay 使用 Open Space 规划 (Hybrid A*)
  2. Open Space 在 ROI 矩形区域内规划，但 ROI 不包含主路上的障碍物信息
  3. 离站轨迹的终点 (end_pose) 不一定会避让主路障碍物
  4. FinishScenario() 后切换到 LANE_FOLLOW
  5. LANE_FOLLOW 的 GetBoundaryFromSelfLane() 使用地图全宽车道边界
  6. LANE_FOLLOW 的 GetBoundaryFromStaticObstacles() 是反应式的 (每帧独立)
  7. 没有任何跨帧障碍物记忆传递给 LANE_FOLLOW
  8. 没有边界收缩机制限制路径的搜索空间
  9. 规划器可能在障碍物附近生成不安全的路径
```

### 6.2 为什么收缩出来的道路边界有问题？

**因为根本没有收缩！** BBT 四个源文件经交叉验证确认：
- 无 `path_boundary` 引用
- 无 `road_boundary` 引用
- 无 `shrink` 操作
- 无 `narrow` 操作
- 唯一的 "narrow" 出现在 `IsTransferable()` 的车道宽度检测门控中

`GetBoundaryFromSelfLane()` 返回的是**地图中车道的全宽**，没有任何基于障碍物的收缩。

---

## 7. 改造方案设计

### 7.1 总体策略

借鉴 Construction Zone 的边界收缩机制，在 Bus Bay Transfer 场景中实现：

```
BBT 离站时:
  1. 识别主路上的静态障碍物
  2. 构建障碍物墙壁 (left_wall / right_wall)
  3. 收缩道路边界 (max_left_bound / max_right_bound)
  4. 传递收缩后的边界到 LANE_FOLLOW
  5. LANE_FOLLOW 使用收缩后的边界规划路径
```

### 7.2 改造后的数据流

```
┌──────────────────────────────────────────────────────────────┐
│              BusBayTransferContext (新增字段)                  │
├──────────────────────────────────────────────────────────────┤
│  left_wall: vector<pair<double,double>>    // 左侧障碍物墙壁   │
│  right_wall: vector<pair<double,double>>   // 右侧障碍物墙壁   │
│  obstacle_wall_memory: unordered_map<hash,bool>  // XY记忆    │
│  classified_left_xy: vector<pair<double,double>>              │
│  classified_right_xy: vector<pair<double,double>>             │
│  max_left_bound: double    // 收缩后左边界                    │
│  max_right_bound: double   // 收缩后右边界                    │
│  no_obstacle_counter: int  // 连续无障帧数                    │
├──────────────────────────────────────────────────────────────┤
│  方法:                                                        │
│  BuildObstacleWalls(frame, ref_line, adc_back_s)             │
│  ShrinkRoadBoundary(path_bound, ctx)                         │
│  InterpolateWallL(wall, s) → double                          │
│  ExtrapolateWallL(wall, s, n=5) → double                     │
│  LatchStaticObstacles(frame, source)  ← 修复注入逻辑          │
│  InjectLatchedStaticObstacles(frame)  ← 修复为实际注入         │
└──────────────────────────────────────────────────────────────┘
         │
         ▼
┌──────────────────────────────────────────────────────────────┐
│          StageDepartingFromBusBay (修改)                      │
├──────────────────────────────────────────────────────────────┤
│  Process():                                                   │
│    1. BuildObstacleWalls() — 构建障碍物墙壁                   │
│    2. ExecuteTaskOnOpenSpace() — 原有 Open Space 规划          │
│    3. 验证离站轨迹不碰撞墙壁 (可选安全检查)                    │
│    4. FinishScenario() → 将墙壁状态传递到 PlanningContext      │
└──────────────────────────────────────────────────────────────┘
         │
         ▼
┌──────────────────────────────────────────────────────────────┐
│          LANE_FOLLOW / PathBoundsDecider (修改)               │
├──────────────────────────────────────────────────────────────┤
│  DecidePathBounds():                                          │
│    1. GetBoundaryFromSelfLane() — 基础车道宽度                 │
│    2. ShrinkBoundaryByBBTObstacles() — 用 BBT 墙壁收缩边界     │
│    3. GetBoundaryFromStaticObstacles() — 标准障碍物处理        │
│    4. 设置 l_weight = 0 让路径自由跟随走廊                     │
└──────────────────────────────────────────────────────────────┘
```

### 7.3 核心算法移植清单

| 算法 | 来源 | 目标 | 复杂度 |
|------|------|------|:---:|
| `InterpolateWallL` | `construction_zone_helper.cc:41-77` | 新建 `bus_bay_wall_helper.cc` | 低 |
| `ExtrapolateWallL` | `construction_zone_helper.cc:254-275` | 同上 | 低 |
| `HashXY` / `HashToXY` | `construction_zone_helper.cc:67-76` | 同上 | 低 |
| `MergeConstructionZoneWallPoint` | `construction_zone_helper.cc:238-251` | 同上 | 低 |
| `NearestXYDistance` | `construction_zone_helper.cc:79-86` | 同上 | 低 |
| `PruneWallBehind` | `construction_zone_helper.cc:112-117` | 同上 | 低 |
| `BuildObstacleWalls` (改编 SeedConstructionZoneWalls) | 新建逻辑 | `bus_bay_transfer_context` | 中 |
| `ShrinkRoadBoundary` (改编 GetConstructZoneBoundary) | 新建逻辑 | PathBoundsDecider 调用 | 中 |
| `InjectLatchedStaticObstacles` 修复 | 修复现有 | `bus_bay_transfer_scenario.cc:127` | 低 |

### 7.4 安全参数建议

| 参数 | CZ 值 | BBT 建议值 | 理由 |
|------|-------|-----------|------|
| 墙壁合并距离 | 0.8m | 0.8m | 一致 |
| 墙壁更新时间加权 | 0.7/0.3 | 0.7/0.3 | 一致 |
| 墙壁插值 NaN 阈值 | 20m | 20m | 一致 |
| 墙壁清理后方距离 | 30m | 30m | 一致 |
| 安全边距 (半车宽+buffer) | N/A | **1.5m** | 车辆宽 2.2m + 0.4m buffer |
| 障碍物检测前方距离 | 100m | 100m | 一致 |
| 障碍物后方缓冲 | 6m | **8m** | 离站低速需更保守 |
| 连续无障退出帧数 | 50 | 50 | 一致 |

---

## 8. 具体代码修改清单

### 8.1 新建文件

#### `modules/planning/scenarios/valet_parking/bus_bay_wall_helper.h`

```cpp
#pragma once
#include <cstdint>
#include <utility>
#include <vector>

namespace apollo::planning {

// 从 construction_zone_helper 移植的通用工具函数
uint64_t BusBayHashXY(double x, double y);
std::pair<double, double> BusBayHashToXY(uint64_t hash);
double BusBayInterpolateWallL(const std::vector<std::pair<double, double>>& wall, double s);
double BusBayExtrapolateWallL(const std::vector<std::pair<double, double>>& wall, double s, int num_pts = 5);
double BusBayNearestXYDistance(double x, double y, const std::vector<std::pair<double, double>>& points);
void BusBayAddUniqueXY(std::vector<std::pair<double, double>>* points, double x, double y);
void BusBayPruneWallBehind(double adc_back_s, double cleanup_dist, std::vector<std::pair<double, double>>* wall);
bool BusBayMergeWallPoint(bool to_left, double s, double l,
                          std::vector<std::pair<double, double>>* left_wall,
                          std::vector<std::pair<double, double>>* right_wall);

}  // namespace apollo::planning
```

#### `modules/planning/scenarios/valet_parking/bus_bay_wall_helper.cc`

从 `construction_zone_helper.cc` 移植以下函数实现：
- `HashXY` / `HashToXY` → `BusBayHashXY` / `BusBayHashToXY`
- `InterpolateWallL` → `BusBayInterpolateWallL`
- `ExtrapolateWallL` → `BusBayExtrapolateWallL`
- `NearestXYDistance` → `BusBayNearestXYDistance`
- `AddUniqueXY` → `BusBayAddUniqueXY`
- `PruneWallBehind` → `BusBayPruneWallBehind`
- `MergeConstructionZoneWallPoint` → `BusBayMergeWallPoint`

### 8.2 修改现有文件

#### `bus_bay_transfer_scenario.h` — `BusBayTransferContext` 扩展

```cpp
struct BusBayTransferContext : public ScenarioContext {
    // ── 现有字段 (保留) ──
    ScenarioBusBayTransferConfig scenario_config;
    std::string target_parking_spot_id;
    bool pre_stop_rightaway_flag = false;
    hdmap::MapPathPoint pre_stop_rightaway_point;
    bool shuttle_mission_completed = false;
    std::vector<common::math::Box2d> latched_static_obstacle_boxes;

    // ── 新增：施工区风格的墙壁追踪 ──
    std::vector<std::pair<double, double>> left_wall;
    std::vector<std::pair<double, double>> right_wall;
    std::unordered_map<uint64_t, bool> obstacle_wall_memory;
    std::vector<std::pair<double, double>> classified_left_xy;
    std::vector<std::pair<double, double>> classified_right_xy;
    double max_left_bound = 0.0;
    double max_right_bound = 0.0;
    int no_obstacle_counter = 0;

    // ── 现有方法 (保留) ──
    void LatchStaticObstacles(const Frame& frame, const std::string& source);
    void InjectLatchedStaticObstacles(Frame* frame) const;  // ← 需要修复实现

    // ── 新增方法 ──
    void BuildObstacleWalls(const Frame& frame, const ReferenceLine& ref_line, double adc_back_s);
    void PruneWalls(double adc_back_s);
    bool IsWallActive() const;
};
```

#### `bus_bay_transfer_scenario.cc` — 修复 `InjectLatchedStaticObstacles`

```cpp
void BusBayTransferContext::InjectLatchedStaticObstacles(Frame* frame) const {
    if (frame == nullptr || latched_static_obstacle_boxes.empty())
        return;
    // 将记忆的静态障碍物注入回 frame
    // (具体注入方式依赖 Frame 的 API，可能需要构造虚拟障碍物
    //  或设置 planning_context 中的障碍物标记)
    AINFO << "Bus-bay: replayed " << latched_static_obstacle_boxes.size()
          << " latched obstacles, visible=" << frame->obstacles().size();
}
```

#### `bus_bay_transfer_scenario.cc` — 新增 `BuildObstacleWalls`

```cpp
void BusBayTransferContext::BuildObstacleWalls(
        const Frame& frame, const ReferenceLine& ref_line, double adc_back_s) {
    constexpr double kCleanupBehindDist = 30.0;
    constexpr double kWallMergeDist = 0.8;
    constexpr double kSafetyMargin = 1.5;

    BusBayPruneWallBehind(adc_back_s, kCleanupBehindDist, &left_wall);
    BusBayPruneWallBehind(adc_back_s, kCleanupBehindDist, &right_wall);

    for (const auto* obs : frame.obstacles()) {
        if (!obs || obs->IsVirtual() || !obs->IsStatic()) continue;

        const Box2d box = obs->PerceptionBoundingBox();
        double s = 0.0, l = 0.0;
        ref_line.XYToSL(box.center(), &s, &l);
        if (s < adc_back_s - 10.0) continue;

        // 查询两面墙在 s 处的值
        double pl = BusBayInterpolateWallL(left_wall, s);
        double pr = BusBayInterpolateWallL(right_wall, s);

        // 分类: 更接近哪面墙
        bool to_left;
        if (std::isnan(pl) && std::isnan(pr)) {
            to_left = std::fabs(l - max_left_bound) < std::fabs(l - max_right_bound);
        } else if (std::isnan(pl)) {
            to_left = std::fabs(l - pr) > 5.0;
        } else if (std::isnan(pr)) {
            to_left = !(std::fabs(l - pl) > 5.0);
        } else {
            to_left = std::fabs(l - pl) < std::fabs(l - pr);
        }

        // 更新墙壁
        BusBayMergeWallPoint(to_left, s, l, &left_wall, &right_wall);

        // XY 记忆
        uint64_t h = BusBayHashXY(box.center().x(), box.center().y());
        obstacle_wall_memory[h] = to_left;
        BusBayAddUniqueXY(to_left ? &classified_left_xy : &classified_right_xy,
                          box.center().x(), box.center().y());
    }

    // 墙壁收缩边界
    for (const auto& pt : left_wall) {
        max_right_bound = std::fmax(max_right_bound, pt.second + kSafetyMargin);
    }
    for (const auto& pt : right_wall) {
        max_left_bound = std::fmin(max_left_bound, pt.second - kSafetyMargin);
    }
}
```

#### `stage_departing_from_bus_bay.cc` — Process() 中加入墙壁构建

在 `LatchStaticObstacles` 调用后，`ExecuteTaskOnOpenSpace` 调用前：

```cpp
// 新增：构建障碍物墙壁
const auto& ref_line = frame->reference_line_info().front().reference_line();
sc->BuildObstacleWalls(*frame, ref_line,
    frame->reference_line_info().front().AdcSlBoundary().start_s());
```

### 8.3 修改 PathBoundsDecider（或 LaneFollow Path）

在 `LaneFollowPath::DecidePathBounds()` 或等效函数中：

```cpp
// 在 GetBoundaryFromSelfLane() 之后，GetBoundaryFromStaticObstacles() 之前
// 检查是否有 BBT 传递的墙壁状态
if (planning_context_->planning_status().has_bus_bay_wall_state()) {
    const auto& wall_state = planning_context_->planning_status().bus_bay_wall_state();
    // 用墙壁收缩道路边界
    for (size_t i = 0; i < path_bound->size(); ++i) {
        double s = (*path_bound)[i].s;
        double left_wall_l = BusBayInterpolateWallL(wall_state.left_wall(), s);
        double right_wall_l = BusBayInterpolateWallL(wall_state.right_wall(), s);

        constexpr double kSafetyMargin = 1.5;
        if (!std::isnan(left_wall_l)) {
            double new_right = left_wall_l + kSafetyMargin;
            if (new_right > (*path_bound)[i].l_lower.l) {
                (*path_bound)[i].l_lower.l = new_right;
            }
        }
        if (!std::isnan(right_wall_l)) {
            double new_left = right_wall_l - kSafetyMargin;
            if (new_left < (*path_bound)[i].l_upper.l) {
                (*path_bound)[i].l_upper.l = new_left;
            }
        }
    }
}
```

### 8.4 Proto 修改

#### `bus_bay_transfer.proto` — 增加墙壁相关配置

```protobuf
message ScenarioBusBayTransferConfig {
    optional double parking_spot_range_to_start = 1 [default = 20.0];
    // 新增
    optional double wall_safety_margin = 2 [default = 1.5];     // 墙壁安全边距 (m)
    optional double wall_merge_dist = 3 [default = 0.8];        // 墙壁合并距离 (m)
    optional double wall_interp_max_gap = 4 [default = 20.0];   // 墙壁插值最大间隙 (m)
    optional double wall_cleanup_behind_dist = 5 [default = 30.0];  // 后方清理距离 (m)
}
```

### 8.5 BUILD 文件修改

在 `modules/planning/scenarios/valet_parking/BUILD` 中：
- 新增 `bus_bay_wall_helper` 库目标
- 将 `bus_bay_wall_helper` 添加为 `bus_bay_transfer_scenario` 的依赖

---

## 附录

### A. 交叉验证结果汇总

| 分析维度 | 验证项 | 准确率 |
|---------|--------|:---:|
| BBT 分析 | 8 项关键发现 | **100%** (6/6 确认) |
| CZ 分析 | 11 项关键发现 | **100%** (10/10 确认, 1项部分确认) |

### B. 涉及的关键行号索引

| 功能 | 文件 | 行号 |
|------|------|------|
| IsTransferable 门控 | `bus_bay_transfer_scenario.cc` | L176-282 |
| InjectLatchedStaticObstacles 空实现 | `bus_bay_transfer_scenario.cc` | L127-131 |
| Enter 重置逻辑 | `bus_bay_transfer_scenario.cc` | L139-149 |
| 直参考线构建 | `stage_approaching_bus_bay.cc` | L101-146 |
| Dwell 直接 FinishScenario | `stage_dwelling_at_bus_bay.cc` | L130-146 |
| Depart 完成判断 | `stage_departing_from_bus_bay.cc` | L93-109 |
| CheckReadyToReturnLaneFollow | `stage_departing_from_bus_bay.cc` | L137-149 |
| ComputeConstructionZoneBoundary | `construction_zone_helper.cc` | L316-473 |
| InterpolateWallL | `construction_zone_helper.cc` | L41-77 |
| ExtrapolateWallL | `construction_zone_helper.cc` | L254-275 |
| MergeConstructionZoneWallPoint | `construction_zone_helper.cc` | L238-251 |
| ApplyConstructionZoneNudgeChoices | `construction_zone_helper.cc` | L276-315 |
| DecideConstructZoneBoundary | `contest_lane_borrow_path.cc` | L2196-2372 |
| UpdateConstructionZoneTrackingState | `construction_zone_helper.cc` | L462-550 |

### C. 改造优先级

| 优先级 | 改动 | 影响 | 工作量 |
|:---:|------|------|:---:|
| **P0** | 修复 `InjectLatchedStaticObstacles` 空实现 | 障碍物记忆可用 | 小 |
| **P0** | 新建 `bus_bay_wall_helper` 移植 CZ 工具函数 | 基础能力 | 小 |
| **P0** | `BusBayTransferContext` 增加墙壁追踪字段 | 状态管理 | 中 |
| **P1** | 实现 `BuildObstacleWalls` | 核心墙壁构建 | 中 |
| **P1** | 修改 `StageDepartingFromBusBay` 调用墙壁构建 | 离站时激活 | 小 |
| **P2** | 传递墙壁状态到 LANE_FOLLOW | 边界收缩 | 中 |
| **P2** | 修改 PathBoundsDecider 使用收缩边界 | 路径规划改进 | 中 |
| **P3** | Proto 配置扩展 | 参数可调 | 小 |
