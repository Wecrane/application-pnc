# Apollo 星火自动驾驶大赛 — 7 题解题方案总览

> 生成时间：2026-06-16 | 分析方法：7 子 Agent 并行分析 + 3 审查 Agent 交叉验证

---

## 一、总览表

| 赛题        | 策略           | 新建/修改 | 核心改动                                                | 工作量                | 风险等级 |
| ----------- | -------------- | --------- | ------------------------------------------------------- | --------------------- | -------- |
| 1. 交通灯   | 纯配置         | 配置仅    | 启用场景 + stop_distance=1.75 + 红灯右转                | **低**          | 🟢 低    |
| 2. 变道     | 改 Task + 配置 | 修改      | 变道限速 + 安全距离配置化 + freeze_time 延长            | **低→中** ⚠️ | 🟢 低    |
| 3. S弯      | 纯配置         | 配置仅    | 曲率限速放宽 + 横向 buffer 增大 + 全局限速 30km/h       | **低**          | 🟢 低    |
| 4. U-Turn   | 扩展场景       | 修改      | large_curvature 增加 U-Turn 分支 + Hybrid A* + 对向检测 | **中**          | 🟡 中    |
| 5. 施工区域 | 移植旧代码     | 修改      | LaneBorrowPath 移植锥桶识别 + 施工区限速 + 倒车恢复     | **中**          | 🟡 中    |
| 6. 站点接驳 | 扩展场景       | 修改+新建 | PullOver 新增 WAITING + DEPARTURE Stage                 | **中**          | 🟡 中    |
| 7. 环岛     | 新建场景       | 新建      | Roundabout Scenario (3 Stage) + GeoFence 检测           | **高**          | 🔴 高    |

> ⚠️ 赛题 2 工作量标注说明：分析方案标注 LOW，审查指出 proto 修改需要 Bazel 重编译，实际工作量上调为 LOW→MEDIUM。

---

## 二、赛题一：xh_2026_交通灯场景

### 2.1 现有能力评估

Apollo 已有完整的红绿灯处理体系：

- **TrafficLight TrafficRule**（`traffic_light.cc`）：遍历参考线上所有信号灯 overlap，对 RED/YELLOW/UNKNOWN 信号构建虚拟停车障碍物。停车位置 = `stop_line_s - config_.stop_distance()`，默认 `stop_distance=1.0m`。
- **traffic_light_protected 场景**（2 Stage）：Approach 阶段等待全部变绿 → IntersectionCruise 通过。
- **traffic_light_unprotected_right_turn 场景**（3 Stage）：通过 `GetPathTurnType()==RIGHT_TURN` 区分右转意图，`enable_right_turn_on_red` 控制红灯右转行为（默认 false）。

**当前 profile 状态**：所有 traffic light 场景被注释，仅 TrafficRule 统一处理红灯，无法区分直行和右转。

### 2.2 解题策略

**纯配置方案，无需修改源码。**

1. **启用红绿灯场景**：在 `public_road_planner_config.pb.txt` 中取消注释，UNPROTECTED_RIGHT_TURN 排在 PROTECTED 之前（高优先级）。
2. **调整停车距离**：新建 profile 级 `traffic_light/conf/default_conf.pb.txt`，`stop_distance: 1.75`（1.5-2.0m 中间值）。
3. **开启红灯右转**：新建 `unprotected_right_turn/conf/scenario_conf.pb.txt`，`enable_right_turn_on_red: true`。
4. **复制 pipeline**：从模块目录复制 `pipeline.pb.txt` 到 profile 目录。

### 2.3 核心机制

```
TrafficLight TrafficRule → 全局停车框架 (stop_distance=1.75)
        ↓
ScenarioManager → 按优先级分配场景
        ├── routing==RIGHT_TURN? → unprotected_right_turn (标记 done, 蠕行通过)
        └── routing!=RIGHT_TURN? → protected (等待绿灯)
                ↓
PiecewiseJerkSpeed Task → 精确平滑停车
```

### 2.4 参数变更

| 参数                         | 文件                                                                              | 旧值  | 新值     |
| ---------------------------- | --------------------------------------------------------------------------------- | ----- | -------- |
| `stop_distance`            | `profiles/default/.../traffic_light/conf/default_conf.pb.txt`（新建）           | 1.0   | 1.75     |
| `enable_right_turn_on_red` | `profiles/default/.../unprotected_right_turn/conf/scenario_conf.pb.txt`（新建） | false | true     |
| 场景列表                     | `profiles/default/.../public_road_planner_config.pb.txt`                        | 注释  | 取消注释 |

### 2.5 风险与缓解

| 风险                          | 缓解                                                               |
| ----------------------------- | ------------------------------------------------------------------ |
| stop_distance=1.75 是全局参数 | 右转场景通过 done_overlap_id 清除，不影响；直行 1.75m 在赛题范围内 |
| 场景优先级排序错误            | 必须 UNPROTECTED_RIGHT_TURN 排在 PROTECTED 之前                    |
| GetPathTurnType 依赖地图标注  | 需验证赛题地图中右转路口的 turn type 正确                          |

---

## 赛题二：xh_2026_变道场景

### 3.1 现有能力评估

- **LaneChangePath Task**：完整变道路径规划（DecidePathBounds→OptimizePath→AssessPath）。
- **IsClearToChangeLane()**：同向后向安全距离 = `max(10m, (v_obs-v_ego)×3s)`，对向安全时间=5s。**全部硬编码，不可配置**。
- **防连续变道**：`change_lane_success_freeze_time=3s`（变道成功后冷却）。
- **变道限速**：**完全缺失**。SpeedSetting 不感知变道状态。

### 3.2 解题策略

**修改 LaneChangePath Task + proto 扩展 + 配置调整。**

1. **变道限速**：在 `LaneChangePath::Process()` 中 `UpdateLaneChangeStatus()` 之后，`IN_CHANGE_LANE` 状态时 `reference_line_info->SetCruiseSpeed(8.33)`。
2. **安全距离配置化**：proto 新增 `min_safe_distance_backward`、`safe_time_same_direction` 等字段，替换硬编码。
3. **增强防连续变道**：`change_lane_success_freeze_time` 从 3s → 5s（审查建议改动：8s 过于激进）。
4. **变道后速度恢复**：变道完成 `CHANGE_LANE_FINISHED` 后，SpeedSetting TrafficRule 自动恢复巡航速度。

### 3.3 参数变更

| 参数                                   | 文件                                          | 旧值           | 新值 |
| -------------------------------------- | --------------------------------------------- | -------------- | ---- |
| `change_lane_success_freeze_time`    | `lane_change_path/conf/default_conf.pb.txt` | 3.0            | 5.0  |
| `min_safe_distance_backward`（新增） | `lane_change_path/conf/default_conf.pb.txt` | 10.0（硬编码） | 15.0 |
| `safe_time_same_direction`（新增）   | `lane_change_path/conf/default_conf.pb.txt` | 3.0（硬编码）  | 5.0  |
| `lane_change_speed_limit`（新增）    | `lane_change_path/conf/default_conf.pb.txt` | 无             | 8.33 |

### 3.4 风险与缓解

| 风险                           | 缓解                                                      |
| ------------------------------ | --------------------------------------------------------- |
| proto 新增字段需重编译         | 提前规划编译时间                                          |
| 限速后安全距离计算变化         | 公式 `max(min_dist, (v_obs-8.33)×time)` 需实际地图验证 |
| lane_change_util.cc 有代码重复 | 两处 IsClearToChangeLane 需同步修改                       |

---

## 赛题三：xh_2026_S弯场景

### 4.1 现有能力评估

- **LaneFollow 场景**：默认场景，使用 PiecewiseJerkPathProblem (QP) 进行路径优化，可处理连续弯道。
- **LargeCurvature 场景**：触发条件 `|kappa|>0.2`（从 proto 读取）且剩余距离<10m（**硬编码，proto 中无对应字段**）+ 前视距离 3.0m（**硬编码**）。仅路线末端触发。
- **曲率限速公式**：$v_{\text{limit}} = \sqrt{a_{\text{max}} / \max(|\kappa|, \kappa_{\text{min}})}$，`max_centric_acceleration_limit` 当前默认 0.8。
- **横向安全距离**：`adc_half_width + obstacle_lat_buffer(0.4) + static_obstacle_nudge_l_buffer(0.3)` ≈ 1.7m。

### 4.2 解题策略

**纯配置方案。**

1. **限速 30km/h**：`planning_upper_speed_limit` 从 20.00 改为 8.33。
2. **曲率限速放宽**：`max_centric_acceleration_limit` 从 0.8 改为 2.0（约 0.2g，审查确认安全）。
3. **横向安全距离增大**：`static_obstacle_nudge_l_buffer` 0.3→0.6，`obstacle_lat_buffer` 0.4→0.6。
4. **路径平滑增强**（可选）：增大 PiecewiseJerk 的 weight_ddl/weight_dddl 参数。

### 4.3 参数变更

| 参数                               | 文件                                              | 旧值  | 新值 |
| ---------------------------------- | ------------------------------------------------- | ----- | ---- |
| `planning_upper_speed_limit`     | `planning.conf`                                 | 20.00 | 8.33 |
| `default_cruise_speed`           | `planning.conf`                                 | 11.18 | 8.33 |
| `max_centric_acceleration_limit` | `speed_bounds_decider/conf/default_conf.pb.txt` | 0.8   | 2.0  |
| `static_obstacle_nudge_l_buffer` | `planning.conf`（新增 gflag）                   | 0.3   | 0.6  |
| `obstacle_lat_buffer`            | `planning.conf`（新增 gflag）                   | 0.4   | 0.6  |

### 4.4 风险与缓解

| 风险                                      | 缓解                                                |
| ----------------------------------------- | --------------------------------------------------- |
| `planning_upper_speed_limit` 是全局限制 | 如直道需更高速度，用区域限速替代全局限速            |
| LargeCurvature IsTransferable 硬编码值    | 10m 剩余距离限制使场景只在末端触发，不影响中途 S 弯 |

---

## 赛题四：xh_2026_U-Turn场景

### 5.1 现有能力评估

- **Hybrid A\* 粗轨迹生成**：`Plan(sx, sy, sphi, ex, ey, ephi)` 接受任意 4D 位姿，支持航向翻转 180°。
- **轨迹平滑器**：`IterativeAnchoringSmoother` + `FemPosDeviationSmoother`（**QP/OSQP 求解器**），终点航向通过软约束锚点逼近。`DistanceApproachIPOPT` 存在但是另一套独立方案，**非当前管线使用**。
- **NARROW_STREET_U_TURN 枚举**：已在 proto 中定义（值=4），但 `Process()` 中**确认无对应 case 分支**（仅有 PARKING/PULL_OVER/PARK_AND_GO/LARGE_CURVATURE + else 报错）。
- **LargeCurvature 场景**：`GetUTurnPath()` + `GetLargeCurvatureEndPose()` 函数骨架存在，但 `GetLargeCurvatureEndPose()` 不翻转航向角。

### 5.2 解题策略

**扩展 large_curvature 场景 + 新增 NARROW_STREET_U_TURN ROI 分支。**

1. **IsTransferable() 扩展**：检测 routing 响应中存在 >150° 航向变化（参考线"掉头"），或检测对向 lane。
2. **Process() 补充 case**：为 NARROW_STREET_U_TURN 新增分支，复用 `GetUTurnPath()` + `GetLargeCurvatureBoundary()` + `GetLargeCurvatureEndPose()`。
3. **终点位姿**：`GetLargeCurvatureEndPose()` 终点航向 = 路径点 heading（非起点+π），需增加航向翻转修正。
4. **对向来车检测**：使用 `obstacle->Perception().velocity()` 和 `theta()` 计算瞬时 TTC，TTC<5s 则零速等待。
5. **速度限制**：不建议全局提升 `max_forward_v` 到 8.33。应在 proto 中为 NARROW_STREET_U_TURN 添加独立 `max_forward_v`（建议 5.0 m/s ≈ 18km/h），或通过 `ReferenceLine::AddSpeedLimit()` 区域限速。
6. **终点航向容差**：接受 <5° 偏差，终点后衔接短直线路径纠正航向。

### 5.3 参数变更

| 参数                             | 文件                                  | 旧值      | 新值      |
| -------------------------------- | ------------------------------------- | --------- | --------- |
| `max_explored_num`             | `planner_open_space_config.proto`   | 10000     | 20000     |
| `traj_gear_switch_penalty`     | `planner_open_space_config.proto`   | 10.0      | 5.0       |
| `u_turn_max_forward_v`（新增） | `open_space_roi_decider_park.proto` | 无        | 5.0       |
| `roi_longitudinal_range`       | `open_space_roi_decider_park.proto` | 10.0/10.0 | 15.0/15.0 |

### 5.4 风险与缓解

| 风险                         | 缓解                                                           |
| ---------------------------- | -------------------------------------------------------------- |
| Hybrid A\* 搜索超时（100ms） | 设置 `astar_max_search_time=0.08s`，启用并行搜索             |
| 终点航向偏差                 | 衔接短直线纠正，放宽航向阈值至 0.15rad                         |
| 对向来车误判                 | 增加速度阈值(>1m/s) + 结合 lane 方向二次确认                   |
| U-Turn 完成后场景切换冲突    | 增加退出条件（航向已翻转 + 曲率正常 → 允许 lane_follow 接管） |

---

## 赛题五：xh_2026_施工区域通行场景

### 6.1 现有能力评估

- **SpeedSetting TrafficRule**：仅处理外部指令巡航速度，不实现区域限速。
- **LaneBorrowPath**：当前仅支持通用借道（左借/右借相邻车道），无施工区域逻辑。
- **旧版代码**：`.oldcode/Apollo-wzr/modules/planning/tasks/lane_borrow_path/` 包含完整的施工区域锥桶处理逻辑。
- **ReferenceLine::AddSpeedLimit()**：原生支持区间限速注入。

### 6.2 解题策略

**将旧版施工区域逻辑移植到当前 LaneBorrowPath。**

1. **锥桶识别**：`IsConstructionSmallObstacle()` — 通过 polygon 面积<0.5m² 识别（审查建议增加高度/长宽比约束）。
2. **construct_zone 状态机**：≥3 锥桶进入施工区模式。
3. **锥桶历史记忆**：`cone_history_` 记录历史锥桶位置，处理感知闪烁。
4. **施工区限速**：`AddSpeedLimit(cone_min_s-10m, cone_max_s+10m, 8.33)`。
5. **路径边界**：`GetConstructZoneBoundary()` 生成跨全部可用车道的双向路径边界。
6. **曲率门控**：仅直道（前方 100m 最大曲率<0.01）启用施工区模式。
7. **倒车恢复**：车辆紧贴锥桶时自动倒车拉开距离（审查补充的遗漏点）。
8. **驶出检测**：`no_cone_counter_>8` 帧 + 驶过最远锥桶 → 恢复正常限速。

### 6.3 参数变更

| 参数                                            | 文件                                          | 旧值 | 新值 |
| ----------------------------------------------- | --------------------------------------------- | ---- | ---- |
| `enable_construction_curvature_gate`（新增）  | `lane_borrow_path/conf/default_conf.pb.txt` | 无   | true |
| `construction_straight_max_abs_kappa`（新增） | `lane_borrow_path/conf/default_conf.pb.txt` | 无   | 0.01 |
| `reverse_distance`（新增）                    | `lane_borrow_path/conf/default_conf.pb.txt` | 无   | 8.0  |
| 施工区限速                                      | 代码中 `AddSpeedLimit` 注入                 | 无   | 8.33 |

### 6.4 风险与缓解

| 风险                         | 缓解                                   |
| ---------------------------- | -------------------------------------- |
| 锥桶面积识别误触发（小石块） | 增加高度范围(0.3-1.2m) + 长宽比约束    |
| 弯道锥桶 SL 投影失真         | 曲率门控强制仅在直道启用               |
| 倒车逻辑合规性               | 可通过 config 参数 disable             |
| Proto 字段大量新增需重编译   | 分阶段移植：先核心锥桶识别，后倒车恢复 |

---

## 赛题六：xh_2026_站点接驳场景

### 7.1 现有能力评估（源码核实）

**PullOver 场景**（pipeline.pb.txt 确认 3 Stage）：
| Stage | Type | Enabled | 功能 |
|-------|------|---------|------|
| `PULL_OVER_APPROACH` | `PullOverStageApproach` | ✅ true | PullOverPath 路径规划 + 精准泊入 |
| `PULL_OVER_RETRY_APPROACH_PARKING` | `PullOverStageRetryApproachParking` | ❌ **false** | OpenSpacePreStop 减速（默认禁用）|
| `PULL_OVER_RETRY_PARKING` | `PullOverStageRetryParking` | ✅ true | OpenSpace (Hybrid A\*) 精确泊入 |

**关键代码逻辑**（`stage_approach.cc:73-76`）：
```cpp
if (state == PullOverState::PARK_COMPLETE) {
    return FinishStage(true);  // → FinishScenario() 直接退出场景！
}
```
PARK_COMPLETE 后直接 `FinishScenario()`，**无任何等待机制**。

**ParkAndGo 场景**（pipeline.pb.txt 确认 **4 Stage**，非 5）：
| Stage | Type | 功能 |
|-------|------|------|
| `PARK_AND_GO_CHECK` | `ParkAndGoStageCheck` | 环境检查 + 位置初始化 |
| `PARK_AND_GO_ADJUST` | `ParkAndGoStageAdjust` | OpenSpace 姿态调整 |
| `PARK_AND_GO_PRE_CRUISE` | `ParkAndGoStagePreCruise` | OpenSpace 预巡航 |
| `PARK_AND_GO_CRUISE` | `ParkAndGoStageCruise` | LaneFollow 正常驶离 |

**🚨 关键发现：ParkAndGo 完全不能用于站点驶离！**

```cpp
// park_and_go_scenario.cc:100-104 — IsTransferable() 条件：
if (adc_speed < max_abs_speed_when_stopped &&
    adc_distance_to_dest > min_dist_to_dest() &&
    (!is_ego_on_lane || !is_lane_type_city_driving)) {  // ← 必须在非 CITY_DRIVING 车道！
    park_and_go = true;
}
```

ParkAndGo 设计要求车辆**不在车道内**或**不在 CITY_DRIVING 类型车道**上才能切入。站点接驳的泊车位通常在正常道路上（CITY_DRIVING），ParkAndGo 根本不会触发。之前方案"复用 ParkAndGo 驶离"完全不可行。

### 7.2 解题策略（修正版）

**在 PullOver 内部新增 2 个 Stage，完全放弃 ParkAndGo。**

核心原因：ParkAndGo 的 `IsTransferable()` 要求 `!is_lane_type_city_driving`（不在城市驾驶车道上），站点泊位通常在正常道路，无法切入。

1. **PULL_OVER_APPROACH（改）**：添加 `LimitCruiseSpeed(8.33)`。PARK_COMPLETE 后不调用 `FinishScenario()`，改为跳转 WAITING Stage。
2. **PULL_OVER_WAITING（新建）**：参考 `EmergencyPullOverStageStandby`，`BuildStopDecision` 构建 stop fence。记录首次进入时间戳，累积 ≥5.0s 后跳转 DEPARTURE。等待期间持续碰撞检测。
3. **PULL_OVER_DEPARTURE（新建）**：使用 LaneFollow Task 链（`LaneFollowPath + LaneBorrowPath + FallbackPath + PathDecider + SpeedBounds + PiecewiseJerkSpeed`）生成驶离轨迹。检测 adc 到达 routing 起点（`dest_sl.s() - adc_s > 5m`）后 `FinishScenario()`。**
4. **压线避免**：`pull_over_road_edge_buffer` 0.15→0.3。停车精度 `max_l_error` 1.0→0.5。

**Pipeline 修改**：
```
原: APPROACH → RETRY_APPROACH_PARKING → RETRY_PARKING
新: APPROACH(改) → WAITING(新, enabled:true) → DEPARTURE(新, enabled:true) → RETRY_APPROACH_PARKING → RETRY_PARKING
```

APPROACH 中：
- `PARK_COMPLETE` → `next_stage_ = "PULL_OVER_WAITING"`（不再 FinishScenario）
- `PARK_FAIL` → 保持原有跳转 RETRY_APPROACH_PARKING

### 7.3 参数变更

| 参数                            | 文件                                    | 旧值 | 新值 |
| ------------------------------- | --------------------------------------- | ---- | ---- |
| `waiting_duration`（新增）    | `pull_over.proto`                     | 无   | 5.0  |
| `station_speed_limit`（新增） | `pull_over.proto`                     | 无   | 8.33 |
| `pull_over_road_edge_buffer`  | `pull_over_path/conf/`                | 0.15 | 0.3  |
| `max_l_error_to_end_point`    | `pull_over/conf/scenario_conf.pb.txt` | 1.0  | 0.5  |

### 7.4 风险与缓解

| 风险                   | 缓解                                                 |
| ---------------------- | ---------------------------------------------------- |
| 精准停车失败无限重试   | 增加重试上限（10 次）或超时（30s）                   |
| 等待期间动态障碍物逼近 | WAITING Stage 中实时碰撞检测，进入警戒范围则延长等待 |
| 站点地图泊位标注缺失   | 地图预先标注或 routing 请求中指定终点为泊位坐标      |

---

## 赛题七：xh_2026_环岛场景

### 8.1 现有能力评估

⚠️ **审查发现**：Apollo EDU 版本 HDMap 中**不存在 JunctionType 枚举或 roundabout 类型标记**。环岛通过 `PNC_JUNCTION` overlap 体现（与普通路口相同），无法从地图直接区分。

可复用组件：

- **StopSignUnprotected**：4 Stage 停车让行+蠕行框架。
- **BareIntersectionUnprotected**：`CheckClear()` S-T 图安全间隙判断。
- **LaneFollow**：环内车道跟随。
- **SetTurnSignal**：转向灯控制（`ReferenceLineInfo::SetTurnSignal()`）。
- **BacksideVehicle**：需扩展为 roundabout_exit 模式（当前仅 ignore 后方车辆）。

### 8.2 解题策略

**新建 Roundabout Scenario（3 Stage），使用外部 GeoFence 配置文件识别环岛。**

1. **环岛检测（核心难点）**：使用外部配置文件指定环岛区域（GeoFence polygon + 关联 lane ID），在 Scenario 初始化时加载。
2. **ENTRY Stage**：复用 StopSign 停车+蠕行，watch_vehicles 通过环岛内部车道 ID 集合筛选环内车辆。
3. **CIRCULATING Stage**：复用 LaneFollow Task 链，`LimitCruiseSpeed(8.33)`，nudge 横向 ≥1m。
4. **EXIT Stage**：`SetTurnSignal(TURN_RIGHT)`，扩展 BacksideVehicle 检测出口后方来车，安全距离<15m 则等待。
5. **场景注册**：`Roundabout` 需在 `public_road_planner_config.pb.txt` 中排在 `BareIntersection` 之前。

### 8.3 参数变更

| 参数                                           | 文件                                     | 旧值 | 新值  |
| ---------------------------------------------- | ---------------------------------------- | ---- | ----- |
| `start_roundabout_scenario_distance`（新增） | `roundabout/conf/scenario_conf.pb.txt` | 无   | 25.0  |
| `stop_duration_sec`（新增）                  | `roundabout/conf/scenario_conf.pb.txt` | 无   | 1.0   |
| `roundabout_speed_limit`（新增）             | `roundabout/conf/scenario_conf.pb.txt` | 无   | 8.333 |
| `turn_signal_distance`（新增）               | `roundabout/conf/scenario_conf.pb.txt` | 无   | 30.0  |
| `exit_rear_check_distance`（新增）           | `roundabout/conf/scenario_conf.pb.txt` | 无   | 15.0  |
| `static_obstacle_nudge_l_buffer`             | `obstacle_nudge_decider/conf/`         | 0.3  | 1.0   |

### 8.4 风险与缓解

| 风险                               | 缓解                                                               |
| ---------------------------------- | ------------------------------------------------------------------ |
| HDMap 无 roundabout 类型           | 使用外部 GeoFence 配置文件指定环岛区域                             |
| 场景优先级与 BareIntersection 冲突 | Roundabout 排在 BareIntersection 之前，IsTransferable 增加互斥逻辑 |
| 环内车辆感知盲区                   | 依赖 V2X 或地图预警，增加保守的安全间隙                            |
| 出口转向灯时机不当                 | 离目标出口前 20-30m 触发，结合 lane turn type 判断                 |
| 工作量被低估                       | 审查建议上调至**VERY HIGH**                                  |

---

## 九、跨赛题共性修改建议

以下参数被多个赛题使用，建议一次性统一配置：

| 共性需求                    | 涉及赛题         | 统一方案                                                                                             |
| --------------------------- | ---------------- | ---------------------------------------------------------------------------------------------------- |
| **限速 30km/h**       | 2, 3, 4, 5, 6, 7 | `planning_upper_speed_limit=8.33` 作为全局上限，各场景内通过 `LimitCruiseSpeed()` 进一步精确控制 |
| **横向安全距离 ≥1m** | 3, 4, 5, 7       | 统一增大 `static_obstacle_nudge_l_buffer` 和 `obstacle_lat_buffer`                               |
| **障碍物绕行增强**    | 3, 4, 5, 7       | 确保 ObstacleNudgeDecider 和 LaneBorrowPath 的 buffer 参数一致                                       |
| **场景注册与优先级**  | 1, 7             | 统一在 `public_road_planner_config.pb.txt` 中管理场景顺序                                          |
| **Proto 重新编译**    | 2, 4, 5, 6       | 建议一次性修改所有 proto，减少编译次数                                                               |

---

## 十、任务完成报告

| 指标              | 数值                                                                                                                |
| ----------------- | ------------------------------------------------------------------------------------------------------------------- |
| 分析轮子 Agent 数 | 7                                                                                                                   |
| 审查轮子 Agent 数 | 3                                                                                                                   |
| 分析轮全部成功    | ✅                                                                                                                  |
| 审查轮全部成功    | ✅                                                                                                                  |
| 失败/重试次数     | 0                                                                                                                   |
| 发现的关键偏差    | 赛题 2（工作量低估 + freeze_time 调低）、赛题 3（LargeCurvature IsTransferable 前视距离 3.0m 和剩余距离 10.0m 均为硬编码，proto 中无对应字段）、赛题 4（平滑器非 IPOPT → IterativeAnchoringSmoother + FemPosDeviationSmoother(QP/OSQP)）、赛题 4（Process() 中确认无 NARROW_STREET_U_TURN case）、**赛题 6（ParkAndGo 仅 4 Stage 且 IsTransferable 要求非 CITY_DRIVING 车道 → 完全不可用于站点驶离）**、赛题 6（RETRY_APPROACH_PARKING 默认 disabled）、赛题 7（HDMap 无环岛标记 → 需 GeoFence；`modules/map/` 目录不在工作区） |
