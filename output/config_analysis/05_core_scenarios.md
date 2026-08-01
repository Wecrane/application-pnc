# 05 核心道路场景配置深度分析

> 工程：`/home/skye/application-pnc`（Apollo 11.0 EDU，官方原始版本）
> 分析对象：`modules/planning/scenarios/` 下 10 个核心场景的 `pipeline.pb.txt` / `scenario_conf.pb.txt` / stage 子配置
> 方法：每个参数 → proto 默认值 → 代码定位（`modules/planning/` 内 .cc 文件）→ 作用说明 → 对车辆行为影响
> 赛题背景标注：🎯=与 8 大赛题直接相关

---

## 0. 场景注册与优先级（public_road_planner_config.pb.txt）

路径：`modules/planning/planning_component/conf/public_road_planner_config.pb.txt`

该文件是 PublicRoadPlanner 的场景注册表，**顺序即优先级**（ScenarioManager 按顺序调用 `IsTransferable`，命中即切入，不再检查后续场景）：

| # | name | type（C++ 类） | 触发条件（IsTransferable 摘要） |
|---|------|---------------|-------------------------------|
| 1 | EMERGENCY_PULL_OVER | EmergencyPullOverScenario | Pad 消息 `PULL_OVER` |
| 2 | EMERGENCY_STOP | EmergencyStopScenario | Pad 消息 `STOP` |
| 3 | VALET_PARKING | ValetParkingScenario | 泊车指令 |
| 4 | BARE_INTERSECTION_UNPROTECTED | BareIntersectionUnprotectedScenario | 前方有 PNC_JUNCTION 且无信号灯/停止牌/让行牌，且距离≤25m，路口无路权 |
| 5 | STOP_SIGN_UNPROTECTED | StopSignUnprotectedScenario | 首个 encounter overlap 为 STOP_SIGN 且距离≤4m |
| 6 | YIELD_SIGN | YieldSignScenario | 首个 overlap 为 YIELD_SIGN 且距离≤10m |
| 7 | TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN | TrafficLightUnprotectedLeftTurnScenario | 前方信号灯非绿/黑且路径转向为 LEFT_TURN |
| 8 | TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN | TrafficLightUnprotectedRightTurnScenario | 前方信号灯非绿/黑且路径转向为 RIGHT_TURN |
| 9 | TRAFFIC_LIGHT_PROTECTED | TrafficLightProtectedScenario | 前方信号灯非绿/黑（直行/任意转向兜底） |
| 10 | PULL_OVER | PullOverScenario | 常规靠边停车指令 |
| 11 | PARK_AND_GO | ParkAndGoScenario | 起停 |
| 12 | LANE_FOLLOW | LaneFollowScenario | **兜底默认场景**，`IsTransferable` 恒为 true |

**关键发现：**
- `SQUARE` 场景**未注册**在本文件中（虽然存在 `square/` 目录和 pipeline），默认不参与运行。若赛事需要启用需在此文件追加注册。
- 红绿灯三兄弟的优先级顺序：**左转 > 右转 > 直行（protected）**，靠 `IsTransferable` 中的 `GetPathTurnType` 区分。
- 裸路口（bare）优先级高于停止牌/让行牌/红绿灯，但其 `IsTransferable` 明确排除有 SIGNAL/STOP_SIGN/YIELD_SIGN 的情况，故互斥不冲突。

---

## 1. LANE_FOLLOW（默认场景，重点）

- 目录：`scenarios/lane_follow/conf/pipeline.pb.txt`
- 场景类：`LaneFollowScenario`（`lane_follow_scenario.cc`），**无 scenario_conf.pb.txt / 无 proto 配置**
- 作用：所有其他场景都不命中时的兜底场景。`IsTransferable` 只要有 `lane_follow_command` 且参考线非空即返回 true。
- Stage 类：`LaneFollowStage`（`lane_follow_stage.cc`）——对每条参考线依次执行 task_list_，选出可行驶参考线（变道参考线成本<10 才采用），出错时执行 fallback_task_。

### Pipeline（Stage/Task 链）

| Stage name | type | enabled |
|-----------|------|:---:|
| LANE_FOLLOW_STAGE | LaneFollowStage | true |

| # | Task name | Task type | 作用 |
|---|-----------|-----------|------|
| 1 | LANE_CHANGE_PATH | LaneChangePath | 生成变道路径（打转向灯、目标车道路径），仅当存在变道意图 |
| 2 | LANE_FOLLOW_PATH | LaneFollowPath | 沿当前车道生成常规路径（lane keep 主路径） |
| 3 | LANE_BORROW_PATH | LaneBorrowPath | 借道绕行路径（压线借相邻车道避让静态障碍） |
| 4 | FALLBACK_PATH | FallbackPath | 路径兜底（生成直行最短路径，防止无路径可用） |
| 5 | PATH_DECIDER | PathDecider | 路径决策：为静态障碍物加横向 nudge/绕行/停止决策，设置静态障碍物 buffer |
| 6 | RULE_BASED_STOP_DECIDER | RuleBasedStopDecider | 规则停止决策：综合信号灯/停止牌/让行牌/人行道/施工区等 TrafficRule 产生的虚拟障碍物，生成纵向 STOP 决策 |
| 7 | SPEED_BOUNDS_PRIORI_DECIDER | SpeedBoundsDecider | 先验速度边界（ST 边界粗算，含限速、停止、跟车、让行） |
| 8 | SPEED_HEURISTIC_OPTIMIZER | PathTimeHeuristicOptimizer | DP 速度启发式（沿 s 采样做动态规划，得到粗糙速度曲线） |
| 9 | SPEED_DECIDER | SpeedDecider | 速度决策（基于 ST 图给障碍物分配跟/停/让/超车决策） |
| 10 | SPEED_BOUNDS_FINAL_DECIDER | SpeedBoundsDecider | 最终速度边界（结合决策结果精化 ST 边界） |
| 11 | PIECEWISE_JERK_SPEED | PiecewiseJerkSpeedOptimizer | QP 分段 jerk 速度优化（最终光滑速度曲线） |
| — | ~~PIECEWISE_JERK_SPEED_NONLINEAR~~ | PiecewiseJerkSpeedNonlinearOptimizer | **被注释**：非线性速度优化备选（官方默认不启用） |

### 说明

- 该 pipeline 是**所有道路场景 Task 链的"母版"**：其余场景的 Approach/Creep/Stop 等 Stage 大多是在此链基础上增删 Task（见下文各场景）。
- 注释掉的 `PIECEWISE_JERK_SPEED_NONLINEAR` 表明官方默认用线性 QP（PIECEWISE_JERK_SPEED）而非非线性优化。
- 本场景无参数文件，行为完全由 gflag 控制，与赛题相关的关键 gflag（`planning_base/gflags/planning_gflags.cc`）：
  - `default_cruise_speed = 5.0 m/s`（18 km/h，默认巡航限速）
  - `max_stop_distance_obstacle = 10.0`、`min_stop_distance_obstacle = 6.0`（同车道障碍物停车距离上下限，🎯 障碍物停车≥2m 相关）
  - `follow_min_distance = 3.0`、`follow_time_buffer = 2.5`（动态跟随）
  - `speed_bump_speed_limit = 4.4704 m/s`（🎯 减速带限速，当前 10 mph≈16 km/h，赛题要求≤3m/s 需调小）

---

## 2. TRAFFIC_LIGHT_PROTECTED（红绿灯直行，重点）

- 目录：`scenarios/traffic_light_protected/`
- 场景类：`TrafficLightProtectedScenario`（`traffic_light_protected_scenario.cc`）

### Pipeline（Stage/Task 链）

| # | Stage name | type | enabled |
|---|-----------|------|:---:|
| 1 | TRAFFIC_LIGHT_PROTECTED_APPROACH | TrafficLightProtectedStageApproach | true |
| 2 | TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE | TrafficLightProtectedStageIntersectionCruise | true |

两个 Stage 的 Task 链完全相同（各 11 个，比 lane_follow 少了 LANE_CHANGE_PATH，多了 ST_BOUNDS_DECIDER）：

| # | Task name | Task type | 说明 |
|---|-----------|-----------|------|
| 1 | LANE_FOLLOW_PATH | LaneFollowPath | 沿车道路径 |
| 2 | LANE_BORROW_PATH | LaneBorrowPath | 借道路径 |
| 3 | FALLBACK_PATH | FallbackPath | 路径兜底 |
| 4 | PATH_DECIDER | PathDecider | 路径决策 |
| 5 | RULE_BASED_STOP_DECIDER | RuleBasedStopDecider | 规则停止（含交通灯规则） |
| 6 | ST_BOUNDS_DECIDER | STBoundsDecider | **ST 边界决策**（红灯停止线/障碍物 ST 边界，左转/右转/本场景独有） |
| 7 | SPEED_BOUNDS_PRIORI_DECIDER | SpeedBoundsDecider | 先验速度边界 |
| 8 | SPEED_HEURISTIC_OPTIMIZER | PathTimeHeuristicOptimizer | DP 速度 |
| 9 | SPEED_DECIDER | SpeedDecider | 速度决策 |
| 10 | SPEED_BOUNDS_FINAL_DECIDER | SpeedBoundsDecider | 最终速度边界 |
| 11 | PIECEWISE_JERK_SPEED | PiecewiseJerkSpeedOptimizer | QP 速度 |

### 参数详解（scenario_conf.pb.txt）

#### 2.1 `start_traffic_light_scenario_distance`
- **当前值**：`5.0`　**proto 默认值**：`5.0`（`traffic_light_protected.proto`，单位 m）
- **代码位置**：`traffic_light_protected_scenario.cc::IsTransferable`
  ```cpp
  const double start_check_distance = context_.scenario_config.start_traffic_light_scenario_distance();
  // 仅当 0 < adc_distance_to_traffic_light <= start_check_distance 且信号非绿/黑 才切入本场景
  ```
- **作用**：车辆前缘距信号灯 overlap（停止线）的**切入距离**。超出该距离不切场景；进入范围内且灯非绿/黑（红/黄/未知）则切入。
- **对车辆行为影响**：值越小，越靠近路口才切换到红绿灯场景。因 protected 场景的 Approach 阶段**不主动限速**（依赖 traffic_light 规则在红灯时生成停止决策），此值主要决定场景切换时机，不影响停车距离。
- 🎯 与赛题"绿灯路口≤5m/s"间接相关（若需路口前提前限速，需关注 unprotected 的 `approach_cruise_speed` 或 traffic_light 规则，本参数本身不限制速度）。

#### 2.2 `max_valid_stop_distance`
- **当前值**：`2.0`　**proto 默认值**：`2.0`（单位 m）
- **代码位置**：`stage_approach.cc::Process`
  ```cpp
  if (distance_adc_to_stop_line > scenario_config.max_valid_stop_distance()) {
    traffic_light_all_done = false;  // 未到停止线附近，继续 APPROACH
  }
  // 且要求 signal_color == GREEN 才算 done
  ```
- **作用**：Approach 阶段的**结束判定**——当车辆前缘距停止线 **≤ 2.0m 且信号灯为绿灯**时，`FinishStage` 进入 IntersectionCruise。
- **对车辆行为影响**：该参数**不是**停车距离（停车距离由 traffic_light 规则的 `stop_distance=1.0m` 控制，见下），而是"贴近停止线后进入巡航"的判定阈值。若红灯且已停在停止线前 2m 内，则一直停在 APPROACH 等绿灯。
- 🎯 **与赛题"红绿灯停车1.5-2.0m"强相关**：实际停车点 = 停止线位置 - `traffic_light` 规则的 `stop_distance`。

#### 2.3 关联的 traffic_light 规则配置（红灯停车距离，重要！）
- 路径：`modules/planning/traffic_rules/traffic_light/conf/default_conf.pb.txt`
  ```
  enabled: true
  stop_distance: 1.0      # 🎯 红灯/黄灯时停车点距停止线距离（默认1.0m）
  max_stop_deceleration: 4.0   # 超过此减速度则放弃红灯停车（防急刹）
  ```
- **代码位置**：`traffic_rules/traffic_light/traffic_light.cc`
  ```cpp
  util::BuildStopDecision(virtual_obstacle_id, traffic_light_overlap.start_s,
                          config_.stop_distance(), STOP_REASON_SIGNAL, ...);
  ```
- **作用**：红灯/黄灯/未知灯色时在停止线前 `stop_distance` 处建立虚拟障碍物（停止墙）。绿灯/黑灯跳过。
- 🎯 **这是红绿灯场景真正的"停车距离"参数**：红灯停车点距停止线 = 1.0m。赛题要求 1.5-2.0m，**需要修改 `traffic_rules/traffic_light/conf/default_conf.pb.txt` 的 `stop_distance`**。

### Stage 行为

- **APPROACH**：执行 Task 链；对每个灯检查"距停止线≤max_valid_stop_distance 且绿灯"→ FinishStage → `TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE`。红灯时由 traffic_light 规则停车等待；不主动限速。
- **INTERSECTION_CRUISE**：`stage_intersection_cruise.cc` 调 `BaseStageCruise::CheckDone(frame, ctx, true)`——若参考线上无 junction，则车尾越过信号灯 overlap **20m** 后 FinishScenario；若在 junction 内则持续巡航直到离开路口。`right_of_way_status=true`（绿灯有路权）。

---

## 3. TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN（红绿灯左转）

- 目录：`scenarios/traffic_light_unprotected_left_turn/`
- 场景类：`TrafficLightUnprotectedLeftTurnScenario`（`traffic_light_unprotected_left_turn_scenario.cc`），`IsTransferable` 额外要求 `GetPathTurnType == LEFT_TURN`

### Pipeline（Stage/Task 链）

| # | Stage name | type | enabled |
|---|-----------|------|:---:|
| 1 | TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_APPROACH | TrafficLightUnprotectedLeftTurnStageApproach | true |
| 2 | TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP | TrafficLightUnprotectedLeftTurnStageCreep | true |
| 3 | TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE | TrafficLightUnprotectedLeftTurnStageIntersectionCruise | true |

三个 Stage 的 Task 链相同（同 protected：11 个 Task，含 ST_BOUNDS_DECIDER）。

### 参数详解（scenario_conf.pb.txt）

#### 3.1 `start_traffic_light_scenario_distance`
- **当前值**：`30.0`　**proto 默认值**：`5.0`（m）
- **代码位置**：`traffic_light_unprotected_left_turn_scenario.cc::IsTransferable`（同 protected 用法）
- **作用**：切入距离，**当前被调大到 30m**（默认 5m）——提前 30m 就进入左转场景，更早开始减速。
- **对车辆行为影响**：值越大越早进入 Approach 阶段并触发 `approach_cruise_speed` 限速，接近路口更安全；但过早限速可能拖慢通行。🎯 与赛题路口限速相关。

#### 3.2 `approach_cruise_speed` 🎯
- **当前值**：`2.78`　**proto 默认值**：`2.78`（m/s，=10 km/h）
- **代码位置**：`stage_approach.cc::Process`
  ```cpp
  frame->mutable_reference_line_info()->front().LimitCruiseSpeed(scenario_config.approach_cruise_speed());
  ```
- **作用**：Approach 阶段的**巡航限速**。进入左转场景后立即把限速压到 2.78 m/s，让车提前减速驶向停止线。
- **对车辆行为影响**：直接决定接近路口的车速。10 km/h 是稳妥值。🎯 赛题"绿灯路口≤5m/s"——左转场景已满足（2.78<5），直行/右转场景见各自参数。

#### 3.3 `max_valid_stop_distance`
- **当前值**：`2.0`　**proto 默认值**：`3.5`（m，**当前值比默认小**）
- **代码位置**：`stage_approach.cc::Process`
  ```cpp
  if (signal_color != TrafficLight::GREEN ||
      distance_adc_to_stop_line >= scenario_config.max_valid_stop_distance()) {
    traffic_light_all_done = false;
  }
  ```
- **作用**：Approach 结束条件——**绿灯且前缘距停止线 < 2.0m** 才结束。当前值比默认 3.5m 更苛刻，车需更贴近停止线才进入下一阶段。
- **对车辆行为影响**：值越小，绿灯起步前越靠近停止线。🎯 相关"红绿灯停车1.5-2.0m"（配合 traffic_light 规则 `stop_distance`）。

#### 3.4 `creep_timeout_sec`
- **当前值**：`10.0`　**proto 默认值**：`10.0`（s）
- **代码位置**：`stage_creep.cc::Process` → `CheckCreepDone(frame, rfl, overlap_end_s, wait_time, timeout_sec)`
- **作用**：蠕行超时。蠕行时间 ≥ 该值（或已越过蠕行终点）即认为蠕行完成，进入 IntersectionCruise。
- **对车辆行为影响**：限制无保护左转在路口蠕行探头的最大时长，避免长时间堵在路口。蠕行速度由 `GenerateFixedDistanceCreepProfile` 控制（固定小距离蠕行）。

#### 3.5 `max_adc_speed_before_creep` 🎯
- **当前值**：`5.56`　**proto 默认值**：`5.56`（m/s，=20 km/h）
- **代码位置**：`stage_approach.cc::FinishStage`
  ```cpp
  if (adc_speed > scenario_config.max_adc_speed_before_creep()) {
    next_stage_ = "...INTERSECTION_CRUISE";   // 速度仍较快 → 跳过蠕行
  } else {
    next_stage_ = "...CREEP";                 // 已低速 → 进蠕行
  }
  ```
- **作用**：Approach 结束瞬间车速的阈值——若车速 > 5.56 m/s 说明车没停稳（直接通过了路口），跳过蠕行直接巡航；否则进入蠕行阶段。
- **对车辆行为影响**：值越大越容易跳过蠕行。左转 5.56 vs 右转 3.0 的差异反映"左转更倾向于直接通过、右转更倾向于停车蠕行"的默认策略。🎯 与路口停车/通过策略相关。

#### 3.6 `creep_stage_config`（`CreepStageConfig`，见 `planning_interface_base/scenario_base/proto/creep_stage.proto`）
- **当前值**：`min_boundary_t=6.0, ignore_max_st_min_t=0.1, ignore_min_st_min_s=15.0`
- **proto 默认值**：`6.0 / 0.1 / 15.0`（当前值=默认值）
- **代码位置**：`planning_interface_base/scenario_base/base_stage_creep.cc::CheckCreepDone`
  ```cpp
  if (obstacle->reference_line_st_boundary().min_t() < creep_config.min_boundary_t()) {
    // 障碍物 6s 内会到达本车区域 → 视为"前方有车"
    if (obstacle_traveled_s < kepsilon &&
        min_t < creep_config.ignore_max_st_min_t() &&       // 0.1s 内到达
        min_s > creep_config.ignore_min_st_min_s()) {       // 且已在本车前方15m外
      continue;  // 同向且足够远 → 忽略
    }
    all_far_away = false;
  }
  // all_far_away 连续 5 帧 → 蠕行完成
  ```
- **作用**：蠕行期间判断路口是否"清空"的 ST 边界阈值：
  - `min_boundary_t=6.0`：忽略 min_t ≥ 6s 的障碍物（6s 后才到，不影响蠕行）
  - `ignore_max_st_min_t=0.1`、`ignore_min_st_min_s=15.0`：忽略"已在本车同向前方 15m 外、且 0.1s 内到达"的同向运动障碍物
- **对车辆行为影响**：决定蠕行时对横向/对向车辆的"让行敏感度"。值越大对障碍物越保守（更不容易完成蠕行）。

### Stage 行为

- **APPROACH**：`LimitCruiseSpeed(2.78)` 限速接近；绿灯且距停止线<2m → FinishStage；按 `max_adc_speed_before_creep` 决定进 Creep 或直接 Cruise。
- **CREEP**：`ProcessCreep` 在 overlap_end+4m 处建蠕行停止墙，`GetCreepFinishS = overlap_end+2m`；蠕行超时或路口清空（连续 5 帧）→ FinishStage → IntersectionCruise。
- **INTERSECTION_CRUISE**：同 protected，`CheckDone(..., right_of_way_status=true)`，离开路口后 FinishScenario。

---

## 4. TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN（红绿灯右转）

- 目录：`scenarios/traffic_light_unprotected_right_turn/`
- 场景类：`TrafficLightUnprotectedRightTurnScenario`，`IsTransferable` 要求 `RIGHT_TURN`

### Pipeline（Stage/Task 链）

| # | Stage name | type | enabled |
|---|-----------|------|:---:|
| 1 | TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_STOP | TrafficLightUnprotectedRightTurnStageStop | true |
| 2 | TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP | TrafficLightUnprotectedRightTurnStageCreep | true |
| 3 | TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE | TrafficLightUnprotectedRightTurnStageIntersectionCruise | true |

Task 链同左转（11 个，含 ST_BOUNDS_DECIDER）。

### 参数详解（scenario_conf.pb.txt）

#### 4.1 `start_traffic_light_scenario_distance`
- **当前值**：`5.0`　**proto 默认值**：`5.0`（m）
- **代码位置**：`traffic_light_unprotected_right_turn_scenario.cc::IsTransferable`
- **作用**：切入距离（同前）。右转保持默认 5m（左转被调到 30m）。注意：**右转场景的 Stop 阶段不主动限速**（无 approach_cruise_speed 参数）。

#### 4.2 `enable_right_turn_on_red` 🎯
- **当前值**：`false`　**proto 默认值**：`false`
- **代码位置**：`stage_stop.cc::Process`
  ```cpp
  if (scenario_config.enable_right_turn_on_red()) {
    // 红灯可右转时，等待 red_light_right_turn_stop_duration_sec 后通过
  }
  ```
- **作用**：是否允许"红灯右转"。`false` 表示红灯时**必须停**（除非地图无 NO_RIGHT_TURN_ON_RED 标志且未启用红灯右转，则按 `CheckTrafficLightNoRightTurnOnRed` 判定）。
- **对车辆行为影响**：🎯 **赛题"红绿灯停车"强相关**——若赛题要求"红灯停车、绿灯通行"（不要求红灯右转），保持 `false` 即可；若要求"红灯可右转但要先停 3s"，需置 `true`。

#### 4.3 `max_valid_stop_distance`
- **当前值**：`2.0`　**proto 默认值**：`3.5`（m，当前值比默认小）
- **代码位置**：`stage_stop.cc::Process`
  ```cpp
  if (distance_adc_to_stop_line > scenario_config.max_valid_stop_distance()) {
    traffic_light_all_stop = false;
  }
  ```
- **作用**：Stop 阶段判定"是否已停到停止线附近"——前缘距停止线 ≤ 2.0m 才算到位。

#### 4.4 `min_pass_s_distance`
- **当前值**：`3.0`　**proto 默认值**：`3.0`（m）
- **代码位置**：`stage_stop.cc::Process`
  ```cpp
  const double distance_adc_pass_stop_line = adc_front_edge_s - current_traffic_light_overlap->end_s;
  if (distance_adc_pass_stop_line > scenario_config.min_pass_s_distance()) {
    return FinishStage(false);  // 前缘越过停止线后端 3m → 即使红灯也可继续
  }
  ```
- **作用**：**闯红灯兜底**——若车头已越过停止线（越过 overlap 后端）3m，即使红灯也放行（已进入路口无法安全停下）。
- 🎯 与赛题相关：值过小容易在红灯时驶入路口，值过大则车已深入路口。

#### 4.5 `red_light_right_turn_stop_duration_sec`
- **当前值**：`3.0`　**proto 默认值**：`3.0`（s）
- **代码位置**：`stage_stop.cc::Process`
  ```cpp
  if (wait_time > scenario_config.red_light_right_turn_stop_duration_sec()) {
    return FinishStage(false);  // 红灯停够3s → 允许右转
  }
  ```
- **作用**：启用 `enable_right_turn_on_red` 时，红灯变绿前（或红灯持续期间）需**至少停车 3s** 才允许右转起步。
- 🎯 赛题若考"红灯右转停车时长"，改此参数。

#### 4.6 `creep_timeout_sec`
- **当前值**：`10.0`　**proto 默认值**：`10.0`（s）
- **代码位置**：`stage_creep.cc`（同左转）
- **作用**：蠕行超时。

#### 4.7 `max_adc_speed_before_creep`
- **当前值**：`3.0`　**proto 默认值**：`3.0`（m/s）
- **代码位置**：`stage_stop.cc::FinishStage`
  ```cpp
  if (adc_speed > context->scenario_config.max_adc_speed_before_creep()) {
    next_stage_ = "...INTERSECTION_CRUISE";  // 跳过蠕行
  } else {
    next_stage_ = "...CREEP";
  }
  ```
- **作用**：Stop 结束瞬间车速阈值（3.0 m/s，比左转 5.56 低）。右转更倾向于停车→蠕行。
- 🎯 若赛题要求右转"先停再蠕行通过"，保持 3.0；若要求直接通过，调大。

#### 4.8 `creep_stage_config`
- **当前值**：`6.0 / 0.1 / 15.0`（=默认），作用同 §3.6。

### Stage 行为

- **STOP**：检查 `CheckTrafficLightNoRightTurnOnRed`（地图有 `NO_RIGHT_TURN_ON_RED` 标志或 `ARROW_RIGHT` 子信号 → 红灯必须停）。绿灯→`FinishStage(true)` 直接巡航；红灯且不可右转→停；可右转→停够 `red_light_right_turn_stop_duration_sec`（3s）或越线 3m → `FinishStage(false)`。
- **CREEP / INTERSECTION_CRUISE**：同左转。

---

## 5. STOP_SIGN_UNPROTECTED（停止标志，重点）

- 目录：`scenarios/stop_sign_unprotected/`
- 场景类：`StopSignUnprotectedScenario`（`stop_sign_unprotected_scenario.cc`），`IsTransferable` 要求首个 overlap 为 STOP_SIGN 且距离≤4m

### Pipeline（Stage/Task 链）

| # | Stage name | type | enabled |
|---|-----------|------|:---:|
| 1 | STOP_SIGN_UNPROTECTED_PRE_STOP | StopSignUnprotectedStagePreStop | true |
| 2 | STOP_SIGN_UNPROTECTED_STOP | StopSignUnprotectedStageStop | true |
| 3 | STOP_SIGN_UNPROTECTED_CREEP | StopSignUnprotectedStageCreep | true |
| 4 | STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE | StopSignUnprotectedStageIntersectionCruise | true |

各 Stage Task 链相同（10 个，**无 ST_BOUNDS_DECIDER**、无 LANE_CHANGE_PATH）：

| # | Task name | Task type | 说明 |
|---|-----------|-----------|------|
| 1 | LANE_FOLLOW_PATH | LaneFollowPath | 沿车道路径 |
| 2 | LANE_BORROW_PATH | LaneBorrowPath | 借道路径 |
| 3 | FALLBACK_PATH | FallbackPath | 路径兜底 |
| 4 | PATH_DECIDER | PathDecider | 路径决策 |
| 5 | RULE_BASED_STOP_DECIDER | RuleBasedStopDecider | 规则停止（含停止牌规则） |
| 6 | SPEED_BOUNDS_PRIORI_DECIDER | SpeedBoundsDecider | 先验速度边界 |
| 7 | SPEED_HEURISTIC_OPTIMIZER | PathTimeHeuristicOptimizer | DP 速度 |
| 8 | SPEED_DECIDER | SpeedDecider | 速度决策 |
| 9 | SPEED_BOUNDS_FINAL_DECIDER | SpeedBoundsDecider | 最终速度边界 |
| 10 | PIECEWISE_JERK_SPEED | PiecewiseJerkSpeedOptimizer | QP 速度 |

### 参数详解（scenario_conf.pb.txt）

#### 5.1 `start_stop_sign_scenario_distance`
- **当前值**：`4.0`　**proto 默认值**：`5.0`（m，**当前值比默认小**）
- **代码位置**：`stop_sign_unprotected_scenario.cc::IsTransferable`
  ```cpp
  stop_sign_scenario = (adc_distance_to_stop_sign > 0.0 &&
       adc_distance_to_stop_sign <= context_.scenario_config.start_stop_sign_scenario_distance());
  ```
- **作用**：切入距离——前缘距停止牌 overlap ≤ 4m 才切入。当前值比默认 5m 小，切入更晚。
- **对车辆行为影响**：值太小可能导致高速接近停止牌时来不及平稳减速；值太大会过早切入。🎯 与停止牌停车位置相关（配合 `max_valid_stop_distance`）。

#### 5.2 `watch_vehicle_max_valid_stop_distance`
- **当前值**：`5.0`　**proto 默认值**：`5.0`（m）
- **代码位置**：`stage_pre_stop.cc::AddWatchVehicle`
  ```cpp
  if (distance_to_stop_line > ...->scenario_config.watch_vehicle_max_valid_stop_distance()) {
    ... "too far from stop line. skip";   // 关联车道上距停止线>5m的车辆不纳入 watch
  }
  ```
- **作用**：在**关联车道**（受同一停止牌约束的其他车道）上，若其他车辆距其停止线 ≤ 5m，将其加入 `watch_vehicles` 观察列表——这些车可能在路口与你冲突。
- **对车辆行为影响**：决定 Stop 阶段等待哪些车辆离开。值越大越保守（等更多车）。

#### 5.3 `max_valid_stop_distance` 🎯
- **当前值**：`2.0`　**proto 默认值**：`3.5`（m，**当前值比默认小**）
- **代码位置**：`stage_pre_stop.cc::CheckADCStop`
  ```cpp
  if (distance_stop_line_to_adc_front_edge > ...->scenario_config.max_valid_stop_distance()) {
    ... "not a valid stop. too far from stop line.";  // 距停止线>2m 不算有效停车
  }
  ```
- **作用**：**停止牌停车位置的判定距离**——本车完全停稳（车速≤`max_abs_speed_when_stopped`）且前缘距停止线 ≤ 2.0m 才算"有效停止"，PreStop 才能结束。
- **对车辆行为影响**：🎯 **赛题"停止牌停车1.5-2.0m"最直接相关参数**。当前 2.0m 表示停车点距停止线 2.0m 内即算合格；若要求更精确（如 1.5m），调小此值。

#### 5.4 `stop_duration_sec`
- **当前值**：`1.0`　**proto 默认值**：`1.0`（s）
- **代码位置**：`stage_stop.cc::Process`
  ```cpp
  if (wait_time < scenario_config.stop_duration_sec()) {
    return result.SetStageStatus(RUNNING);  // 停车未满1s继续等
  }
  ```
- **作用**：Stop 阶段**最短停车时长**——完全停稳后至少停 1s 才能进入下一步。
- 🎯 赛题若要求"停止牌停车后停顿 N 秒"，改此参数。

#### 5.5 `stop_timeout_sec`
- **当前值**：`8.0`　**proto 默认值**：`8.0`（s）
- **代码位置**：`stage_stop.cc::Process`
  ```cpp
  if (wait_time > scenario_config.stop_timeout_sec() && watch_vehicle_ids.size() <= 1) {
    return FinishStage();  // 等单辆车超过8s → 强制结束 Stop
  }
  ```
- **作用**：等待**单辆** watch 车辆时的超时——若只有 1 辆需观察的车且等满 8s，强制放行（防止被"僵尸车"永久卡住）。
- **对车辆行为影响**：值越大越耐心等待，越小越激进放行。注意：多辆车时不受此超时限制（需等全部离开）。

#### 5.6 `creep_timeout_sec`
- **当前值**：`10.0`　**proto 默认值**：`10.0`（s）
- **代码位置**：`stage_creep.cc::Process` → `CheckCreepDone`
- **作用**：蠕行超时——蠕行 ≥10s 或路口清空 → FinishStage 进入 IntersectionCruise。

#### 5.7 `creep_stage_config`
- **当前值**：`6.0 / 0.1 / 15.0`（=默认），作用同 §3.6（`base_stage_creep.cc::CheckCreepDone`）。

### Stage 行为

- **PRE_STOP**（`stage_pre_stop.cc`）：执行 Task 链（停车由 RuleBasedStopDecider/停止牌规则完成）；车头越过停止线>0.3m 直接 FinishStage；未越过则检查 `CheckADCStop`（停稳 + 距停止线≤2m）→ FinishStage；期间持续 `AddWatchVehicle` 收集关联车道观察车辆。
- **STOP**（`stage_stop.cc`）：`SetJunctionRightOfWay(false)`；等 `stop_duration_sec`(1s)；若 watch_vehicles 非空则等待其离开（超 10m 移除）或单辆超时 8s；全部满足 → FinishStage → Creep。
- **CREEP**：蠕行通过停止牌（overlap_end+4m 建墙，+2m 算完成），超时/清空 → IntersectionCruise。
- **INTERSECTION_CRUISE**：`CheckDone(..., right_of_way_status=false)`，离开路口后 FinishScenario。

---

## 6. BARE_INTERSECTION_UNPROTECTED（裸路口/无信号交叉口）

- 目录：`scenarios/bare_intersection_unprotected/`
- 场景类：`BareIntersectionUnprotectedScenario`，`IsTransferable` 要求：首个 overlap 为 PNC_JUNCTION（排除 SIGNAL/STOP_SIGN/YIELD_SIGN 在 10m 内的干扰）、路口无路权（`GetIntersectionRightofWayStatus` 为 false）、距离≤25m

### Pipeline（Stage/Task 链）

| # | Stage name | type | enabled |
|---|-----------|------|:---:|
| 1 | BARE_INTERSECTION_UNPROTECTED_APPROACH | BareIntersectionUnprotectedStageApproach | true |
| 2 | BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE | BareIntersectionUnprotectedStageIntersectionCruise | true |

Task 链同停止牌（10 个，无 ST_BOUNDS_DECIDER）。

### 参数详解（scenario_conf.pb.txt）

#### 6.1 `start_bare_intersection_scenario_distance`
- **当前值**：`25.0`　**proto 默认值**：`25.0`（m）
- **代码位置**：`bare_intersection_unprotected_scenario.cc::IsTransferable`
- **作用**：切入距离——距 PNC_JUNCTION 25m 内且无路权即切入。

#### 6.2 `enable_explicit_stop` 🎯
- **当前值**：`false`　**proto 默认值**：`false`
- **代码位置**：`stage_approach.cc::Process`
  ```cpp
  if (scenario_config_.enable_explicit_stop()) {
    bool stop = false;
    // 距路口 2~5m 且不清空 → 停；<2m 进入蠕行区，连续5帧清空才放行
    if (stop) { BuildStopDecision(...STOP_REASON_STOP_SIGN...); }
  }
  ```
- **作用**：是否在路口前**显式停车等待**。`false` 时不主动停车，仅减速巡航；`true` 时若路口有冲突车辆则停车（2~5m 判断区），清空后放行。
- **对车辆行为影响**：🎯 **裸路口"停车让行 vs 减速通过"策略开关**。赛题若要求无保护路口减速让行（不强制停车），保持 `false`；若要求停车观察，置 `true`。

#### 6.3 `approach_cruise_speed` 🎯
- **当前值**：`6.7056`　**proto 默认值**：`6.7056`（m/s，=15 mph≈24.1 km/h）
- **代码位置**：`stage_approach.cc::Process`
  ```cpp
  frame->mutable_reference_line_info()->front().LimitCruiseSpeed(scenario_config_.approach_cruise_speed());
  ```
- **作用**：接近路口的**巡航限速**（24 km/h）。所有场景中裸路口限速最高（无信号灯，靠低速观察）。
- **对车辆行为影响**：🎯 赛题"绿灯路口≤5m/s"——裸路口当前 6.7m/s 超限，若赛题涵盖"通过路口限速≤5m/s"需调小此参数（如 5.0）。

#### 6.4 `stop_distance`
- **当前值**：`0.5`　**proto 默认值**：`0.5`（m）
- **代码位置**：`stage_approach.cc::Process`
  ```cpp
  BuildStopDecision(virtual_obstacle_id, current_pnc_junction->start_s,
                    scenario_config_.stop_distance(), STOP_REASON_STOP_SIGN, ...);
  ```
- **作用**：启用 `enable_explicit_stop` 时，停车点距路口起点 0.5m。
- **对车辆行为影响**：仅 `enable_explicit_stop=true` 时生效；当前 false 不生效。

> 注：`stage_approach.cc::CheckClear` 中的冲突判断阈值（min_boundary_t=6.0 / ignore_max_st_min_t=0.1 / ignore_min_st_min_s=15.0）为**硬编码常量**（代码注释 `TODO: move to conf`），未走配置。

### Stage 行为

- **APPROACH**：`LimitCruiseSpeed(6.7056)` 减速接近；`SetJunctionRightOfWay(false)`；`enable_explicit_stop=false` 时纯减速通过；`true` 时按 2~5m 判断区停车等待。车头越过路口起点 0.3m → FinishStage。
- **INTERSECTION_CRUISE**：离开路口后 FinishScenario。

---

## 7. YIELD_SIGN（让行标志）

- 目录：`scenarios/yield_sign/`
- 场景类：`YieldSignScenario`，`IsTransferable` 要求首个 overlap 为 YIELD_SIGN 且距离≤10m

### Pipeline（Stage/Task 链）

| # | Stage name | type | enabled |
|---|-----------|------|:---:|
| 1 | YIELD_SIGN_APPROACH | YieldSignStageApproach | true |
| 2 | YIELD_SIGN_CREEP | YieldSignStageCreep | true |

Task 链同停止牌（10 个）。

### 参数详解（scenario_conf.pb.txt）

#### 7.1 `start_yield_sign_scenario_distance`
- **当前值**：`10.0`　**proto 默认值**：`10.0`（m）
- **代码位置**：`yield_sign_scenario.cc::IsTransferable`
- **作用**：切入距离。让行场景无 `approach_cruise_speed`，**不主动限速**，靠让行规则在接近时决策。

#### 7.2 `max_valid_stop_distance`
- **当前值**：`4.5`　**proto 默认值**：`4.5`（m）
- **代码位置**：`stage_approach.cc::Process`
  ```cpp
  if (distance_adc_to_stop_line < scenario_config_.max_valid_stop_distance()) {
    // 距让行牌停止线 4.5m 内 → 检查路口是否清空
    yield_sign_done = true;  // 无冲突车辆 → 完成 Approach
  }
  ```
- **作用**：Approach 结束判定——距停止线 ≤ 4.5m 且路口无冲突车辆（ST 边界判断，阈值硬编码 6.0/0.1/15.0）→ FinishStage。
- **对车辆行为影响**：值越小越贴近让行线才判断；让行逻辑本质是"接近后观察清空即走"，不强制停车。

#### 7.3 `creep_timeout_sec`
- **当前值**：`10.0`　**proto 默认值**：`10.0`（s）
- **代码位置**：`stage_creep.cc`（同停止牌蠕行）
- **作用**：蠕行超时。

#### 7.4 `creep_stage_config`
- **当前值**：`6.0 / 0.1 / 15.0`（=默认），作用同 §3.6。

### Stage 行为

- **APPROACH**：`SetJunctionRightOfWay(false)`；接近到 4.5m 内后检查路口清空（ST 边界 + 让行规则），清空即 FinishStage；有车则把障碍物 id 写入 `yield_sign.wait_for_obstacle_id` 等待。
- **CREEP**：蠕行通过，超时/清空 → FinishScenario。

---

## 8. EMERGENCY_STOP（紧急停车）

- 目录：`scenarios/emergency_stop/`
- 场景类：`EmergencyStopScenario`，`IsTransferable` 仅当 Pad 消息为 `STOP`

### Pipeline（Stage/Task 链）

| # | Stage name | type | enabled |
|---|-----------|------|:---:|
| 1 | EMERGENCY_STOP_APPROACH | EmergencyStopStageApproach | true |
| 2 | EMERGENCY_STOP_STANDBY | EmergencyStopStageStandby | true |

Task 链（9 个，**无 LANE_BORROW_PATH**、无 ST_BOUNDS_DECIDER、无 LANE_CHANGE_PATH）：

| # | Task name | Task type |
|---|-----------|-----------|
| 1 | LANE_FOLLOW_PATH | LaneFollowPath |
| 2 | FALLBACK_PATH | FallbackPath |
| 3 | PATH_DECIDER | PathDecider |
| 4 | RULE_BASED_STOP_DECIDER | RuleBasedStopDecider |
| 5 | SPEED_BOUNDS_PRIORI_DECIDER | SpeedBoundsDecider |
| 6 | SPEED_HEURISTIC_OPTIMIZER | PathTimeHeuristicOptimizer |
| 7 | SPEED_DECIDER | SpeedDecider |
| 8 | SPEED_BOUNDS_FINAL_DECIDER | SpeedBoundsDecider |
| 9 | PIECEWISE_JERK_SPEED | PiecewiseJerkSpeedOptimizer |

### 参数详解（scenario_conf.pb.txt）

#### 8.1 `max_stop_deceleration`
- **当前值**：`6.0`　**proto 默认值**：`6.0`（m/s²）
- **代码位置**：`stage_approach.cc::Process`
  ```cpp
  const double travel_distance = std::ceil(std::pow(adc_speed, 2) / (2 * deceleration));
  stop_line_s = adc_front_edge_s + travel_distance + stop_distance + kBuffer;  // kBuffer=2.0
  ```
- **作用**：紧急停车假设的**最大减速度**。用 v²/(2a) 估算从当前车速到 0 的制动距离，决定停止墙位置。a 越大→制动距离越短→停车点越近（更激进）；a 越小→提前停车（更安全但更远）。
- **对车辆行为影响**：🎯 赛题"障碍物停车≥2m"相关——此值决定急停距离预留，通常保持较大值保证能停住。

#### 8.2 `stop_distance`
- **当前值**：`1.0`　**proto 默认值**：`1.0`（m）
- **代码位置**：`stage_approach.cc::Process` / `stage_standby.cc::Process`
  ```cpp
  BuildStopDecision(virtual_obstacle_id, stop_line_s, stop_distance, STOP_REASON_EMERGENCY, ...);
  ```
- **作用**：停车点相对停止墙的**缓冲距离**（停止墙设在 stop_line_s，车停在其后 stop_distance 处）。
- 🎯 紧急停车场景下"停车距"参数；赛题障碍物停车≥2m 时，此值 + 制动距离共同决定最终停车点。

### Stage 行为

- **APPROACH**：`SetEmergencyLight()`；按 `v²/(2a)+stop_distance+2m` 计算并固化停止墙（写入 `planning_status.emergency_stop.stop_fence_point`）；`BuildStopDecision("EMERGENCY_STOP")` 急停；车速≤`max_abs_speed_when_stopped` → FinishStage → Standby。
- **STANDBY**：持续重建停止墙保持停车；Pad 消息不再是 STOP → FinishScenario（解除急停）。

---

## 9. EMERGENCY_PULL_OVER（紧急靠边停车）

- 目录：`scenarios/emergency_pull_over/`
- 场景类：`EmergencyPullOverScenario`，`IsTransferable` 仅当 Pad 消息为 `PULL_OVER`

### Pipeline（Stage/Task 链）

| # | Stage name | type | enabled |
|---|-----------|------|:---:|
| 1 | EMERGENCY_PULL_OVER_SLOW_DOWN | EmergencyPullOverStageSlowDown | true |
| 2 | EMERGENCY_PULL_OVER_APPROACH | EmergencyPullOverStageApproach | true |
| 3 | EMERGENCY_PULL_OVER_STANDBY | EmergencyPullOverStageStandby | true |

各 Stage Task 链：
- **SLOW_DOWN / STANDBY**：10 个 Task（同停止牌链：LANE_FOLLOW_PATH / LANE_BORROW_PATH / FALLBACK_PATH / PATH_DECIDER / RULE_BASED_STOP_DECIDER / SPEED_BOUNDS_PRIORI / SPEED_HEURISTIC / SPEED_DECIDER / SPEED_BOUNDS_FINAL / PIECEWISE_JERK_SPEED）
- **APPROACH**：11 个 Task，**头部多了 `PULL_OVER_PATH`（PullOverPath，生成靠边停车路径）**：
  1. PULL_OVER_PATH (PullOverPath)
  2. LANE_FOLLOW_PATH (LaneFollowPath)
  3. FALLBACK_PATH (FallbackPath)
  4. PATH_DECIDER (PathDecider)
  5. RULE_BASED_STOP_DECIDER (RuleBasedStopDecider)
  6-11. 速度链（同前）

### 参数详解（scenario_conf.pb.txt）

#### 9.1 `max_stop_deceleration`
- **当前值**：`4.5`　**proto 默认值**：`3.0`（m/s²，**当前值比默认大**）
- **代码位置**：`stage_slow_down.cc::Process`
  ```cpp
  target_slow_down_speed = std::max(
      scenario_config_.target_slow_down_speed(),
      adc_speed - scenario_config_.max_stop_deceleration() * scenario_config_.slow_down_deceleration_time());
  ```
- **作用**：SlowDown 阶段允许的**最大减速度**，参与计算目标减速速度（保证 a×t 的减速量）。
- **对车辆行为影响**：值越大减速越猛、越快降到目标速度。

#### 9.2 `slow_down_deceleration_time`
- **当前值**：`3.0`　**proto 默认值**：`3.0`（s）
- **代码位置**：`stage_slow_down.cc::Process`（同上式）
- **作用**：减速时间窗。目标速度 = max(配置的 target_slow_down_speed, 当前车速 - a×t)。

#### 9.3 `target_slow_down_speed` 🎯
- **当前值**：`1.0`　**proto 默认值**：`2.5`（m/s，**当前值比默认小**）
- **代码位置**：`stage_slow_down.cc::Process`
  ```cpp
  reference_line_info.LimitCruiseSpeed(target_slow_down_speed);  // 限速到1.0 m/s
  ```
- **作用**：SlowDown 阶段**目标巡航速度**。当前 1.0 m/s（3.6 km/h）比默认 2.5 m/s 更慢，减速更彻底。
- **对车辆行为影响**：🎯 决定靠边前的车速；1.0 m/s 接近蠕行速度，便于平稳靠边停车。

#### 9.4 `stop_distance` 🎯
- **当前值**：`1.5`　**proto 默认值**：`1.5`（m）
- **代码位置**：`stage_approach.cc::Process`
  ```cpp
  stop_line_s = pull_over_sl.s() + stop_distance + front_edge_to_center;
  BuildStopDecision("EMERGENCY_PULL_OVER", stop_line_s, stop_distance, STOP_REASON_PULL_OVER, ...);
  ```
- **作用**：靠边停车点距目的地的距离（停止墙建在目的地前 stop_distance+车头中心距 处）。
- 🎯 赛题若考"靠边停车距离"，改此参数。

### Stage 行为

- **SLOW_DOWN**：`LimitCruiseSpeed(1.0)` 逐步减速；车速与目标速度差≤1.0 → FinishStage（置 `plan_pull_over_path=true` 通知 PullOverPath task）。
- **APPROACH**：`SetTurnSignal(RIGHT)`；继续限速 1.0；`PullOverPath` 生成靠边路径（读 `pull_over_status.position()` 目的地）；在 `pull_over_sl.s + stop_distance + front_edge` 建停止墙；车速≤停止阈值且距停止墙≤3m → FinishStage。
- **STANDBY**：保持停车，等待指令变化退出。

### 子配置：`conf/emergency_pull_over_approach/pull_over_path.pb.txt`

该文件是 APPROACH 阶段 **PullOverPath 任务**的配置（任务配置按 stage 目录存放，`config_dir` 指向 stage 名）。

| 参数 | 当前值 | proto 默认值（`tasks/pull_over_path/proto/pull_over_path.proto`） | 代码位置/作用 |
|------|:---:|:---:|------|
| `path_optimizer_config.l_weight` | 1.0 | — | `PullOverPath` 调 `PathOptimizerUtil::OptimizePath`；横向偏移惩罚权重 |
| `path_optimizer_config.dl_weight` | 20.0 | — | 横向一阶导数（航向角）权重 |
| `path_optimizer_config.ddl_weight` | 1000.0 | — | 横向二阶导数（曲率）权重 |
| `path_optimizer_config.dddl_weight` | 50000.0 | — | 横向三阶导数（曲率变化率）权重 |
| `path_optimizer_config.lateral_derivative_bound_default` | 2.0 | — | 横向导数默认边界（限制路径摆动） |
| `pull_over_destination_to_adc_buffer` | 25.0 | 25.0 | `FindDestinationPullOverS`：目的地前 25m 内开始搜索靠边点 |
| `pull_over_destination_to_pathend_buffer` | 4.0 | 10.0（**当前更小**） | 目的地到参考线末端的最小缓冲，决定靠边点距终点距离 |
| `pull_over_road_edge_buffer` | 0.15 | 0.15 | `GetBoundaryFromRoads`：靠边时离道路边缘 0.15m 缓冲 |
| `pull_over_approach_lon_distance_adjust_factor` | 1.6 | 1.5 | `FindNearestPullOverS`：靠边纵向距离调整系数 |
| `pull_over_weight` | 2000 | 10（**当前远大于默认**） | `UpdatePullOverBoundaryByLaneBoundary`：靠边代价权重，越大越贴边 |
| `pull_over_direction` | BOTH_SIDE | RIGHT_SIDE | 靠边方向枚举（LEFT/RIGHT/BOTH）。`DecidePathBounds`：BOTH 时选择离边更近的一侧 |
| `pull_over_position` | NEAREST_POSITION | DESTINATION | 靠边位置策略：NEAREST（就近可行点）/DESTINATION（目的地）。当前改成了就近 |

- 🎯 与赛题"靠边停车"相关：`pull_over_weight`、`pull_over_road_edge_buffer`、`pull_over_direction`、`pull_over_position` 共同决定靠边停车的贴边程度与位置选择。

---

## 10. SQUARE（广场/停车场脱困场景）

- 目录：`scenarios/square/`
- 场景类：`SquareScenario`（`square_scenario.cc`），`IsTransferable`：车辆完全处于某个 JUNCTION 内部（前缘后缘都在 junction 范围内）
- ⚠️ **该场景未注册到 `public_road_planner_config.pb.txt`，默认不参与运行**（需手动启用）
- scenario_conf.pb.txt 为**空文件**（全部用 proto 默认值）

### Pipeline（Stage/Task 链）

| # | Stage name | type | enabled |
|---|-----------|------|:---:|
| 1 | SQUARE_LANE_FOLLOW_STAGE | SquareLaneFollowStage | true |
| 2 | EXTRICATE_STAGE | ExtricateStage | true |

**SQUARE_LANE_FOLLOW_STAGE** Task 链：

| # | Task name | Task type | 说明 |
|---|-----------|-----------|------|
| 1 | OBSTACLE_NUDGE_DECIDER | ObstacleNudgeDecider | 障碍物横向推挤决策（方形区域专用，提前规划 nudge 量） |
| 2 | SQUARE_PATH | SquarePath | **方形区域专用路径生成**（按 junction 边界生成 path bound） |
| 3 | LANE_BORROW_PATH_GENERIC | LaneBorrowPathGeneric | 通用借道路径 |
| 4 | FALLBACK_PATH | FallbackPath | 路径兜底 |
| 5 | RULE_BASED_STOP_DECIDER | RuleBasedStopDecider | 规则停止决策 |
| 6 | SPEED_BOUNDS_PRIORI_DECIDER | SpeedBoundsDecider | 先验速度边界 |
| 7 | SPEED_HEURISTIC_OPTIMIZER | PathTimeHeuristicOptimizer | DP 速度 |
| 8 | SPEED_DECIDER | SpeedDecider | 速度决策 |
| 9 | SPEED_BOUNDS_FINAL_DECIDER | SpeedBoundsDecider | 最终速度边界 |
| 10 | PIECEWISE_JERK_SPEED | PiecewiseJerkSpeedOptimizer | QP 速度 |

**EXTRICATE_STAGE** Task 链（倒车脱困）：

| # | Task name | Task type | 说明 |
|---|-----------|-----------|------|
| 1 | OBSTACLE_NUDGE_DECIDER | ObstacleNudgeDecider | 障碍物推挤决策 |
| 2 | LANE_FOLLOW_PATH | LaneFollowPath | 沿车道路径 |
| 3 | LANE_BORROW_PATH_GENERIC | LaneBorrowPathGeneric | 通用借道路径 |
| 4 | REVERSE_PATH | ReversePath | 生成倒车路径 |
| 5 | REVERSE_SPEED_DECIDER | ReverseSpeed | 倒车速度决策 |
| 6 | PIECEWISE_JERK_SPEED | PiecewiseJerkSpeedOptimizer | QP 速度 |

### 参数详解

#### 10.1 scenario_conf.pb.txt（空文件）
- 场景 proto（`square.proto`）：`filtering_distance=100`（默认）、`perception_obstacle_buffer=2`（默认）
- **代码位置**：`square_scenario.cc` 仅 `LoadConfig<ScenarioSquareConfig>`，**两个字段在 `square/` 目录代码中均未被引用**（`grep filtering_distance|perception_obstacle_buffer` 仅命中 proto 本身）→ **当前为无效/预留参数**，改配置无效果。

#### 10.2 `square_lane_follow_stage/lane_borrow_path_generic.pb.txt`

| 参数 | 当前值 | proto 默认值（`tasks/lane_borrow_path_generic/proto/lane_borrow_path_generic.proto`） | 作用 |
|------|:---:|:---:|------|
| `is_allow_lane_borrowing` | true | true | 是否允许借道绕行 |
| `lane_borrow_max_speed` | 10.0 | 5.0（**当前更大**） | 🎯 触发借道的**最大车速**（≤10m/s 才允许借道；越大越容易借道） |
| `long_term_blocking_obstacle_cycle_threshold` | 3 | 3 | 障碍物持续阻塞 N 帧判定为"长期停放"，触发稳定借道 |
| `enable_extend_boundary_by_adc` | false | false | 是否按车身扩展借道边界 |
| `enable_ignore_boundary_type` | true | false（**当前开启**） | 忽略实线/虚线边界类型限制，方形区域可任意借道 |
| `path_optimizer_config.l_weight` | 10.0 | — | 横向偏移惩罚 |
| `path_optimizer_config.dl_weight` | 2.0 | — | 航向角惩罚 |
| `path_optimizer_config.ddl_weight` | 100.0 | — | 曲率惩罚 |
| `path_optimizer_config.dddl_weight` | 500.0 | — | 曲率变化率惩罚 |
| `path_optimizer_config.path_reference_l_weight` | 2000.0 | — | 参考线横向跟随权重（大→路径贴近参考线） |
| `path_optimizer_config.lateral_derivative_bound_default` | 2.0 | — | 横向导数默认边界 |

#### 10.3 `square_lane_follow_stage/path_decider.pb.txt`
| 参数 | 当前值 | proto 默认值（`tasks/path_decider/proto/path_decider.proto`） | 作用 |
|------|:---:|:---:|------|
| `static_obstacle_buffer` | 0.1 | 0.3（**当前更小**） | 🎯 静态障碍物横向缓冲（m）。越小越贴近障碍物通过（方形区域空间窄，调小便于绕行） |

#### 10.4 `extricate_stage/lane_borrow_path_generic.pb.txt`
- 内容与 `square_lane_follow_stage` 版**完全相同**（同参同值），作用同上。

#### 10.5 `extricate_stage/piecewise_jerk_speed.pb.txt`
| 参数 | 当前值 | proto 默认值（`tasks/piecewise_jerk_speed/proto/piecewise_jerk_speed.proto`） | 作用 |
|------|:---:|:---:|------|
| `acc_weight` | 1.0 | 1.0 | 加速度惩罚权重 |
| `jerk_weight` | 3.0 | 10.0（**当前更小**） | 加加速度惩罚（越小速度曲线越"敢变"，倒车更灵活） |
| `kappa_penalty_weight` | 200.0 | 1000.0（**当前更小**） | 曲率惩罚（越小越不避讳大曲率路径） |
| `ref_s_weight` | 0.0 | 10.0（**当前=0**） | 参考 s 跟随权重（0=不强制贴参考位置） |
| `ref_v_weight` | 10.0 | 10.0 | 参考速度跟随权重 |
| `follow_distance_buffer` | 0.1 | 8.0（**当前远小于默认**） | 🎯 跟车距离缓冲（m）。倒车时对后方障碍物留 0.1m 缓冲——**非常激进**，倒车贴障碍物很近 |

### Stage 行为

- **SQUARE_LANE_FOLLOW_STAGE**（`square_lane_follow_stage.cc`）：
  - 若当前为倒挡 → 输出原地静止轨迹并切到 D 挡（换挡过渡）。
  - 执行 Task 链；若 `path_data.path_label` 非 regular、或有 `blocking_obstacle_id`、或 square 路径且车未动 → `blocking_times_` 累加；**连续 50 帧被堵** → 记录 `blocking_obstacle_id` → FinishStage → EXTRICATE_STAGE。
- **EXTRICATE_STAGE**（`extricate_stage.cc`）：
  - 若非倒挡 → 输出静止轨迹并切到 R 挡（准备倒车）。
  - 倒挡下执行 Task 链（ReversePath 生成向后 10m 直线路径），在起点后 10m 处建 `reverse_stop` 停止墙；持续检测是否出现 `regular` 且无阻塞障碍物的路径；连续 >10 帧找到 → 设停点并停车 → FinishStage 回 SQUARE_LANE_FOLLOW_STAGE。

---

## 11. 赛题相关性速查表

| 赛题要求 | 涉及场景 | 关键参数（当前值 → 建议关注） |
|---------|---------|------------------------------|
| 🎯 减速带 ≤3m/s | lane_follow（gflag） | `speed_bump_speed_limit=4.47`（gflag，需调≤3） |
| 🎯 人行道/红绿灯/停止牌停车 1.5-2.0m | traffic_light_* / stop_sign / 人行道规则 | **红绿灯实际停车点**=`traffic_light` 规则 `stop_distance=1.0`；**停止牌**=`max_valid_stop_distance=2.0`（判定） |
| 🎯 障碍物停车 ≥2m | lane_follow（gflag） | `max_stop_distance_obstacle=10.0` / `min_stop_distance_obstacle=6.0`（gflag，上下限） |
| 🎯 动态跟随 | lane_follow（gflag） | `follow_min_distance=3.0`、`follow_time_buffer=2.5` |
| 🎯 无人人行道 ≤5m/s | 人行道规则（traffic_rules） | 见 `02_traffic_rules.md`（本次未含） |
| 🎯 绿灯路口 ≤5m/s | bare_intersection / unprotected_left_turn | 裸路口 `approach_cruise_speed=6.7056`（超 5 需调）；左转 `approach_cruise_speed=2.78`（已满足） |

### 最关键的 5 个参数（按赛题影响度）

1. **`traffic_rules/traffic_light/conf/default_conf.pb.txt → stop_distance=1.0`** —— 红灯/黄灯实际停车点距停止线距离（🎯 1.5-2.0m 要求的主要调控点，注意它不是场景配置而是 TrafficRule 配置）。
2. **`stop_sign_unprotected/conf/scenario_conf.pb.txt → max_valid_stop_distance=2.0`** —— 停止牌有效停车判定距离（🎯 停车 1.5-2.0m）。
3. **`bare_intersection_unprotected/conf/scenario_conf.pb.txt → approach_cruise_speed=6.7056`** —— 裸路口通过限速（🎯 若要求路口 ≤5m/s 必须调小）。
4. **`traffic_light_unprotected_left_turn/conf/scenario_conf.pb.txt → approach_cruise_speed=2.78`** —— 无保护左转接近限速（已满足 ≤5m/s）。
5. **`emergency_pull_over/conf/scenario_conf.pb.txt → stop_distance=1.5` / `target_slow_down_speed=1.0`** —— 紧急靠边停车距离与速度。

> 补充提醒：`lane_follow` 等默认行为由 `planning_base/gflags/planning_gflags.cc` 的 gflag 控制（`default_cruise_speed=5.0`、`speed_bump_speed_limit=4.47`、`max/min_stop_distance_obstacle`、`follow_*`），这些不在场景 conf 中但直接影响赛题表现。
