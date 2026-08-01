# Apollo Planning 路径相关 Task 配置参数全量分析

> 工程：Apollo 11.0 EDU 赛事工程（官方原始版本，代码未修改）
> 分析日期：2026-08-01
> 分析范围：`modules/planning/tasks/` 下 13 个路径 Task 的 `conf/default_conf.pb.txt` 及对应 proto、源码
>
> **赛题关联标注说明**：
> - 🔴 直接相关（影响赛题 5/6 障碍物停车/跟随、赛题 4 障碍物停车≥2m）
> - 🟡 间接相关（绕行/借道等躲避行为）
> - ⚪ 一般相关（路径平滑/变道/靠边等）

---

## 目录

1. [共享结构：path_optimizer_config（分段 Jerk 路径优化器）](#0-共享结构path_optimizer_config分段-jerk-路径优化器)
2. [lane_follow_path（直行跟车道路径）](#1-lane_follow_path直行跟车道路径重点)
3. [lane_change_path（变道路径，重点）](#2-lane_change_path变道路径重点)
4. [lane_change_path_generic（变道路径通用）](#3-lane_change_path_generic变道路径通用)
5. [lane_borrow_path（借道路径）](#4-lane_borrow_path借道路径)
6. [lane_borrow_path_generic（借道通用）](#5-lane_borrow_path_generic借道通用)
7. [fallback_path（兜底路径）](#6-fallback_path兜底路径)
8. [path_decider（路径决策，重点）](#7-path_decider路径决策重点)
9. [path_reference_decider（路径参考线决策）](#8-path_reference_decider路径参考线决策)
10. [obstacle_nudge_decider（绕行决策，重点）](#9-obstacle_nudge_decider绕行决策重点)
11. [reuse_path（路径复用）](#10-reuse_path路径复用)
12. [reverse_path（倒车路径）](#11-reverse_path倒车路径)
13. [pull_over_path（靠边停车路径）](#12-pull_over_path靠边停车路径)
14. [square_path（路口/广场路径）](#13-square_path路口广场路径)
15. [赛题关联总结](#14-赛题关联总结)

---

## 0. 共享结构：path_optimizer_config（分段 Jerk 路径优化器）

多个任务共享 `PiecewiseJerkPathConfig`（定义于 `modules/planning/planning_base/proto/piecewise_jerk_path_config.proto`）。

**proto 定义与默认值：**

| 字段 | proto 默认值 |
|------|-------------|
| `l_weight` | 1.0 |
| `dl_weight` | 100.0 |
| `ddl_weight` | 1000.0 |
| `dddl_weight` | 10000.0 |
| `path_reference_l_weight` | 0.0 |
| `weight_end_state_l` | 1000.0 |
| `weight_end_state_dl` | 0.0 |
| `weight_end_state_ddl` | 0.0 |
| `lateral_derivative_bound_default` | 2.0 |
| `max_iteration` | 4000 |

**代码位置**：`modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.cc`
- `OptimizePath()`（约 101-240 行）：`set_weight_x(l_weight)`、`set_weight_dx(dl_weight)`、`set_weight_ddx(ddl_weight)`、`set_weight_dddx(dddl_weight)`、`set_dx_bounds(-lateral_derivative_bound_default, +lateral_derivative_bound_default)`、`set_end_state_ref({weight_end_state_l, weight_end_state_dl, weight_end_state_ddl}, end_state)`、`Optimize(max_iteration)`。
- `UpdatePathRefWithBound()`（约 330 行）：当路径边界类型为 `OBSTACLE` 时，把参考 l 设为上下界中点，权重设为 `path_reference_l_weight`；否则权重为 0。

**各参数作用与对车辆行为影响：**

### 0.1 l_weight（横向位置平滑权重）
- **作用**：分段 Jerk 优化器目标函数中 $l$（横向偏差）项的权重。权重越大，优化出的路径越贴近参考线（$l=0$），横向偏移被"拉回"。
- **对车辆行为影响**：在跟车/直行时控制车辆贴线程度。权重过大 → 路径不敢偏出车道中心（不利于绕障碍）；过小 → 路径可以自由偏摆。
- **赛题关联**：🟡 间接影响绕行/借道后的回正。

### 0.2 dl_weight（横向速度/航向角权重）
- **作用**：目标函数中 $\dot{l}$（横向变化率，即路径点航向相对参考线的偏差）的权重。控制路径与参考线的夹角。
- **对车辆行为影响**：权重越大，路径越"直"地平行于参考线，减少大角度切入；过小则路径可能出现明显斜插。
- **赛题关联**：⚪ 一般（路径平滑性）。

### 0.3 ddl_weight（横向加速度/曲率权重）
- **作用**：目标函数中 $\ddot{l}$（横向二阶导，近似路径曲率）的权重。控制路径曲率大小。
- **对车辆行为影响**：权重越大路径越平缓（大半径转弯），绕障碍时曲线更柔和，但也可能"切不到"最贴障碍的横向距离。
- **赛题关联**：⚪ 一般。

### 0.4 dddl_weight（横向 jerk 权重）
- **作用**：目标函数中 $\dddot{l}$（横向三阶导，jerk）的权重。控制路径曲率变化率，直接决定路径平滑度与转向连续性。
- **对车辆行为影响**：权重越大路径越平滑（曲率变化缓慢），转向越柔和；过小会出现"折线"路径，甚至超出车辆转向能力。**注意**：当前各任务普遍设为 50000，远大于 proto 默认 10000，说明官方调参偏重平滑。
- **赛题关联**：⚪ 一般（影响乘坐感与控制稳定性）。

### 0.5 path_reference_l_weight（障碍物处参考路径权重）
- **作用**：`UpdatePathRefWithBound` 中，仅当路径边界被障碍物（`BoundType::OBSTACLE`）收窄时，将参考 l 设为"上下界中点"并赋予此权重。即：**在绕障通道内，把优化路径"吸引"到通道中心**。
- **对车辆行为影响**：该值越大，绕障时车辆越贴近障碍物两侧剩余空间的中点（既不贴障碍、也不贴车道线），通过更从容；为 0 时路径完全由边界约束决定。
- **赛题关联**：🟡 影响绕行/借道时车辆在通道内的横向位置。

### 0.6 lateral_derivative_bound_default（横向速度/航向角硬约束）
- **作用**：`set_dx_bounds(-value, +value)`，限制每个路径点的 $\dot{l}$ 在 ±2.0 内，即路径点航向相对参考线夹角约为 ±63°（实际限制的是横向变化率，配合参考线曲率使用）。
- **对车辆行为影响**：限制路径与参考线的最大夹角，防止路径"横着走"。过大 → 可大角度切入绕障；过小 → 变道/绕障需要更长纵向距离完成。
- **赛题关联**：⚪ 一般。

### 0.7 weight_end_state_l / weight_end_state_dl / weight_end_state_ddl（终点状态权重）
- **作用**：`set_end_state_ref`，约束路径终点逼近给定 `end_state`（各任务传入 `{0,0,0}`，即终点回到参考线、航向对齐、曲率为 0）。
- **对车辆行为影响**：默认 `weight_end_state_l=1000` 较大，保证路径末端回到车道中心，避免终点偏移。各任务配置中均未显式设置 → 使用默认值。
- **赛题关联**：⚪ 一般。

### 0.8 max_iteration（优化迭代上限）
- **作用**：OSQP 迭代上限。默认 4000，各任务未覆盖。
- **对车辆行为影响**：迭代不足可能优化失败（路径规划失败会走 fallback）；过大会增加耗时。赛事中一般无需修改。
- **赛题关联**：⚪ 一般。

---

## 1. lane_follow_path（直行跟车道路径，重点）

**配置文件**：`modules/planning/tasks/lane_follow_path/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/lane_follow_path/proto/lane_follow_path.proto`（`LaneFollowPathConfig`）
**源码**：`modules/planning/tasks/lane_follow_path/lane_follow_path.cc`

```protobuf
is_extend_lane_bounds_to_include_adc: true
extend_buffer: 0.2
path_optimizer_config {
  l_weight: 1.0
  dl_weight: 20.0
  ddl_weight: 1000.0
  dddl_weight: 50000.0
  lateral_derivative_bound_default: 2.0
  path_reference_l_weight:100
}
```

### 1.1 is_extend_lane_bounds_to_include_adc
- **当前值**：`true` ｜ **proto 默认值**：无显式默认（即 false）
- **代码位置**：`lane_follow_path.cc` `DecidePathBounds()` 约 100 行：
  ```cpp
  bool is_include_adc = config_.is_extend_lane_bounds_to_include_adc() &&
                        !injector_->...path_decider().is_in_path_lane_borrow_scenario();
  ```
  为 true 且**不在借道场景**时，调用 `PathBoundsDeciderUtil::ExtendBoundaryByADC(...)`。
- **作用**：决定是否把"自车外形边界"计入路径上下界（用 `extend_buffer` 再外扩），保证优化出的路径一定在自车可通行范围内。
- **对车辆行为影响**：为 true 时直行跟车道路径更安全（边界包含车宽），但会略微压缩可绕空间；借道场景下自动禁用（避免与借道边界冲突）。
- **赛题关联**：⚪ 一般。

### 1.2 extend_buffer
- **当前值**：`0.2` ｜ **proto 默认值**：无显式默认（0）
- **代码位置**：同上，作为 `ExtendBoundaryByADC(*reference_line_info_, init_sl_state_, config_.extend_buffer(), &path_bound)` 的参数。
- **作用**：按自车边界外扩的缓冲距离（米）。数值越大，路径边界离车体越远，越保守。
- **对车辆行为影响**：0.2m 的缓冲使直行路径与路边/障碍保留额外余量。
- **赛题关联**：⚪ 一般。

### 1.3 path_optimizer_config（见第 0 节）
- **当前值**：`l_weight=1.0, dl_weight=20.0, ddl_weight=1000.0, dddl_weight=50000.0, lateral_derivative_bound_default=2.0, path_reference_l_weight=100`
- **与 proto 默认差异**：`dl_weight` 20（默认 100）、`dddl_weight` 50000（默认 10000）、`path_reference_l_weight` 100（默认 0）。
- **作用**：直行路径优化权重。`path_reference_l_weight=100` 使遇到静态障碍收窄边界时，路径被拉向通道中点，实现"贴障碍绕行但不蹭"。
- **对车辆行为影响**：直行遇静态障碍时路径会在障碍两侧中点通过；平滑度较高（jerk 权重 50000）。
- **赛题关联**：🟡 影响赛题 5/6 中静态障碍（锥桶/车辆）旁的通行轨迹。

---

## 2. lane_change_path（变道路径，重点）

**配置文件**：`modules/planning/tasks/lane_change_path/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/lane_change_path/proto/lane_change_path.proto`（`LaneChangePathConfig`）
**源码**：`modules/planning/tasks/lane_change_path/lane_change_path.cc`

```protobuf
extend_adc_buffer: 0.5
change_lane_success_freeze_time: 3
change_lane_fail_freeze_time: 1.0
path_optimizer_config {
  l_weight: 1.0
  dl_weight: 20.0
  ddl_weight: 1000.0
  dddl_weight: 50000.0
  path_reference_l_weight: 100.0
  lateral_derivative_bound_default: 2.0
}
```

### 2.1 extend_adc_buffer
- **当前值**：`0.5` ｜ **proto 默认值**：无显式默认（0）
- **代码位置**：`lane_change_path.cc` `DecidePathBounds()` 约 120 行：
  ```cpp
  if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(
          *reference_line_info_, init_sl_state_, config_.extend_adc_buffer(),
          &path_bound)) {...}
  ```
  变道路径**无条件**按 ADC 边界外扩 0.5m（不像 lane_follow 有开关）。
- **作用**：变道过程中自车横跨两车道，路径边界必须包含车体，0.5m 缓冲保证安全。
- **对车辆行为影响**：缓冲越大变道越保守，路径边界越宽。
- **赛题关联**：⚪ 一般（赛题无变道科目）。

### 2.2 change_lane_success_freeze_time
- **当前值**：`3`（秒）｜ **proto 默认值**：无显式默认（0）
- **代码位置**：`lane_change_path.cc` `UpdateLaneChangeStatus()` 约 300 行：
  ```cpp
  } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FINISHED) {
    if (now - prev_status->timestamp() > config_.change_lane_success_freeze_time()) {
      UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
  ```
- **作用**：一次变道**成功后**需等待 3 秒才允许发起下一次变道（防抖）。
- **对车辆行为影响**：防止连续快速变道导致车辆摆动。
- **赛题关联**：⚪ 一般。

### 2.3 change_lane_fail_freeze_time
- **当前值**：`1.0`（秒）｜ **proto 默认值**：无显式默认（0）
- **代码位置**：`UpdateLaneChangeStatus()` 约 290 行：
  ```cpp
  if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
    if (now - prev_status->timestamp() > config_.change_lane_fail_freeze_time()) {
      UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
  ```
- **作用**：变道**失败后**等待 1 秒再重试（防止因瞬时障碍反复失败抖动）。
- **对车辆行为影响**：失败后短暂冷却再尝试。
- **赛题关联**：⚪ 一般。

### 2.4 lane_change_prepare_length（配置中未设置 → proto 默认 0）
- **当前值**：未设置 ｜ **proto 默认值**：无显式默认（0）
- **代码位置**：
  - `GetLaneChangeStartPoint()`：`lane_change_start_s = config_.lane_change_prepare_length() + adc_frenet_s`
  - `GetBoundaryFromLaneChangeForbiddenZone()`：当 `is_clear_to_change_lane_ == false`（前方有障碍不清晰）时，把 `lane_change_start_s` 之前的路径边界限制在本车道内，禁止提前越线。
- **作用**：变道"准备段"长度——在发起变道前在本车道内直行的距离。当前为 0，表示障碍不清晰时立即不允许越线；若调大则先在本车道走一段再变道。
- **对车辆行为影响**：增大可让变道更晚/更从容；当前 0 使变道起点即切换。
- **赛题关联**：⚪ 一般。

### 2.5 path_optimizer_config
- 与 lane_follow 相同（`l_weight=1, dl_weight=20, ddl_weight=1000, dddl_weight=50000, path_reference_l_weight=100, lateral_derivative_bound_default=2`），作用见第 0 节。
- **对车辆行为影响**：变道路径平滑、末端回正参考线。

---

## 3. lane_change_path_generic（变道路径通用）

**配置文件**：`modules/planning/tasks/lane_change_path_generic/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/lane_change_path/proto/lane_change_path.proto`（与 lane_change_path **共用同一 proto 类型 `LaneChangePathConfig`**）

```protobuf
extend_adc_buffer: 0.5
change_lane_success_freeze_time: 3
change_lane_fail_freeze_time: 1.0
path_optimizer_config {
  l_weight: 1.0
  dl_weight: 20.0
  ddl_weight: 1000.0
  dddl_weight: 50000.0
  path_reference_l_weight: 100.0
  lateral_derivative_bound_default: 2.0
}
```

- **与 lane_change_path 完全一致**，所有参数含义同第 2 节。
- 该任务为"通用变道"版本（`lane_change_path_generic`），在同一 pipeline 中可能与 `lane_change_path` 二选一启用，参数无差异。
- **赛题关联**：⚪ 一般。

---

## 4. lane_borrow_path（借道路径）

**配置文件**：`modules/planning/tasks/lane_borrow_path/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/lane_borrow_path/proto/lane_borrow_path.proto`（`LaneBorrowPathConfig`）
**源码**：`modules/planning/tasks/lane_borrow_path/lane_borrow_path.cc`

```protobuf
is_allow_lane_borrowing: true
lane_borrow_max_speed: 5.0
long_term_blocking_obstacle_cycle_threshold: 3
path_optimizer_config {
  l_weight: 1.0
  dl_weight: 20.0
  ddl_weight: 1000.0
  dddl_weight: 50000.0
  path_reference_l_weight: 100.0
  lateral_derivative_bound_default: 2.0
}
```

### 4.1 is_allow_lane_borrowing
- **当前值**：`true` ｜ **proto 默认值**：`true`
- **代码位置**：`lane_borrow_path.cc` `Process()` 约 60 行：
  ```cpp
  if (!config_.is_allow_lane_borrowing() || reference_line_info->path_reusable()) {
    return Status::OK();  // 直接跳过借道
  }
  ```
- **作用**：总开关。为 false 时**完全不生成借道路径**（车辆只能停车等前方障碍消失或变道）。
- **对车辆行为影响**：此开关决定赛题中"借道绕行"是否可用。若某赛题要求停车等待（如障碍物停车≥2m 赛题），关闭它可强制停车；若要求绕行，必须保持 true。
- **赛题关联**：🟡 关键开关（控制是否允许借道绕障）。

### 4.2 lane_borrow_max_speed
- **当前值**：`5.0`（m/s）｜ **proto 默认值**：`5.0`
- **代码位置**：`lane_borrow_path.cc` `IsWithinSidePassingSpeedADC()` 约 432 行：
  ```cpp
  bool LaneBorrowPath::IsWithinSidePassingSpeedADC(const Frame& frame) {
    return frame.PlanningStartPoint().v() < config_.lane_borrow_max_speed();
  }
  ```
  在 `IsNecessaryToBorrowLane()` 中被调用，速度不满足则直接不借道。
- **作用**：只有**车速 < 5 m/s（18 km/h）**时才允许借道。低速意味着前方有阻塞（接近停车），此时才借道绕行；高速时不允许借道（防止高速变道借道危险）。
- **对车辆行为影响**：前方静态障碍导致车辆减速到 5m/s 以下才会触发借道。若调大（如 8-10），车辆在更高速度下也敢借道，反应更快但更激进。
- **赛题关联**：🟡 影响借道绕行的触发速度点。

### 4.3 long_term_blocking_obstacle_cycle_threshold
- **当前值**：`3`（周期）｜ **proto 默认值**：`3`
- **代码位置**：`lane_borrow_path.cc` `IsLongTermBlockingObstacle()` 约 438 行：
  ```cpp
  if (injector_->...path_decider().front_static_obstacle_cycle_counter() >=
      config_.long_term_blocking_obstacle_cycle_threshold()) {
    return true;  // 障碍已持续阻塞 ≥3 个周期，判定为"长期阻塞"
  }
  ```
  `front_static_obstacle_cycle_counter` 由 `path_decider.cc` 维护（每次前方有阻塞 +1、无阻塞 -1，上限 ±10）。
- **作用**：只有同一静态障碍**连续阻塞 ≥3 个规划周期**才认为是"长期停驻障碍"，才允许借道（防止瞬时误判）。
- **对车辆行为影响**：阈值越小借道触发越快（如 1 周期立即借道）；阈值大则要先停车观察 3 周期。
- **赛题关联**：🟡 决定借道绕行的响应速度（赛题 5/6 遇静态障碍）。

### 4.4 path_optimizer_config
- 同第 0 节（`path_reference_l_weight=100`）。借道路径在障碍收窄处拉向通道中点。

---

## 5. lane_borrow_path_generic（借道通用）

**配置文件**：`modules/planning/tasks/lane_borrow_path_generic/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/lane_borrow_path_generic/proto/lane_borrow_path_generic.proto`（`LaneBorrowPathGenericConfig`）
**源码**：`modules/planning/tasks/lane_borrow_path_generic/lane_borrow_path_generic.cc`

```protobuf
is_allow_lane_borrowing: true
lane_borrow_max_speed: 10.0
long_term_blocking_obstacle_cycle_threshold: 3
enable_extend_boundary_by_adc: false
enable_ignore_boundary_type: false
path_optimizer_config {
  l_weight: 10.0
  dl_weight: 2.0
  ddl_weight: 100.0
  dddl_weight: 500.0
  path_reference_l_weight: 2000.0
  lateral_derivative_bound_default: 2.0
}
enable_nudge_destination_threshold: 8.0
```

> 说明：这是与 `obstacle_nudge_decider` 联动的"新一代"借道实现——先由 NudgeDecider 生成绕障关键点（`extra_nudge_key_points`），本任务据此生成借道边界。

### 5.1 is_allow_lane_borrowing
- **当前值**：`true` ｜ **proto 默认值**：`true`
- **代码位置**：`Process()` 约 55 行。同第 4.1 节，总开关。
- **赛题关联**：🟡 关键开关。

### 5.2 lane_borrow_max_speed
- **当前值**：`10.0`（m/s，36 km/h）｜ **proto 默认值**：`5.0`
- **代码位置**：`IsWithinSidePassingSpeedADC()` 约 575 行：
  ```cpp
  return frame.PlanningStartPoint().v() < config_.lane_borrow_max_speed();
  ```
- **作用**：与 4.2 相同，但阈值提高到 10 m/s。**比非 generic 版本激进**：车辆在 36 km/h 以下都可能借道绕障（非 generic 仅 18 km/h）。
- **对车辆行为影响**：动态/静态障碍前无需减速到很低就能借道，绕行更主动；但高速借道更危险，需配合 Nudge 关键点安全评估。
- **赛题关联**：🟡 决定绕行的速度容忍度（赛题 5/6 若希望更早绕行可依赖此值）。

### 5.3 long_term_blocking_obstacle_cycle_threshold
- **当前值**：`3` ｜ **proto 默认值**：`3`
- **代码位置**：`IsLongTermBlockingObstacle()` 约 579 行。**注意**：在 `IsNecessaryToBorrowLane()` 中该检查已被**注释禁用**（generic 版不要求长期阻塞判定，改由 NudgeDecider 的概率累积替代）。
- **作用**：当前实际未生效（保留参数）。
- **赛题关联**：🟡 潜在相关（若重新启用则影响触发）。

### 5.4 enable_extend_boundary_by_adc
- **当前值**：`false` ｜ **proto 默认值**：`false`
- **代码位置**：`DecidePathBounds()` 约 110 行：
  ```cpp
  if (config_.enable_extend_boundary_by_adc()) {
    if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(*reference_line_info_, init_sl_state_, 0.5, &path_bound)) {...}
  }
  ```
- **作用**：是否在借道边界上额外按 ADC 外形外扩（硬编码 0.5m）。当前 false → 借道边界完全由邻道宽度 + Nudge 关键点决定，不额外外扩。
- **对车辆行为影响**：开启后借道更保守（边界更宽，避碰更稳但可通行空间判断更严，可能减少借道成功率）。
- **赛题关联**：🟡 借道安全裕度。

### 5.5 enable_ignore_boundary_type
- **当前值**：`false` ｜ **proto 默认值**：`false`
- **代码位置**：`CheckLaneBorrow()` 约 645 行：
  ```cpp
  if (!config_.enable_ignore_boundary_type()) {
    // 检查实线边界类型：SOLID_YELLOW / DOUBLE_YELLOW / SOLID_WHITE → 禁止借道
  }
  ```
- **作用**：为 false 时，若邻道与本道之间是**实线**（黄实线/双黄/白实线），禁止借道（合法合规）；为 true 时忽略边界类型，实线也允许借道。
- **对车辆行为影响**：赛事地图中若障碍物停在实线车道旁，默认**不会**借道（保持停车）；设 true 后可强制跨实线借道。
- **赛题关联**：🟡 关键合规开关（决定实线路段能否绕障）。

### 5.6 path_optimizer_config
- **当前值**：`l_weight=10.0, dl_weight=2.0, ddl_weight=100.0, dddl_weight=500.0, path_reference_l_weight=2000.0, lateral_derivative_bound_default=2.0`
- **与默认差异**：`l_weight` 10（默认 1）、`dl_weight` 2（默认 100）、`ddl_weight` 100（默认 1000）、`dddl_weight` 500（默认 10000）、`path_reference_l_weight` 2000（默认 0）。
- **作用**：借道路径优化权重。`l_weight=10` 较大 → 路径尽量贴近参考线（借道幅度受边界约束但不随意偏摆）；`dl/ddl/dddl` 权重大幅降低 → **允许路径以较大横向速度/曲率/曲率变化率绕障**（路径更"敢拐"）；`path_reference_l_weight=2000` 极大 → 在 Nudge 关键点给出的障碍通道内，路径被强拉向通道中心（精确贴障碍边缘绕行）。
- **对车辆行为影响**：这是整套参数中最激进的绕障配置——路径可以在障碍旁快速、紧密地绕行，同时保持通道内居中。若希望绕障更柔和，可增大 `dddl_weight`。
- **赛题关联**：🟡 核心影响赛题 5/6 的绕行轨迹形态。

### 5.7 enable_nudge_destination_threshold
- **当前值**：`8.0`（米）｜ **proto 默认值**：`8.0`
- **代码位置**：`IsNecessaryToBorrowLane()` 约 519 行调用 `obstacle_blocking_analyzer.cc` 的 `IsBlockingObstacleWithinDestination`：
  ```cpp
  if (blocking_obstacle_s - adc_end_s + threshold > reference_line_info.SDistanceToDestination()) {
    return false;  // 障碍离终点太近（< threshold），不借道
  }
  ```
- **作用**：阻塞障碍必须距终点**至少 8m** 以上才允许借道绕行。防止临近终点时借道导致错过目的地。
- **对车辆行为影响**：离终点 8m 内的障碍不再绕行（改为停车等）。若赛题终点附近有障碍需绕行，可调小此值。
- **赛题关联**：🟡 影响终点附近障碍的处理。

### 5.8 enable_active_trigger（配置中未设置 → proto 默认 true）
- **当前值**：未设置 ｜ **proto 默认值**：`true`
- **代码位置**：`IsNecessaryToBorrowLane()` 约 528 行：
  ```cpp
  if (!config_.enable_active_trigger() && blocking_obstacle_id_.empty()) {
    return false;
  }
  ```
- **作用**：为 true 时即使没有明确的 blocking obstacle（`front_static_obstacle_id` 为空），只要 NudgeDecider 给出了绕障关键点也可主动借道；为 false 则必须有阻塞障碍才借道。
- **对车辆行为影响**：true 支持"主动预防性借道"，false 只做"被动响应性借道"。
- **赛题关联**：🟡 决定借道是否可主动触发。

---

## 6. fallback_path（兜底路径）

**配置文件**：`modules/planning/tasks/fallback_path/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/fallback_path/proto/fallback_path.proto`（`FallbackPathConfig`）
**源码**：`modules/planning/tasks/fallback_path/fallback_path.cc`

```protobuf
extend_buffer: 0.5
path_optimizer_config {
  l_weight: 1.0
  dl_weight: 20.0
  ddl_weight: 1000.0
  dddl_weight: 50000.0
  lateral_derivative_bound_default: 2.0
}
```

### 6.1 is_extend_lane_bounds_to_include_adc（配置中未设置 → proto 默认 false）
- **当前值**：未设置 ｜ **proto 默认值**：无显式默认（false）
- **作用**：proto 中有此字段，但 fallback_path.cc 中**未读取**。Fallback 路径总是调用 `ExtendBoundaryByADC`（见下），此字段对本任务无效。
- **赛题关联**：⚪ 无效参数。

### 6.2 extend_buffer
- **当前值**：`0.5` ｜ **proto 默认值**：无显式默认（0）
- **代码位置**：`fallback_path.cc` `DecidePathBounds()` 约 105 行：
  ```cpp
  if (!PathBoundsDeciderUtil::ExtendBoundaryByADC(
          *reference_line_info_, init_sl_state_, config_.extend_buffer(),
          &path_bound)) {...}
  ```
  无条件执行（无开关）。
- **作用**：兜底路径（其他路径全部失败时的保底方案）按自车边界外扩 0.5m 生成本车道路径，保证至少能沿当前车道直行。
- **对车辆行为影响**：缓冲越大兜底路径越宽裕；兜底路径通常用于速度降级/安全停车。
- **赛题关联**：⚪ 一般（兜底路径主要服务安全）。

### 6.3 path_optimizer_config
- `l_weight=1, dl_weight=20, ddl_weight=1000, dddl_weight=50000, lateral_derivative_bound_default=2`。
- **注意**：`fallback_path.cc` 的 `OptimizePath()` 中 `weight_ref_l` 被赋为 `config.path_reference_l_weight()`（未设置 → 默认 0），即兜底路径无参考权重，完全跟随边界。
- **赛题关联**：⚪ 一般。

---

## 7. path_decider（路径决策，重点）

**配置文件**：`modules/planning/tasks/path_decider/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/path_decider/proto/path_decider.proto`（`PathDeciderConfig`）
**源码**：`modules/planning/tasks/path_decider/path_decider.cc`

```protobuf
static_obstacle_buffer: 0.3
skip_overlap_stop_check: true
```

### 7.1 static_obstacle_buffer 🔴
- **当前值**：`0.3`（米）｜ **proto 默认值**：`0.3`
- **代码位置**：`path_decider.cc` `MakeStaticObstacleDecision()`：
  ```cpp
  const double min_nudge_l = half_width + config_.static_obstacle_buffer() / 2.0;
  // ① IGNORE：横向距离太远
  // ② STOP：横向重叠（sl_boundary.end_l() >= curr_l - min_nudge_l && sl_boundary.start_l() <= curr_l + min_nudge_l）
  // ③ NUDGE：横向很近
  //   LEFT_NUDGE: set_distance_l(config_.static_obstacle_buffer())   // +0.3
  //   RIGHT_NUDGE: set_distance_l(-config_.static_obstacle_buffer()) // -0.3
  ```
- **作用**：双重身份：
  1. 决定静态障碍"重叠/绕行/忽略"的横向判定阈值（`min_nudge_l = 车宽/2 + buffer/2`）；
  2. 作为 NUDGE 决策的横向绕行距离（`distance_l = ±0.3m`）。
- **对车辆行为影响**：
  - 增大（如 0.5）→ 车辆更早把前方静态障碍判定为"重叠需停车"或"需绕行"，且绕行时离障碍更远（更安全，但可能压线/错过可通行间隙）；
  - 减小 → 更贴近障碍，可能穿过更窄间隙。
  - **注意**：本参数只影响 path_decider 生成的 `ObjectNudge.distance_l` 决策，实际路径偏移还由后续任务（lane_borrow/nudge）决定。
- **赛题关联**：🔴 **直接相关赛题 5（障碍物停车≥2m）/赛题 6（动态跟随/障碍）。** 该 buffer 决定静态障碍是否进入"重叠→STOP"分支；配合 `skip_overlap_stop_check` 使用。

### 7.2 skip_overlap_stop_check 🔴
- **当前值**：`true` ｜ **proto 默认值**：`false`
- **代码位置**：`path_decider.cc` `MakeStaticObstacleDecision()` 约 200 行：
  ```cpp
  } else if (sl_boundary.end_l() >= curr_l - min_nudge_l &&
             sl_boundary.start_l() <= curr_l + min_nudge_l) {
    if (config_.skip_overlap_stop_check()) {
      AINFO << "skip_overlap_stop_check";   // 跳过"横向重叠 → STOP"分支
    } else {
      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
      ...
    }
  }
  ```
- **作用**：为 true 时，**跳过"横向重叠的静态障碍 → 生成 STOP 决策"**这一分支（路径决策层面不再因横向重叠而停车）。
- **对车辆行为影响**：⚠️ **重要**：当前 true 意味着横向与路径重叠的静态障碍**不会**从 path_decider 获得 STOP 决策，停车决策改由 speed_decider / 阻塞障碍 STOP（`blocking_obstacle_id` 分支仍保留）等其他机制产生。**注意**：阻塞障碍（`obstacle->Id() == blocking_obstacle_id`）的 STOP 分支在上方独立保留，不受此开关影响。
- **赛题关联**：🔴 **直接相关赛题 4/5（障碍物停车）**。若赛题要求"障碍物前停车≥2m"且依赖 path_decider 生成 STOP，需把此值改为 false 使横向重叠障碍也能生成 STOP；当前 true 时主要依赖"阻塞障碍 STOP"分支（该分支始终生效）。

### 7.3 ignore_backward_obstacle（配置中未设置 → proto 默认 false）
- **当前值**：未设置 ｜ **proto 默认值**：`false`
- **代码位置**：`MakeObjectDecision()`：
  ```cpp
  if (config_.ignore_backward_obstacle()) {
    IgnoreBackwardObstacle(path_decision);
  }
  ```
  `IgnoreBackwardObstacle()` 把自车后方（`end_s < adc_start_s`）的动态障碍全部置 IGNORE。
- **作用**：为 true 时忽略后方动态障碍（防止倒车/被超越时误判）。
- **赛题关联**：⚪ 一般（赛事多为前进场景）。

---

## 8. path_reference_decider（路径参考线决策）

**配置文件**：`modules/planning/tasks/path_reference_decider/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/path_reference_decider/proto/path_reference_decider.proto`（`PathReferenceDeciderConfig`）
**源码**：`modules/planning/tasks/path_reference_decider/path_reference_decider.cc`

```protobuf
min_path_reference_length: 20
skip_path_reference_in_side_pass: false
skip_path_reference_in_change_lane: true
```

> 背景：该任务决定是否采用"学习模型输出轨迹"作为路径参考（Hybrid 规划）。纯规则规划（赛事默认）下学习模型通常无输出，本任务主要作为开关。

### 8.1 min_path_reference_length
- **当前值**：`20` ｜ **proto 默认值**：`20`
- **代码位置**：**当前 `path_reference_decider.cc` 中未使用**（仅存在于 proto/conf）。旧版本用于判断拼接后的参考轨迹长度是否足够。
- **作用**：保留参数，当前代码路径不读取 → **无效参数**。
- **赛题关联**：⚪ 无效。

### 8.2 skip_path_reference_in_side_pass
- **当前值**：`false` ｜ **proto 默认值**：`false`
- **代码位置**：`Process()` 约 100 行：
  ```cpp
  if (config_.skip_path_reference_in_side_pass() &&
      reference_line_info->is_path_lane_borrow()) {
    ... set_is_valid_path_reference(false); return Status::OK();
  }
  ```
- **作用**：为 true 时，在**借道（side pass）场景跳过**学习模型路径参考；当前 false → 借道时也可使用路径参考。
- **赛题关联**：⚪ 一般（依赖学习模型，赛事纯规则默认无输出）。

### 8.3 skip_path_reference_in_change_lane
- **当前值**：`true` ｜ **proto 默认值**：`true`
- **代码位置**：`Process()` 约 85 行：
  ```cpp
  if (config_.skip_path_reference_in_change_lane() &&
      frame->reference_line_info().size() > 1) {
    ... set_is_valid_path_reference(false); return Status::OK();
  }
  ```
- **作用**：为 true 时，**变道（双参考线）场景跳过**学习模型路径参考（因为变道时参考线切换频繁，学习模型输出不可靠）。当前 true。
- **赛题关联**：⚪ 一般。

---

## 9. obstacle_nudge_decider（绕行决策，重点）

**配置文件**：`modules/planning/tasks/obstacle_nudge_decider/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/obstacle_nudge_decider/proto/obstacle_nudge_decider.proto`（`ObstacleNudgeDeciderConfig`）
**源码**：`modules/planning/tasks/obstacle_nudge_decider/obstacle_nudge_decider.cc` + `nudge_calculation.cc`（核心算法）

```protobuf
enbale_nugde_static_obs: true
enbale_nugde_dynamic_obs: false
min_forward_check_dis: 30.0
max_forward_check_dis: 60.0

max_lateral_left_nudge_dis: 3.0
max_lateral_right_nudge_dis: 3.0

expand_polygon_buffer: 0.2

obs_passable_buffer: 0.7

obs_nudge_time_out_threshold: 3.0

obs_clean_time_out_threshold: 2.0

full_nudge_buffer: 1.5

nudge_key_point_extend_length_coefficient: 0.7
```

> 功能：对静态障碍生成"绕行关键点"（`extra_nudge_key_points`），供 `lane_borrow_path_generic` 生成借道边界。**这是"借道通用"方案的决策前端，直接决定赛题 5/6 绕障行为。**

### 9.1 enbale_nugde_static_obs 🔴
- **当前值**：`true` ｜ **proto 默认值**：无显式默认（false）
- **代码位置**：`obstacle_nudge_decider.cc` `Process()` 约 55 行：
  ```cpp
  if (!FLAGS_enable_nudge_decider || !config_.enbale_nugde_static_obs() || ...) {
    return Status::OK();  // 跳过整个 Nudge 计算
  }
  ```
- **作用**：静态障碍绕行决策总开关。为 false 时完全不生成绕行关键点 → generic 借道无法触发（`IsEnableNudge` 返回 false）。
- **赛题关联**：🔴 **直接相关赛题 5/6**：决定是否对静态障碍（锥桶/故障车）做绕行评估。

### 9.2 enbale_nugde_dynamic_obs（⚪ 无效参数）
- **当前值**：`false` ｜ **proto 默认值**：无显式默认（false）
- **代码位置**：**当前代码中未读取**（proto 保留字段）。`nudge_calculation.cc` 中动态障碍分支只有 `ADEBUG << "...is dynamic"` 占位，无实际逻辑。
- **作用**：保留参数。动态障碍绕行未实现。
- **赛题关联**：⚪ 无效。

### 9.3 min_forward_check_dis / max_forward_check_dis 🔴
- **当前值**：`30.0` / `60.0`（米）｜ **proto 默认值**：无显式默认（0）
- **代码位置**：`nudge_calculation.cc` `BuildNudgeDecisionWithObs()` 约 70 行：
  ```cpp
  double ego_v = std::abs(reference_line_info->vehicle_state().linear_velocity());
  check_forward_dis_ = common::math::Clamp(ego_v, 0.0, 8.0) / 8.0
          * (config_.max_forward_check_dis() - config_.min_forward_check_dis())
          + config_.min_forward_check_dis();
  ```
  车速 0→8 m/s 时，前方检查距离 30→60 m 线性插值。在 `IsWithinNudgeScopeObstacle()` 中：障碍 `start_s - adc_end_s > check_forward_dis_` 则忽略。
- **作用**：确定"多远的静态障碍会被纳入绕行评估"。低速看 30m，高速看 60m。
- **对车辆行为影响**：调大 → 更早识别远处障碍并准备绕行；调小 → 只评估近处障碍（可能来不及绕）。
- **赛题关联**：🔴 **直接影响赛题 5/6 障碍识别范围**（决定多早进入绕障准备）。

### 9.4 max_lateral_left_nudge_dis / max_lateral_right_nudge_dis（⚪ 无效参数）
- **当前值**：`3.0` / `3.0`（米）｜ **proto 默认值**：无显式默认（0）
- **代码位置**：**当前代码中未读取**（proto 保留字段）。
- **作用**：原设计用于限制左右绕行最大横向距离，当前实现未使用。
- **赛题关联**：⚪ 无效。

### 9.5 expand_polygon_buffer 🟡
- **当前值**：`0.2`（米）｜ **proto 默认值**：无显式默认（0）
- **代码位置**：`nudge_calculation.cc` `UpdateTrackingObsInfo()` 约 327 行：
  ```cpp
  track_obs_info->expand_polygon = track_obs_info->origin_polygon.ExpandByDistance(config_.expand_polygon_buffer());
  ```
- **作用**：把障碍多边形外扩 0.2m 用于**跨帧匹配**（`MatchTrackingObsInfo` 中多边形包含/IoU>0.6 判断），保证同一障碍的跟踪连续性。
- **对车辆行为影响**：外扩越大跨帧匹配越稳（障碍晃动不丢跟踪），但可能误合并相邻障碍。
- **赛题关联**：🟡 影响绕障跟踪稳定性。

### 9.6 obs_passable_buffer 🔴
- **当前值**：`0.7`（米）｜ **proto 默认值**：无显式默认（0）
- **代码位置**：`nudge_calculation.cc` `CalcObsRemainSpace()` 约 266 行：
  ```cpp
  bool is_passable = remain_space->left_lane_remain > config_.obs_passable_buffer()
          || remain_space->right_lane_remain > config_.obs_passable_buffer();
  ```
  其中 `left_lane_remain = 车道左宽 - 等效车宽 - 障碍右沿 l - static_obstacle_nudge_l_buffer`。
- **作用**：障碍物某一侧剩余可通行空间**必须 > 0.7m** 才认为"可绕行"。小于此值 → 判定不可通过 → 不产生绕行关键点（车辆停车等待）。
- **对车辆行为影响**：**核心可通行性阈值**。调小（如 0.3）→ 允许更窄间隙绕行（更激进）；调大 → 间隙不足就停车（更保守）。赛题中若障碍+路宽只留窄通道，此值决定能否绕过去。
- **赛题关联**：🔴 **直接相关赛题 5/6**（障碍物可绕/不可绕的判断核心）。

### 9.7 obs_nudge_time_out_threshold（⚪ 无效参数）
- **当前值**：`3.0`（秒）｜ **proto 默认值**：无显式默认（0）
- **代码位置**：**当前代码中未读取**（proto 保留字段）。
- **作用**：原设计用于绕行状态超时重置，当前未使用。
- **赛题关联**：⚪ 无效。

### 9.8 obs_clean_time_out_threshold 🟡
- **当前值**：`2.0`（秒）｜ **proto 默认值**：无显式默认（0）
- **代码位置**：`nudge_calculation.cc` `BuildNudgeDecisionWithObs()` 约 101 行：
  ```cpp
  if (current_time - iter->second.last_track_time > config_.obs_clean_time_out_threshold()) {
    tracking_nudge_obs_info->erase(iter);  // 清理长时间未更新的跟踪障碍
  }
  ```
- **作用**：跟踪中的绕障目标若 **>2 秒未更新**（障碍消失/离开视野）则清除跟踪记录，停止绕行。
- **对车辆行为影响**：影响障碍消失后绕行状态退出的快慢。
- **赛题关联**：🟡 影响绕障结束/恢复直行的时机。

### 9.9 full_nudge_buffer（⚪ 无效参数）
- **当前值**：`1.5`（米）｜ **proto 默认值**：无显式默认（0）
- **代码位置**：**当前代码中未读取**（proto 保留字段）。
- **作用**：原设计用于全绕行缓冲，当前未使用。
- **赛题关联**：⚪ 无效。

### 9.10 nudge_key_point_extend_length_coefficient 🟡
- **当前值**：`0.7` ｜ **proto 默认值**：无显式默认（0）
- **代码位置**：`nudge_calculation.cc` `CalcNudgeExtraSpace()` 约 532 行：
  ```cpp
  double extend_length = veh_config_.length * config_.nudge_key_point_extend_length_coefficient();
  ...
  start_s = std::min(sl_polygon.MinS() - extend_length, start_s);
  end_s = std::max(sl_polygon.MaxS() + extend_length, end_s);
  ```
- **作用**：绕行关键点的**纵向延伸长度 = 车长 × 0.7**。即在障碍前后各延伸 `车长×0.7` 米生成绕行起点/终点，保证车辆有足够空间提前变向、绕完回正。
- **对车辆行为影响**：系数越大，绕行段越长越平缓（提前开始、延后结束）；越小绕行越"贴"障碍（短促）。
- **赛题关联**：🟡 影响绕障轨迹的平滑与长度。

> **补充**：NudgeDecider 内还有两个**硬编码**（非配置）阈值值得关注：
> - `nudge_probability` 每周期 +0.08，> 0.8 才启用绕行（`IsProbEnoughToNudge`）→ 需连续约 10 个周期确认障碍不可通过才绕行；
> - `FLAGS_static_obstacle_nudge_l_buffer = 0.3`（gflag，`planning_gflags.cc`），用于计算剩余空间时额外扣除的横向缓冲。

---

## 10. reuse_path（路径复用）

**配置文件**：`modules/planning/tasks/reuse_path/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/reuse_path/proto/reuse_path.proto`（`ReusePathConfig`）
**源码**：`modules/planning/tasks/reuse_path/reuse_path.cc`

```protobuf
enable_reuse_path_in_lane_follow: true
```

### 10.1 enable_reuse_path_in_lane_follow 🟡
- **当前值**：`true` ｜ **proto 默认值**：无显式默认（false）
- **代码位置**：`reuse_path.cc` `IsPathReusable()` 约 75 行：
  ```cpp
  if (!reference_line_info->IsChangeLanePath() &&
      !config_.enable_reuse_path_in_lane_follow()) {
    ADEBUG << "skipping reusing path: not in lane_change";
    return false;
  }
  ```
- **作用**：允许在**直行跟车道（非变道）**时也复用上一帧路径（默认只允许变道时复用）。为 true 时直行场景若环境无变化（无碰撞、障碍未变化、速度优化成功）则直接沿用上帧路径，跳过路径重规划。
- **对车辆行为影响**：**关键**：路径复用开启时，若障碍物/环境不变，planning 会跳过 path_decider 等路径任务（`FLAGS_enable_skip_path_tasks`），沿用旧路径。这可能影响"障碍物停车"的响应——**但注意**：`IsPathReusable` 中有 `IsCollisionFree` 检查，且阻塞障碍消失/远离（`IsIgnoredBlockingObstacle`）时会重新规划，所以遇到新障碍会退出复用。
- **赛题关联**：🟡 影响规划响应速度与任务执行（若关闭则每帧重算路径，行为更"新鲜"但更耗算力）。

### 10.2 short_path_threshold（配置中未设置 → proto 默认 60）
- **当前值**：未设置 ｜ **proto 默认值**：`60`（路径点数）
- **代码位置**：`reuse_path.cc` `NotShortPath()` 约 296 行（被 `TrimHistoryPath()` 约 380 行调用）：
  ```cpp
  bool ReusePath::NotShortPath(const DiscretizedPath& current_path) {
    return current_path.size() >= config_.short_path_threshold();
  }
  ```
- **作用**：裁剪后的历史路径点数必须 ≥60（约 60 个路径点，结合 `path_bounds_decider_resolution` 通常 0.1-0.2m → 约 6-12m）才允许复用，防止复用太短的路径。
- **赛题关联**：⚪ 一般。

---

## 11. reverse_path（倒车路径）

**配置文件**：`modules/planning/tasks/reverse_path/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/reverse_path/proto/reverse_path.proto`（`ReversePathConfig`）
**源码**：`modules/planning/tasks/reverse_path/reverse_path.cc`

```protobuf
max_s_distance:10.0
max_lateral_distance:3.0
```

### 11.1 max_s_distance
- **当前值**：`10.0`（米）｜ **proto 默认值**：`10.0`
- **代码位置**：`reverse_path.cc` `InitPathBoundary()`：
  ```cpp
  double end_s = init_sl_state.first[0] - std::min(config_.max_s_distance(), reference_line_backward_length);
  ```
- **作用**：倒车路径**向后规划的最大纵向距离**（最多倒 10m）。
- **对车辆行为影响**：限制倒车距离，防止倒车过远。
- **赛题关联**：⚪ 一般（赛事少用倒车）。

### 11.2 max_lateral_distance
- **当前值**：`3.0`（米）｜ **proto 默认值**：`3.0`
- **代码位置**：
  - `InitPathBoundary()`：初始边界 `emplace_back(curr_s, -max_lateral_distance, max_lateral_distance)`（±3m）；
  - `GetBoundaryFromSquare()`：路口内边界 `l_min/l_max` 被 clamp 到 ±max_lateral_distance。
- **作用**：倒车路径横向边界宽度 ±3m。
- **赛题关联**：⚪ 一般。

### 11.3 is_considered_square_boundary（未设置 → 默认 false）/ is_considered_lane_boundary（未设置 → 默认 true）
- **代码位置**：`DecidePathBounds()`：`is_considered_square_boundary=true` 时用路口边界；否则若 `is_considered_lane_boundary=true` 用本车道边界。
- **作用**：决定倒车边界来源。当前用本车道边界。
- **赛题关联**：⚪ 一般。

### 11.4 path_optimizer_config（未设置 → 全部默认）
- `OptimizePathOsqp()` 使用默认权重（`l_weight=1, dl_weight=100, ddl_weight=1000, dddl_weight=10000, path_reference_l_weight=0`）。

---

## 12. pull_over_path（靠边停车路径）

**配置文件**：`modules/planning/tasks/pull_over_path/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/pull_over_path/proto/pull_over_path.proto`（`PullOverPathConfig`）
**源码**：`modules/planning/tasks/pull_over_path/pull_over_path.cc`

```protobuf
path_optimizer_config {
  l_weight: 1.0
  dl_weight: 20.0
  ddl_weight: 1000.0
  dddl_weight: 50000.0
  lateral_derivative_bound_default: 2.0
}
pull_over_destination_to_adc_buffer: 25.0
pull_over_destination_to_pathend_buffer: 4.0
pull_over_road_edge_buffer: 0.15
pull_over_approach_lon_distance_adjust_factor: 1.6
pull_over_weight: 10
```

### 12.1 path_optimizer_config
- 同第 0 节（直行权重组合）。
- **赛题关联**：⚪ 一般。

### 12.2 pull_over_destination_to_adc_buffer
- **当前值**：`25.0`（米）｜ **proto 默认值**：`25.0`
- **代码位置**：`pull_over_path.cc` `FindDestinationPullOverS()`：
  ```cpp
  if (destination_s - adc_end_s < config_.pull_over_destination_to_adc_buffer()) {
    AERROR << "Destination is too close to ADC..."; return false;
  }
  ```
- **作用**：目的地距自车**至少 25m** 才按目的地规划靠边（防止刚到终点就靠边，距离不足）。
- **对车辆行为影响**：终点前 25m 内不启动"按目的地靠边"。
- **赛题关联**：⚪ 一般（赛题无人行道靠边停车场景可用）。

### 12.3 pull_over_destination_to_pathend_buffer
- **当前值**：`4.0`（米）｜ **proto 默认值**：`10.0`（当前值比默认更小）
- **代码位置**：`FindDestinationPullOverS()`：
  ```cpp
  if (destination_s + destination_to_pathend_buffer >= path_bound.back().s) {
    AERROR << "Destination is not within path_bounds search scope"; return false;
  }
  ```
- **作用**：目的地必须在路径边界末端**之前至少 4m**（当前值；默认要求 10m）。即靠边停车位置搜索起点离路径末端要有余量。
- **对车辆行为影响**：当前 4m 比默认 10m 更宽松 → 终点更接近路径末端也可靠边。
- **赛题关联**：🟡 影响"终点靠边停车"场景（若赛题含停车位/靠边）。

### 12.4 pull_over_road_edge_buffer
- **当前值**：`0.15`（米）｜ **proto 默认值**：`0.15`
- **代码位置**：`SearchPullOverPosition()`：
  ```cpp
  if (curr_road_right_width - (curr_right_bound + adc_half_width) > config_.pull_over_road_edge_buffer()) {
    AERROR << "Not close enough to road-edge..."; is_feasible_window = false; break;
  }
  ```
- **作用**：靠边窗口必须离路边**≤0.15m**（车身边缘到路沿的距离），否则认为"不够靠边"不可靠。
- **对车辆行为影响**：决定靠边停车的贴边程度。调大 → 允许离路边稍远（更易找到位置但不够贴边）；调小 → 必须非常贴边。
- **赛题关联**：🟡 靠边停车场景贴边距离。

### 12.5 pull_over_approach_lon_distance_adjust_factor
- **当前值**：`1.6` ｜ **proto 默认值**：`1.5`（当前值略大）
- **代码位置**：`FindNearestPullOverS()`：
  ```cpp
  const double pull_over_distance = min_turn_radius * 2 * adjust_factor;
  *pull_over_s = adc_end_s + pull_over_distance;
  ```
- **作用**：最近靠边位置的纵向距离 = `最小转弯半径 × 2 × 1.6`。控制"从当前位置开始多远后找靠边点"。
- **对车辆行为影响**：越大靠边点越远（提前量越大），越小越近。
- **赛题关联**：🟡 靠边场景接近距离。

### 12.6 pull_over_weight
- **当前值**：`10` ｜ **proto 默认值**：`10`
- **代码位置**：`OptimizePath()`：
  ```cpp
  end_state[0] = pull_over_sl.l();
  ref_l.assign(path_boundary_size, end_state[0]);
  weight_ref_l.assign(path_boundary_size, config_.pull_over_weight());
  ```
- **作用**：找到靠边位置后，路径优化时把全路径参考 l 设为靠边点的 l，权重为 10，把路径"拉向路边"。
- **对车辆行为影响**：权重越大靠边越坚决；过大会导致路径横向切角过陡。
- **赛题关联**：🟡 靠边轨迹。

### 12.7 pull_over_direction / pull_over_position（未设置 → 默认 RIGHT_SIDE / DESTINATION）
- **proto 默认**：`pull_over_direction = RIGHT_SIDE`，`pull_over_position = DESTINATION`。
- **作用**：右侧靠边、按目的地搜索靠边位置。
- **赛题关联**：🟡 靠边方向/策略（中国靠右行驶，RIGHT_SIDE 正确）。

---

## 13. square_path（路口/广场路径）

**配置文件**：`modules/planning/tasks/square_path/conf/default_conf.pb.txt`
**proto**：`modules/planning/tasks/square_path/proto/square_path.proto`（`SquarePathConfig`）
**源码**：`modules/planning/tasks/square_path/square_path.cc`

```protobuf
max_lateral_distance:6.0
enable_road_boundary_constraint: true
path_optimizer_config {
  l_weight: 10.0
  dl_weight: 2.0
  ddl_weight: 10.0
  dddl_weight: 500.0
  lateral_derivative_bound_default: 2.0
  path_reference_l_weight:2000
}
```

> 功能：在路口（Junction）内生成路径，允许车辆在路口宽阔区域自由通行（如转弯、绕行路口内障碍）。

### 13.1 max_lateral_distance 🟡
- **当前值**：`6.0`（米）｜ **proto 默认值**：无显式默认（0）
- **代码位置**：`GetBoundaryFromSquare()`：
  ```cpp
  double road_left_width = config_.max_lateral_distance();
  double road_right_width = config_.max_lateral_distance();
  ...
  l_min = std::max<double>(l_min, -road_right_width);
  l_max = std::min<double>(l_max, road_left_width);
  ```
- **作用**：路口内路径的横向搜索范围 ±6m（超宽路口内可大范围横向移动）。
- **对车辆行为影响**：决定车辆在路口内可横向偏移多大（转弯切角/绕路口内障碍）。6m 较大，路口内机动灵活。
- **赛题关联**：🟡 路口/广场内绕行、转弯。

### 13.2 enable_road_boundary_constraint 🟡
- **当前值**：`true` ｜ **proto 默认值**：`true`
- **代码位置**：`GetBoundaryFromSquare()`：
  ```cpp
  if (config_.enable_road_boundary_constraint()) {
    reference_line.GetRoadWidth(point.s, &road_left_width, &road_right_width);
    road_left_width = std::min(road_left_width, config_.max_lateral_distance());
    road_right_width = std::min(road_right_width, config_.max_lateral_distance());
  }
  ```
- **作用**：为 true 时用**真实道路宽度**（再被 max_lateral_distance=6 截断）作为路口边界；为 false 时直接用 ±6m。
- **对车辆行为影响**：true 更贴合实际道路（不会让路径超出路面），false 在路口更大胆。
- **赛题关联**：🟡 路口路径的合法边界。

### 13.3 path_optimizer_config
- **当前值**：`l_weight=10.0, dl_weight=2.0, ddl_weight=10.0, dddl_weight=500.0, path_reference_l_weight=2000, lateral_derivative_bound_default=2.0`
- **与默认差异**：`l_weight` 10、`dl_weight` 2（默认 100）、`ddl_weight` 10（默认 1000）、`dddl_weight` 500（默认 10000）、`path_reference_l_weight` 2000。
- **作用**：与 generic 借道类似但更激进——**允许路口内大横向速度/曲率机动**（`dl/ddl/dddl` 权重极小），`path_reference_l_weight=2000` 使路径强拉向路口内障碍通道中点。
- **对车辆行为影响**：路口内转弯/绕障路径可以较"紧"地走，机动性强；平滑性相对直行任务低。
- **赛题关联**：🟡 影响赛题 8（绿灯路口≤5m/s）等路口场景的路径灵活性。

---

## 14. 赛题关联总结

### 🔴 直接相关（赛题 4/5/6：障碍物停车≥2m / 障碍物停车 / 动态跟随）

| 参数 | 所在任务 | 当前值 | 作用一句话 |
|------|---------|--------|-----------|
| `static_obstacle_buffer` | path_decider | 0.3 | 静态障碍"重叠/绕行/忽略"横向判定阈值 + NUDGE 距离 |
| `skip_overlap_stop_check` | path_decider | **true** | 跳过"横向重叠静态障碍→STOP"，停车改由阻塞障碍 STOP 分支负责 |
| `enbale_nugde_static_obs` | obstacle_nudge_decider | true | 静态障碍绕行评估总开关 |
| `min/max_forward_check_dis` | obstacle_nudge_decider | 30/60 | 障碍纳入绕行评估的前方距离（随车速 30-60m） |
| `obs_passable_buffer` | obstacle_nudge_decider | 0.7 | 障碍两侧剩余空间 >0.7m 才判定"可绕行"（**赛题 5/6 可绕性核心**） |

### 🟡 间接相关（绕行/借道行为）

| 参数 | 所在任务 | 当前值 | 作用一句话 |
|------|---------|--------|-----------|
| `is_allow_lane_borrowing` | lane_borrow(_generic) | true | 借道总开关 |
| `lane_borrow_max_speed` | lane_borrow / generic | 5.0 / **10.0** | 允许借道的最大车速 |
| `long_term_blocking_obstacle_cycle_threshold` | lane_borrow(_generic) | 3 | 长期阻塞判定周期（generic 已禁用） |
| `enable_extend_boundary_by_adc` | lane_borrow_generic | false | 借道边界是否按 ADC 外扩 |
| `enable_ignore_boundary_type` | lane_borrow_generic | false | 是否忽略实线边界（false=实线不借道） |
| `enable_nudge_destination_threshold` | lane_borrow_generic | 8.0 | 障碍距终点 ≥8m 才借道 |
| `enable_active_trigger` | lane_borrow_generic | true(默认) | 无阻塞障碍也可主动借道 |
| `path_reference_l_weight` | 各路径任务 | 100~2000 | 绕障通道内拉向中点强度 |
| `expand_polygon_buffer` | obstacle_nudge_decider | 0.2 | 障碍多边形外扩用于跨帧匹配 |
| `obs_clean_time_out_threshold` | obstacle_nudge_decider | 2.0 | 跟踪障碍超时清理 |
| `nudge_key_point_extend_length_coefficient` | obstacle_nudge_decider | 0.7 | 绕行关键点前后延伸 = 车长×0.7 |
| `enable_reuse_path_in_lane_follow` | reuse_path | true | 直行时复用上帧路径（跳过重规划） |
| `max_lateral_distance` / `enable_road_boundary_constraint` | square_path | 6.0 / true | 路口内横向机动范围 |
| `pull_over_destination_to_pathend_buffer` 等 | pull_over_path | 4.0 等 | 终点靠边停车行为 |

### ⚪ 一般相关（平滑/变道/兜底/参考线）
- 各任务 `path_optimizer_config` 的 `l/dl/ddl/dddl_weight`、`lateral_derivative_bound_default`、`weight_end_state_*`、`max_iteration`
- `is_extend_lane_bounds_to_include_adc` / `extend_buffer`（lane_follow / fallback）
- `extend_adc_buffer`、`change_lane_success/fail_freeze_time`、`lane_change_prepare_length`（lane_change）
- `min_path_reference_length`、`skip_path_reference_in_*`（path_reference_decider，与学习模型相关，纯规则赛事基本无效）
- `max_s_distance`、`max_lateral_distance`（reverse_path）
- `short_path_threshold`（reuse_path）

### ⚪ 当前代码中未使用的"保留参数"（改了也无效，勿浪费调参时间）
- `min_path_reference_length`（path_reference_decider）
- `enbale_nugde_dynamic_obs`、`max_lateral_left_nudge_dis`、`max_lateral_right_nudge_dis`、`obs_nudge_time_out_threshold`、`full_nudge_buffer`（obstacle_nudge_decider）
- `long_term_blocking_obstacle_cycle_threshold`（lane_borrow_path_generic，检查已注释）
- `is_extend_lane_bounds_to_include_adc`（fallback_path，代码未读取）

---

## 附：涉及的关键 GFlag（非 Task 配置，但影响行为）

来自 `modules/planning/planning_base/gflags/planning_gflags.cc`：

| GFlag | 默认值 | 作用 |
|-------|--------|------|
| `FLAGS_enable_nudge_decider` | true | Nudge 决策总开关（与 `enbale_nugde_static_obs` 同时为 true 才生效） |
| `FLAGS_static_obstacle_nudge_l_buffer` | 0.3 | 计算剩余空间时额外扣除的横向缓冲 |
| `FLAGS_static_obstacle_speed_threshold` | 0.5 | 障碍"静态"判定速度阈值（>0.5 视为动态） |
| `FLAGS_lateral_ignore_buffer` | 3.0 | path_decider 横向忽略缓冲 |
| `FLAGS_enable_skip_path_tasks` | false | 复用路径时跳过路径任务 |
| `FLAGS_num_extra_tail_bound_point` | 20 | 路径尾部补充点数 |
| `FLAGS_path_trim_destination_threshold` | 20.0 | 距终点 20m 内裁剪路径 |
| `FLAGS_use_front_axe_center_in_path_planning` | false | 是否用前轴中心规划 |
