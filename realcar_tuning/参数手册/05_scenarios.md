# 05 planning scenarios 17 个（回归代码版）

> 配置：`realcar_tuning/ORIGINAL/modules/planning/scenarios/<scenario>/conf/`
> 源码：`modules/planning/scenarios/<scenario>/`（✅ 已恢复官方基线 2b48d5e，无魔改）
> 标注：⭐=比赛核心 ○=一般 △=无关/泊车

---

## 1. traffic_light_protected（红灯场景）⭐

| 参数 | 实车值 | 含义（回归代码） | 影响/备注 |
|------|--------|------------------|-----------|
| start_traffic_light_scenario_distance | 5.0 | `IsTransferable`：车头到首个信号灯 ≤5m 且非绿灯进入场景 | 调大→更早进红灯场景 ⭐ |
| max_valid_stop_distance | 2.0 | Approach 需车头距停止线 ≤2.0 且绿灯才切巡航 | 调大→停车更靠后、绿灯放行更早 ⭐ |

**pipeline**：`TRAFFIC_LIGHT_PROTECTED_APPROACH`（停车等待）→ `TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE`（绿灯过路口）。任务链 = LaneFollowPath → LaneBorrowPath → FallbackPath → PathDecider → RuleBasedStopDecider → STBoundsDecider → 速度五件套。红灯 STOP fence 在 RuleBasedStopDecider 生成。

## 2. stop_sign_unprotected（停止标志场景）⭐

| 参数 | 实车值 | 含义 | 影响/备注 |
|------|--------|------|-----------|
| start_stop_sign_scenario_distance | 15.0 | 车头到停止牌 ≤15m 进入场景 | 实车 15 > 官方 conf 4.0 → 更早触发 ⭐ |
| watch_vehicle_max_valid_stop_distance | 5.0 | 关联车道车辆距停止线 ≤5m 加入 watch | ⭐ |
| max_valid_stop_distance | 6.0 | 官方判定有效停车距离 | ⚠️ 本地魔改已删位置检查，仅参考 ⭐ |
| stop_duration_sec | 1.0 | STOP 等待时间 ≥1s 放行 | 调大→停更久 ⭐ |
| stop_timeout_sec | 8.0 | 等待单个车辆超时强制放行 | 调大→更久等车 ⭐ |
| creep_timeout_sec | 10.0 | 蠕行超时 | ⚠️ 魔改已跳过蠕行 ⭐ |
| creep_stage_config.min_boundary_t | 6.0 | 蠕行判定只查 min_t<6s 障碍 | ⭐ |
| creep_stage_config.ignore_max_st_min_t | 0.1 | 忽略同向移动障碍阈值 | ⭐ |
| creep_stage_config.ignore_min_st_min_s | 15.0 | 忽略障碍最小 s 距离 | ⭐ |

**pipeline**（4 stage）：`PRE_STOP` → `STOP` → `CREEP` → `INTERSECTION_CRUISE`。⚠️ 魔改版 FinishStage 从 STOP 直接跳 INTERSECTION_CRUISE（省蠕行）。

## 3. lane_follow（巡航场景）⭐

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| （无场景参数） | — | `IsTransferable` 恒 true，默认兜底场景 | ⭐ 巡航是兜底 |

**pipeline**（1 stage，巡航核心任务链）：
1. `OBSTACLE_NUDGE_DECIDER` 障碍绕行
2. `LANE_CHANGE_PATH_GNERIC` 换道
3. `LANE_FOLLOW_PATH` 车道保持
4. `LANE_BORROW_PATH_GENERIC` 借道
5. `FALLBACK_PATH` 兜底
6. `PATH_DECIDER` 路径决策
7. `RULE_BASED_STOP_DECIDER` ⭐ 红绿灯/停止牌/人行道 STOP 在此生成
8. `SPEED_BOUNDS_PRIORI_DECIDER` → 9. `SPEED_HEURISTIC_OPTIMIZER`(DP) → 10. `SPEED_DECIDER` → 11. `SPEED_BOUNDS_FINAL_DECIDER` → 12. `PIECEWISE_JERK_SPEED`(QP)
- fallback：`SMOOTH_STOP_TRAJECTORY_FALLBACK`；NONLINEAR 被注释

## 4. traffic_light_unprotected_left_turn（无保护左转）⭐

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| start_traffic_light_scenario_distance | 5.0 | 进入场景距离 | ⚠️ 实车 5 << 官方 conf 30.0（差异 6 倍，更晚进入红灯场景）⭐ |
| approach_cruise_speed | 2.78 | Approach 限速 2.78 m/s 接近 | 调大→接近更快 ⭐ |
| max_valid_stop_distance | 2.0 | 停止线判定距离（实车 2.0 < 默认 3.5） | ⭐ |
| creep_timeout_sec | 10.0 | 蠕行超时 | ⭐ |
| max_adc_speed_before_creep | 5.56 | 车速 >5.56 跳过蠕行 | 调大→更易跳过蠕行 ⭐ |
| creep_stage_config（3 项） | 6.0/0.1/15.0 | 蠕行判定 | ⭐ |

**pipeline**：`APPROACH` → `CREEP` → `INTERSECTION_CRUISE`。

## 5. traffic_light_unprotected_right_turn（无保护右转）⭐

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| start_traffic_light_scenario_distance | 5.0 | 进入场景距离 | ⭐ |
| enable_right_turn_on_red | false | false=红灯必须停 | 调 true 允许红灯右转 ⭐ |
| max_valid_stop_distance | 2.0 | 停止线判定距离 | ⭐ |
| min_pass_s_distance | 3.0 | 车头越过停止线 >3m 可继续通过 | 调大→要求越过更多 ⭐ |
| red_light_right_turn_stop_duration_sec | 3.0 | 红灯右转前等待 3s | ○ enable=false 不生效 |
| creep_timeout_sec | 10.0 | 蠕行超时 | ⭐ |
| max_adc_speed_before_creep | 3.0 | 车速 >3.0 跳过蠕行 | 比左转保守 ⭐ |
| creep_stage_config（3 项） | 6.0/0.1/15.0 | 蠕行判定 | ⭐ |

**pipeline**：`STOP`（TrafficLightUnprotectedRightTurnStageStop）→ `CREEP` → `INTERSECTION_CRUISE`。

## 6. bare_intersection_unprotected（无保护裸路口）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| start_bare_intersection_scenario_distance | 25.0 | 距 PNC_JUNCTION ≤25m 进入 | ○ |
| enable_explicit_stop | false | false=不停车直接减速通过 | 调 true 会停车 ○ |
| approach_cruise_speed | 6.7056 | 接近限速 6.7 m/s | ○ |
| stop_distance | 0.5 | 仅 explicit_stop 时生效 | △ |

**pipeline**：`APPROACH` → `INTERSECTION_CRUISE`。

## 7. yield_sign（让行牌）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| start_yield_sign_scenario_distance | 10.0 | 距让行牌 ≤10m 进入 | ○ |
| max_valid_stop_distance | 4.5 | 车头距让行牌 <4.5 且路口清空完成 | ○ |
| creep_timeout_sec | 10.0 | 蠕行超时 | ○ |
| creep_stage_config（3 项） | 6.0/0.1/15.0 | 蠕行判定 | ○ |

**pipeline**：`APPROACH` → `CREEP`。

## 8. park_and_go（停车后起步）⭐

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| front_obstacle_buffer | 10.0 | 前方 10m 内非虚拟障碍即判被堵不能起步 | 实车 10 > 默认 4 → 起步更保守 ⭐ |
| heading_buffer | 0.3 | 航向与参考线夹角须 ∈ (-0.1, 0.3) rad 才起步（下限由 heading_buffer-0.2 硬编码） | 实车 0.3 < 默认 0.5 → 更严格 ⭐ |
| min_dist_to_dest | 10.0 | 距目的地 >10m 才进入 ParkAndGo | 实车 10 < 默认 25 → 更易触发 ⭐ |
| max_steering_percentage_when_cruise | 20.0 | 转角 <20% 才切 CRUISE | 实车 20 < 默认 90 → 要求方向回正 ⭐ |

**pipeline**（4 stage）：`CHECK` → `ADJUST` → `PRE_CRUISE` → `CRUISE`。

## 9. pull_over（靠边停车）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| start_pull_over_scenario_distance | 20.0 | 距目的地 ≤20m 进入 | 实车 20 < 默认 50 → 更晚触发 ○ |
| pull_over_min_distance_buffer | 5.0 | 距目的地 ≥5m 才允许进入 | ○ |
| max_distance_stop_search | 5.0 | 距目的地 <5m 禁止进入 | ○ |
| max_s_error_to_end_point | 0.5 | 完成判定 s 误差 | ○ |
| max_l_error_to_end_point | 0.5 | 完成判定 l 误差 | ○ |
| max_theta_error_to_end_point | 0.2 | 航向误差 | ○ |
| pass_destination_threshold | 15.0 | 超过目标 15m 判冲过 | ○ |
| max_distance_error_to_end_point | 0.2 | 距离误差 | △ |

**pipeline**：`APPROACH` → `RETRY_PARKING`。

## 10. emergency_pull_over（紧急靠边）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| max_stop_deceleration | 4.5 | 减速斜率 4.5 m/s² | 实车 4.5 > 默认 3.0 → 减速更猛 ○ |
| slow_down_deceleration_time | 3.0 | 减速时间窗 | ○ |
| target_slow_down_speed | 2.0 | 目标减速速度 | ○ |
| stop_distance | 1.5 | 停车点距靠边点 | ○ |

**pipeline**：`SLOW_DOWN` → `APPROACH` → `STANDBY`。

## 11. emergency_stop（紧急停车）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| max_stop_deceleration | 6.0 | 制动距离计算 6.0 | 调小→制动距离变长 ○ |
| stop_distance | 1.0 | 停车点缓冲 | ○ |

**pipeline**：`APPROACH` → `STANDBY`，fallback=FAST_STOP。

## 12. free_space（自由空间泊车）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| enabled_perception_obstacles | true | 感知障碍加载进 ROI | △ |
| perception_obstacle_buffer | 0.2 | 障碍膨胀 0.2m | △ |

**pipeline**：单 stage `STAGE_FREE_SPACE`。

## 13. valet_parking / valet_parking_park（代客泊车）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| parking_spot_range_to_start | 20.0 | 距停车位 ≤20m 进入 | △ |
| max_valid_stop_distance | 3.0 / 6.0 | 距预停点判完成（park 版极宽松） | △ |
| max_heading_error / max_lat_error / max_lon_error | 0.35/0.45/0.35 | 泊入误差触发 retry | △ |

## 14. lane_follow_park（排队/泊出）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| escape_reverse_distance | 10.0 | 倒车逃逸距离 | 实车 10 > 默认 5 ○ |
| escape_stop_distance | 2.5 | 倒车停止点 | ○ |
| success_path_count | 10 | 连续成功次数 | ○ |
| queue_check_count | 100 | 排队检测窗口 | ○ |
| stable_block_count | 5 | 稳定阻塞 ≥5 判被堵 | 实车 5 < 默认 30 → 更早逃逸 ○ |

**pipeline**：`LANE_FOLLOW_STAGE` → `LANE_ESCAPE_PARK_STAGE`。

## 15. large_curvature（大曲率/掉头）△

> 实车含 open_space 系列 stage 配置（fallback_decider_park/path_planning/replan/roi/rt_optimizer/trajectory_optimizer/post_process），全部为掉头（U-Turn）开空间规划，比赛无掉头题 → △。

## 16. square（场地赛 Square）△

> SQUARE 场景已在 public_road_planner_config 注释禁用；实车 conf 含 extricate_stage/square_lane_follow_stage 配置（lane_borrow_path_generic/piecewise_jerk_speed/path_decider），无比赛影响 → △。
