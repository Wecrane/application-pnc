# 10 快速调参指南（Planning + Control 全参数手册）

> **用途**：实车赛现场快速查找/修改参数。覆盖 **Planning 全局 gflags + TrafficRules + Tasks + Scenarios + Control 全局 + LQR/PID** 全部可调参数。
> **生效铁律（官方确认）**：只改参数 → 改哪个模块就重启哪个模块，**不用编译、不用 `aem profile use default`**！
> **🚨 铁律：所有情况以实车为准！** 本表参数值/默认值基于本地官方代码 + 仿真沉淀推断，**实车可能不同**（代码版本、插件、已有配置、评分细则）——现场以 `cat /apollo/modules/.../conf/xxx` 实车实际内容为准，本表仅作参考基线。
> **⚠️ 关键提示**：以下参数中，**"标注 ⭐需手动添加"** 的是源码已定义但**当前配置文件里没有**的——需要自己加进 conf 才能生效（语法正确、默认值已知）。
> 配置文件路径基座：`profiles/default/modules/...`（比赛只改这里）；源码默认值见各 proto/`*_gflags.cc`。

---

## 0. 快速使用流程

```bash
# 1. 找到参数所在 conf（本表已标路径）
# 2. 修改 profiles/ 下对应文件（⭐ 参数需手动添加）
# 3. 重启对应模块：
bash shiche/car_connect/apply_config.sh planning    # 改 planning 相关
bash shiche/car_connect/apply_config.sh control     # 改 control 相关
# 4. 验证：ssh car "aem exec -- cat /apollo/modules/planning/.../conf/xxx"
```

**修改生效关系速查**：
| 改的文件 | 重启模块 |
|---------|---------|
| `planning_component/conf/planning.conf` | Planning |
| `traffic_rules/*/conf/default_conf.pb.txt` | Planning |
| `tasks/*/conf/default_conf.pb.txt` | Planning |
| `scenarios/*/conf/*.pb.txt` | Planning |
| `control_component/conf/control.conf` | Control |
| `controllers/*/conf/controller_conf.pb.txt` | Control |

---

## 1. Planning 全局 gflags（`planning_component/conf/planning.conf`）

> 源码全量定义：`modules/planning/planning_base/gflags/planning_gflags.cc`。以下为**实车赛相关**的高价值参数。

### 1.1 已在 planning.conf 中的参数（直接改）

| 参数 | 当前值 | 作用 | 赛题 |
|------|--------|------|------|
| `--planning_upper_speed_limit` | 20.00 | 规划最高速度上限（m/s） | 通用 ⚠️别调小（自锁陷阱） |
| `--default_cruise_speed` | 11.18 | 巡航目标速度（m/s） | 赛题0 定速巡航 |
| `--destination_check_distance` | 4.0 | 距终点多近判定到达 | 赛题5 |
| `--nonstatic_obstacle_nudge_l_buffer` | 0.4 | 动态障碍绕行横向缓冲 | 通用 |
| `--min_length_for_lane_change` | 5.0 | 变道最小长度 | 通用 |
| `--enable_reference_line_stitching` | false | 参考线拼接 | 通用 |
| `--enable_pull_over_at_destination` | false | 终点自动靠边停车 | 赛题5（若误触发可关） |
| `--use_st_drivable_boundary` | false | 用 ST 可行驶边界 | 通用 |

### 1.2 ⭐ 需手动添加的关键 gflags（源码默认值已查）

| 参数 | 源码默认值 | 作用 | 赛题 |
|------|-----------|------|------|
| `--speed_bump_speed_limit` | 4.4704 | 减速带限速（m/s） | **赛题1**（若不用插件） |
| `--min_stop_distance_obstacle` | 6.0 | 车道内障碍最小停车距离（m） | **赛题5** ⚠️官方 6m，别改小 |
| `--max_stop_distance_obstacle` | 10.0 | 车道内障碍最大停车距离（m） | 赛题5 |
| `--follow_min_distance` | 3.0 | 最小跟车距离（m） | 赛题6 |
| `--yield_distance` | 5.0 | 让行距离（m） | 通用 |
| `--follow_time_buffer` | 2.5 | 跟车时间缓冲（s） | 赛题6 |
| `--static_obstacle_nudge_l_buffer` | 0.3 | 静态障碍绕行横向缓冲 | 通用 |
| `--lateral_ignore_buffer` | 3.0 | 横向超过此距离忽略障碍 | 通用 |
| `--obstacle_lat_buffer` | 0.4 | 路径边界障碍横向缓冲 | 通用 |
| `--obstacle_lon_start_buffer` | 3.0 | 路径边界障碍纵向起始缓冲 | 通用 |
| `--obstacle_lon_end_buffer` | 2.0 | 路径边界障碍纵向结束缓冲 | 通用 |
| `--speed_lower_bound` | -0.1 | 最低速度（m/s） | 通用 |
| `--speed_upper_bound` | 40.0 | 最高速度（m/s） | 通用 |
| `--longitudinal_acceleration_upper_bound` | 4.0 | 最大纵向加速度（m/s²） | 起步超冲 |
| `--longitudinal_acceleration_lower_bound` | -6.0 | 最小纵向加速度（m/s²） | 刹车强度 |
| `--longitudinal_jerk_upper_bound` | 2.0 | 最大纵向 jerk | 平顺性 |
| `--trajectory_time_length` | 8.0 | 轨迹时长（s） | 通用 |
| `--slowdown_profile_deceleration` | -4.0 | 减速曲线减速度 | 通用 |
| `--near_stop_speed` | 0.5 | 接近停止判定速度 | 停车精度 |
| `--near_stop_deceleration` | -1.0 | 接近停止减速度 | 停车精度 |
| `--default_city_road_speed_limit` | 15.67 | 城市道路默认限速（m/s） | 通用 |
| `--default_highway_speed_limit` | 29.06 | 高速默认限速（m/s） | 通用 |
| `--replan_lateral_distance_threshold` | 0.5 | 重规划横向距离阈值 | 通用 |
| `--replan_longitudinal_distance_threshold` | 2.5 | 重规划纵向距离阈值 | 通用 |
| `--message_latency_threshold` | 0.02 | 消息延迟阈值（s） | 通用 |

**gflags 语法**（bool 类型）：
```text
--enable_xxx            # true
--noenable_xxx          # false（注意前缀 no）
--enable_xxx=true       # 也可以
```

---

## 2. TrafficRules（`traffic_rules/<rule>/conf/default_conf.pb.txt`）

### 2.1 Crosswalk（人行道）⭐ 赛题2/7 核心

> 文件：`traffic_rules/crosswalk/conf/default_conf.pb.txt` ｜ proto：`crosswalk.proto`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `stop_distance` | 1.0 | 1.0 | 人行道停止线前停车距离（m） |
| `max_stop_deceleration` | 4.0 | 4.0 | 最大停车减速度（m/s²） |
| `min_pass_s_distance` | 1.0 | 1.0 | 通过判定距离 |
| `expand_s_distance` | 2.0 | 2.0 | 人行道纵向扩展（行人检测范围） |
| `stop_strict_l_distance` | 5.0 | 4.0 | 横向<此值必须停车 |
| `stop_loose_l_distance` | 5.0 | 5.0 | 横向>此值可忽略 |
| `stop_timeout` | 10.0 | 10.0 | 行人静止超时后放行（s） |
| `start_watch_timer_distance` | ⭐缺失 | 40.0 | 开始计时距离（**需手动添加**） |

### 2.2 TrafficLight（红绿灯）⭐ 赛题3/8 核心

> 文件：`traffic_rules/traffic_light/conf/default_conf.pb.txt` ｜ proto：`traffic_light.proto`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `enabled` | true | true | 是否启用 |
| `stop_distance` | 1.0 | 1.0 | 停止线前停车距离（m） |
| `max_stop_deceleration` | 4.0 | 4.0 | 最大停车减速度 |

### 2.3 StopSign（停止标志）⭐ 赛题4 核心

> 文件：`traffic_rules/stop_sign/conf/default_conf.pb.txt` ｜ proto：`stop_sign.proto`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `enabled` | true | true | 是否启用 |
| `stop_distance` | 1.0 | 1.0 | 停止线前停车距离（m） |

### 2.4 Destination（终点）⭐ 赛题5

> 文件：`traffic_rules/destination/conf/default_conf.pb.txt` ｜ proto：`destination.proto`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `stop_distance` | 0.2 | 0.5 | 终点停车距离（m） |

### 2.5 ReferenceLineEnd（参考线终点）

> 文件：`traffic_rules/reference_line_end/conf/default_conf.pb.txt`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `stop_distance` | 0.5 | 0.5 | 参考线终点停车距离 |
| `min_reference_line_remain_length` | 50.0 | 50.0 | 剩余参考线最短长度 |

### 2.6 KeepClear（禁停区）

> 文件：`traffic_rules/keepclear/conf/default_conf.pb.txt`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `enable_keep_clear_zone` | true | — | 启用禁停区 |
| `enable_junction` | true | — | 启用路口禁停 |
| `min_pass_s_distance` | 2.0 | 2.0 | 通过判定距离 |
| `align_with_traffic_sign_tolerance` | 4.5 | 4.5 | 与标志对齐容差 |

### 2.7 Rerouting（重路由）

> 文件：`traffic_rules/rerouting/conf/default_conf.pb.txt`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `cooldown_time` | 3.0 | 3.0 | 重路由冷却时间（s） |
| `prepare_rerouting_time` | 2.0 | 2.0 | 准备重路由时间（s） |

### 2.8 YieldSign（让行标志）

> 文件：`traffic_rules/yield_sign/conf/default_conf.pb.txt`

| 参数 | 源码默认 | 作用 |
|------|---------|------|
| `stop_distance` | 1.0 | 让行线停车距离 |

### 2.9 BacksideVehicle（后方车辆）

> 文件：`traffic_rules/backside_vehicle/conf/default_conf.pb.txt`

| 参数 | 源码默认 | 作用 |
|------|---------|------|
| `backside_lane_width` | 4.0 | 后方车道宽度 |

### 2.10 ⭐ SceneSpeedLimit（限速插件）赛题1/7/8 核心

> 文件：`traffic_rules/scene_speed_limit/conf/default_conf.pb.txt` ｜ proto：`scene_speed_limit.proto`
> ✅ **实车预计已预装编译好**（主办方提供，SCENE_SPEED_LIMIT 已在 traffic_rule_config 注册）→ 现场确认后**直接改配置即可**，无需自己加注册项/编译。
> ⚠️ 若现场确认实车**没有**该插件（`ls ~/apollo*/modules/planning/traffic_rules/scene_speed_limit` 无结果）：才需按 references/03 手动安装 + 添加 `rule{name:"SCENE_SPEED_LIMIT",type:"SceneSpeedLimit"}` + 编译一次。

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `crosswalk_enabled` | false | true | 人行道限速开关 |
| `crosswalk_forward_buffer` | 1.0 | 5.0 | 人行道前缓冲（m） |
| `crosswalk_backward_buffer` | 1.0 | 2.0 | 人行道后缓冲（m） |
| `crosswalk_limit_speed` | 1.0 | 3.0 | 人行道限速（m/s）⭐赛题7 |
| `traffic_light_enabled` | false | true | 红绿灯限速开关 |
| `traffic_light_forward_buffer` | 1.0 | 8.0 | 红绿灯前缓冲（m） |
| `traffic_light_backward_buffer` | 1.0 | 2.0 | 红绿灯后缓冲（m） |
| `traffic_light_limit_speed` | 1.0 | 4.0 | 红绿灯限速（m/s）⭐赛题8 |
| `speed_bump_enabled` | false | true | 减速带限速开关 |
| `speed_bump_forward_buffer` | 1.0 | 3.0 | 减速带前缓冲（m） |
| `speed_bump_backward_buffer` | 1.0 | 1.0 | 减速带后缓冲（m） |
| `speed_bump_limit_speed` | 1.0 | 2.0 | 减速带限速（m/s）⭐赛题1 |

---

## 3. Tasks（`tasks/<task>/conf/default_conf.pb.txt`）

### 3.1 SpeedDecider（速度决策）⭐ 赛题5/6 核心

> 文件：`tasks/speed_decider/conf/default_conf.pb.txt` ｜ proto：`speed_decider.proto`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `follow_min_obs_lateral_distance` | 2.5 | 2.5 | 跟车障碍最小横向距离 |
| `follow_min_time_sec` | 2.0 | 2.0 | 跟车最短时间 ⭐赛题6（调小提速） |
| `is_stop_for_pedestrain` | ⭐缺失 | false | 是否为行人停车（需手动添加） |
| `keep_clear_last_point_speed` | ⭐缺失 | 0.8 | 禁停区最后点速度 |
| `overtake_time_buffer` | ⭐缺失 | 3.0 | 超车时间缓冲 |
| `overtake_min_distance` | ⭐缺失 | 10.0 | 超车最小距离 |
| `yield_distance_buffer` | ⭐缺失 | 5.0 | 让行距离缓冲 |
| `stop_follow_distance` | 2.0 | 2.0 | 停止跟车距离 |
| `follow_distance_scheduler` | 有 | — | 跟车距离调度（速度→斜率） |

### 3.2 RuleBasedStopDecider（规则停车决策）

> 文件：`tasks/rule_based_stop_decider/conf/default_conf.pb.txt`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `max_adc_stop_speed` | 0.5 | 0.5 | 判定已停的最大速度 |
| `max_valid_stop_distance` | 1.0 | 1.0 | 有效停车最大距离 |
| `search_beam_length` | 20.0 | 20.0 | 搜索光束长度 |
| `search_range` | 3.14 | 3.14 | 搜索范围（rad） |
| `is_block_angle_threshold` | 0.5 | 0.5 | 阻塞角度阈值 |
| `enable_lane_change_urgency_checking` | false | false | 变道紧急检查 |
| `short_path_length_threshold` | 20.0 | 20.0 | 短路径长度阈值 |

### 3.3 PiecewiseJerkSpeed（速度优化）⭐ 平顺性

> 文件：`tasks/piecewise_jerk_speed/conf/default_conf.pb.txt`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `acc_weight` | 1.0 | 1.0 | 加速度权重 |
| `jerk_weight` | 3.0 | 10.0 | jerk 权重（越大越平顺） |
| `kappa_penalty_weight` | 200.0 | 1000.0 | 曲率惩罚权重 |
| `ref_s_weight` | 1.0 | 10.0 | 参考 s 权重 |
| `ref_v_weight` | 10.0 | 10.0 | 参考速度权重（越大越贴限速） |
| `follow_distance_buffer` | 0.1 | 8.0 | 跟车距离缓冲 |

### 3.4 LaneFollowPath（路径优化）⭐ 贴线/切弯

> 文件：`tasks/lane_follow_path/conf/default_conf.pb.txt`

| 参数 | 当前值 | 作用 |
|------|--------|------|
| `is_extend_lane_bounds_to_include_adc` | true | 扩展车道边界含自车 |
| `extend_buffer` | 0.2 | 扩展缓冲 |
| `path_optimizer_config.l_weight` | 1.0 | 横向偏移权重 |
| `path_optimizer_config.dl_weight` | 20.0 | 横向速度权重 |
| `path_optimizer_config.ddl_weight` | 1000.0 | 横向加速度权重 |
| `path_optimizer_config.dddl_weight` | 50000.0 | 横向 jerk 权重 |
| `path_optimizer_config.lateral_derivative_bound_default` | 2.0 | 横向导数边界 |
| `path_optimizer_config.path_reference_l_weight` | 100 | 贴参考线权重 ⭐切弯/贴线调这个 |

### 3.5 SpeedBoundsDecider（速度边界）

> 文件：`tasks/speed_bounds_decider/conf/default_conf.pb.txt`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `total_time` | 7.0 | — | 总时间 |
| `boundary_buffer` | 0.25 | — | 边界缓冲 |
| `max_centric_acceleration_limit` | 0.8 | — | 最大向心加速度 |
| `point_extension` | 0.0 | — | 点扩展 |
| `lowest_speed` | 0.1 | — | 最低速度 |
| `static_obs_nudge_speed_ratio` | 0.6 | — | 静态障碍绕行速度比 |
| `dynamic_obs_nudge_speed_ratio` | 0.8 | — | 动态障碍绕行速度比 |
| `enable_nudge_slowdown` | true | — | 绕行减速开关 |
| `lane_change_obstacle_nudge_l_buffer` | 0.3 | — | 变道绕行横向缓冲 |
| `max_trajectory_len` | 1000.0 | — | 最大轨迹长度 |

### 3.6 STBoundsDecider

> 文件：`tasks/st_bounds_decider/conf/default_conf.pb.txt`

| 参数 | 当前值 | 作用 |
|------|--------|------|
| `total_time` | 7.0 | ST 总时间 |

### 3.7 PathDecider（路径决策）

> 文件：`tasks/path_decider/conf/default_conf.pb.txt`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `static_obstacle_buffer` | 0.3 | 0.3 | 静态障碍缓冲 |
| `skip_overlap_stop_check` | true | true | 跳过重叠停止检查 |

### 3.8 ObstacleNudgeDecider（障碍绕行决策）

> 文件：`tasks/obstacle_nudge_decider/conf/default_conf.pb.txt`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `enbale_nugde_static_obs` | true | — | 静态障碍绕行开关 |
| `enbale_nugde_dynamic_obs` | false | — | 动态障碍绕行开关 |
| `min_forward_check_dis` | 30.0 | — | 最小前向检查距离 |
| `max_forward_check_dis` | 60.0 | — | 最大前向检查距离 |
| `max_lateral_left_nudge_dis` | 3.0 | — | 最大左绕距离 |
| `max_lateral_right_nudge_dis` | 3.0 | — | 最大右绕距离 |
| `expand_polygon_buffer` | 0.2 | — | 多边形扩展缓冲 |
| `obs_passable_buffer` | 0.7 | — | 障碍可通过缓冲 |
| `obs_nudge_time_out_threshold` | 3.0 | — | 绕行超时 |
| `obs_clean_time_out_threshold` | 2.0 | — | 清理超时 |
| `full_nudge_buffer` | 1.5 | — | 完全绕行缓冲 |
| `nudge_key_point_extend_length_coefficient` | 0.7 | — | 关键点扩展系数 |

### 3.9 PathReferenceDecider（路径参考）

> 文件：`tasks/path_reference_decider/conf/default_conf.pb.txt`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `min_path_reference_length` | 20 | — | 最小路径参考长度 |
| `skip_path_reference_in_side_pass` | false | — | 侧方通过跳过参考 |
| `skip_path_reference_in_change_lane` | true | — | 变道跳过参考 |

### 3.10 PathTimeHeuristic（DP 速度搜索）

> 文件：`tasks/path_time_heuristic/conf/default_conf.pb.txt`

| 参数 | 当前值 | 作用 |
|------|--------|------|
| `default_speed_config.max_acceleration` | 2.0 | 最大加速度 |
| `default_speed_config.max_deceleration` | -4.0 | 最大减速度 |
| `default_speed_config.speed_weight` | 0.0 | 速度权重 |
| `default_speed_config.accel_weight` | 10.0 | 加速度权重 |
| `default_speed_config.jerk_weight` | 10.0 | jerk 权重 |
| `default_speed_config.obstacle_weight` | 1.0 | 障碍权重 |
| `default_speed_config.exceed_speed_penalty` | 1e3 | 超速惩罚 |
| `default_speed_config.low_speed_penalty` | 10.0 | 低速惩罚 |
| `default_speed_config.enable_dp_reference_speed` | true | DP 参考速度 |
| `lane_change_speed_config.*` | — | 变道 DP 配置（同上结构） |

---

## 4. Scenarios（`scenarios/<scenario>/conf/scenario_conf.pb.txt`）

### 4.1 TrafficLightProtected（红绿灯场景）⭐ 赛题3/8

> 文件：`scenarios/traffic_light_protected/conf/scenario_conf.pb.txt`

| 参数 | 当前值 | 作用 |
|------|--------|------|
| `start_traffic_light_scenario_distance` | 5.0 | 进入场景距离 |
| `max_valid_stop_distance` | 2.0 | 有效停车最大距离 |

### 4.2 StopSignUnprotected（停止标志场景）⭐ 赛题4

> 文件：`scenarios/stop_sign_unprotected/conf/scenario_conf.pb.txt`

| 参数 | 当前值 | 作用 |
|------|--------|------|
| `start_stop_sign_scenario_distance` | 4.0 | 进入场景距离 |
| `watch_vehicle_max_valid_stop_distance` | 5.0 | 观察车辆有效停车距离 |
| `max_valid_stop_distance` | 2.0 | 有效停车最大距离 |
| `stop_duration_sec` | 1.0 | 停车时长 |
| `stop_timeout_sec` | 8.0 | 停车超时 |
| `creep_timeout_sec` | 10.0 | 蠕行超时 |
| `creep_stage_config.min_boundary_t` | 6.0 | 蠕行最小边界 t |
| `creep_stage_config.ignore_max_st_min_t` | 0.1 | 忽略最大 ST 最小 t |
| `creep_stage_config.ignore_min_st_min_s` | 15.0 | 忽略最小 ST 最小 s |

### 4.3 BareIntersectionUnprotected（无保护路口）

> 文件：`scenarios/bare_intersection_unprotected/conf/scenario_conf.pb.txt`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `start_bare_intersection_scenario_distance` | 25.0 | 25.0 | 进入场景距离 |
| `enable_explicit_stop` | false | false | 显式停车开关 |
| `approach_cruise_speed` | 6.7056 | 6.7056 | 接近路口巡航速度（m/s） |
| `stop_distance` | 0.5 | 0.5 | 停车距离 |

### 4.4 YieldSign（让行场景）

> 文件：`scenarios/yield_sign/conf/scenario_conf.pb.txt`

| 参数 | 当前值 | 作用 |
|------|--------|------|
| `start_yield_sign_scenario_distance` | 10.0 | 进入场景距离 |
| `max_valid_stop_distance` | 4.5 | 有效停车最大距离 |
| `creep_timeout_sec` | 10.0 | 蠕行超时 |
| `creep_stage_config.*` | — | 蠕行阶段配置 |

### 4.5 ParkAndGo（停车起步）

> 文件：`scenarios/park_and_go/conf/scenario_conf.pb.txt`

| 参数 | 当前值 | 作用 |
|------|--------|------|
| `front_obstacle_buffer` | 10.0 | 前方障碍缓冲 |
| `heading_buffer` | 0.3 | 航向缓冲 |
| `min_dist_to_dest` | 10.0 | 到终点最小距离 |
| `max_steering_percentage_when_cruise` | 20.0 | 巡航最大转向百分比 |

---

## 5. Control 全局 gflags（`control_component/conf/control.conf`）

> 源码全量定义：`modules/control/control_component/common/control_gflags.cc`

| 参数 | 当前值 | 源码默认 | 作用 |
|------|--------|---------|------|
| `--pipeline_file` | /apollo/.../pipeline.pb.txt | — | 控制器流水线 |
| `--control_period` | 0.01 | 0.01 | 控制周期（s） |
| `--enable_gain_scheduler` | true | false | 增益调度开关 |
| `--set_steer_limit` | true | false | 转向限位开关 |
| `--use_vehicle_epb` | false | false | 车辆 EPB |
| `--use_control_submodules` | false | false | 子模块模式 |
| `--steer_angle_rate` | 100 | 100.0 | 转向角速率 |
| `--max_acceleration_when_stopped` | 0.01 | 0.01 | 已停加速度阈值 |
| `--soft_estop_brake` | 15.0 | 50.0 | 软急停制动 |
| `--max_path_remain_when_stopped` | 0.2 | 0.3 | 已停最大路径余量 |
| `--enable_persistent_estop` | 缺失→false? | true | 持续急停 |
| `--use_preview_speed_for_table` | false | false | 预瞄速度查表 |
| `--enable_csv_debug` | false | false | CSV 调试 |
| `--control_test_duration` | -1 | -1.0 | 控制测试时长 |
| `--pitch_offset_deg` | ⭐缺失 | 0.0 | 俯仰偏移（坡道） |
| `--use_speed_filter` | ⭐缺失 | false | 速度平滑滤波 |
| `--use_throttle_filter` | ⭐缺失 | false | 油门平滑滤波 |
| `--speed_smoothing_factor` | ⭐缺失 | 0.05 | 速度平滑系数 |
| `--throttle_smoothing_factor` | ⭐缺失 | 0.05 | 油门平滑系数 |
| `--minimum_speed_resolution` | 0.2 | 0.2 | 最小速度分辨率 |
| `--minimum_speed_protection` | 0.1 | 0.1 | 最小速度保护 |
| `--is_control_test_mode` | false | false | 控制测试模式 |
| `--enable_input_timestamp_check` | false | false | 输入时间戳检查 |
| `--max_localization_miss_num` | 20 | 20 | 定位丢失上限 |
| `--max_chassis_miss_num` | 20 | 20 | 底盘丢失上限 |
| `--max_planning_miss_num` | 5 | 20 | 规划丢失上限 |
| `--action` | 1 | 1 | START=1/RESET=2 |

**control.conf 语法注意**：文件里 `--steer_angle_rate = 100` 有空格也可以，但建议统一 `--xxx=值` 格式。

---

## 6. Control 控制器（`controllers/*/conf/controller_conf.pb.txt`）

### 6.1 纵向 PID（`lon_based_pid_controller`）⭐ 停车精度/速度平顺

| 参数 | 当前值 | 作用 |
|------|--------|------|
| `ts` | 0.01 | 控制周期 |
| `brake_minimum_action` | 0.0 | 最小制动 |
| `throttle_minimum_action` | 0.0 | 最小油门 |
| `speed_controller_input_limit` | 1.5 | 速度控制器输入限幅 |
| `station_error_limit` | 2.0 | 位置误差限幅 |
| `preview_window` | 20.0 | 预瞄窗口 |
| `standstill_acceleration` | -0.3 | 静止目标加速度 |
| `standstill_narmal_acceleration` | -0.5 | 静止正常加速度 |
| `pedestrian_stop_time` | 15 | 行人停车时间 |
| `full_stop_long_time` | 3.0 | 长时间停车判定 |
| `stop_gain_acceleration` | -1.0 | 停车增益加速度 ⭐停车精度 |
| `full_stop_path_remain_gain` | 0.3 | 停车路径余量增益 |
| `speed_itfc_full_stop_speed` | 0.09 | 完全停止速度 |
| `speed_itfc_dcc_emergency` | -1.5 | 紧急制动减速度 |
| `speed_itfc_speed_cmd` | 0.10 | 速度指令 |
| `speed_itfc_path_remain_max` | 0.6 | 路径余量上限 |
| `speed_itfc_path_remain_min` | 0.10 | 路径余量下限 |
| `station_pid_conf` | kp:0.2 ki:0 kd:0 | 位置 PID |
| `low_speed_pid_conf` | kp:2.0 ki:0.3 kd:0 | **低速 PID** ⭐停车段 |
| `high_speed_pid_conf` | kp:1.0 ki:0.3 kd:0 | 高速 PID |
| `switch_speed` | 3.0 | 高低速切换速度 |
| `pit_station_pid_conf` | kp:0.5 ki:0.3 | 泊位位置 PID |
| `pit_speed_pid_conf` | kp:1.0 ki:0.2 | 泊位速度 PID |

### 6.2 横向 LQR（`lat_based_lqr_controller`）⭐ 跑偏/转向

| 参数 | 当前值 | 作用 |
|------|--------|------|
| `ts` | 0.01 | 控制周期 |
| `preview_window` | 0 | 预瞄窗口 |
| `cf` / `cr` | 155494.663 | 前后轮侧偏刚度 |
| `mass_fl/fr/rl/rr` | 520 | 各轮质量 |
| `matrix_q` | 0.05/0/1.0/0 | Q 矩阵（横向误差权重） |
| `cutoff_freq` | 10 | 截止频率 |
| `mean_filter_window_size` | 10 | 均值滤波窗口 |
| `max_lateral_acceleration` | 5.0 | 最大横向加速度 |
| `enable_steer_mrac_control` | false | 转向 MRAC 控制 |
| `enable_look_ahead_back_control` | true | 前视后视控制 |
| `lookahead_station` | 1.4224 | 前视距离 ⭐低速 |
| `lookback_station` | 2.8448 | 后视距离 |
| `lookahead_station_high_speed` | 1.4224 | 高速前视 |
| `lookback_station_high_speed` | 2.8448 | 高速后视 |
| `switch_speed` | 2.0 | 高低速切换 |
| `query_relative_time` | 0.8 | 查询相对时间 |
| `lock_steer_speed` | -0.081 | 锁定转向速度 |
| `lat_err_gain_scheduler` | 4→1.0, 8→0.6... | 横向误差增益调度 ⭐低速增益 |
| `heading_err_gain_scheduler` | 4→1.0, 8→0.6... | 航向误差增益调度 |
| `steer_mrac_conf.*` | — | 转向 MRAC 参数 |
| `reverse_leadlag_conf.*` | — | 倒车 leadlag |

---

## 7. 9 赛题 → 参数速查表（终极速查）

| # | 赛题 | 改哪个文件 | 参数 | 建议值 |
|---|------|-----------|------|--------|
| 0 | 定速巡航 | planning.conf | `--default_cruise_speed` | ≤3.06（按现场） |
| 1 | 减速带 | scene_speed_limit conf 或 planning.conf | `speed_bump_limit_speed` 或 `--speed_bump_speed_limit` | ≤1.5 留裕量（插件） / 2.5~2.7（gflag） |
| 2 | 人行道停车 | crosswalk conf | `stop_distance` / `expand_s_distance` / `stop_timeout` | 1.25 + 4~6 + 30 |
| 3 | 红绿灯停车 | traffic_light conf | `stop_distance` | 1.25 |
| 4 | 停止牌停车 | stop_sign conf | `stop_distance` | 1.25 |
| 5 | 障碍物避让 | planning.conf | `--min_stop_distance_obstacle` | ⚠️官方 6m 不改 |
| 6 | 动态跟随 | speed_decider conf | `follow_min_time_sec` / `stop_follow_distance` | 1.0 / 2.0 |
| 7 | 路口减速 | scene_speed_limit conf | `crosswalk_limit_speed` | ≤1.5 留裕量 |
| 8 | 交通灯减速 | scene_speed_limit conf | `traffic_light_limit_speed` | ≤1.5 留裕量 |

---

## 8. 通用调参注意事项

1. **一次只改一个参数**：多参数同时改无法归因
2. **留裕量**：限速值 = 赛题上限 − 0.2~0.5（控制超调）；停车距离取区间中值
3. **⭐ 手动添加参数**：proto 中定义但 conf 里没有的字段，直接在 conf 加 `字段名: 值` 即可（如 `stop_timeout: 30`、`--speed_bump_speed_limit=2.5`）
4. **gflag 添加**：planning.conf/control.conf 加 `--参数名=值`（bool 用 `--参数名` 或 `--no参数名`）
5. **改错文件不生效**：确认改的是 `profiles/default/modules/...`（软链指向这里）
6. **编译后配置丢失**：只有改 C++ 编译后才需 `aem profile use default` 恢复
7. **验证生效**：`ssh car "aem exec -- cat /apollo/modules/.../conf/xxx"` 看软链值
8. **回滚**：改前 `backup_car.sh` 备份，出问题 30 秒还原

---

## 9. 参数来源索引

| 来源 | 文件 |
|------|------|
| Planning 全部 gflags | `modules/planning/planning_base/gflags/planning_gflags.cc` |
| Control 全部 gflags | `modules/control/control_component/common/control_gflags.cc` |
| TrafficRule proto | `modules/planning/traffic_rules/<rule>/proto/*.proto` |
| Task proto | `modules/planning/tasks/<task>/proto/*.proto` |
| Scenario conf | `modules/planning/scenarios/<scenario>/conf/scenario_conf.pb.txt` |
| 限速插件 proto | `modules/planning/traffic_rules/scene_speed_limit/proto/scene_speed_limit.proto` |
| Control LQR/PID | `modules/control/controllers/*/conf/controller_conf.pb.txt` |
