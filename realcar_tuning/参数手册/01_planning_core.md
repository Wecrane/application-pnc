# 01 planning_component/conf 全参数（回归代码版）

> 配置：`realcar_tuning/ORIGINAL/modules/planning/planning_component/conf/`
> 源码：`modules/planning/planning_base/gflags/planning_gflags.cc`、`planning_base/proto/`、`planning_open_space/proto/planner_open_space_config.proto`
> 标注：⭐=比赛核心 ○=一般 △=无关/泊车
> ⚠️ 本地 `modules/planning` 已恢复官方基线（2b48d5e）；正文中"本地魔改/本地竞赛代码"备注指恢复前 add5 版（备份于 backup/），仅为历史对照

---

## 1. planning.conf（gflags 全局参数）

| 参数 | 实车值 | 默认值 | 含义（回归代码） | 影响/备注 |
|------|--------|--------|------------------|-----------|
| `--flagfile` | `/apollo/modules/common/data/global_flagfile.txt` | — | 引入全局 flagfile | 全局公共参数入口 |
| `--traffic_rule_config_filename` | `/apollo/modules/planning/planning_component/conf/traffic_rule_config.pb.txt` | 相对路径 | TrafficRule 插件列表配置文件名 | 指定规则管线配置 ⭐ |
| `--planning_upper_speed_limit` | **5.0** | 31.3 | 规划最大速度 (m/s)。`reference_line.cc::GetSpeedLimitFromS` 默认限速兜底，与车道限速取 min；`piecewise_jerk_speed_optimizer.cc:174` 与 cruise 取 min | **全局限速 18 km/h**。调大→车速上限提高；调小→更保守 ⭐（铁律 [1.5,5.1] 别动） |
| `--default_cruise_speed` | 5.0 | 5.0 | 默认巡航速度。`frame.cc:185` 等无显式限速时取此值 | 各 Stage 默认目标速度 ⭐（评分 ≤3.0556，**推荐 2.5** 留 0.56 裕量） |
| `--ignore_overlapped_obstacle` | true | ❌未定义 | **当前代码无此 gflags 定义**，遗留参数 | **no-op 无效果** △ |
| `--prioritize_change_lane` | true | false | 变道优先级更高：`reference_line_provider.cc:624` 始终优先选有效变道路径 | 开启更积极变道绕行 ⭐ |
| `--min_length_for_lane_change` | 5.0 | ❌未定义 | **当前代码无此定义**，旧版"变道路段最小长度" | no-op；现役为 `change_lane_min_length`(30.0 未设置) △ |
| `--nouse_multi_thread_to_add_obstacles` | true(=false) | false | 禁用多线程添加障碍物 | 单线程确定性添加，求稳定 ○ |
| `--enable_print_curve` | false | false | 是否打印曲线信息到日志 | 调试用 ○ |
| `--destination_check_distance` | **20.0** | 5.0 | `reference_line_info.cc:891`：主 stop 为 DEST 且距终点 > 此值不发 mission_complete | 调大→更晚触发完成判定；配合 passed_destination_threshold ⭐ |
| `--smoother_config_filename` | `.../discrete_points_smoother_config.pb.txt` | qp_spline | 参考线平滑器配置，`reference_line_provider.cc:72` 加载 | 实车改用离散点 FEM 平滑器 ⭐ |
| `--enable_reference_line_stitching` | **false** | true | `reference_line_provider.cc:656`：false 不拼接上周期参考线 | 关闭→每帧全新生成，更稳更确定 ⭐ |
| `--export_chart` | false | false | 是否导出图表 | 调试用 ○ |
| `--longitudinal_jerk_lower_bound` | **-5** | -4.0 | 纵向 jerk 下界，`piecewise_jerk_speed_optimizer.cc:180/409` set_dddx_bound | 实车放宽到 -5（更陡急刹可行）⭐ |
| `--longitudinal_jerk_upper_bound` | **5** | 2.0 | 纵向 jerk 上界 | 实车放宽到 5（加速更灵活）⭐ |
| `--obstacle_lat_buffer` | 0.4 | 0.4 | 障碍物横向 buffer（m），path 边界让出宽度 | 调大→绕障更安全但更贴线外 ○ |
| `--min_stop_distance_obstacle` | **0.0** | 6.0 | 车道内障碍最小停车距离。`obstacle.cc:311-312` 钳制 | **钳到 0：可紧贴障碍停车** ⭐ |
| `--max_stop_distance_obstacle` | **0.0** | 10.0 | 障碍最大停车距离上限 | 与 min 一致→停车距离=障碍边界 ⭐ |
| `--static_obstacle_nudge_l_buffer` | **0.6** | 0.3 | 静态障碍绕行最小横向距离 | 实车 2 倍→绕障余量更大 ⭐ |
| `--enable_smoother_failsafe` | true | false | 平滑器失败用 warm start 结果兜底 | 开空间 △ |
| `--enable_parallel_trajectory_smoothing` | true | false | 分段并行平滑 | 开空间 △ |
| `--use_s_curve_speed_smooth` | true | false | s-curve 平滑 Hybrid A* 速度 | 开空间 △ |
| `--use_iterative_anchoring_smoother` | true | false | 用 iterative anchoring smoother | 开空间 △ |
| `--speed_fallback_distance` | **0.0** | 3.0 | 速度求解失败 fallback 急刹距离 | 实车 0→失败在障碍处才急刹 ⭐ |
| `--nonstatic_obstacle_nudge_l_buffer` | **0.15** | 0.4 | 动态障碍绕行最小横向距离 | 实车压缩→对移动障碍很贴 ⭐ |
| `--use_st_drivable_boundary` | false | false | 速度规划用 st drivable boundary | 走传统 ST 边界 ⭐ |
| `--enable_pull_over_at_destination` | false | false | 到终点靠边停车 | 到终点直接停 ⭐ |
| `--replan_time_threshold` | 7.0 | 7.0 | 时间差超此值重规划，`trajectory_stitcher.cc:219` | 调小→更频繁重规划 ○ |
| `--obstacle_lon_start_buffer` | **0.4** | 3.0 | 障碍纵向起始 buffer | 实车大幅压缩→更晚触发绕障 ⭐ |
| `--check_collision_freespace_obstacle_vertices` | false | false | 检测车辆与投影障碍碰撞 | 开空间 △ |
| `--yield_distance` | **1.0** | 5.0 | 让行最小距离，`st_gap_estimator.cc:45/65` | 实车 1.0 极紧→跟车更短 ⭐ |
| `--passed_destination_threshold` | **0.5** | 0.05 | `reference_line_info.cc:801` 距终点 ≤ 此值判到达 | 实车 0.5m 配合终点提前完成 ⭐ |
| `--open_space_delta_t` | 0.2 | 1.0 | 开空间速度采样时间 | △ |
| `--open_space_acc_weight` | 1.0 | 1.0 | 开空间加速度权重 | △ |
| `--open_space_jerk_weight` | 10.0 | 10.0 | 开空间 jerk 权重 | △ |
| `--open_space_kappa_weight` | **100.0** | 1000.0 | 开空间曲率权重 | 实车调小→路径可更弯 △ |
| `--open_space_reference_s_weight` | 10.0 | 10.0 | 开空间参考 s 权重 | △ |
| `--open_space_reference_v_weight` | 10.0 | 10.0 | 开空间参考 v 权重 | △ |
| `--open_space_max_forward_v` | 2.0 | 2.0 | 开空间最大前进速度 | △ |
| `--open_space_max_reverse_v` | 2.0 | 2.0 | 开空间最大倒车速度 | △ |
| `--open_space_max_forward_acc` | **1.0** | 3.0 | 开空间最大前进加速度 | 实车调低→更缓 △ |
| `--open_space_max_reverse_acc` | **0.3** | 2.0 | 开空间最大倒车加速度 | 极保守 △ |
| `--open_space_max_jerk` | **2.0** | 4.0 | 开空间最大 jerk | 实车调低→更平顺 △ |
| `--enable_reference_line_provider_thread` | **false** | true | 是否独立线程生成参考线 | 实车关→主线程同步，行为确定 ⭐ |
| `--enable_smooth_reference_line` | true | true | 平滑地图参考线 | 走 FEM 离散点平滑 ⭐ |
| `--enable_corner_constraint` | **true** | false | `path_bounds_decider_util.cc:1088` 障碍物角点约束 | 实车开→绕障贴角点防擦碰 ⭐ |
| `--speed_optimize_fail_relax_velocity_constraint` | **false** | true | 速度优化失败放宽速度上界重试 | 实车关→失败直接 fallback ⭐ |
| `--path_bounds_horizon` | **12.0** | 100 | path bounds 前向计算距离 | 实车 12m 只算近障碍→少变道 ⭐ |
| `--longitudinal_acceleration_lower_bound` | **-2.0** | -6.0 | 纵向加速度下界（最大减速度） | 实车限 2 m/s²→制动极缓 ⭐ |
| `--longitudinal_acceleration_upper_bound` | **2.0** | 4.0 | 纵向加速度上界 | 最大加速 2→起步平缓 ⭐ |
| `--smoother_stop_distance` | 10 | 10.0 | 接近停车点距离阈值 | 近停车点平滑过渡 ○ |
| `--passed_referenceline_end_threshold` | **5** | 1.5 | 距参考线末端 ≤ 此值判到达 | 实车 5→终点判定更宽松 ⭐ |
| `# --follow_min_obs_lateral_distance=6.0` | 注释 | — | 被注释旧参数 | 未生效 △ |

---

## 2. planning_config.pb.txt

### topic_config（消息主题）

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| `chassis_topic` | `/apollo/canbus/chassis` | 底盘消息 | 标准 ○ |
| `hmi_status_topic` | `/apollo/hmi/status` | HMI 状态 | 标准 ○ |
| `localization_topic` | `/apollo/localization/pose` | 定位消息 | 标准 ○ |
| `planning_pad_topic` | `/apollo/planning/pad` | 手动接管 pad | 标准 ○ |
| `planning_trajectory_topic` | `/apollo/planning` | 规划轨迹发布（control 订阅） | 标准 ⭐ |
| `prediction_topic` | `/apollo/prediction` | 预测消息 | 标准 ○ |
| `relative_map_topic` | `/apollo/relative_map` | 相对地图 | 标准 ○ |
| `routing_request_topic` | `/apollo/external_command/lane_follow` | 路由请求（LaneFollow 入口） | 标准 ⭐ |
| `routing_response_topic` | `/apollo/routing_response` | 路由响应 | 标准 ○ |
| `planning_command_topic` | `/apollo/planning/command` | 规划控制命令 | 标准 ○ |
| `story_telling_topic` | `/apollo/storytelling` | 场景叙事 | 标准 ○ |
| `traffic_light_detection_topic` | `/apollo/perception/traffic_light` | 红绿灯检测 | 标准 ○ |
| `planning_learning_data_topic` | `/apollo/planning/learning_data` | 学习数据采集 | NO_LEARNING 不用 △ |
| `perception_edge_info_topic` | `/apollo/perception/edge` | 感知路沿信息 | 标准 ○ |

### 其他字段

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| `learning_mode` | `NO_LEARNING` | 学习模式 | 纯规则规划，保持 △ |
| `reference_line_config.pnc_map_class` | `apollo::planning::LaneFollowMap` | 参考线生成器类名 | **LaneFollowMap 车道跟随地图** ⭐ |

---

## 3. traffic_rule_config.pb.txt（TrafficRule 插件列表）

> 只控制"启用哪些规则"，各规则内部参数在其各自 conf。

| 规则 | type | 含义 | 备注 |
|------|------|------|------|
| `BACKSIDE_VEHICLE` | BacksideVehicle | 后方来车处理 | ○ |
| `SCENE_SPEED_LIMIT` | SceneSpeedLimit | **主办方限速插件**：crosswalk/traffic_light/speed_bump/junction 区段限速 | ⭐ 9 赛题限速核心 |
| `CROSSWALK` | Crosswalk | 人行横道规则 | ⭐ 人行道/路口核心 |
| `DESTINATION` | Destination | 终点规则：DEST 障碍+终点停车 | ⭐ |
| `KEEP_CLEAR` | KeepClear | 禁停区 | ○ |
| `REFERENCE_LINE_END` | ReferenceLineEnd | 参考线末端虚拟障碍 | ⭐ |
| `REROUTING` | Rerouting | 偏离路由重路由 | ○ |
| `TRAFFIC_LIGHT` | TrafficLight | 红绿灯规则 | ⭐ |
| `YIELD_SIGN` | YieldSign | 让行标志 | ○ |
| `SPEED_SETTING` | SpeedSetting | 通用限速设置 | ○ |
| `SCENE_SPEED_LIMIT`（末尾重复） | SceneSpeedLimit | **与第 2 条重复** | 重复注册通常无害，冗余 ○ |

---

## 4. public_road_planner_config.pb.txt（Scenario 插件列表）

| 场景 | type | 含义 | 备注 |
|------|------|------|------|
| EMERGENCY_PULL_OVER | EmergencyPullOverScenario | 紧急靠边 | △ |
| EMERGENCY_STOP | EmergencyStopScenario | 紧急停车 | △ |
| VALET_PARKING_PARK | ValetParkingParkScenario | 代客泊车入库 | △ |
| BARE_INTERSECTION_UNPROTECTED | BareIntersectionUnprotectedScenario | 无保护裸路口 | ○ |
| STOP_SIGN_UNPROTECTED | StopSignUnprotectedScenario | 停止标志路口 | ○ |
| YIELD_SIGN | YieldSignScenario | 让行路口 | ○ |
| TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN | TrafficLightUnprotectedLeftTurnScenario | 无保护左转 | ○ |
| TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN | TrafficLightUnprotectedRightTurnScenario | 无保护右转 | ○ |
| TRAFFIC_LIGHT_PROTECTED | TrafficLightProtectedScenario | 有保护红绿灯直行 | ⭐ |
| PULL_OVER | PullOverScenario | 靠边停车 | △ |
| PARK_AND_GO | ParkAndGoScenario | 停车后起步 | ○ |
| FREE_SPACE | FreeSpaceScenario | 自由空间 | △ |
| SQUARE | ~~SquareScenario~~ | **已注释**：场地赛 Square | △ 禁用 |
| LANE_FOLLOW | LaneFollowScenario | **车道跟随默认场景（兜底）** | ⭐ 赛题主场景 |

---

## 5. discrete_points_smoother_config.pb.txt（参考线 FEM 平滑器）

> 由 `--smoother_config_filename` 指向，平滑参考线用。

| 参数 | 实车值 | 默认值 | 含义 | 备注 |
|------|--------|--------|------|------|
| `max_constraint_interval` | **0.25** | 5.0 | 离散点平滑输出分辨率 | 0.25m 极细→质量高计算大 ⭐ |
| `longitudinal_boundary_bound` | **0.05** | 1.0 | 纵向边界约束界限 | 调小→更贴原线 ⭐ |
| `max_lateral_boundary_bound` | **0.05** | 0.5 | 横向边界上界 | 实车 0.05→横向偏移极小 ⭐ |
| `min_lateral_boundary_bound` | 0.0 | 0.2 | 横向边界下界 | 0.0 允许完全贴线 ⭐ |
| `curb_shift` | 0.2 | 0.2 | 路缘偏移 | ○ |
| `lateral_buffer` | 0.2 | 0.2 | 横向 buffer | ○ |
| `discrete_points.smoothing_method` | FEM_POS_DEVIATION | 同 | FEM 位置偏差法 | ⭐ |
| `weight_fem_pos_deviation` | 1e10 | 1e10 | FEM 相邻点偏差权重 | 越大越平滑 ⭐ |
| `weight_ref_deviation` | 1.0 | 1.0 | 相对原始参考线偏差权重 | ○ |
| `weight_path_length` | 1.0 | 1.0 | 路径长度权重 | ○ |
| `apply_curvature_constraint` | false | false | 是否施加曲率约束 | ○ |
| `max_iter` | 500 | 500 | OSQP 最大迭代 | ○ |
| `time_limit` | 0.0 | 0.0 | 求解时间上限 | ○ |
| `verbose` | false | false | 求解器日志 | ○ |
| `scaled_termination` | true | true | 缩放终止条件 | ○ |
| `warm_start` | true | true | 热启动 | ○ |

---

## 6. planner_open_space_config.pb.txt（开空间/泊车规划）△

> 全部用于 FREE_SPACE / VALET_PARKING 等开空间场景，与实车 9 赛题（车道内行驶）基本无关，**保持原值**。

| 参数 | 实车值 | 含义 |
|------|--------|------|
| `roi_config.roi_longitudinal_range_start/end` | 15.0/15.0 | 泊车 ROI 纵向范围 |
| `roi_config.parking_start_range` | 12.0 | 车位检测范围 |
| `roi_config.parking_inwards` | false | 倒车入库方向 |
| `warm_start_config.xy_grid_resolution` | 0.3 | Hybrid A* 栅格 XY 分辨率 |
| `warm_start_config.phi_grid_resolution` | 0.1 | 航向角栅格分辨率 |
| `warm_start_config.next_node_num` | 10 | 下一步节点数 |
| `warm_start_config.step_size` | 0.5 | 节点间轨迹点距 |
| `warm_start_config.traj_forward/back_penalty` | 1.0/1.0 | 前进/后退惩罚 |
| `warm_start_config.traj_gear_switch_penalty` | 10.0 | 换挡惩罚 |
| `warm_start_config.traj_steer_penalty` | 0.0 | 转向惩罚 |
| `warm_start_config.traj_steer_change_penalty` | 0.0 | 转向变化惩罚 |
| `warm_start_config.grid_a_star_xy_resolution` | 0.5 | A* 栅格分辨率 |
| `warm_start_config.node_radius` | 0.25 | 碰撞检测节点半径 |
| `warm_start_config.s_curve_config.*` | acc1.0/jerk0.0/... | 开空间 S 曲线权重 |
| `dual_variable_warm_start_config.*` | OSQP / min_safety_distance0.01 | 对偶变量热启动 |
| `distance_approach_config.*` | IPOPT / max_speed_forward2.0 / ... | 距离逼近法权重 |
| `iterative_anchoring_smoother_config.*` | delta_s0.1 / FEM 1e8 / ... | 迭代锚点平滑 |
| `delta_t` | 0.5 | 开空间速度采样时间 |
| `near_destination_threshold` | 0.05 | 接近终点阈值 |
| `enable_linear_interpolation` | false | 线性插值输出 |
