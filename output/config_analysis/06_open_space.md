# Apollo 11.0 EDU — OpenSpace / 泊车相关场景与任务配置分析

> 分析范围：6 个 OpenSpace/泊车场景（free_space、valet_parking、valet_parking_park、park_and_go、pull_over、large_curvature）+ 11 个 OpenSpace 任务默认配置。
> 代码位置均定位到 `modules/planning/` 下实际 .cc 文件（.oldcode/ 为备份目录，已忽略）。
> 结论先行：**这些配置仅在 OpenSpace 泊车/脱困/大曲率场景内生效，与 8 个常规道路赛题（车道保持/路口/人行道/减速带等）基本无关**，唯一需要留意的是 `pull_over` 场景（靠边停车）与 `valet_parking` 的接近停车位阶段，见文末总结。

---

## 一、场景级配置

### 1. free_space（自由空间/脱困）

**`conf/pipeline.pb.txt`** — 单 Stage `STAGE_FREE_SPACE`，任务链：`OpenSpaceTrajectoryProvider → OpenSpaceTrajectoryPartition → OpenSpaceFallbackDecider`。各任务参数见任务节。

**`conf/scenario_conf.pb.txt`**（2 参数，作用于 `scenarios/free_space/stage_free_space.cc`）：

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `enabled_perception_obstacles` | `true` | `stage_free_space.cc:96` | 是否把感知障碍物加入 OpenSpace 可行驶区域（ROI）边界 |
| `perception_obstacle_buffer` | `0.2` m | `stage_free_space.cc:99` | 感知障碍物外扩缓冲距离，值越大避让越保守 |

**`conf/stage_free_space/open_space_trajectory_provider.pb.txt`** — 见任务 8（内容相同，含全部 warm_start / 优化器参数）。

**`conf/stage_free_space/open_space_trajectory_partition.pb.txt`** — 见任务 9（`use_gear_shift_trajectory: true` 等）。

### 2. valet_parking（代客泊车）

**`conf/pipeline.pb.txt`** — 两 Stage：
- `VALET_PARKING_APPROACHING_PARKING_SPOT`：`OpenSpacePreStopDecider` + 常规车道任务链（LaneFollowPath/LaneBorrowPath/FallbackPath/PathDecider/RuleBasedStopDecider/SpeedBounds×2/PathTimeHeuristic/PiecewiseJerkSpeed 等），即接近车位时仍按车道行驶；
- `VALET_PARKING_PARKING`：`OpenSpaceRoiDecider → OpenSpaceTrajectoryProvider → OpenSpaceTrajectoryPartition → OpenSpaceFallbackDecider`。

**`conf/scenario_conf.pb.txt`**（2 参数）：

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `parking_spot_range_to_start` | `20.0` m | `scenarios/valet_parking/valet_parking_scenario.cc:96` | 车距停车位中心 s 距小于该值即切换进入泊车 Stage |
| `max_valid_stop_distance` | `1.0` m | `scenarios/valet_parking/stage_approaching_parking_spot.cc:111` | 接近阶段自车距停止线 ≤ 该值判定"已到位"并转入泊车 |

**Stage 配置**：
- `valet_parking_approaching_parking_spot/open_space_pre_stop_decider.pb.txt`：`stop_type: PARKING`（任务 12）
- `valet_parking_parking/open_space_roi_decider.pb.txt`：`roi_type: PARKING`（任务 7）
- `valet_parking_parking/open_space_trajectory_partition.pb.txt`：泊车版切挡参数，`heading_search_range: 0.79`、`heading_track_range: 1.57`、`distance_search_range: 2.0`、`vehicle_box_iou_threshold_to_midpoint: 0.5`（任务 9）
- `valet_parking_parking/open_space_trajectory_provider.pb.txt`：与任务 8 同构，`max_explored_num: 10000`、`traj_short_length_penalty: 20`、`traj_expected_shortest_length: 2.0`、`traj_steer_penalty: 3.0`、`traj_steer_change_penalty: 1.0`、`astar_max_search_time: 20`

### 3. valet_parking_park（代客泊车·升级版）

**`conf/pipeline.pb.txt`** — 三 Stage：
- `VALET_PARKING_APPROACHING_PARKING_SPOT_PARK`：`OpenSpacePreStopDecider` + `ObstacleNudgeDecider`/`LaneChangePathGeneric`/`LaneBorrowPathGeneric` 等（比旧版多了避障/换道），带 `fallback_task: SmoothStopTrajectoryFallback`；
- `VALET_PARKING_PARKING_PARK`：`OpenSpaceReplanDecider → OpenSpaceRoiDecider → OpenSpacePathPlanning → OpenSpaceTrajectoryOptimizerPark → OpenSpaceTrajectoryPostProcess → OpenSpaceFallbackDeciderPark`（带实时重规划闭环）；
- `VALET_PARKING_RETRY_PARK`：同上 6 任务（重试泊车）。

**`conf/scenario_conf.pb.txt`**（5 参数，其中 3 个为此版独有）：

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `parking_spot_range_to_start` | `20.0` m | `scenarios/valet_parking_park/valet_parking_scenario.cc:99` | 距车位中心该距离内进入泊车 Stage |
| `max_valid_stop_distance` | `1.0` m | `scenarios/valet_parking_park/stage_approaching_parking_spot_park.cc:136` | 接近阶段判定"到位"阈值 |
| `max_heading_error` | `0.08` rad | `scenarios/valet_parking_park/stage_parking_park.cc:114` | 泊车完成判定：航向误差上限 |
| `max_lat_error` | `0.3` m | `scenarios/valet_parking_park/stage_parking_park.cc:112` | 泊车完成判定：横向误差上限 |
| `max_lon_error` | `0.15` m | `scenarios/valet_parking_park/stage_parking_park.cc:113` | 泊车完成判定：纵向误差上限（三者同时满足才判泊车成功） |

**Stage 配置**（对比 valet_parking 更精细）：
- `valet_parking_approaching_parking_spot_park/open_space_pre_stop_decider.pb.txt`：`stop_type: PARKING` + `stop_distance_to_target: 5.0`（提前 5m 停车，任务 12）
- `valet_parking_parking_park/open_space_roi_decider.pb.txt`：`roi_type: PARKING`、`roi_longitudinal_range_start/end: 20`、`expand_polygon_of_obstacle_by_distance: true`、`perception_obstacle_buffer: 0.3`（任务 7）
- `valet_parking_parking_park/open_space_replan_decider.pb.txt`：**空文件**（用 proto 默认，任务 15）
- `valet_parking_parking_park/open_space_path_planning.pb.txt`：`enable_path_planning_thread: true`、`enable_vertical_parking_last_trajectory_straight: false`、`traj_kappa_contraint_ratio: 0.7`、`astar_max_search_time: 5.0`、`desired_explored_num: 100`（任务 11）
- `valet_parking_parking_park/open_space_trajectory_optimizer.pb.txt`：`enable_trajectory_optimize_thread: true`、`collision_decrease_ratio: 0.9`、`use_sqp: true`、`weight_curvature_constraint_slack_var: 1e3`（任务 10）
- `valet_parking_parking_park/open_space_trajectory_post_process.pb.txt`：`heading_offset_to_midpoint: 1.79`、`lateral_offset_to_midpoint: 1.5`、`longitudinal_offset_to_midpoint: 0.4`、`vehicle_box_iou_threshold_to_midpoint: 0.25`、`speed_replan_distance: 2.0`、`stop_check_window: 10`（任务 17）
- `valet_parking_parking_park/open_space_fallback_decider_park.pb.txt`：4 参数（任务 14）
- `valet_parking_retry_park/*`：与 parking_park 各文件内容相同（重试阶段复用同一套参数）

### 4. park_and_go（停车后起步）

**`conf/pipeline.pb.txt`** — 四 Stage：`PARK_AND_GO_CHECK`（OpenSpace 三任务判定是否可起步）→ `PARK_AND_GO_ADJUST`（OpenSpace 三任务原地调整）→ `PARK_AND_GO_PRE_CRUISE`（OpenSpace 三任务驶出）→ `PARK_AND_GO_CRUISE`（常规车道任务链 + RSS）。

**`conf/scenario_conf.pb.txt`**（4 参数）：

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `front_obstacle_buffer` | `10.0` m | `scenarios/park_and_go/util.cc:51,80` | 起步/调整时自车前方障碍物判定框延长量 |
| `heading_buffer` | `0.3` rad | `scenarios/park_and_go/util.cc:54` | 障碍物/参考线航向差允许缓冲 |
| `min_dist_to_dest` | `10.0` m | `scenarios/park_and_go/park_and_go_scenario.cc:107,113` | 距目的地 < 该值才进入 park_and_go 场景（决定场景何时触发） |
| `max_steering_percentage_when_cruise` | `20.0` % | `scenarios/park_and_go/stage_adjust.cc:67`、`stage_pre_cruise.cc:54` | 调整/预巡航阶段转向百分比上限（判断是否可切巡航） |

**Stage 配置**：
- `park_and_go_adjust/open_space_roi_decider.pb.txt`：`roi_type: PARK_AND_GO`、`roi_line_segment_min_angle: 0.15`、`roi_line_segment_length: 0.2`、`perception_obstacle_buffer: 0.5`、`end_pose_s_distance: 10.0`、`roi_longitudinal_range_start: 10.0`、`roi_longitudinal_range_end: 20.0`（任务 7/16）
- `park_and_go_adjust/open_space_trajectory_provider.pb.txt`：与任务 8 同构，`traj_kappa_contraint_ratio: 0.6`、`max_explored_num/desired_explored_num: 10000`
- `park_and_go_check/open_space_roi_decider.pb.txt`：`roi_type: PARK_AND_GO`、`perception_obstacle_buffer: 0.0`（检查阶段不扩大障碍物）
- `park_and_go_pre_cruise/open_space_roi_decider.pb.txt`：`roi_type: PARK_AND_GO`、`perception_obstacle_buffer: 0.5`、`end_pose_s_distance: 5.0`
- `park_and_go_pre_cruise/open_space_trajectory_provider.pb.txt`：仅覆写 `ref_s_weight: 100.0`（s_curve 参考纵向权重，其余同任务 8）
- `park_and_go_cruise/path_decider.pb.txt`：`static_obstacle_buffer: 0.2`（常规 PathDecider 参数，`tasks/path_decider/path_decider.cc`）

### 5. pull_over（靠边停车）⭐ 与常规道路最相关

**`conf/pipeline.pb.txt`** — 三 Stage：
- `PULL_OVER_APPROACH`（enabled，`PullOverPath` + 常规车道任务链 + RssDecider）；
- `PULL_OVER_RETRY_APPROACH_PARKING`（**enabled: false**，`OpenSpacePreStopDecider` + `PullOverPath`，默认关闭）；
- `PULL_OVER_RETRY_PARKING`（enabled，OpenSpace 四任务，OpenSpace 二次靠边）。

**`conf/scenario_conf.pb.txt`**（8 参数，决定"何时进入/退出 pull_over"）：

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `start_pull_over_scenario_distance` | `50.0` m | `scenarios/pull_over/pull_over_scenario.cc:89` | 距目的地 ≤ 该值才开始搜索靠边停车点（场景触发条件） |
| `pull_over_min_distance_buffer` | `10.0` m | `scenarios/pull_over/pull_over_scenario.cc:87` | 目的地前保留的最短缓冲距离 |
| `max_distance_stop_search` | `25.0` m | `scenarios/pull_over/pull_over_scenario.cc:93` | 在目的地前方该范围内搜索可停靠点 |
| `max_s_error_to_end_point` | `0.5` m | `scenarios/pull_over/util.cc:147` | 判定靠边完成：纵向误差上限 |
| `max_l_error_to_end_point` | `1.0` m | `scenarios/pull_over/util.cc:143` | 判定靠边完成：横向误差上限 |
| `max_theta_error_to_end_point` | `0.2` rad | `scenarios/pull_over/util.cc:144`、`stage_retry_parking.cc:103` | 判定靠边完成：航向误差上限 |
| `pass_destination_threshold` | `15.0` m | `scenarios/pull_over/util.cc:56` | 超过目的地该距离视为"越过"，退出 pull_over |
| `max_distance_error_to_end_point` | `0.2` m | `scenarios/pull_over/stage_retry_parking.cc:102` | 重试靠边时距离终点误差上限 |

**`conf/pull_over_approach/pull_over_path.pb.txt`**（12 参数，作用于 `tasks/pull_over_path/pull_over_path.cc`，这是常规道路靠边停车路径生成的核心）：

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `l_weight` | `1.0` | `pull_over_path.cc`（path_optimizer 权重） | 横向偏移代价权重 |
| `dl_weight` | `20.0` | 同上 | 横向一阶（角度）代价权重 |
| `ddl_weight` | `1000.0` | 同上 | 横向二阶（曲率）代价权重 |
| `dddl_weight` | `50000.0` | 同上 | 横向三阶（曲率变化率）代价权重 |
| `lateral_derivative_bound_default` | `2.0` | `planning_interface_base/.../path_optimizer_util.cc:173` | 横向一阶导数（角度）默认边界 |
| `pull_over_destination_to_adc_buffer` | `25.0` m | `pull_over_path.cc:569` | 靠边路径终点相对目的地前置缓冲距离 |
| `pull_over_destination_to_pathend_buffer` | `4.0` m | `pull_over_path.cc:577` | 路径末端到目的地的缓冲 |
| `pull_over_road_edge_buffer` | `0.15` m | `pull_over_path.cc:459` | 靠边时与道路边缘（路沿）的安全缓冲 |
| `pull_over_approach_lon_distance_adjust_factor` | `1.6` | `pull_over_path.cc:549` | 接近段纵向距离调整系数 |
| `pull_over_weight` | `10` | `pull_over_path.cc:210` | 靠边目标横向参考权重（越大越贴边） |
| `pull_over_direction` | `RIGHT_SIDE` | `pull_over_path.cc:107-113` | 靠边方向（右/左/双侧） |
| `pull_over_position` | `DESTINATION` | `pull_over_path.cc:367-373` | 靠边位置策略（目的地旁 / 最近可用位置） |

**Stage 配置**：
- `pull_over_retry_approach_parking/open_space_pre_stop_decider.pb.txt`：`stop_type: PULL_OVER` + `rightaway_stop_distance: 1.0`（任务 12）
- `pull_over_retry_parking/open_space_roi_decider.pb.txt`：`roi_type: PULL_OVER`、`perception_obstacle_buffer: 0.5`
- `pull_over_retry_parking/open_space_trajectory_partition.pb.txt`：`heading_offset_to_midpoint: 0.79`、`vehicle_box_iou_threshold_to_midpoint: 0.8`、`linear_velocity_threshold_on_ego: 0.2`（任务 9）

### 6. large_curvature（大曲率/回头弯）

**`conf/pipeline.pb.txt`** — 单 Stage `LARGE_CURVATURE`：`OpenSpaceReplanDecider → OpenSpaceRoiDeciderPark → OpenSpacePathPlanning → OpenSpaceTrajectoryOptimizerPark → OpenSpaceTrajectoryPostProcess → OpenSpaceFallbackDeciderPark`。

**`conf/scenario_conf.pb.txt`**：**空文件**，使用 proto 默认值（`scenarios/large_curvature/proto/large_curvature_scenario.proto`）：
- `min_curvature = 0.2`（默认）— `large_curvature_scenario.cc:79`，参考线曲率 > 该值才进入大曲率场景
- `curvature_check_delta_s = 0.3`、`curvature_check_distance = 3.0`、`curvature_check_max = 0.1` — `stage_large_curvature.cc:78-88`，Stage 内每 0.3m 检查曲率，3m 内曲率全 < 0.1 即退出

**Stage 配置**：
- `open_space_roi_decider_park.pb.txt`：`roi_type: LARGE_CURVATURE`、`roi_longitudinal_range_start: 20`、`roi_longitudinal_range_end: 40`、`parking_depth_buffer: 0.3`、`expand_polygon_of_obstacle_by_distance: false`、`perception_obstacle_buffer: 0.3`、`roi_line_segment_min_angle: 0.1`、`roi_line_segment_length: 0.1`（任务 16）
- `open_space_path_planning.pb.txt`：`enable_vertical_parking_last_trajectory_straight: true`（唯一为 true 处）、`traj_kappa_contraint_ratio: 0.95`、`max_explored_num: 20000`、`astar_max_search_time: 10.0`、**`esdf_range: 2.0`、`soft_boundary_penalty: 8.0`、`use_esdf: true`**（ESDF 距离场障碍惩罚，`planning_open_space/coarse_trajectory_generator/grid_search.cc:36-37,260` 与 `hybrid_a_star.cc:94,216,306`）
- `open_space_trajectory_optimizer.pb.txt`：同任务 10，`collision_decrease_ratio: 0.7`
- `open_space_trajectory_post_process.pb.txt`：`heading_offset_to_midpoint: 0.3`、`speed_replan_distance: 2.0`、`scale_destination: 1.0`、`stop_check_window: 40`（任务 17）
- `open_space_fallback_decider_park.pb.txt`：`open_space_fallback_collision_distance: 1.5`（任务 14）
- `open_space_rt_optimizer.pb.txt`：**实时优化器参数**（大曲率专用，LBFGS 数值优化）：`traj_resolution: 16`、`wei_sta_obs: 1000.0`、`wei_dyn_obs: 5000.0`、`wei_feas: 2500.0`、`wei_sqrvar: 500.0`、`wei_time: 500.0`、`dyn_obs_clearance: 0.4`、`max_forward_acc: 8.0`、`max_backward_acc: 4.0`、`max_frontend_cur: 1.0`、`traj_piece_duration: 1.0`、`half_margin: 0.15`、`max_forward_vel: 5.0`、`max_backward_vel: 2.0`、`max_latacc: 5.0`、`gearopt: true`、`lbfgs_memsize: 256`、`mini_t: 0.1`、`non_sinv: 0.05`、`wei_smooth: 1.0` 等（`planning_open_space/rtk_replan/` 相关优化器）
- `open_space_replan_decider.pb.txt`：**空文件**（任务 15）

---

## 二、任务级默认配置（11 个 Task）

### 7. open_space_roi_decider — `tasks/open_space_roi_decider/conf/default_conf.pb.txt`
主代码：`tasks/open_space_roi_decider/open_space_roi_decider.cc`

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `roi_longitudinal_range_start` | `15` m | `open_space_roi_decider.cc:410,480` | ROI 中心线起点向后回退距离 |
| `roi_longitudinal_range_end` | `15` m | 同上（end_s 计算） | ROI 终点向前延伸距离 |
| `parking_start_range` | `20.0` m | 场景层使用（valet_parking_scenario.cc） | 距车位判定切换距离 |
| `parking_inwards` | `false` | `open_space_roi_decider.cc:272` | 车位朝向是否向道路内侧 |
| `enable_perception_obstacles` | `true` | `open_space_roi_decider.cc:1413` | 是否将感知障碍物纳入 ROI 边界 |
| `parking_depth_buffer` | `0.2` m | `open_space_roi_decider.cc:270-291` | 车位深度方向外扩缓冲（叠加到车长上） |
| `roi_line_segment_min_angle` | `0.15` rad | `open_space_roi_decider.cc:428` | 路沿点转角大于该值才视为拐点 |
| `roi_line_segment_length` | `1.0` m | `open_space_roi_decider.cc:452` | 路沿采样步长 |
| `perception_obstacle_filtering_distance` | `1000.0` m | `open_space_roi_decider.cc:1524` | 感知障碍物过滤距离（基本全量纳入） |
| `perception_obstacle_buffer` | `0.0` m | `open_space_roi_decider.cc:1432-1439` | 障碍物外扩缓冲 |
| `curb_heading_tangent_change_upper_limit` | `0.4` rad | `open_space_roi_decider.cc:663` | 路沿切线方向突变上限（判定不连续） |
| `use_road_boundary_from_map` | `false` | `open_space_roi_decider.cc:1115` | 是否用地图道路边界替代感知 ROI |

### 8. open_space_trajectory_provider — `tasks/open_space_trajectory_provider/conf/default_conf.pb.txt`
主代码：`tasks/open_space_trajectory_provider/open_space_trajectory_provider.cc` + `open_space_trajectory_optimizer.cc`

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `open_space_trajectory_optimizer_config.planner_open_space_config.warm_start_config.*` | 见下 | `open_space_trajectory_optimizer.cc` / `planning_open_space/coarse_trajectory_generator/*` | Hybrid A* 粗轨迹搜索参数 |
| ├ `xy_grid_resolution` | `0.3` m | grid_search.cc | 位置栅格分辨率 |
| ├ `phi_grid_resolution` | `0.1` rad | 同上 | 航向栅格分辨率 |
| ├ `next_node_num` | `10` | hybrid_a_star.cc | 每步候选节点数 |
| ├ `step_size` | `0.25` m | 同上 | 搜索步长 |
| ├ `traj_forward_penalty / traj_back_penalty` | `1.0` / `1.0` | 同上 | 前进/后退代价 |
| ├ `traj_gear_switch_penalty` | `10.0` | 同上 | 换挡惩罚（越大越少倒车） |
| ├ `traj_steer_penalty / traj_steer_change_penalty` | `0.0` / `0.0` | 同上 | 转向/转向变化惩罚（默认版关闭） |
| ├ `grid_a_star_xy_resolution` | `0.25` m | grid_search.cc | A* 启发栅格 |
| ├ `node_radius` | `0.5` m | 同上 | 节点碰撞半径 |
| ├ `dual_variable_warm_start_config.*` | — | `planning_open_space/tools/distance_approach_problem_wrapper.cc` | 对偶变量 QP 初值（OSQP，`qp_format: OSQP`） |
| ├ `distance_approach_config.*` | — | 同上 | 距离约束 NLP 优化（IPOPT）权重与速度/加速度上限 |
| │ ├ `weight_steer/weight_a/...` | `0.3/1.1/3.0/2.5/2.3/0.7/1.5/0.0` | 同上 | 各状态量代价权重 |
| │ ├ `max_speed_forward / max_speed_reverse` | `2.0` / `1.0` m/s | 同上 | 前进/倒车最大速度（OpenSpace 内低速） |
| │ ├ `max_acceleration_forward / max_acceleration_reverse` | `2.0` / `1.0` m/s² | 同上 | 前进/倒车最大加速度 |
| │ └ `distance_approach_mode` | `DISTANCE_APPROACH_IPOPT_RELAX_END_SLACK` | 同上 | 末端松弛求解模式 |
| ├ `iterative_anchoring_smoother_config.*` | — | `planning_open_space/.../iterative_anchoring_smoother.cc` | 迭代锚点平滑 |
| │ ├ `max_forward_v / max_reverse_v` | `1.0` / `1.0` m/s | 同上 | 平滑后速度上限 |
| │ ├ `max_forward_acc / max_reverse_acc` | `0.5` / `1.0` m/s² | 同上 | 平滑后加速度上限 |
| │ ├ `max_acc_jerk` | `0.5` m/s³ | 同上 | 加速度变化率上限 |
| │ └ `curvature_constraint` | `0.18` 1/m | fem_pos_deviation_smoother | 曲率约束 |
| ├ `delta_t` | `0.5` s | 同上 | 轨迹采样周期 |
| ├ `near_destination_threshold` | `0.1` m | `open_space_trajectory_provider.cc:346` | 距终点距离 < 该值视为到达 |
| ├ `enable_linear_interpolation` | `false` | `distance_approach_problem_wrapper.cc:321` | 是否线性插值 |
| └ `is_near_destination_theta_threshold` | `0.1` rad | `open_space_trajectory_provider.cc:349` | 到达判定航向差 |
| `open_space_planning_period` | `0.1` s | proto 字段（trajectory_provider.proto:12，默认 4.0） | OpenSpace 规划周期（当前配置值小） |
| `open_space_trajectory_stitching_preserved_length` | `inf` | proto 字段（trajectory_provider.proto:16） | 拼接保留长度 |
| `open_space_standstill_acceleration` | `0.3` m/s² | `open_space_trajectory_provider.cc:384` | 静止起步/刹停加速度 |

### 9. open_space_trajectory_partition — `tasks/open_space_trajectory_partition/conf/default_conf.pb.txt`
主代码：`tasks/open_space_trajectory_partition/open_space_trajectory_partition.cc`

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `gear_shift_max_t` | `3.0` s | `open_space_trajectory_partition.cc:706` | 换挡搜索时间窗 |
| `gear_shift_unit_t` | `0.02` s | `open_space_trajectory_partition.cc:707` | 换挡时间步长 |
| `gear_shift_period_duration` | `2.0` s | `open_space_trajectory_partition.cc:680` | 换挡间隔周期 |
| `interpolated_pieces_num` | `10` | `open_space_trajectory_partition.cc:266` | 轨迹段插值份数 |
| `initial_gear_check_horizon` | `15` | 同上（默认） | 初始挡位检查视界 |
| `heading_search_range` | `0.79` rad | `open_space_trajectory_partition.cc`（终点搜索） | 终点航向搜索范围 |
| `heading_track_range` | `1.57` rad | `open_space_trajectory_partition.cc:55` | 航向跟踪范围 |
| `distance_search_range` | `2.0` m | `open_space_trajectory_partition.cc:56,189,607` | 终点距离搜索范围 |
| `heading_offset_to_midpoint` | `0.79` rad | `open_space_trajectory_partition.cc:57,539` | 分段中点航向偏移 |
| `lateral_offset_to_midpoint` | `0.5` m | 同上 | 分段中点横向偏移 |
| `longitudinal_offset_to_midpoint` | `0.40` m | 同上 | 分段中点纵向偏移 |
| `vehicle_box_iou_threshold_to_midpoint` | `0.30` | `open_space_trajectory_partition.cc:60,550` | 分段终点与中点车辆框 IoU 阈值 |
| `speed_replan_distance` | `2.0` m | `open_space_trajectory_partition.cc:239` | 重规划触发距离 |
| `linear_velocity_threshold_on_ego` | `0.1` m/s | `open_space_trajectory_partition.cc:62,540` | 判定自车静止的线速度阈值 |
| `use_gear_shift_trajectory` | `true` | `open_space_trajectory_partition.cc:149` | 是否生成换挡（倒车）轨迹 |

### 10. open_space_trajectory_optimizer_park — `tasks/open_space_trajectory_optimizer_park/conf/default_conf.pb.txt`
主代码：`tasks/open_space_trajectory_optimizer_park/open_space_trajectory_optimizer_park.cc`

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `enable_trajectory_optimize_thread` | `true` | `open_space_trajectory_optimizer_park.cc:43,49,101` | 是否用独立线程做轨迹优化（异步） |
| `dual_variable_warm_start_config.*` | 同任务 8 | 同上 | 对偶变量 QP 初值 |
| `distance_approach_config.*` | 同任务 8（`max_speed_forward: 2.0` 等） | 同上 | 距离约束 NLP 优化 |
| `iterative_anchoring_smoother_config.collision_decrease_ratio` | `0.9` | 同上 | 碰撞约束松弛比例（越大越易避障） |
| `iterative_anchoring_smoother_config.fem_pos_deviation_smoother_config.use_sqp` | `true` | 同上 | 使用 SQP 求解平滑 |
| `...weight_curvature_constraint_slack_var` | `1e3` | 同上 | 曲率约束松弛权重（park 版比 provider 版 1e8 小） |

### 11. open_space_path_planning — `tasks/open_space_path_planning/conf/default_conf.pb.txt`
主代码：`tasks/open_space_path_planning/open_space_path_planning.cc`

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `enable_path_planning_thread` | `true` | `open_space_path_planning.cc:46,52,94,108` | 是否独立线程跑路径规划（异步） |
| `warm_start_config.next_node_num` | `10` | 同上（Hybrid A*） | 每步候选节点数 |
| `warm_start_config.step_size` | `0.25` m | 同上 | 搜索步长 |
| `warm_start_config.traj_gear_switch_penalty` | `10.0` | 同上 | 换挡惩罚 |
| `warm_start_config.traj_steer_penalty / traj_steer_change_penalty` | `0.0` / `0.0` | 同上 | 转向惩罚（默认关闭） |
| `warm_start_config.traj_kappa_contraint_ratio` | `0.7` | 同上 | 曲率约束比例 |
| `warm_start_config.node_radius` | `0.5` m | 同上 | 节点碰撞半径 |
| `warm_start_config.grid_a_star_xy_resolution` | `0.25` m | 同上 | A* 栅格分辨率 |

### 12. open_space_pre_stop_decider — `tasks/open_space_pre_stop_decider/conf/default_conf.pb.txt`
**默认文件为空**，参数全部来自场景 Stage 配置。主代码：`tasks/open_space_pre_stop_decider/open_space_pre_stop_decider.cc`

| 参数 | 值（Stage 覆盖） | 代码位置 | 作用 |
|------|----|---------|------|
| `stop_type` | `PARKING` / `PULL_OVER` | `open_space_pre_stop_decider.cc:58-59` | 停车类型，决定目标点（PARKING→车位，PULL_OVER→靠边点） |
| `stop_distance_to_target` | `5.0` m（valet_parking_park 接近阶段） | `open_space_pre_stop_decider.cc:160,183,187` | 距目标停车距离阈值（提前减速停车） |
| `rightaway_stop_distance` | `1.0` m（pull_over retry） | `open_space_pre_stop_decider.cc:193` | 立即停车距离（对向有车时） |

### 13. open_space_fallback_decider — `tasks/open_space_fallback_decider/conf/default_conf.pb.txt`
主代码：`tasks/open_space_fallback_decider/open_space_fallback_decider.cc`

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `open_space_prediction_time_period` | `5.0` s | `open_space_fallback_decider.cc:322` | 预测动态障碍物时间窗 |
| `open_space_fallback_collision_distance` | `5.0` m | 同上（碰撞判定） | 预测碰撞距离阈值（宽松版） |
| `open_space_fallback_stop_distance` | `2.0` m | 同上 | 触发 fallback 的停车距离 |
| `open_space_fallback_collision_time_buffer` | `5.0` s | `open_space_fallback_decider.cc:412` | 碰撞时间缓冲 |

### 14. open_space_fallback_decider_park — `tasks/open_space_fallback_decider_park/conf/default_conf.pb.txt`
主代码：`tasks/open_space_fallback_decider_park/open_space_fallback_decider_park.cc`

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `open_space_prediction_time_period` | `5.0` s | `open_space_fallback_decider_park.cc:446` | 预测时间窗 |
| `open_space_fallback_collision_distance` | `1.0` m | `open_space_fallback_decider_park.cc:42,113,123,143,200-229` | 预测碰撞距离阈值（park 版更严格 1.0 vs 5.0） |
| `open_space_fallback_stop_distance` | `2.0` m | 同上 | 触发 fallback 停车距离 |
| `open_space_fallback_collision_time_buffer` | `5.0` s | `open_space_fallback_decider_park.cc:556` | 碰撞时间缓冲 |

### 15. open_space_replan_decider — `tasks/open_space_replan_decider/conf/default_conf.pb.txt`
**默认文件为空**，无参数，纯逻辑判定：何时触发重新规划（`tasks/open_space_replan_decider/open_space_replan_decider.cc`，依据轨迹与障碍物距离/到达状态）。各场景 Stage 中该文件也均为空。

### 16. open_space_roi_decider_park — `tasks/open_space_roi_decider_park/conf/default_conf.pb.txt`
主代码：`tasks/open_space_roi_decider_park/open_space_roi_decider_park.cc`（参数与任务 7 同名同义，唯一差异）

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `roi_longitudinal_range_start / end` | `15` / `15` m | `open_space_roi_decider_park.cc:471,572` | ROI 纵向范围 |
| `parking_start_range` | `20.0` m | 同任务 7 | 车位切换距离 |
| `parking_inwards` | `false` | `open_space_roi_decider_park.cc:267` | 车位朝向 |
| `enable_perception_obstacles` | `true` | `open_space_roi_decider_park.cc:1589` | 感知障碍物入 ROI |
| `parking_depth_buffer` | `0.2` m | `open_space_roi_decider_park.cc:265-289` | 车位深度缓冲 |
| `roi_line_segment_min_angle` | `0.15` rad | `open_space_roi_decider_park.cc:488` | 路沿拐点判定角 |
| `roi_line_segment_length` | `1.0` m | `open_space_roi_decider_park.cc:526,697-698` | 路沿采样步长 |
| `perception_obstacle_filtering_distance` | `1000.0` m | `open_space_roi_decider_park.cc:1687` | 障碍物过滤距离 |
| `perception_obstacle_buffer` | `0.0` m | `open_space_roi_decider_park.cc:1607-1614` | 障碍物外扩缓冲 |
| `curb_heading_tangent_change_upper_limit` | **`0.01` rad** | `open_space_roi_decider_park.cc:730` | 路沿切线突变上限（**park 版 0.01 比普通版 0.4 严格得多**，拐点判定更敏感） |
| `use_road_boundary_from_map` | `false` | `open_space_roi_decider_park.cc:1176,1292` | 是否用地图边界 |

### 17. open_space_trajectory_post_process — `tasks/open_space_trajectory_post_process/conf/default_conf.pb.txt`
主代码：`tasks/open_space_trajectory_post_process/open_space_trajectory_post_process.cc`（与任务 9 同名参数语义一致）

| 参数 | 值 | 代码位置 | 作用 |
|------|----|---------|------|
| `gear_shift_max_t` | `3.0` s | `open_space_trajectory_post_process.cc:671` | 换挡搜索时间窗 |
| `gear_shift_unit_t` | `0.02` s | `open_space_trajectory_post_process.cc:672` | 换挡时间步长 |
| `gear_shift_period_duration` | `2.0` s | `open_space_trajectory_post_process.cc:643` | 换挡间隔周期 |
| `interpolated_pieces_num` | `10` | `open_space_trajectory_post_process.cc:287` | 插值份数 |
| `initial_gear_check_horizon` | `15` | 同上 | 初始挡位检查视界 |
| `heading_search_range` | `0.79` rad | 同上 | 终点航向搜索范围 |
| `heading_track_range` | `1.57` rad | `open_space_trajectory_post_process.cc:55` | 航向跟踪范围 |
| `distance_search_range` | `2.0` m | `open_space_trajectory_post_process.cc:56,201,566` | 终点距离搜索范围 |
| `heading_offset_to_midpoint` | `0.79` rad | `open_space_trajectory_post_process.cc:57,524,784,808` | 分段中点航向偏移 |
| `lateral_offset_to_midpoint` | `0.5` m | 同上 | 分段中点横向偏移 |
| `longitudinal_offset_to_midpoint` | `0.2` m | 同上 | 分段中点纵向偏移 |
| `vehicle_box_iou_threshold_to_midpoint` | `0.25` | `open_space_trajectory_post_process.cc:60,816` | 分段终点 IoU 阈值 |
| `speed_replan_distance` | `2.0` m | `open_space_trajectory_post_process.cc:260` | 重规划触发距离 |
| `linear_velocity_threshold_on_ego` | `0.1` m/s | 同上 | 自车静止判定阈值 |
| `use_gear_shift_trajectory` | `true` | `open_space_trajectory_post_process.cc:160` | 是否生成换挡轨迹 |

---

## 三、对常规道路行驶的影响评估

**总体结论：这 17 个配置仅当车辆进入 OpenSpace 泊车/脱困/大曲率场景时生效，8 个常规道路赛题（车道保持、借道绕行、路口、人行道、减速带、停止牌、红绿灯等）走的是 LANE_FOLLOW 场景及常规任务链，几乎不读取这些参数。**

需留意、可能影响常规道路行为的点：

1. **`pull_over` 场景（靠边停车）** — 唯一与"常规道路 + 目的地"直接相关的场景：
   - `start_pull_over_scenario_distance = 50.0`：距目的地 50m 内开始搜索靠边点；`max_distance_stop_search = 25.0`、`pull_over_min_distance_buffer = 10.0` 决定停靠点位置与缓冲；
   - `pull_over_path.pb.txt` 中 `pull_over_destination_to_adc_buffer = 25.0`、`pull_over_road_edge_buffer = 0.15`、`pull_over_weight = 10`、`pull_over_direction = RIGHT_SIDE`、`pull_over_position = DESTINATION`：决定靠边路径在目的地旁的停车位置、贴边程度。若赛题要求"到达目的地即停/不靠边"，需在此场景调整。
   - **但注意**：`pull_over` 场景是否启用由 `public_road_planner_config.pb.txt` 的 Scenario 注册表控制，且 `PULL_OVER_RETRY_APPROACH_PARKING` Stage 已 `enabled: false`。常规赛题如不配置 pull_over 触发，则不影响。

2. **`valet_parking` / `valet_parking_park` 接近停车位阶段**：`stage_approaching_parking_spot.cc` 在距车位 `parking_spot_range_to_start = 20.0` 内、`max_valid_stop_distance = 1.0` 处停车。仅在用户下发泊车任务（Routing 带停车位）时进入，常规道路赛题不触发。

3. **`park_and_go` 起步场景**：`min_dist_to_dest = 10.0`、`front_obstacle_buffer = 10.0` 仅在停车后起步（PARK_AND_GO 场景）使用，不影响正常巡航。

4. **`large_curvature`**：`scenario_conf.pb.txt` 为空，靠 proto 默认 `min_curvature = 0.2`（曲率 > 0.2 才触发）。该场景走 OpenSpace 重规划，**若赛题地图含回头弯/U 型弯，此场景可能被触发**并接管规划（属于 8 赛题边缘情况）。

5. **其余任务（7/8/9/10/11/13/14/15/16/17）**：全部只在 OpenSpace 任务链（泊车 Stage）中调用，常规道路任务的 pipeline 里不出现这些 Task，无影响。

> ⚠️ 唯一真正的"常规道路交叉点"是 `pull_over`（靠边停车）场景及其 `pull_over_path` 任务——若 8 个赛题中任何一题在终点/目的地附近要求特殊停车行为，需要重点检查这里的 `pull_over_destination_to_adc_buffer`、`pull_over_position`、`start_pull_over_scenario_distance` 等参数。
