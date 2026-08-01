# Apollo Planning 全局配置逐参数分析报告

- **工程**：/home/skye/application-pnc（Apollo 11.0 EDU，官方原始代码）
- **分析范围**：planning_component/conf 与 planners 目录下 12 个全局配置文件
- **分析方法**：逐行读取配置 → 在 `modules/planning/planning_base/gflags/planning_gflags.cc` 查默认值 → grep 定位使用代码 → 阅读代码说明作用
- **赛题关联标注**：🏁 = 与 8 大赛题（减速带≤3m/s、人行道停车 1.5-2.0m、红绿灯停车、停止标志停车、障碍物停车≥2m、动态跟随、无人人行道≤5m/s、绿灯路口≤5m/s）直接相关

> ⚠️ 注意：`--min_length_for_lane_change`、`--ignore_overlapped_obstacle`、`--use_navigation_mode`、`--noenable_nudge_decision`、`--planning_config_file` 等 flag 在本工作区源码中**没有定义**（位于预编译的 `pnc_map`/`planning_base` 二进制依赖包中），文中已标注为"上游定义"。

---

## 一、planning.conf（gflags，重点文件）

> 被 `planning_navi.conf` 通过 `--flagfile` 引用，是标准模式（on-lane planning）的核心配置。

### 1. `--flagfile=/apollo/modules/common/data/global_flagfile.txt`
- **当前值**：`/apollo/modules/common/data/global_flagfile.txt`
- **作用**：gflags 的 flagfile 引用机制，导入 Apollo 全局公共 gflags（如日志级别等）。
- **影响**：无直接驾驶行为影响。

### 2. `--traffic_rule_config_filename=modules/planning/planning_component/conf/traffic_rule_config.pb.txt`
- **当前值**：同上；**默认值**：同（`planning_gflags.cc:35`）
- **代码位置**：`planning_interface_base/traffic_rules_base/traffic_decider.cc:37-39`
  ```cpp
  AINFO << "Load config path:" << FLAGS_traffic_rule_config_filename;
  if (!apollo::cyber::common::LoadConfig(FLAGS_traffic_rule_config_filename, &rule_pipeline_))
  ```
- **作用**：指定交通规则流水线（TrafficRule 插件列表）配置文件。`TrafficDecider::Init` 用它加载启用的规则集合。
- **影响**：决定启用哪些交通规则（见文件四）。🏁 与所有人行道/信号灯/停止标志/目的地停车赛题相关（规则开关的总闸）。

### 3. `--planning_upper_speed_limit=20.00` 🏁
- **当前值**：`20.00` m/s（72 km/h）；**默认值**：`31.3` m/s（`planning_gflags.cc:95`）
- **代码位置**：
  - `planning_base/reference_line/reference_line.cc:927` — `GetSpeedLimitFromS()`：作为参考线限速的**上限兜底**（对车道限速取 min）
  - `tasks/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.cc:156,193` — 速度规划的 `v_upper_bound`
  - `tasks/path_time_heuristic/gridded_path_time_graph.cc:378` — DP 速度搜索网格的速度上限
  - `tasks/piecewise_jerk_speed_nonlinear/...optimizer.cc:206,455` — 非线性速度优化 `s_dot_max_`
  - `tasks/fast_stop_trajectory_fallback/...cc:63`、`planners/navi/.../navi_speed_decider.cc:77`
- **作用**：整个 Planning 模块的**绝对速度上限**。任何道路/场景限速都会被它截断（`std::fmin`）。
- **影响**：🏁 **全局最高车速**。所有赛题限速（3/5 m/s 等）均受此上限约束；当前 20 m/s 不构成对低速赛题的约束，但决定高速跟车/巡航段的最高车速。若某赛题需限速 5 m/s，主要靠 speed_bump/限速区/规则实现，而非本参数。

### 4. `--default_cruise_speed=11.18` 🏁
- **当前值**：`11.18` m/s（≈40.2 km/h）；**默认值**：`5.0` m/s（`planning_gflags.cc:218`）
- **代码位置**：
  - `planning_base/common/frame.cc:185` — `Frame::CreateReferenceLineInfo` 初始化 `target_speed = FLAGS_default_cruise_speed`
  - `planning_base/common/reference_line_info.cc:167,171` — `GetBaseCruiseSpeed()/GetCruiseSpeed()` 兜底
  - `planning_base/common/st_graph_data.cc:62` — ST 图巡航速度
  - `scenarios/bare_intersection_unprotected/stage_approach.cc:181`、`scenarios/emergency_pull_over/stage_standby.cc:54`、`scenarios/traffic_light_unprotected_left_turn/stage_approach.cc:150` — 各场景 `LimitCruiseSpeed(FLAGS_default_cruise_speed)` 限巡航速度
- **作用**：无目标车速指令时的**默认巡航速度**，也是各场景限制巡航速度的基准。
- **影响**：🏁 **决定车辆正常行驶段的目标速度**。当前 11.18 m/s（40 km/h）比默认 5 m/s 高很多，直道跟车/巡航更快；但在"无人人行道≤5m/s""绿灯路口≤5m/s"赛题中，若场景通过 `LimitCruiseSpeed` 限速，会覆盖此值。

### 5. `--ignore_overlapped_obstacle=true`
- **当前值**：`true`；**默认值**：`false`（上游 `modules/map/pnc_map/pnc_map.cc` 定义，预编译包内，工作区无源码）
- **代码位置**：工作区无源码引用（在预编译 pnc_map 库 `LaneFollowMap`/`PncMap` 内使用）。
- **作用**（基于上游 Apollo）：当障碍物与自车（ADC）发生重叠时，是否忽略该障碍物，避免生成错误的自车-障碍物重叠 ST 边界导致异常停车。
- **影响**：开启后，自车与障碍物边界重叠时不会因"障碍物穿车"产生异常急停/决策，起步/绕行更鲁棒。🏁 与"障碍物停车≥2m"赛题间接相关（避免重叠误判）。

### 6. `--prioritize_change_lane` 🏁
- **当前值**：`true`（裸 flag 置真）；**默认值**：`false`（`planning_gflags.cc:75`）
- **代码位置**：
  - `planning_base/reference_line/reference_line_provider.cc:624` — `CreateRouteSegments()` 后调用 `PrioritizeChangeLane(segments)`
  - `PrioritizeChangeLane()`（同文件 :350-364）— 将不在当前 segment 上的**变道路段提升到 list 最前**（优先生成变道参考线）
  - `planners/rtk/rtk_replay_planner.cc:62` — RTK planner 判断
- **作用**：变道策略优先级提升——只要存在合法变道路径就优先执行变道，而不是先走完当前车道再变。
- **影响**：🏁 车辆更"积极"变道/绕行。对"障碍物停车/绕行"类赛题：若前方有静止障碍物且相邻车道可通行，车辆会尽早变道而不是停车等待。

### 7. `--min_length_for_lane_change=5.0`
- **当前值**：`5.0` m；**默认值**：上游 pnc_map 中默认 `30.0` m（预编译包内定义；工作区仅 `pnc_map/lane_follow_map/lane_follow_map_test.cc:30` 有 `DECLARE_double(min_length_for_lane_change)`）
  - ⚠️ 注意：`planning_gflags.cc` 中另有 `change_lane_min_length`（默认 30.0，仅 RTK planner 用），与 `min_length_for_lane_change` 是**两个不同 flag**。
- **代码位置**：预编译 pnc_map `PncMap::GetRouteSegments`（上游语义：变道路段长度小于该阈值时不产生变道路径）。
- **作用**：变道所需最小长度。路段太短（不足以完成变道）时禁止变道。
- **影响**：当前 5.0 m 比上游默认 30 m 小很多 → **几乎任何能变道的路段都允许变道**，配合 `prioritize_change_lane` 使车辆非常积极地绕行/变道。🏁 与障碍物绕行类赛题相关。

### 8. `--nouse_multi_thread_to_add_obstacles`（= `use_multi_thread_to_add_obstacles=false`）
- **当前值**：`false`；**默认值**：`false`（`planning_gflags.cc:213`）
- **代码位置**：`planning_base/common/reference_line_info.cc:437` — `AddObstacles()`：为 true 时用 `cyber::Async` 多线程并行添加障碍物，false 时串行。
- **作用**：控制障碍物 SL 边界计算是否多线程并行。
- **影响**：串行计算更稳定、便于复现；性能稍低。不影响驾驶行为本身。

### 9. `# --min_past_history_points_len=10`（已注释）
- **当前值**：注释未生效；**默认值**：`0`（`planning_gflags.cc:398`）
- **代码位置**：工作区无引用（用于规则规划→学习规划切换的学习模块）。
- **作用**：从规则规划切换到学习（learning-based）规划所需的最小历史轨迹点数。
- **影响**：无（NO_LEARNING 模式不使用）。

### 10. `--enable_print_curve=true`
- **当前值**：`true`；**默认值**：`false`（`planning_gflags.cc:348`）
- **代码位置**：`planning_base/common/util/print_debug_info.cc:36-79` — `PrintPoints::PrintToLog()`、`PrintCurves::AddPoint()` 等，为 true 时把曲线点打印到日志。
- **作用**：调试开关，控制是否把路径/曲线点输出到 INFO 日志。
- **影响**：仅调试日志，不影响驾驶行为（会增大日志量）。

### 11. `--destination_check_distance=4.0` 🏁
- **当前值**：`4.0` m；**默认值**：`5.0` m（`planning_gflags.cc:160`）
- **代码位置**：`planning_base/common/reference_line_info.cc:911-912` — `MakeMainMissionCompleteDecision()`：
  ```cpp
  if (distance_destination > FLAGS_destination_check_distance &&
      distance_to_reference_end > FLAGS_destination_check_distance) {
    return;  // 距离目的地/参考线终点都超过阈值，不判任务完成
  }
  ```
- **作用**：判定"任务完成（mission complete）"的阈值——当 STOP 原因来自目的地/靠边停车/参考线终点且自车距终点小于该距离时，将主决策置为 mission_complete。
- **影响**：🏁 **决定到达终点后何时判定任务完成并停车**。当前 4.0 m（比默认 5.0 更严）：自车距终点≤4m 才判完成，配合 Destination 规则实际停车距离（见 `destination_stop_distance` 等）决定最终停车位置。若赛题要求在终点附近特定距离停车，此参数是重要调优点。

### 12. `--smoother_config_filename=.../discrete_points_smoother_config.pb.txt`
- **当前值**：discrete_points（FEM_POS_DEVIATION）；**默认值**：`qp_spline_smoother_config.pb.txt`（`planning_gflags.cc:39`）
- **代码位置**：`planning_base/reference_line/reference_line_provider.cc:68-82` — 构造时加载 smoother 配置并按 `has_qp_spline()/has_spiral()/has_discrete_points()` 选择 `QpSplineReferenceLineSmoother` / `SpiralReferenceLineSmoother` / `DiscretePointsReferenceLineSmoother`；`smoother_util.cc:60` 同样加载。
- **作用**：选择**参考线平滑器**（注意：这是参考线平滑，不是轨迹平滑；`planner_open_space_config` 才是 open space 轨迹平滑）。
- **影响**：🏁 **决定参考线平滑质量**。当前 FEM_POS_DEVIATION（离散点法）对急弯/窄路适应更好、计算快；QP 样条法更平滑但可能贴边。直接影响绕行/变道/过弯时的路径平滑度与横向偏移。

### 13. `--enable_reference_line_stitching=false`
- **当前值**：`false`；**默认值**：`true`（`planning_gflags.cc:63`）
- **代码位置**：`planning_base/reference_line/reference_line_provider.cc:656`：
  ```cpp
  if (is_new_command_ || !FLAGS_enable_reference_line_stitching) {
    // 每帧全量重建参考线
  } else {
    // 用上一帧参考线拼接（stitching）
  }
  ```
  以及 `qp_spline_reference_line_smoother.cc:172`。
- **作用**：为 false 时，每个周期都**从路由重建整条参考线**；为 true 时复用上一帧的参考线并做重叠拼接（减少计算、提升稳定性）。
- **影响**：关闭 stitching 后参考线每帧全量重建——若发生路由更新/地图变化能更快反映，但可能造成参考线抖动（尤其用离散点平滑时）。对变道/绕行的实时响应有影响。

### 14. `# --speed_bump_speed_limit=3`（已注释）🏁
- **当前值**：注释未生效 → 使用默认值；**默认值**：`4.4704` m/s（10 mph，`planning_gflags.cc` speed_bump_speed_limit 定义处）
- **代码位置**：`planning_base/common/reference_line_info.cc:124-128`：
  ```cpp
  for (const auto& speed_bump : map_path.speed_bump_overlaps()) {
    reference_line_.AddSpeedLimit(speed_bump.start_s - 1.0,
                                  speed_bump.end_s + 1.0,
                                  FLAGS_speed_bump_speed_limit);
  }
  ```
  `AddSpeedLimit`（`reference_line.cc:951`）对重叠区间取 min 合并限速。
- **作用**：对地图中所有**减速带（speed bump）重叠区域**施加统一限速，区间向两侧各扩展 1.0m 保证可采样。
- **影响**：🏁 **与"减速带≤3m/s"赛题直接相关**！当前配置**注释掉了该行**，车辆过减速带限速为默认 **4.47 m/s（>3 m/s，可能超速扣分）**。若赛题要求≤3 m/s，应取消注释并设为 `3`。此参数是减速带赛题的头号调优点。

### 15. `# --parking_inwards=false`（已注释）
- **当前值**：注释未生效；**默认值**：本版本无此 gflag（`parking_inwards` 是 `planner_open_space_config.pb.txt` 中 `roi_config` 的 pb 字段，默认 false）
- **代码位置**：`planning_open_space/tools/open_space_roi_wrapper.cc:121` — 根据 `parking_inwards()` 决定泊车末端位姿（车头朝内/朝外，`end_y` 取 1/4 或 3/4 车位深度）。
- **作用**：open space 泊车时车位"头进"还是"尾进"。
- **影响**：仅影响开放空间泊车（VALET_PARKING）场景，标准道路赛题一般不触发。

### 16. `# --use_dual_variable_warm_start=true`（已注释，即默认开启）
- **当前值**：`true`（默认）；**默认值**：`true`（`planning_gflags.cc` use_dual_variable_warm_start）
- **代码位置**：`tasks/open_space_trajectory_provider/open_space_trajectory_optimizer.cc:571` — 为 true 时先解对偶变量 warm start 问题再进 distance approach 平滑。
- **作用**：open space 平滑的对偶变量热启动。
- **影响**：仅 open space（泊车/掉头）场景，提升求解成功率与速度。

### 17. `# --enable_record_debug=true`（已注释，即默认开启）
- **当前值**：`true`（默认）；**默认值**：`true`（`planning_gflags.cc` enable_record_debug）
- **代码位置**：`planners/public_road/public_road_planner.cc:48`、`on_lane_planning.cc:337`、`tasks/speed_bounds_decider/speed_bounds_decider.cc:172`、`stage.cc:266`、`open_space_trajectory_optimizer.cc:264` 等多处——为 false 时跳过 ST 图/调试信息记录。
- **作用**：是否记录 ST 图、speed profile 等调试信息（供 DreamView 显示）。
- **影响**：仅调试/可视化，不影响驾驶行为。

### 18. `# --enable_parallel_hybrid_a=true`（已注释）
- **当前值**：`false`（默认）；**默认值**：`false`（`planning_gflags.cc` enable_parallel_hybrid_a）
- **代码位置**：`planning_open_space/coarse_trajectory_generator/reeds_shepp_path.cc:187` — 并行 Hybrid A* 实现分支。
- **作用**：open space 粗轨迹搜索是否并行。
- **影响**：仅 open space 性能，不影响行为。

### 19. `--export_chart=true`
- **当前值**：`true`；**默认值**：`false`（`planning_gflags.cc:345`）
- **代码位置**：`planning_component/on_lane_planning.cc:616` — 为 true 时 `ExportOnLaneChart(best_ref_info->debug(), ptr_debug)`，否则 `ExportReferenceLineDebug` + 失败的变道 ST 图。
- **作用**：是否导出规划图表（chart）到 trajectory debug。
- **影响**：仅调试可视化（DreamView 显示图表），不影响驾驶行为。开启时对变道失败 ST 图不再单独导出。

### 20. `# --use_front_axe_center_in_path_planning=true`（已注释）
- **当前值**：`false`（默认）；**默认值**：`false`（`planning_gflags.cc` use_front_axe_center_in_path_planning）
- **代码位置**：`planning_interface_base/task_base/common/path_generation.cc:110`（起点前移一个轴距）、`path_bounds_decider_util.cc:76`、`tasks/lane_follow_path.cc:224`、`lane_borrow_path.cc:179`、`lane_change_path.cc:183`、`pull_over_path.cc:226`、`fallback_path.cc:125`、`square_path.cc:170`、`reverse_path.cc:205` 等全部路径任务。
- **作用**：路径规划是否用**前轴中心**代替后轴中心作为车辆参考点（前轴更"敏捷"、更贴内弯，但需更大转向）。
- **影响**：为 false 时用后轴中心（标准），过弯更保守。若赛题急弯需要更激进路径可开启，但通常保持 false。

### 21. `--enable_smoother_failsafe`
- **当前值**：`true`；**默认值**：`false`（`planning_gflags.cc` enable_smoother_failsafe）
- **代码位置**：`tasks/open_space_trajectory_provider/open_space_trajectory_optimizer.cc:596` — distance approach 求解失败时：
  ```cpp
  if (FLAGS_enable_smoother_failsafe) { UseWarmStartAsResult(...); } else { return false; }
  ```
- **作用**：open space 平滑求解失败时，**用 Hybrid A* 的 warm start 结果兜底**输出，而不是整体失败。
- **影响**：仅 open space 场景鲁棒性提升（避免泊车/掉头时无轨迹可用）。

### 22. `--enable_parallel_trajectory_smoothing`
- **当前值**：`true`；**默认值**：`false`（`planning_gflags.cc` enable_parallel_trajectory_smoothing）
- **代码位置**：`tasks/open_space_trajectory_provider/open_space_trajectory_optimizer.cc:129` — 先 `TrajectoryPartition` 分段，再对每段并行平滑后 `CombineTrajectories` 拼接；`tools/distance_approach_problem_wrapper.cc:513` 同样。
- **作用**：open space 轨迹是否分段并行平滑。
- **影响**：仅 open space 性能与轨迹一致性（分段点可能引入轻微不连续）。不涉及标准道路赛题。

### 23. `--nouse_s_curve_speed_smooth`（= `use_s_curve_speed_smooth=false`）
- **当前值**：`false`；**默认值**：`false`（`planning_gflags.cc` use_s_curve_speed_smooth）
- **代码位置**：`planning_open_space/coarse_trajectory_generator/hybrid_a_star.cc:654` — 为 true 用 `GenerateSCurveSpeedAcceleration`，false 用 `GenerateSpeedAcceleration`。
- **作用**：Hybrid A* 粗轨迹的速度/加速度是否用 S 曲线（piecewise_jerk）平滑。
- **影响**：仅 open space 粗轨迹速度 profile 的平滑方式。false 用简单前向差分法（更快）。

### 24. `--use_iterative_anchoring_smoother`
- **当前值**：`true`；**默认值**：`false`（`planning_gflags.cc` use_iterative_anchoring_smoother）
- **代码位置**：`tasks/open_space_trajectory_provider/open_space_trajectory_optimizer.cc:175,252`（为 true 走 `GenerateDecoupledTraj` 迭代锚定平滑，否则 distance approach；非并行分支强制置 false）；`tasks/open_space_trajectory_partition/...cc:261`、`tasks/open_space_trajectory_post_process/...cc:282`。
- **作用**：open space 平滑器选择——迭代锚定（iterative anchoring）vs 距离逼近（distance approach）。
- **影响**：仅 open space。iterative anchoring 更快、更适合窄空间泊车，但精度略低。与标准道路赛题无关。

### 25. `--nonstatic_obstacle_nudge_l_buffer=0.4` 🏁
- **当前值**：`0.4` m；**默认值**：`0.4` m（`planning_gflags.cc:140`）
- **代码位置**：
  - `planning_base/common/obstacle.cc:463,470` — `BuildReferenceLineStBoundary` 中计算动静态障碍物 SL 边界时加横向缓冲
  - `planning_interface_base/task_base/common/path_util/path_bounds_decider_util.cc:824,976` — 路径边界（path bounds）生成时对**非静态障碍物**的绕行横向距离 = 障碍物半宽 + 自车半宽 + 该 buffer
  - `tasks/speed_bounds_decider/st_boundary_mapper.cc:285` — ST 边界横向扩展
- **作用**：**非静态（动态）障碍物**的横向避让缓冲（静态障碍物用 `static_obstacle_nudge_l_buffer`=0.3）。
- **影响**：🏁 决定绕行动态障碍物（如慢车、锥桶移动物）时路径偏离障碍物的**横向间距**。当前 0.4m 较小 → 绕行更贴障碍物（占道少、不易压线），但若赛题要求与障碍物保持更大横向距离需调大。

### 26. `--use_st_drivable_boundary=false`
- **当前值**：`false`；**默认值**：`false`（`planning_gflags.cc` use_st_drivable_boundary）
- **代码位置**：
  - `tasks/speed_decider/speed_decider.cc:105,111,264` — 为 false 时用 ST 边界线段相交法判断 `CROSS/ABOVE/BELOW`，并走 KEEP_CLEAR 特殊处理
  - `tasks/speed_bounds_decider/speed_bounds_decider.cc:67`、`st_boundary_mapper.cc:166,420`、`tasks/path_time_heuristic/dp_st_cost.cc:119`、`gridded_path_time_graph.cc:52`
- **作用**：速度规划是否使用"ST 可行驶边界（st_drivable_boundary）"新方法（false 用传统 STBoundary 方法）。
- **影响**：为 false 时使用 Apollo 经典 ST 图 + 障碍物 ST boundary 决策（跟随/让行/超车/停车），行为成熟稳定。此开关决定速度决策的核心算法分支，改动需谨慎。🏁 影响所有涉及跟车/停车/让行的赛题。

### 27. `--enable_pull_over_at_destination=false` 🏁
- **当前值**：`false`；**默认值**：`false`（`planning_gflags.cc:407`）
- **代码位置**：`scenarios/pull_over/pull_over_scenario.cc:63` — `IsTransferable()`：
  ```cpp
  if (!FLAGS_enable_pull_over_at_destination) { return false; }
  ```
- **作用**：是否允许在到达目的地时进入 PULL_OVER（靠边停车）场景。
- **影响**：🏁 当前 false → 到目的地后**不触发靠边停车场景**，而是由 Destination 规则直接停车（原地停）。若赛题要求"终点靠边停车"需置 true。对"人行道/红绿灯/停止标志停车"类赛题无影响（那些走的是对应 TrafficRule）。

---

## 二、planning_navi.conf（导航模式，一般不用于标准赛题）

> 通过 `--flagfile` 包含 planning.conf，再叠加导航模式参数。标准 on-lane 模式下该文件**不加载**（由 `planning.conf` 决定），仅 DreamView 选择 navigation mode 时使用。

### 1. `--planning_config_file=.../planning_config_navi.pb.txt`
- **作用**：导航模式使用的 PlanningConfig（topic/learning_mode/pnc_map_class）。
- **影响**：仅导航模式。

### 2. `--planning_upper_speed_limit=24.587`
- **作用**：导航模式下全局速度上限（覆盖 planning.conf 的 20）。
- **影响**：仅导航模式。

### 3. `--noenable_nudge_decision`（= `enable_nudge_decision=false`）
- **作用**：导航模式下关闭绕行（nudge）决策。上游默认 true。
- **影响**：仅导航模式。

### 4. `--replan_lateral_distance_threshold=0.5`
- **当前值**：`0.5` m；**默认值**：`0.5`（`planning_gflags.cc:409`）
- **代码位置**：`planning_base/common/trajectory_stitcher.cc:197`（横向偏差超阈值触发重新规划）、`pnc_map/lane_follow_map/lane_follow_map.cc:192`（参考线外推边界）。
- **作用**：轨迹拼接时横向偏差超过该值则判定需重新规划。
- **影响**：标准模式下同样生效（flagfile 继承）——横向偏差>0.5m 时放弃拼接重新规划，影响变道/绕行后的轨迹连续性。

### 5. `--use_navigation_mode`
- **作用**：开启导航模式（使用 relative map 而非高精地图 routing）。
- **影响**：标准赛题不使用。

---

## 三、planning_config.pb.txt（PlanningConfig 主配置）

> 由 planning_component 通过 `ComponentBase::GetProtoConfig` 加载（`planning_component.cc:48`），决定 topic 与学习模式。

### 1. `topic_config { ... }`（15 个 topic）
- **字段**：chassis_topic、hmi_status_topic、localization_topic、planning_pad_topic、planning_trajectory_topic、prediction_topic、relative_map_topic、routing_request_topic、routing_response_topic、planning_command_topic、story_telling_topic、traffic_light_detection_topic、planning_learning_data_topic、perception_edge_info_topic、control_interative_topic
- **代码位置**：`planning_component.cc:57+` — 用 `config_.topic_config().xxx_topic()` 创建各 Cyber Reader/Writer。
- **作用**：规划模块订阅/发布的所有话题名。
- **影响**：话题名与仿真/DreamView 不符会导致收不到数据（如红绿灯、路由）。**通常不需要改**；若赛事平台话题名不同才需调整。

### 2. `learning_mode: NO_LEARNING`
- **代码位置**：`planning_component.cc:59,213,227` — `config_.learning_mode() != PlanningConfig::NO_LEARNING` 时才初始化学习模块/消息处理。
- **作用**：学习模式选择。NO_LEARNING = 纯规则规划。
- **影响**：当前为规则规划，是赛题标准配置。若误设为其他值会启用学习模块并加载 `planning_semantic_map_config.pb.txt`（文件十），增加开销。

### 3. `reference_line_config { pnc_map_class: "apollo::planning::LaneFollowMap" }`
- **代码位置**：`on_lane_planning.cc:128-133` 传入 `ReferenceLineProvider`；`reference_line_provider.cc:88-99` — 按 `pnc_map_class()` 通过插件管理器创建 PncMap 实例（空则默认 LaneFollowMap）。
- **作用**：指定 PnC Map 插件类名（把路由转换成参考线的地图插件）。
- **影响**：LaneFollowMap = 基于高精地图 + 路由的标准 PnC Map。赛题标准配置。

---

## 四、traffic_rule_config.pb.txt（交通规则启用清单）

> 被 `TrafficDecider::Init`（`traffic_decider.cc:37-55`）加载：对每个 `rule`，按 `type` 用插件管理器创建 TrafficRule 实例，按 `name` 作为实例名。

### 启用的 9 条规则（name → type → 插件类）

| # | name | type | 插件类 | 作用 |
|---|------|------|--------|------|
| 1 | BACKSIDE_VEHICLE | BacksideVehicle | `traffic_rules/backside_vehicle` | 后方来车（变道时后车让行/速度决策） |
| 2 | CROSSWALK | Crosswalk | `traffic_rules/crosswalk` | 🏁 **人行道规则**：检测人行道重叠，有人时停车、无人时限速 |
| 3 | DESTINATION | Destination | `traffic_rules/destination` | 🏁 **目的地停车**：到达终点生成 STOP_DESTINATION |
| 4 | KEEP_CLEAR | KeepClear | `traffic_rules/keepclear` | 禁停区（网格/路口）内不停车 |
| 5 | REFERENCE_LINE_END | ReferenceLineEnd | `traffic_rules/reference_line_end` | 参考线终点停车（PATH_END） |
| 6 | REROUTING | Rerouting | `traffic_rules/rerouting` | 偏离路线/卡死时请求重新路由 |
| 7 | STOP_SIGN | StopSign | `traffic_rules/stop_sign` | 🏁 **停止标志规则**：路口停止线停车/让行 |
| 8 | TRAFFIC_LIGHT | TrafficLight | `traffic_rules/traffic_light` | 🏁 **信号灯规则**：红灯/黄灯停车、绿灯通行 |
| 9 | YIELD_SIGN | YieldSign | `traffic_rules/yield_sign` | 让行标志规则 |

- **影响**：🏁 **这是赛题的规则总闸**——人行道/信号灯/停止标志/目的地停车都依赖这里启用对应规则。各规则内部参数（停车距离等）在各自模块的 config 中（如 `crosswalk_config.pb.txt`、`traffic_light_config.pb.txt`、`destination_config.pb.txt`、`stop_sign_config.pb.txt`），不在本文件。注意：**SpeedSetting 规则（对无人人行道/路口限速 5m/s 常用）未启用**——若赛题"无人人行道≤5m/s""绿灯路口≤5m/s"需要该机制，需在此追加 `SpeedSetting` 规则。

---

## 五、public_road_planner_config.pb.txt（PublicRoadPlanner 场景清单）

> 由 `public_road_planner.cc:32` `LoadConfig<PlannerPublicRoadConfig>(FLAGS_planner_config_path, &config_)` 加载，传给 `ScenarioManager::Init`（场景注册表）。每个 `scenario { name, type }` 中 `type` 为场景插件类名。

### 启用的 12 个场景（type → 插件）

| name | type | 说明 |
|------|------|------|
| EMERGENCY_PULL_OVER | EmergencyPullOverScenario | 紧急靠边停车 |
| EMERGENCY_STOP | EmergencyStopScenario | 紧急停车 |
| VALET_PARKING | ValetParkingScenario | 代客泊车（开放空间） |
| BARE_INTERSECTION_UNPROTECTED | BareIntersectionUnprotectedScenario | 无保护十字路口 |
| STOP_SIGN_UNPROTECTED | StopSignUnprotectedScenario | 🏁 无保护停止标志路口 |
| YIELD_SIGN | YieldSignScenario | 让行路口 |
| TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN | TrafficLightUnprotectedLeftTurnScenario | 🏁 无保护左转（信号灯） |
| TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN | TrafficLightUnprotectedRightTurnScenario | 🏁 无保护右转（信号灯） |
| TRAFFIC_LIGHT_PROTECTED | TrafficLightProtectedScenario | 🏁 保护型信号灯路口 |
| PULL_OVER | PullOverScenario | 靠边停车 |
| PARK_AND_GO | ParkAndGoScenario | 停车再起步 |
| LANE_FOLLOW | LaneFollowScenario | 🏁 车道保持（默认场景，含绕行/借道/变道任务） |

- **影响**：🏁 场景框架决定车辆在哪种"剧本"下运行。红灯/停止标志/人行道主要走对应 TrafficRule（文件四），路口直行/左右转走上面路口场景。**LANE_FOLLOW 是大多数赛题的默认场景**。

---

## 六、discrete_points_smoother_config.pb.txt（离散点参考线平滑器）

> 由 `FLAGS_smoother_config_filename` 指向（planning.conf 第 12 项），`reference_line_provider.cc:68` 加载，`discrete_points_reference_line_smoother.cc` 使用。

### 公共字段（ReferenceLineSmootherConfig）
| 参数 | 当前值 | 代码位置 | 作用 |
|------|--------|----------|------|
| `max_constraint_interval` | 0.25 m | `reference_line_provider.cc:945`（锚点间隔 = 参考线长度/interval） | 参考线上锚点（anchor point）的采样间距。越小锚点越密、平滑越精细 |
| `longitudinal_boundary_bound` | 2.0 m | `reference_line_provider.cc:869` | 锚点纵向约束（沿道路方向的允许偏差） |
| `max_lateral_boundary_bound` | 0.5 m | `:873,:936-937` | 锚点横向约束上限（平滑允许的最大横向偏移） |
| `min_lateral_boundary_bound` | 0.1 m | `:936-937` | 锚点横向约束下限 |
| `curb_shift` | 0.2 m | `:902-918` | 路边有 CURB 时，安全宽度额外减去该值并让参考线向内侧偏移（避让路缘） |
| `lateral_buffer` | 0.2 m | `:924` | 锚点横向范围再减 2×buffer 作为缓冲（贴边冗余） |

- **影响**：🏁 这些参数决定**参考线平滑的横向贴边程度**。max_lateral_boundary_bound=0.5 表示平滑最多偏离原始参考线 0.5m；curb_shift/lateral_buffer 使车辆在路边让出安全距离。对绕行、借道、过窄路、贴边行驶均有影响。

### 离散点平滑段（discrete_points {}）
| 参数 | 当前值 | 代码位置 | 作用 |
|------|--------|----------|------|
| `smoothing_method` | FEM_POS_DEVIATION_SMOOTHING | `discrete_points_reference_line_smoother.cc:57-64` | 平滑算法：FEM 位置偏差法（当前）或 COS_THETA 法 |
| `weight_fem_pos_deviation` | 1e10 | `:139-142` → `FemPosDeviationSmoother` | FEM 相邻点偏差权重（极大 → 强制相邻点距离均匀/平滑） |
| `weight_ref_deviation` | 1.0 | 同上 | 相对原始参考线的偏差权重（越小越贴原始线） |
| `weight_path_length` | 1.0 | 同上 | 路径长度最小化权重 |
| `apply_curvature_constraint` | false | 同上 | 是否施加曲率约束（false 不约束曲率，可更快但可能急弯） |
| `max_iter` | 500 | 同上 | OSQP 最大迭代次数 |
| `time_limit` | 0.0 | 同上 | 求解时间上限（0=不限） |
| `verbose` | false | 同上 | 求解器日志 |
| `scaled_termination` | true | 同上 | 缩放终止条件 |
| `warm_start` | true | 同上 | 热启动 |

- **影响**：🏁 `weight_fem_pos_deviation=1e10` 是官方调好的平滑强度；`apply_curvature_constraint=false` 使参考线可能带小曲率尖点（对高速过弯略不利，但对窄路绕行更灵活）。

---

## 七、qp_spline_smoother_config.pb.txt（QP 样条参考线平滑器，未启用）

> 当前 `smoother_config_filename` 指向 discrete_points，本文件**未使用**（默认值即指向它，仅作备份）。

### 公共字段
| 参数 | 当前值 | 作用 |
|------|--------|------|
| `max_constraint_interval` | 5.0 m | 锚点采样间隔 |
| `longitudinal_boundary_bound` | 2.0 m | 纵向约束 |
| `max_lateral_boundary_bound` | 0.5 m | 横向约束上限 |
| `min_lateral_boundary_bound` | 0.1 m | 横向约束下限 |
| `num_of_total_points` | 500 | QP 样条总采样点数 |
| `curb_shift` | 0.2 m | 路缘偏移 |
| `lateral_buffer` | 0.2 m | 横向缓冲 |

### qp_spline 段
| 参数 | 当前值 | 作用 |
|------|--------|------|
| `spline_order` | 5 | 样条阶数 |
| `max_spline_length` | 25.0 m | 每段样条最大长度 |
| `regularization_weight` | 1.0e-5 | 正则化权重 |
| `second_derivative_weight` | 200.0 | 二阶导（曲率）惩罚 |
| `third_derivative_weight` | 1000.0 | 三阶导（曲率变化率）惩罚 |

- **影响**：若切回 QP 样条平滑，参考线更平滑连续（适合高速），但对窄路/急弯的适应性不如离散点法。

---

## 八、spiral_smoother_config.pb.txt（螺旋线参考线平滑器，未启用）

> 当前未使用（smoother_config_filename 指向 discrete_points）。

### 公共字段（同 QP，`resolution: 0.02` 为新增）
### spiral 段
| 参数 | 当前值 | 作用 |
|------|--------|------|
| `max_deviation` | 0.05 m | 最大偏差 |
| `piecewise_length` | 10.0 m | 分段长度 |
| `max_iteration` | 500 | 迭代上限 |
| `opt_tol` | 1.0e-6 | 优化容差 |
| `opt_acceptable_tol` | 1e-4 | 可接受容差 |
| `opt_acceptable_iteration` | 15 | 可接受迭代数 |
| `weight_curve_length` | 1.0 | 曲线长度权重 |
| `weight_kappa` | 1.0 | 曲率权重 |
| `weight_dkappa` | 100.0 | 曲率变化率权重 |

- **影响**：螺旋线平滑能严格满足曲率约束（最适合高速弯道），但计算慢、对复杂道路鲁棒性差，官方一般不用。

---

## 九、planner_open_space_config.pb.txt（开放空间规划配置）

> 由 `FLAGS_planner_open_space_config_filename` 加载，`OpenSpaceTrajectoryOptimizer`/`open_space_roi_wrapper`/`hybrid_a_star`/`distance_approach_problem_wrapper` 使用。**仅影响开放空间（泊车/掉头/窄空间）场景，标准道路赛题一般不触发**，此处简列。

### roi_config（ROI 提取）
| 参数 | 当前值 | 作用 |
|------|--------|------|
| `roi_longitudinal_range_start` | 15.0 m | ROI 终点沿参考线向前扩展距离（`open_space_roi_wrapper.cc:65`） |
| `roi_longitudinal_range_end` | 15.0 m | ROI 终点向后扩展距离（`:66`） |
| `parking_start_range` | 12.0 m | 开始寻找车位距离 |
| `parking_inwards` | false | 泊车头进/尾进（`:121`，决定末端位姿） |

### warm_start_config（Hybrid A* 热启动）
| 参数 | 当前值 | 作用 |
|------|--------|------|
| `xy_grid_resolution` | 0.3 m | XY 网格分辨率（`node3d.cc:50`） |
| `phi_grid_resolution` | 0.1 rad | 航向网格分辨率（`:52`） |
| `next_node_num` | 10 | 每节点扩展数（`hybrid_a_star.cc:42`） |
| `step_size` | 0.5 m | 搜索步长（`:45`） |
| `traj_forward_penalty` | 1.0 | 前进惩罚（`reeds_shepp_path.cc:33`） |
| `traj_back_penalty` | 1.0 | 倒车惩罚（`:34`） |
| `traj_gear_switch_penalty` | 10.0 | 换挡惩罚 |
| `traj_steer_penalty` | 0.0 | 转向惩罚 |
| `traj_steer_change_penalty` | 0.0 | 转向变化率惩罚 |
| `grid_a_star_xy_resolution` | 0.5 m | A* 网格分辨率（`grid_search.cc:29`） |
| `node_radius` | 0.25 m | 节点碰撞半径（`:30,:55`） |
| `s_curve_config` | acc 1.0 / jerk 0.0 / kappa 100.0 / ref_s 0.1 / ref_v 0.0 | 速度 S 曲线权重 |

### dual_variable_warm_start_config（对偶变量热启动）
- `weight_d` 1.0、`qp_format: OSQP`、`min_safety_distance` 0.01、`beta` 1.0、ipopt/osqp 求解器参数（`ipopt_max_iter:100`、`ipopt_tol:1e-05` 等）。
- **作用**：`open_space_trajectory_optimizer.cc:571` 中对偶变量 warm start 问题的求解配置。

### distance_approach_config（距离逼近平滑，DISTANCE_APPROACH_IPOPT）
- 关键行为参数：`weight_steer` 0.3、`weight_a` 1.1、`weight_steer_rate` 2.0、`weight_a_rate` 2.5、`weight_x` 18.0、`weight_y` 14.0、`weight_phi` 10.0、`max_speed_forward` 2.0、`max_speed_reverse` 1.0、`max_acceleration_forward` 2.0、`max_acceleration_reverse` 1.0、`min_time_sample_scaling` 0.5、`max_time_sample_scaling` 1.5、`use_fix_time` false、`ipopt_max_iter:1000`。
- **作用**：open space 轨迹平滑的代价权重与速度/加速度/时间约束。

### iterative_anchoring_smoother_config（迭代锚定平滑，当前启用）
| 参数 | 当前值 | 作用 |
|------|--------|------|
| `interpolated_delta_s` | 0.1 m | 插值步长 |
| `reanchoring_trails_num` | 50 | 重锚定轨迹数 |
| `reanchoring_pos_stddev` | 0.25 | 位置扰动标准差 |
| `reanchoring_length_stddev` | 1.0 | 长度扰动标准差 |
| `estimate_bound` | false | 是否估计边界 |
| `default_bound` | 2.0 | 默认边界 |
| `vehicle_shortest_dimension` | 1.04 m | 车辆最短维度 |
| `fem_pos_deviation_smoother_config` | weight 1e8/1.0/1e3, curvature_constraint 0.2, max_iter 500 | 内部 FEM 平滑 |
| `collision_decrease_ratio` | 0.9 | 碰撞率递减 |
| `max_forward_v` / `max_reverse_v` | 2.0 / 2.0 m/s | 前后向最大速度 |
| `max_forward_acc` / `max_reverse_acc` | 3.0 / 2.0 | 前后向最大加速度 |
| `max_acc_jerk` | 4.0 | 最大加加速度 |
| `delta_t` | 0.2 s | 时间步长 |
| `s_curve_config` | acc 1.0 / jerk 1.0 / kappa 100.0 / ref_s 10.0 / ref_v 0.0 | S 曲线权重 |

### 顶层字段
| 参数 | 当前值 | 作用 |
|------|--------|------|
| `delta_t` | 0.5 s | open space 轨迹时间步长（`hybrid_a_star.cc:53`、`distance_approach_problem_wrapper.cc:311`） |
| `near_destination_threshold` | 0.05 | 接近终点阈值 |
| `enable_linear_interpolation` | false | 是否线性插值（`:321`，短轨迹时用） |

---

## 十、planning_semantic_map_config.pb.txt（语义图渲染配置，学习模式用）

> 由 `FLAGS_planning_birdview_img_feature_renderer_config_file` 加载（`on_lane_planning.cc:141`），仅当 `learning_mode != NO_LEARNING` 时才初始化 `BirdviewImgFeatureRenderer`。**当前 NO_LEARNING，本文件不生效**。

| 参数 | 当前值 | 代码位置 | 作用 |
|------|--------|----------|------|
| `resolution` | 0.2 m/px | `birdview_img_feature_renderer.cc` | 鸟瞰图分辨率 |
| `height` / `width` | 200 / 200 px | `:72-118` | 渲染图尺寸（cv::Mat） |
| `ego_idx_x` / `ego_idx_y` | 100 / 160 | `:279` | 自车在图中像素位置 |
| `max_rand_delta_phi` | 25° | 学习数据增强随机旋转角 |
| `max_ego_future_horizon` | 2 s | 自车未来轨迹渲染时长 |
| `max_ego_past_horizon` | 6 s | 自车历史轨迹渲染时长（`:329-333`） |
| `max_obs_future_horizon` | 2 s | 障碍物未来轨迹渲染时长 |
| `max_obs_past_horizon` | 1 s | 障碍物历史轨迹渲染时长（`:367`） |
| `base_map_padding` | 100 px | 底图填充 |
| `city_driving_max_speed` | 22.22 m/s | 城市驾驶最大速度（渲染速度色标用） |

- **影响**：仅学习（learning-based）规划的数据渲染。赛题（规则规划）不生效。

---

## 十一、planners/public_road/conf/planner_config.pb.txt（PublicRoadPlanner 场景清单）

- **内容**：与文件五 `public_road_planner_config.pb.txt` **完全相同**（12 个场景，LANE_FOLLOW 等）。
- **加载方式**：`public_road_planner.cc:32` `LoadConfig<PlannerPublicRoadConfig>(config_path, &config_)`，其中 `config_path = FLAGS_planner_config_path`。
- **注意**：实际生效的是 `FLAGS_planner_config_path`（默认 `modules/planning/planning_component/conf/public_road_planner_config.pb.txt`，即文件五）。本文件是**冗余副本**，仅当 `FLAGS_planner_config_path` 被改为指向它时才生效。
- **影响**：🏁 若官方/赛事把 `FLAGS_planner_config_path` 指向本文件，则此处场景清单生效；与文件五保持一致即可。

---

## 十二、planners/navi/conf/planner_config.pb.txt（NaviPlanner 配置，导航模式）

> 仅导航模式（`use_navigation_mode=true`）时由 `navi_planner.cc:90` `LoadConfig<PlannerNaviConfig>(config_path, &planner_conf)` 加载，注册 `NAVI_PATH_DECIDER`/`NAVI_SPEED_DECIDER` 两个 task。标准模式不使用。

### task 列表
- `NAVI_PATH_DECIDER`、`NAVI_SPEED_DECIDER`：导航模式仅两个任务（路径决策 + 速度决策）。

### navi_path_decider_config
| 参数 | 当前值 | 作用 |
|------|--------|------|
| `min_path_length` | 5 m | 最小路径长度 |
| `min_look_forward_time` | 2 s | 最小前视时间 |
| `max_keep_lane_distance` | 0.4 m | 保道最大横向距离 |
| `max_keep_lane_shift_y` | 0.15 m | 保道最大横移 |
| `min_keep_lane_offset` | 0.20 m | 最小保道偏移 |
| `keep_lane_shift_compensation` | 0.01 | 保道偏移补偿 |
| `move_dest_lane_config_talbe` | lateral_shift { max_speed 34, max_move_dest_lane_shift_y 0.45 } | 移动目标车道横向偏移表 |
| `move_dest_lane_compensation` | 0.35 | 目标车道偏移补偿 |
| `max_kappa_threshold` / `kappa_move_dest_lane_compensation` | 0.0 / 0.0 | 曲率阈值 |
| `start_plan_point_from` | 0 | 规划起点来源 |

### navi_speed_decider_config
| 参数 | 当前值 | 作用 |
|------|--------|------|
| `preferred_accel` / `preferred_decel` | 1.5 / 1.5 | 舒适加减速（`navi_speed_decider.cc:73-74`） |
| `preferred_jerk` | 2.0 | 舒适 jerk |
| `max_accel` / `max_decel` | 4.0 / 5.0 | 最大加减速 |
| `obstacle_buffer` | 1.0 m | 障碍物缓冲 |
| `safe_distance_base` | 10.0 m | 安全距离基准 |
| `safe_distance_ratio` | 1.0 | 安全距离比例 |
| `following_accel_ratio` | 0.5 | 跟车加速度比例 |
| `soft_centric_accel_limit` | 1.0 | 软向心加速度限制 |
| `hard_centric_accel_limit` | 1.5 | 硬向心加速度限制 |
| `hard_speed_limit` | 40.0 m/s | 硬速度上限 |
| `hard_accel_limit` | 8.0 | 硬加速度上限 |
| `enable_safe_path` | false | 是否启用安全路径 |
| `enable_planning_start_point` | true | 启用规划起点 |
| `enable_accel_auto_compensation` | false | 加速度自动补偿 |
| `kappa_preview` / `kappa_threshold` | 80.0 / 0.02 | 曲率预瞄/阈值（弯道减速） |

### navi_obstacle_decider_config
| 参数 | 当前值 | 作用 |
|------|--------|------|
| `min_nudge_distance` / `max_nudge_distance` | 0.2 / 1.1 m | 绕行距离范围 |
| `max_allow_nudge_speed` | 16.667 m/s | 允许绕行的最大速度 |
| `safe_distance` | 0.2 m | 安全距离 |
| `nudge_allow_tolerance` | 0.05 | 绕行允许容差 |
| `cycles_number` | 3 | 判定周期数 |
| `judge_dis_coeff` / `basis_dis_value` | 2.0 / 30.0 | 判定距离系数/基准 |
| `lateral_velocity_value` | 0.5 | 横向速度值 |
| `speed_decider_detect_range` | 0.35 | 速度决策检测范围 |
| `max_keep_nudge_cycles` | 110 | 保持绕行最大周期数 |

- **影响**：全部仅导航模式（relative map）。标准 on-lane 赛题不使用。

---

## 关键参数总结（对赛题影响最大）

### 🏁 与赛题直接相关的"头号"参数
1. **`speed_bump_speed_limit`（当前被注释 → 实际 4.47 m/s）** — 减速带赛题要求≤3 m/s，**必须取消注释并设 3**，否则超速。
2. **`planning_upper_speed_limit=20.0`** — 全局最高车速上限（兜底）。
3. **`default_cruise_speed=11.18`** — 正常行驶段目标速度（40 km/h）。
4. **`destination_check_distance=4.0`** — 目的地任务完成判定阈值（配合 Destination 规则停车距离）。
5. **`enable_pull_over_at_destination=false`** — 终点是否触发靠边停车场景。
6. **`prioritize_change_lane` + `min_length_for_lane_change=5.0`** — 变道/绕行优先级与最小变道长度（决定遇障是绕行还是停车）。
7. **`ignore_overlapped_obstacle=true`** — 障碍物与自车重叠时是否忽略（避免异常急停）。
8. **`traffic_rule_config.pb.txt`** — 人行道/信号灯/停止标志/目的地规则启用总闸（注意未启用 SpeedSetting 规则，无人人行道≤5m/s、绿灯路口≤5m/s 可能需要它）。
9. **`nonstatic_obstacle_nudge_l_buffer=0.4`** — 绕行动态障碍物的横向间距。
10. **`use_st_drivable_boundary=false`** — 速度决策算法分支（ST 边界法）。

### ⚠️ 调参提示
- 人行道停车 1.5-2.0m、红绿灯停车 1.5-2.0m、停止标志停车 1.5-2.0m、障碍物停车≥2m 的**具体停车距离**不在本组全局文件里，而在各 TrafficRule 的独立配置（`crosswalk_config.pb.txt`、`traffic_light_config.pb.txt`、`stop_sign_config.pb.txt`、`destination_config.pb.txt`、speed_decider 的 `max_stop_distance_obstacle/min_stop_distance_obstacle`）中，需另行分析。
- `min_length_for_lane_change` / `ignore_overlapped_obstacle` / `use_navigation_mode` / `enable_nudge_decision` 定义于预编译包，工作区无源码，改前需确认 flag 名拼写合法（gflags 对未知 flag 会报错）。
