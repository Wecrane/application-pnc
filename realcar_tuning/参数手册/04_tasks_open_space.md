# 04 planning tasks open_space 系列 16 个（回归代码版）

> 配置：`realcar_tuning/ORIGINAL/modules/planning/tasks/<task>/conf/default_conf.pb.txt`
> 源码：`modules/planning/tasks/<task>/` + `planning_open_space/`
> **总体结论**：16 个文件中除 pull_over_path / reverse_* / square_path / rss_decider 外，其余均为 **open_space 泊车/掉头专用**，与实车 9 赛题（车道内行驶）无关（△），**保持原值即可**。
> ⚠️ 本地 `modules/planning` 已恢复官方基线（2b48d5e）；`open_space_fallback_decider_park.cc` 仍含基线自带 mayaochang 一处

---

## 1. open_space_fallback_decider（open_space 兜底停止）△

| 参数 | 实车值 | 含义（回归代码） | 备注 |
|------|--------|------------------|------|
| open_space_prediction_time_period | 5.0 | 动态障碍预测时间视界（s） | 调大→碰撞检查更保守 △ |
| open_space_fallback_collision_distance | 5.0 | fallback 碰撞检查距离 | △ |
| open_space_fallback_stop_distance | 2.0 | 停止轨迹安全间隙 | △ |
| open_space_fallback_collision_time_buffer | 5.0 | 碰撞时间缓冲 | 调大→更早判碰撞 △ |

## 2. open_space_fallback_decider_park（泊车兜底）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| open_space_prediction_time_period | 5.0 | 同上 | △ |
| open_space_fallback_collision_distance | 1.0 | 碰撞距离（算停止减速度） | 调小→制动更急（封顶 4 m/s²）△ |
| open_space_fallback_stop_distance | 2.0 | 停止间隙 | △ |
| open_space_fallback_collision_time_buffer | 5.0 | 碰撞时间缓冲 | △ |

## 3. open_space_path_planning（Hybrid A* 路径搜索）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| enable_path_planning_thread | true | 异步线程跑路径搜索 | △ |
| warm_start_config.xy_grid_resolution | 0.3 | 栅格 XY 分辨率 | 调小→更精细更慢 △ |
| warm_start_config.phi_grid_resolution | 0.1 | 航向角网格分辨率 | △ |
| warm_start_config.next_node_num | 10 | 每步扩展节点数 | △ |
| warm_start_config.step_size | 0.25 | 节点间轨迹点步长 | △ |
| warm_start_config.traj_forward_penalty | 1.0 | 前进代价惩罚 | △ |
| warm_start_config.traj_back_penalty | 1.0 | 后退代价惩罚 | △ |
| warm_start_config.traj_gear_switch_penalty | 10.0 | 换挡惩罚 | △ |
| warm_start_config.traj_steer_penalty | 0.0 | 转向惩罚（实车 0） | △ |
| warm_start_config.traj_steer_change_penalty | 0.0 | 转向变化惩罚（实车 0） | △ |
| warm_start_config.grid_a_star_xy_resolution | 0.25 | A* 栅格分辨率 | △ |
| warm_start_config.node_radius | 0.5 | 碰撞检查节点半径 | △ |
| warm_start_config.traj_kappa_contraint_ratio | 0.7 | warm start 最大曲率比例 | △ |

## 4. open_space_pre_stop_decider（预停车决策）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| stop_buffer_to_target | 3.0 | 停车线相对目标点缓冲 | 调大→提前 3m 停车 △ |
| stop_distance_to_target | (默认5.0) | 停车线距目标阈值 | △ |
| rightaway_stop_distance | (默认2.0) | 立即停车距离 | △ |

## 5. open_space_replan_decider（重规划触发）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| （无参数） | — | 上帧无优化轨迹且车速≈0 → 触发重规划 | 纯逻辑无参数 △ |

## 6. open_space_roi_decider（ROI 区域构建）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| roi_longitudinal_range_start | 15 | ROI 纵向向后范围 | △ |
| roi_longitudinal_range_end | 15 | ROI 纵向向前范围 | △ |
| parking_start_range | 20.0 | 车位检测起始范围 | 实车偏大→更早探测 △ |
| parking_inwards | false | 车头朝向（false=倒车入库） | △ |
| enable_perception_obstacles | true | 感知障碍纳入约束 | △ |
| parking_depth_buffer | 0.2 | 车位末端缓冲 | △ |
| roi_line_segment_min_angle | 0.15 | ROI 线段最小航向角变化 | △ |
| roi_line_segment_length | 1.0 | ROI 采样步长 | △ |
| perception_obstacle_filtering_distance | 1000.0 | 障碍过滤距离（≈不滤除） | △ |
| perception_obstacle_buffer | 0.0 | 感知障碍膨胀缓冲（实车 0） | △ |
| curb_heading_tangent_change_upper_limit | 0.4 | 路缘切线变化上限 | △ |
| use_road_boundary_from_map | false | 从 HD map 取道路边界 | △ |

## 7. open_space_roi_decider_park（泊车 ROI）△

> 与 roi_decider 同构，仅 `curb_heading_tangent_change_upper_limit=0.01`（对路缘转角极敏感）不同。其余参数同上。

## 8. open_space_trajectory_optimizer_park（轨迹数值优化器）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| enable_trajectory_optimize_thread | true | 线程做轨迹优化 | △ |
| dual_variable_warm_start_config.* | weight_d1.0 / OSQP / min_safety_distance0.01 / ipopt+osqp 参数 | 对偶变量热启动 | △ |
| distance_approach_config.* | weight_steer0.3/weight_a1.1/weight_steer_rate3.0/weight_a_rate2.5/weight_x2.3/weight_y0.7/weight_phi1.5/weight_v0.0/max_speed_forward2.0/max_speed_reverse1.0/max_acc_forward2.0/max_acc_reverse1.0/IPOPT | 距离逼近法权重 | △ |
| iterative_anchoring_smoother_config.* | delta_s0.1/reanchor50/default_bound2.0/vehicle_shortest1.04/FEM 1e7/ref 1e3/curvature0.18 | 迭代锚点平滑器 | △ |
| delta_t | 0.5 | 开空间速度采样时间 | △ |
| near_destination_threshold | 0.05 | 接近终点阈值 | △ |
| enable_linear_interpolation | false | 线性插值输出 | △ |

## 9. open_space_trajectory_partition（轨迹分段器）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| gear_shift_max_t | 3.0 | 换挡轨迹最大时长 | △ |
| gear_shift_unit_t | 0.02 | 换挡轨迹时间步长 | △ |
| gear_shift_period_duration | 2.0 | 换挡周期时长 | △ |
| interpolated_pieces_num | 10 | 插值段数 | △ |
| initial_gear_check_horizon | 15 | 初始挡位检查视界 | △ |
| heading_search_range | 0.79 | 航向搜索范围 | △ |
| heading_track_range | 1.57 | 航向跟踪范围 | △ |
| distance_search_range | 2.0 | 距离搜索范围 | △ |
| heading_offset_to_midpoint | 0.79 | 到达判定航向偏差阈值 | △ |
| lateral_offset_to_midpoint | 0.5 | 到达判定横向偏移阈值 | △ |
| longitudinal_offset_to_midpoint | 0.40 | 到达判定纵向偏移阈值 | △ |
| vehicle_box_iou_threshold_to_midpoint | 0.40 | 车辆包围盒 IOU 阈值 | △ |
| speed_replan_distance | 2.0 | 速度重规划距离 | △ |
| linear_velocity_threshold_on_ego | 0.1 | 判定静止线速度阈值 | △ |
| use_gear_shift_trajectory | true | 插入换挡轨迹 | △ |

## 10. open_space_trajectory_post_process / open_space_trajectory_provider（后处理/提供器）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| （post_process） | — | 轨迹后处理（平滑/去抖） | △ |
| （provider） | — | 提供开空间轨迹给下游 | △ |

## 11. open_space_roi_decider_park（同 7）△

## 12. pull_over_path（靠边路径）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| pull_over_destination_to_adc_buffer | 25.0 | 靠边点距自车缓冲（实车 25.0=官方默认） | ○ 靠边停车用 |
| pull_over_destination_to_pathend_buffer | 4.0 | 终点到路径末端缓冲（官方默认 10.0，实车改 4.0） | ○ |
| pull_over_road_edge_buffer | 0.15 | 靠边路缘缓冲 | ○ |
| pull_over_approach_lon_distance_adjust_factor | 1.6 | 靠边接近纵向距离调整系数（官方默认 1.5，实车改 1.6） | ○ |
| pull_over_weight | 10 | 靠边路径权重 | ○ |

## 13. reverse_path / reverse_speed（倒车路径/速度）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| （reverse_path） | — | 倒车路径生成 | △ 掉头/泊车用 |
| （reverse_speed） | — | 倒车速度规划 | △ |

## 14. rss_decider（RSS 安全模型）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| （rss 参数） | — | RSS 责任敏感安全模型 | △ 多数场景不启用 |

## 15. square_path（场地赛路径）△

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| （square 参数） | — | Square 场地赛路径 | △ SQUARE 场景已注释禁用 |
