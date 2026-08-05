# 03 planning tasks 核心 16 个（回归代码版）

> 配置：`realcar_tuning/ORIGINAL/modules/planning/tasks/<task>/conf/default_conf.pb.txt`
> 源码：`modules/planning/tasks/<task>/{proto,*.cc}`
> 标注：⭐=比赛核心 ○=一般 △=无关/当前未生效
> ⚠️ 本地 `modules/planning` 已恢复官方基线（2b48d5e）；正文"本地魔改"备注指恢复前版本，仅为历史对照

---

## 1. speed_decider（速度决策）⭐

| 参数 | 实车值 | 含义（回归代码） | 影响/备注 |
|------|--------|------------------|-----------|
| follow_min_obs_lateral_distance | 2.5 | `CheckIsFollow`：障碍 SL 边界 `min(|start_l|,|end_l|) > 2.5` 则不 FOLLOW | 调大→更少跟车；调小→更容易跟车 ⭐ |
| follow_min_time_sec | 2.0 | `CheckIsFollow`：ST 边界时间跨度 `< 2.0s` 不 FOLLOW（区分横穿后驶离障碍） | 调大→只有长时同向才跟；调小→更容易跟 ⭐ |
| is_stop_for_pedestrain | false | `MakeObjectDecision`：true 时官方行人 STOP 分支 | **false=不走官方强制 STOP**（实车自行管理）⭐ |
| keep_clear_last_point_speed | 0.8 | `CheckKeepClearCrossable`：keep-clear 内末点 v<0.8 判阻塞 | 调大→更易判阻塞 ○ |
| yield_distance_buffer | 2.0 | `CreateYieldDecision` 让行 s 缓冲 | 调大→让行更保守 ⭐ |
| stop_follow_distance | 1.5 | `EstimateProperFollowGap` 基础跟车距离 | 调大→跟车远；调小→更近 ⭐ |
| follow_distance_scheduler | 0/2.7/5.4/8.1；1.25/1.0/0.75/0.5 | 按速度插值跟车距离：`follow_distance = 1.5 + Σslope·Δv` | 速度越高跟越远 ⭐ |

## 2. path_decider（路径决策）⭐

| 参数 | 实车值 | 含义 | 影响/备注 |
|------|--------|------|-----------|
| static_obstacle_buffer | 0.3 | 静态障碍缓冲：NUDGE 距离 `±buffer` | 调大→绕行/停车更保守 ⭐ |
| skip_overlap_stop_check | true | 障碍与自车横向重叠时跳过 STOP 只 NUDGE | **true=重叠不再停车只绕行** ⭐ |

## 3. rule_based_stop_decider（规则停车决策）⭐

| 参数 | 实车值 | 含义 | 影响/备注 |
|------|--------|------|-----------|
| max_adc_stop_speed | 0.5 | 速度 >0.5 判"未停" | ○ |
| max_valid_stop_distance | 1.0 | 车头距停止点 >1.0 判无效停 | ○ |
| search_beam_length | 20.0 | 感知阻塞检测光束长度 | △ 仅 side_pass 生效 |
| search_beam_radius_intensity | 0.08 | 光束半径 | △ |
| search_range | 3.14 | 光束搜索范围 | △ |
| is_block_angle_threshold | 0.5 | 阻塞角度阈值 | △ |
| enable_lane_change_urgency_checking | false | 变道紧迫性检查 | △ |
| short_path_length_threshold | 5.0 | 路径长度 <5.0 在末端加 STOP | 调大→更早末端停车 ⭐ |

## 4. piecewise_jerk_speed（QP 速度优化）⭐

| 参数 | 实车值 | 含义 | 影响/备注 |
|------|--------|------|-----------|
| acc_weight | 1.0 | QP 加速度项权重 set_weight_ddx | 调大→更平稳但响应慢 ⭐ |
| jerk_weight | 3.0 | QP jerk 项权重 set_weight_dddx | 调大→起步/停车更柔 ⭐ |
| kappa_penalty_weight | 200.0 | 曲率惩罚：`|κ|·200` 弯道限速 | 调大→弯道降速明显 ○ |
| ref_s_weight | 1.0 | 参考位置权重 | △ |
| ref_v_weight | 10.0 | 参考速度权重 dx_ref_weight | 调大→更努力维持巡航 ⭐ |
| follow_distance_buffer | 0.1 | FOLLOW 边界收紧 | ⚠️ 本地魔改当前未生效 ⭐ |

## 5. piecewise_jerk_speed_nonlinear（NLP 速度优化）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| acc_weight | 5.0 | NLP 加速度权重 | ○ 仅 NLP 路径 |
| jerk_weight | 5.0 | NLP jerk 权重 | ○ |
| lat_acc_weight | 1000.0 | 向心加速度权重（弯道减速） | ○ |
| s_potential_weight | 0.05 | 最短通行时间权重 | 调大→更快通过 ○ |
| ref_v_weight | 1.0 | 参考速度权重 | ○ |
| ref_s_weight | 50.0 | 参考位置权重 | ○ |
| soft_s_bound_weight | 1e3 | 软安全边界惩罚 | ○ |
| use_warm_start | true | QP 结果热启动 | ○ |
| use_smoothed_dp_guide_line | true | 平滑 DP 引导线 | ○ |
| use_soft_bound_in_nonlinear_speed_opt | false | 只用硬边界 | ○ |

## 6. speed_bounds_decider（ST 边界映射+限速）⭐

| 参数 | 实车值 | 含义 | 影响/备注 |
|------|--------|------|-----------|
| total_time | 7.0 | ST 图时间轴总长 | 调大→看得更远 ⭐ |
| boundary_buffer | 0.25 | STOP 边界 s 缓冲 | 调大→更保守 ○ |
| max_centric_acceleration_limit | 1.0 | 向心加速度限速：`v=sqrt(a/κ)` | 调大→弯道更快 ⭐ |
| point_extension | 0.0 | 轨迹点延伸 | △ |
| lowest_speed | 0.1 | 限速下限（不低于 0.1） | 调大→限速区最低速度抬高 ⭐ |
| static_obs_nudge_speed_ratio | 0.6 | 静态障碍绕行减速比 | 调大→绕行更快 ⭐ |
| dynamic_obs_nudge_speed_ratio | 0.8 | 动态障碍绕行减速比 | ○ |
| enable_nudge_slowdown | true | 绕行减速总开关 | ⭐ |
| lane_change_obstacle_nudge_l_buffer | 0.3 | 变道障碍横向缓冲 | ○ |
| max_trajectory_len | 1000.0 | ST 映射最大搜索长度 | △ |

## 7. path_time_heuristic（DP 速度搜索）⭐

**default_speed_config 段：**

| 参数 | 实车值 | 含义 | 影响/备注 |
|------|--------|------|-----------|
| unit_t | 1.0 | 时间采样步长 | △ |
| dense_dimension_s | 101 | 稠密 s 采样点数 | 调大→近区更精细 △ |
| dense_unit_s | 0.1 | 稠密 s 分辨率 | △ |
| sparse_unit_s | 1.0 | 稀疏 s 分辨率 | △ |
| speed_weight | 0.0 | 速度项权重 | 实车 0=不生效 △ |
| accel_weight | 10.0 | DP 加速度权重 | 调大→DP 更倾向匀速 ⭐ |
| jerk_weight | 10.0 | DP jerk 权重 | 调大→DP 曲线更平滑 ⭐ |
| obstacle_weight | 1.0 | 障碍成本权重 | 调大→更早减速 ⭐ |
| reference_weight | 0.0 | 参考线位置权重 | 实车 0=不用 △ |
| go_down_buffer / go_up_buffer | 5.0/5.0 | DP s 边界外扩缓冲 | ○ |
| default_obstacle_cost | 1e4 | 障碍基础成本 | 调大→更早/更保守减速 ⭐ |
| default_speed_cost | 1.0e3 | 速度成本系数 | ○ |
| exceed_speed_penalty | 1.0e3 | 超速惩罚 | 调大→更严压制超速 ⭐ |
| low_speed_penalty | 10.0 | 低速惩罚 | 调大→防低速蠕行 ⭐ |
| reference_speed_penalty | 10.0 | 巡航速度偏差惩罚 | 调大→更贴巡航 ⭐ |
| keep_clear_low_speed_penalty | 10.0 | keep-clear 低速惩罚 | ○ |
| accel_penalty | 1.0 | 加速惩罚系数 | 调大→抑制加速 ⭐ |
| decel_penalty | 1.0 | 减速惩罚系数 | 调大→抑制急减速 ⭐ |
| positive_jerk_coeff | 1.0 | jerk>0 系数 | ○ |
| negative_jerk_coeff | 1.0 | jerk<0 系数 | ⚠️ 实车 1.0 远小于官方默认 300（负 jerk 惩罚放宽）○ |
| max_acceleration | 2.0 | DP 最大加速度 | 调大→起步更猛 ⭐ |
| max_deceleration | -2.0 | DP 最大减速度 | 调大(绝对值)→急刹车 ⭐ |
| spatial_potential_penalty | 1.0e2 | 空间势惩罚（引导 s 向前） | 调大→防原地不动 ⭐ |
| enable_multi_thread_in_dp_st_graph | false | DP 多线程 | △ |
| enable_dp_reference_speed | true | DP 贴巡航速度惩罚 | 调 false→更自由 ⭐ |

**lane_change_speed_config 段**（变道差异）：

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| dense_dimension_s / dense_unit_s | 21 / 0.25 | 变道 DP 稠密区更粗 | ○ |
| max_deceleration | -2.5 | 变道允许更急减速 | ○ |
| spatial_potential_penalty | 1.0e5 | 变道强向前牵引 | ○ |
| is_lane_changing | true | 变道 DP 开关 | ○ |

## 8. lane_follow_path（常规路径优化）⭐

| 参数 | 实车值 | 含义 | 影响/备注 |
|------|--------|------|-----------|
| is_extend_lane_bounds_to_include_adc | true | 用 ADC 边界扩展路径边界 | 调 false→严格按车道线 ⭐ |
| extend_buffer | 0.2 | 扩展缓冲 | 调大→边界更宽 ○ |
| path_optimizer_config.l_weight | 600.0 | 横向偏移 l 惩罚 | 调大→更贴车道中心 ⭐ |
| path_optimizer_config.dl_weight | 20.0 | 横向速度惩罚 | 调大→横向更稳 ○ |
| path_optimizer_config.ddl_weight | 600.0 | 横向加速度惩罚 | 调大→变道更平滑 ⭐ |
| path_optimizer_config.dddl_weight | 2000.0 | 横向 jerk 惩罚 | 调大→横向更柔 ○ |
| path_optimizer_config.lateral_derivative_bound_default | 2.0 | 横向导数边界 | △ |
| path_optimizer_config.path_reference_l_weight | 200 | 路径参考横向权重 | 调大→更贴历史路径 ⭐ |

## 9. lane_change_path（变道路径）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| extend_adc_buffer | 0.5 | 变道 ADC 扩展缓冲 | ○ |
| change_lane_success_freeze_time | 3 | 变道成功冷却 3s | 调大→冷却更长 ○ |
| change_lane_fail_freeze_time | 1.0 | 变道失败重试冷却 | ○ |
| lane_change_prepare_length | 2.0 | 变道准备段长度 | 调大→提前更远准备 ○ |
| path_optimizer_config | l=100/dl=20/ddl=1000/dddl=5000/ref_l=100 | 变道优化权重 | ○ |

## 10. lane_change_path_generic（通用变道）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| extend_adc_buffer / freeze_time / prepare_length | 0.5 / 3 / 1.0 / 2.0 | 同 lane_change_path | ○ |
| path_optimizer_config | l=10/dl=80/ddl=1000/dddl=10000/ref_l=1000 | 通用变道更贴参考线、横向更柔 | ○ |

## 11. lane_borrow_path（借道绕行）⭐

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| is_allow_lane_borrowing | true | 借道总开关 | ⭐ |
| lane_borrow_max_speed | 5.0 | 速度 <5.0 才允许借道 | 调大→高速也借 ⭐ |
| long_term_blocking_obstacle_cycle_threshold | 3 | 静态障碍持续 ≥3 帧判长期阻塞 | 调大→更稳更迟 ○ |
| path_optimizer_config | l=100/.../ref_l=20000 | 借道 ref_l 极大→防大角度偏出 | ⭐ |

## 12. lane_borrow_path_generic（通用借道）⭐

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| is_allow_lane_borrowing | true | 借道总开关 | ⭐ |
| lane_borrow_max_speed | 10.0 | **比 lane_borrow_path 宽松**（高速也借道） | ⭐ 触发差异 |
| long_term_blocking_obstacle_cycle_threshold | 3 | 同上 | ○ |
| enable_extend_boundary_by_adc | false | 借道边界用 ADC 扩展 | ○ |
| enable_ignore_boundary_type | false | false=检查实线禁止借道 | 调 true→无视实线 ⭐ |
| path_optimizer_config | l=100/dl=20/ddl=300/dddl=1000/ref_l=20 | 通用借道更柔和 | ○ |
| enable_nudge_destination_threshold | 8.0 | 阻塞障碍离终点 <8m 不借道 | 调大→距终点更远不借 ⭐ |

## 13. obstacle_nudge_decider（绕行决策）⭐

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| enbale_nugde_static_obs | true | 静态障碍绕行总开关 | ⭐ |
| enbale_nugde_dynamic_obs | false | 动态障碍绕行开关 | △ 动态分支未实现 |
| min_forward_check_dis | 30.0 | 前向检查距离下限 | 调大→低速也检查更远 ⭐ |
| max_forward_check_dis | 60.0 | 前向检查距离上限 | 调大→高速检查更远 ⭐ |
| max_lateral_left/right_nudge_dis | 3.0/3.0 | 左右最大绕行横向距离 | △ 未直接引用 |
| expand_polygon_buffer | 0.2 | 障碍多边形外扩 | ⭐ 绕行安全余量 |
| obs_passable_buffer | 0.7 | 横向余量 >0.7 才可绕行通过 | 调大→需要更宽才绕 ⭐ |
| obs_nudge_time_out_threshold | 3.0 | 绕行超时 | △ |
| obs_clean_time_out_threshold | 2.0 | 跟踪障碍超时清除 | ○ |
| full_nudge_buffer | 1.5 | 全量绕行缓冲 | △ |
| nudge_key_point_extend_length_coefficient | 0.4 | 绕行关键点前后延伸（车长×0.4） | ○ |

## 14. path_reference_decider（路径参考）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| min_path_reference_length | 20 | 最小路径参考长度 | ○ |
| skip_path_reference_in_side_pass | false | 侧方通过跳过参考 | ○ |
| skip_path_reference_in_change_lane | true | 变道跳过参考 | ○ |

## 15. fallback_path（兜底路径）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| extend_buffer | 0.5 | 兜底路径边界扩展缓冲（`FallbackPathConfig` 实际字段） | ○ 主路径失败时用；proto 仅 is_extend_lane_bounds_to_include_adc/extend_buffer/path_optimizer_config 三组字段 |

## 16. reuse_path（路径复用）○

| 参数 | 实车值 | 含义 | 备注 |
|------|--------|------|------|
| enable_reuse_path_in_lane_follow | true | 车道跟随中启用上周期路径复用（`ReusePathConfig` 实际字段） | ○ 上周期路径复用；short_path_threshold 官方默认 60 未显式配置 |
