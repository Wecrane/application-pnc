# 06 control 模块全参数（回归代码版）⭐

> 配置：`realcar_tuning/ORIGINAL/modules/control/`
> 源码：`modules/control/control_component/` + `controllers/` + `control_task_base_extend/`
> **🚨 首要结论（回归源码确认）**：实车 pipeline 用 **Plus 控制器链**（`LatPlusController → LonPlusController → AntiSlipControlTask → DebugInfoControlTask`），**不是**普通 Lat/LonController。
> - `lat_based_lqr_controller/` 和 `lon_based_pid_controller/` 的配置**不生效**（仅对照）
> - **实际生效 = 场景参数链选中的 `public_road_forward_param.pb.txt`**（赛题全是公共道路前向，`LonPlusController::ChooseCurrentParam` 源码证实），**完全覆盖 controller_conf 默认值**
> - 机制：`ControlTaskExtend::LoadParamsPipelineConfig` 加载 `params_pipeline.pb.txt` → 每周期按场景选参数

---

## 一、control.conf（全局 gflags）

> 对照 `modules/control/control_component/common/control_gflags.cc` 默认值

| 参数 | 实车值 | 默认值 | 源码含义 | 影响/备注 |
|------|--------|--------|----------|-----------|
| `--pipeline_file` | `/apollo/modules/control/control_component/conf/pipeline.pb.txt` | — | 控制器流水线文件，`ControlComponent::Init` 加载创建控制器 | 定了 Plus 链 △ |
| `--control_conf_file` / `--mpc_controller_conf_file` / `--lateral_controller_conf_file` / `--longitudinal_controller_conf_file` | /apollo/modules/control/conf/* | — | MPC/旧配置入口（本 Plus 链不用，指向不存在路径） | use_control_submodules=false 无影响 △ |
| `---calibration_table_file` | `/apollo/modules/control/control_component/conf/calibration_table.pb.txt` | — | 标定表文件（三个横线为实车原样，gflags 兼容解析） | 油门/刹车查找表 △ |
| `--control_test_duration` | -1 | -1 | 测试模式时长，-1=不限 | ○ |
| `--enable_csv_debug` | false | false | 输出 csv 调试文件 | 开 true 可离线分析速度曲线 ○ |
| `--preprocessor_submodule_name` 等 4 个 | PreprocessorSubmodule / MPCControllerSubmodule / PostprocessorSubmodule / LatLonControllerSubmodule | — | 子模块模式节点名 | use_control_submodules=false 不用 △ |
| `--is_control_test_mode` | false | false | 测试模式开关 | △ |
| `--use_preview_speed_for_table` | false | false | 标定表查询用 preview 还是当前速度 | false=用当前速度更稳 ○ |
| `--steer_angle_rate` | 100 | 100 | 转向角变化率 | △ |
| `--enable_gain_scheduler` | **true** | false | 按速度对 matrix_q 乘插值增益（`lat_controller.cc`） | **实车已开**：高速自动降横向增益 ⭐ |
| `--set_steer_limit` | **true** | false | 按 max_lateral_acceleration 限制转向角上限 | **实车已开**：低速大转角受限 ⭐ |
| `--use_vehicle_epb` | **true** | false | 启用 EPB 电子手刹停车逻辑 | **实车已开**：停车拉手刹防溜 ⭐ |
| `--chassis_pending_queue_size` 等 4 个 | 10 | 10 | 各通道 pending 队列 | △ |
| `--reverse_heading_control` | false | false | 倒车航向控制 | ○ |
| `--reverse_heading_vehicle_state` | false | — | 倒车航向反转 | ○ |
| `--state_transform_to_com_drive` | **true** | — | 前进轨迹/状态变换到质心 | 实车开 △ |
| `--state_transform_to_com_reverse` | true | — | 倒车变换到质心 | △ |
| `--query_forward_time_point_only` | false | false | 只查询未来轨迹点 | ○ |
| `--query_forward_station_point_only` | false | false | 只查询未来 station 点 | ○ |
| `--enable_gear_drive_negative_speed_protection` | false | false | D 挡负速度保护 | ○ |
| `--use_control_submodules` | false | false | 子模块模式 | 定了 Plus 链单进程 △ |
| `--enable_input_timestamp_check` | **true** | false | 输入时间戳超时检查 | **实车已开**：超时判定 estop ○ |
| `--max_localization_miss_num` | 20 | 20 | 定位丢失容差 | ○ |
| `--max_chassis_miss_num` | 20 | 20 | 底盘丢失容差 | ○ |
| `--max_planning_miss_num` | **5** | 20 | 规划丢失容差（0.5s 即 estop） | **实车收紧** ○ |
| `--max_acceleration_when_stopped` | 0.01 | — | "已停止"判定加速度阈值 | ⭐ 停车判定相关 |
| `--enable_persistent_estop` | true | true | 持续 estop | △ |
| `--control_period` | 0.01 | 0.01 | 控制周期 100Hz | △ |
| `--soft_estop_brake` | **15.0** | 50 | estop 刹车指令 | **实车改小**（50→15 防急刹）⭐ |
| `--trajectory_period` | 0.1 | 0.1 | 规划周期 | △ |
| `--chassis_period` / `--localization_period` | 0.01 | 0.01 | 底盘/定位周期 | △ |
| `--minimum_speed_resolution` | 0.2 | 0.2 | 速度分辨率 | △ |
| `--minimum_speed_protection` | 0.1 | 0.1 | 最小速度保护（防除零） | △ |
| `--action` | 1 | 1 | 默认驾驶动作 | △ |
| `--max_path_remain_when_stopped` | **0.1** | 0.3 | 停止时最大路径余量 | **实车改小**：停更接近目标点 ⭐ |
| `--pitch_offset_deg` | **0.9** | 0 | 车辆俯仰角偏移补偿 | **实车标定值**（坡度补偿都用）△ |
| `--publish_control_debug_info` | **true** | false | 发布 ControlDebugInfo | 实车开 ○ |
| `--use_throttle_filter` | **true** | false | 油门一阶平滑滤波 | **实车开**：防起步冲 ⭐ |
| `--throttle_smoothing_factor` | **0.08** | 0.05 | 油门平滑系数 | ⭐ 起步/停车平顺 |
| `--use_speed_filter` | **true** | false | 车速一阶平滑滤波 | **实车开** ⭐ |
| `--speed_smoothing_factor` | **0.1** | 0.05 | 速度平滑系数 | ⭐ |
| `--enable_park_gear_when_stopped` | true（内置） | true | 完全停止切 P 挡 | 实车二进制内置 △ |
| `--park_gear_max_speed` / `--park_gear_debounce_frames` | 0.05 / 30 | — | P 挡切换阈值/防抖 | △ |
| `--enable_brake_slew_limit` / `--brake_slew_rate_up` | true / 22.0 | — | 刹车上升速率限制 | ⭐ 停车冲击大→调小 |
| `--enable_throttle_slew_limit` / `--throttle_slew_rate_up` | true / 35.0 | — | 油门上升速率限制 | ⭐ 起步窜→调小 |
| `--enable_throttle_suppress_on_parking` | true（内置） | — | 手刹未释放油门强制 0 | △ |

---

## 二、pipeline.pb.txt（控制器链）⭐

| 控制器 | type | 含义 | 备注 |
|--------|------|------|------|
| `LAT_PLUS_CONTROLLER` | LatPlusController | 横向 LQR Plus：输出 steering_target | ⭐ 横向调参 |
| `LON_PLUS_CONTROLLER` | LonPlusController | 纵向 PID Plus：输出 throttle/brake/gear/parking_brake/speed | ⭐ 纵向调参（赛题主战场） |
| `ANTISLIP_TASK` | AntiSlipControlTask | 防溜坡：按坡度/障碍改写 throttle/brake/EPB | ⭐ 坡道停车防溜 |
| `DEBUG_INFO_TASK` | DebugInfoControlTask | 发布 ControlDebugInfo | 仅调试 ○ |

> ⚠️ 顺序即执行顺序（`ControlTaskAgent::ComputeControlCommand` 循环），**改配置只影响对应控制器，不要改顺序**。

---

## 三、calibration_table.pb.txt（油门/刹车标定查找表）△

| 要点 | 说明 |
|------|------|
| 结构 | 数百行 `calibration { speed; acceleration; command }` 三元组（speed≈0~3 m/s、acceleration 从刹车-0.94 到油门 2.15、command 从 -36 到 +36%） |
| 作用 | 把"期望加速度"映射为踏板指令 %（负=刹车，正=油门），纵向控制地基 |
| 实车 | HQEV_BDN 专用标定（与车型强绑定） |
| 标注 | **△ 绝对不动**（动了油门刹车映射全乱） |

---

## 四、横向 LQR（LatPlusController）⭐

> 源码：lat_controller.cc（与普通 LQR 一致）。LQR 状态 = [横向误差, 横向误差率, 航向误差, 航向误差率]。

### 4a. controller_conf.pb.txt（默认值，open_space 兜底）

| 参数 | 实车值 | 源码含义 | 影响/备注 |
|------|--------|----------|-----------|
| ts | 0.01 | 采样周期 | △ |
| preview_window | 0 | 预瞄窗口 | 0=无预瞄 ○ |
| cf / cr | 275159.24 | 前后轮侧偏刚度 | 车型标定 △不动 |
| mass_fl/fr/rl/rr | 563×4 | 四轮质量 | 车型标定 △ |
| eps | 0.01 | LQR Riccati 收敛阈值 | ○ |
| matrix_q | 5.5/1.0/1.5/0.0 | Q 矩阵=[横向误差,横向误差率,航向误差,航向误差率]权重 | ⭐ 越大控制越硬 |
| cutoff_freq | 5 | 横向误差低通滤波截止频率 | ○ |
| mean_filter_window_size | 10 | 误差均值滤波窗口 | ○ |
| max_iteration | 300 | LQR 最大迭代 | ○ |
| max_lateral_acceleration | 4.0 | 最大横向加速度（转向限幅） | ⭐ 低速转向限幅 |
| enable_reverse_leadlag_compensation | false | 倒车超前滞后 | ○ |
| enable_steer_mrac_control | false | 转向 MRAC | ○ |
| enable_look_ahead_back_control | false | 前视后视控制 | ○ |
| lookahead_station | 0.5 | 前视距离 | ⭐ 越小越贴线 |
| lookback_station | 1.05 | 后视距离 | ○ |
| switch_speed | 2.0 | 高低速切换速度 | ○ |
| query_relative_time | 0.0 | 查询相对时间 | ○ |
| straight_steering_percentage / straight_min_velocity / straight_max_ref_cur | 5/0.5/0.01 | 直行小转向抑制 | ○ 防直行抖动 |
| enable_enhance_qparam | false | 增强 Q 参数 | ○ |
| speed_segment: 1.0/2.0/3.0/5.0 | — | 速度分段点 | ○ |
| changing_rate: 0.01/0.005/0.003/0.001 | — | 增益变化率 | ○ |
| lat_speed_input: 2.0~4.5 | — | 横向误差增益调度输入速度点 | ⭐ |
| lat_err_speed_gain_output: 1.0/0.8/0.6/0.3/0.1/0.08 | — | 横向误差增益随速度输出 | ⭐ 高速自动降增益 |
| heading_err_speed_gain_output: 1.0/0.8/0.6/0.2/0.1/0.08 | — | 航向误差增益随速度 | ⭐ |
| lat_safety_check_conf { max_lateral_error_e:0.1, w:0.06 } | — | 横向安全检查阈值 | ○ |

### 4b. ⭐ public_road_forward_param.pb.txt（公共道路前向，**赛题实际生效**）

> 经 params_pipeline 声明 + ChooseCurrentParam 选中，**覆盖默认**。

| 参数 | 实车值 | vs 默认 | 含义 | 标注 |
|------|--------|---------|------|------|
| cf / cr | **126050** | (275159→126050) | 侧偏刚度（公共道路标定） | △ |
| mass_fl/fr/rl/rr | **103/102/133/131** | (563→) | 四轮质量（总~469kg） | △ |
| matrix_q | **2.5/1.0/1.5/0.0** | (5.5→2.5) | **实际生效 Q 矩阵**：横向误差 2.5、航向误差 1.5 | ⭐ 想更贴线↑、更柔↓ |
| max_iteration | 1000 | (300→1000) | LQR 迭代上限 | ○ |
| lookback_station | 0.85 | (1.05→0.85) | 后视距离收短 | ○ |
| handle_lat_err_window / handle_heading_err_window | 120/120 | (75→120) | 误差处理窗口拉长 | ○ |
| handle_lat_err_prarm_domain | 5 | (30→5) | 横向误差参数域收窄 | ○ |
| com_change_rate | 0.05 | (0.3→0.05) | 质心变化率 | ○ |
| lat_speed_input / lat_err_speed_gain_output | 2.0~4.5 / 1.0/0.8/0.6/0.3/0.1/0.08 | 相同 | 速度增益调度 | ⭐ |
| 其余（cutoff_freq=5、max_lateral_acceleration=4.0、lookahead=0.5、switch_speed=2.0 等） | 同 4a | 同 | — | 同上 |

---

## 五、纵向 PID（LonPlusController）⭐⭐ 赛题主战场

> **两级 PID**：`station_pid`（位置误差→速度修正）+ `speed_pid`（速度误差→加速度）→ 查标定表→踏板。
> **kp/ki 最终值 = 基础值 × 按速度插值增益**（`CalcultePidParams` + `InterpolationPlus1D`）。

### 5a. ⭐ public_road_forward_param.pb.txt（赛题实际生效）

| 参数 | 实车值 | 源码含义 | 影响/备注 |
|------|--------|----------|-----------|
| ts | 0.01 | 采样周期 | △ |
| brake_minimum_action | **2.0** | 最小刹车指令 | ⭐ 保证基础刹车 |
| throttle_minimum_action | 0.0 | 最小油门指令 | ○ |
| speed_controller_input_limit | 2.5 | speed_pid 输入限幅 | ⭐ 防大误差冲击 |
| station_error_limit | 2.5 | station_pid 输入限幅 | ⭐ |
| preview_window | 20.0 | 预瞄窗口×0.01=0.2s | ⭐ 预瞄越大越提前响应 |
| standstill_acceleration | **-2.0** | **停止目标减速度**（full stop 用） | ⭐ 刹停强度：|大|→更急更准 |
| standstill_narmal_acceleration | -0.5 | full stop soft 目标减速度 | ⭐ |
| stop_gain_acceleration | -1.0 | 超过 path remain 后的停车增益减速度 | ⭐ |
| speed_itfc_full_stop_speed | 0.09 | 速度接口 full stop 速度上限 | ○ |
| speed_itfc_path_remain_min / max | 0.10 / 0.6 | 速度接口路径余量 | ○ |
| speed_itfc_dcc_emergency | -1.5 | 紧急减速度阈值 | ○ |
| speed_itfc_speed_cmd | 0.10 | full stop 速度指令 | ○ |
| speed_itfc_acc_thres / speed_itfc_stop_speed | 0.0 / 0.05 | 速度接口阈值 | ○ |
| full_stop_pitch_brake_param | 150 | full stop 刹车坡度补偿系数 | ⭐ 坡道停车 |
| max_brk_downhill_change_rate | **0.4** | full stop 刹车爬坡速率上限 | ⭐ 停车平顺：过大→点头 |
| station_pid_conf.pid_conf.kp | **0.4** | **站距 PID 比例增益** | ⭐ 停车精度 |
| station_pid_conf.pid_conf.ki | **0.2** | **站距 PID 积分增益** | ⭐ 消除稳态位置误差 |
| station_pid_conf.pid_conf.kd | 0.0 | 微分 | △ |
| station_pid_conf.integrator_enable / integrator_saturation_level | true / 0.4 | 积分使能/限幅 | ○ |
| station_pid_conf.speed_input | 0/0.55/1.05/2.0/5.0 | 增益调度速度点 | ○ |
| station_pid_conf.kp/ki_speed_gain_output | 1.0×5 | 站距增益（全 1.0=不随速变） | ○ |
| speed_pid_conf.pid_conf.kp | **0.5** | **速度 PID 比例增益** | ⭐ 速度跟踪核心 |
| speed_pid_conf.pid_conf.ki | **0.05** | **速度 PID 积分增益** | ⭐ 消除稳态速度误差 |
| speed_pid_conf.integrator_enable / integrator_saturation_level | true / 0.3 | 积分使能/限幅 | ○ |
| speed_pid_conf.speed_input | 0/0.95/1.05/2.0/5.0 | 增益调度速度点 | ⭐ |
| speed_pid_conf.kp_speed_gain_output | **3.0/3.0/2.0/2.0/1.0** | **speed_pid kp 增益**：低速×3.0→kp=1.5（低速强化） | ⭐⭐ 低速停车/起步跟踪 |
| speed_pid_conf.ki_speed_gain_output | **6.0/6.0/4.0/4.0/1.0** | **speed_pid ki 增益**：低速×6.0→ki=0.3 | ⭐⭐ 低速消稳态误差 |
| pitch_angle_filter_conf.cutoff_freq | 5 | 俯仰角滤波 | ○ |
| station/speed_leadlag_conf | beta 0.8/1.0 | 超前滞后（未启用） | ○ |
| enable_speed_station_preview | false | preview 误差做 PID 输入 | ○ |
| enable_slope_offset | false | 坡度加速度补偿 | ○ |
| use_acceleration_lookup_limit | false | 加速度查表上限 | ○ |
| use_steering_check | true | 大转向清零纵向加速度 | ○ |
| pedestrian_stop_time | **15** | 行人停车等待 15s 后 EPB | ⭐ 人行道停车拉手刹时机 |
| full_stop_long_time | **300.0** | 目的地 full stop 持续→EPB | ⭐ |
| full_stop_path_remain_gain | **0.5** | full stop 后 path_remain 附加量 | ⭐ 防停稳误判又动 |
| epb_change_count | 40 | EPB 切换防抖 | ○ |
| epb_brake | **20.0** | EPB 刹车指令 % | ⭐ |
| full_stop_max_brake | **10** | full stop 最大刹车 % | ⭐ 停车最大制动力 |
| path_remain | **0.15** | 停车判定路径余量 | ⭐⭐ 停车精度核心：越小停越贴近 |
| pitch_path_remain_param / min | 9.0 / -3.0 | 下坡按坡度附加 path_remain | ○ |
| downhill_full_stop_min_speed | 0.4 | 下坡附加最小速度 | ○ |
| battery_threshold | 90 | 电池 SOC>90 用 high_battery 标定表 | ○ |

### 5b. controller_conf.pb.txt（默认值，OPEN_SPACE 兜底，非赛题场景）

> 与 5a 差异项（赛题不生效，仅对比）：standstill_acceleration=-1.1、station kp0.3/ki0.3、speed kp0.5/ki0.05、speed_controller_input_limit 等。**赛题以 public_road_forward_param 为准。**

---

## 六、防溜坡（AntiSlipControlTask）⭐

> 文件：`controllers/slope_anti_slip_control_task/conf/controller_conf.pb.txt`
> 在 LonPlus 之后执行，按坡度/障碍改写 throttle/brake/EPB。

| 参数 | 实车值 | 源码含义 | 备注 |
|------|--------|----------|------|
| （防溜坡参数） | — | 坡道检测（pitch 阈值）+ 溜坡判定 + 刹车/EPB 介入 | ⭐ 坡道停车防溜；苏州赛道基本平坦，一般无需调 |

---

## 七、控制调参总结（赛题视角）

| 目标 | 参数 | 当前值 | 建议 |
|------|------|--------|------|
| 停车更准 | `path_remain` | 0.15 | 已很小，一般不动 |
| 停车更平顺（防点头） | `max_brk_downhill_change_rate` / `standstill_acceleration` | 0.4 / -2.0 | 停车点头→调小 |
| 起步/停车速度跟踪 | `speed_pid kp/ki`（低速增益 3.0/6.0） | 已低速强化 | 一般不动 |
| 横向贴线 | `matrix_q(0,0)` | 2.5 | 想更贴线→调大 |
| 急刹防扣分 | `soft_estop_brake` / `brake_slew_rate_up` | 15.0 / 22.0 | 已偏柔 |
| **🚨 结论** | — | — | **control 实车已调好（Plus 控制器+车厂标定），评分不涉及 control 参数，比赛不建议动 control**（除非现场出现控制层问题） |
