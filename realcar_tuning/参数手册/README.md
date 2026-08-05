# 实车 profiles 全参数手册（回归代码版 · 2026-08-05）

> **🚨 本文档逐一解释实车 HQEV_BDN profiles 配置**（`realcar_tuning/ORIGINAL/`）中**每一个参数**的作用意义，并**回归源码**（`modules/planning/`、`modules/control/` 的 .cc/.h/proto/gflags）给出精确含义。
> **覆盖范围**：planning（planning_component + 10 traffic_rules + 32 tasks + 17 scenarios）+ control（19 文件），共 **~7500 行参数**。
> **标注说明**：⭐=比赛核心 ○=一般 △=无关/泊车等。⚠️=实车与官方默认/本地魔改有差异处。

## 目录（6 个 part + 配套）

| 文件 | 内容 |
|------|------|
| `README.md` | 本总览 |
| `01_planning_core.md` | **planning_component/conf**：planning.conf 全部 gflags + planning_config + traffic_rule_config + public_road_planner_config + smoother + open_space 配置 |
| `02_traffic_rules.md` | **10 个 TrafficRule**：crosswalk / traffic_light / stop_sign / scene_speed_limit(限速插件) / destination / reference_line_end / keepclear / rerouting / yield_sign / backside_vehicle |
| `03_tasks_core.md` | **16 个核心 Task**：speed_decider / path_decider / rule_based_stop_decider / piecewise_jerk_speed / speed_bounds_decider / path_time_heuristic / lane_follow_path / lane_change_path / lane_borrow_path / obstacle_nudge_decider 等 |
| `04_tasks_open_space.md` | **16 个 open_space/泊车 Task**：open_space_* 系列 / pull_over_path / reverse_* / rss_decider / square_path（△ 比赛无关） |
| `05_scenarios.md` | **17 个 Scenario**：traffic_light_protected / stop_sign_unprotected / lane_follow / park_and_go 等（含 pipeline 任务链） |
| `06_control.md` | **Control 19 文件**：control.conf 全部 gflags + pipeline（Plus 控制器链）+ LQR 横向 + PID 纵向 + 防溜坡 + 标定表 |

## 比赛最相关的参数速查（详见各 part）

| 模块 | 关键参数 | 实车值 | 含义 |
|------|---------|--------|------|
| planning | `default_cruise_speed` | 5.0 | 巡航速度（评分 ≤3.0556，建议 3.0）⭐ |
| planning | `planning_upper_speed_limit` | 5.0 | 全局限速（铁律 [1.5,5.1] 别动）⭐ |
| crosswalk | `stop_distance` / `expand_s_distance` / `stop_timeout` | 0.10 / 0.0 / 3.0 | 人行道停车距离/检测区/等待超时 ⭐ |
| traffic_light | `stop_distance` | 0.10 | 红灯停车距离 ⭐ |
| scene_speed_limit | `crosswalk_*` / `traffic_light_*` | 全 false | 限速插件（减速带/路口限速核心）⭐ |
| speed_decider | `follow_min_time_sec` / `stop_follow_distance` | 2.0 / 1.5 | 跟车判定/距离 |
| control PID | `station_pid kp/ki`、`speed_pid kp/ki` | 0.4/0.2、0.5/0.05 | 纵向停车/速度控制 ⭐ |
| control LQR | `matrix_q`（public_road） | 2.5/1.0/1.5/0.0 | 横向控制增益 ⭐ |
| control | `path_remain` / `standstill_acceleration` | 0.15 / -2.0 | 停车精度/刹停强度 ⭐ |

## 源码依据

- planning gflags：`modules/planning/planning_base/gflags/planning_gflags.cc`
- 规则/任务/场景：`modules/planning/{traffic_rules,tasks,scenarios}/<name>/{proto,*.cc}`
- control：`modules/control/control_component/` + `controllers/` + `control_task_base_extend/`

> ⚠️ 注意：本地 `modules/` 部分源码含竞赛魔改（mayaochang add），已在对应参数备注中标出"本地魔改/实车官方无"；实车是**预编译官方二进制 + profile 覆盖**，以实车官方语义为准。
