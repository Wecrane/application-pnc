# 06 官方代码 + 限速插件 —— 9 赛题纯配置方案（实车等价环境研究）

> 环境：官方原始 planning（本地 `baseline-official` 分支 = 官方仓库最新）+ 主办方限速插件 `scene_speed_limit` + 官方默认 profile。
> 目标：**只改 `profiles/` 配置**完成 9 赛题（符合"非必要不动源码"分级策略）。
> 搭建环境：`bash shiche/car_connect/restore_official.sh`
> 更新：2026-08-04

---

## 1. 官方默认插件全景（实测 baseline-official）

### 1.1 TrafficRule（traffic_rule_config.pb.txt，9 个启用）
`BACKSIDE_VEHICLE`、`CROSSWALK`、`DESTINATION`、`KEEP_CLEAR`、`REFERENCE_LINE_END`、`REROUTING`、`STOP_SIGN`、`TRAFFIC_LIGHT`、`YIELD_SIGN`
> 官方没有 `scene_speed_limit`（需插件）；`speed_setting` 存在但默认未启用。

### 1.2 Scenario（public_road_planner_config.pb.txt，12 个注册）
`EMERGENCY_PULL_OVER`、`EMERGENCY_STOP`、`VALET_PARKING`、`BARE_INTERSECTION_UNPROTECTED`、`STOP_SIGN_UNPROTECTED`、`YIELD_SIGN`、`TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN`、`TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN`、`TRAFFIC_LIGHT_PROTECTED`、`PULL_OVER`、`PARK_AND_GO`、`LANE_FOLLOW`（兜底）

### 1.3 LaneFollow 默认任务链（lane_follow pipeline.pb.txt）
`LANE_CHANGE_PATH` → `LANE_FOLLOW_PATH` → `LANE_BORROW_PATH` → `FALLBACK_PATH` → `PATH_DECIDER` → `RULE_BASED_STOP_DECIDER` → `SPEED_BOUNDS_PRIORI` → `PATH_TIME_HEURISTIC` → `SPEED_DECIDER` → `SPEED_BOUNDS_FINAL` → `PIECEWISE_JERK_SPEED`

### 1.4 官方关键配置默认值（baseline-official 实测）
| 配置 | 官方默认 | 说明 |
|------|---------|------|
| `planning.conf: planning_upper_speed_limit` | 16.667 | 全局限速上限 |
| `planning.conf: default_cruise_speed` | 16.667 | 巡航目标速度 |
| `planning.conf: speed_bump_speed_limit` | 3.0 | **减速带限速 gflag 机制存在**（±1m 窗口） |
| `planning.conf: destination_check_distance` | 4.0 | 终点建 STOP 距离 |
| crosswalk: `stop_distance` | 1.0 | 人行道停车距离 |
| traffic_light: `stop_distance` | 1.0 | 红绿灯停车距离 |
| stop_sign: `stop_distance` | 1.0 | 停止标志停车距离 |
| speed_decider: `follow_min_time_sec` | 2.0 | 跟车判定时间阈值 |
| speed_decider: `stop_follow_distance` | 2.0 | 跟车距离基准 |
| speed_decider: `min_stop_distance_obstacle`(gflag) | 6.0 | 静态障碍 STOP 距离（≥2m 达标） |

---

## 2. 9 赛题 → 插件/配置需求矩阵

| # | 赛题 | 依赖插件 | 开关 | 关键配置 |
|---|------|---------|------|---------|
| 0 | 定速巡航 | LANE_FOLLOW（默认） | 开 | planning.conf `default_cruise_speed` |
| 1 | 减速带通行 | gflag 机制 或 限速插件 speed_bump | 开 | planning.conf `speed_bump_speed_limit` 或插件 `speed_bump_*` |
| 2 | 人行道(有人) | CROSSWALK（TrafficRule） | 开 | crosswalk conf `stop_distance` |
| 3 | 红绿灯停车 | TRAFFIC_LIGHT + TRAFFIC_LIGHT_PROTECTED 场景 | 开 | traffic_light conf `stop_distance` |
| 4 | 停止标志 | STOP_SIGN + STOP_SIGN_UNPROTECTED 场景 | 开 | stop_sign conf `stop_distance` |
| 5 | 障碍物停车避让 | speed_decider / path_decider（默认） | 开 | 无需改（6m 达标） |
| 6 | 动态障碍物跟随 | speed_decider（默认）+ LANE_FOLLOW | 开 | speed_decider conf `follow_*` |
| 7 | 人行道路口减速 | **限速插件 crosswalk** | **需装插件** | 插件 conf `crosswalk_*` |
| 8 | 交通灯路口减速 | **限速插件 traffic_light** | **需装插件** | 插件 conf `traffic_light_*` |

### 插件开关建议
- **必须开**（停车/避让核心）：`CROSSWALK`、`TRAFFIC_LIGHT`、`STOP_SIGN`、`DESTINATION`、`BACKSIDE_VEHICLE`；场景 `LANE_FOLLOW`、`STOP_SIGN_UNPROTECTED`、`TRAFFIC_LIGHT_PROTECTED`
- **限速插件**：赛题 1/7/8 需要，分别开 `speed_bump` / `crosswalk` / `traffic_light` 三个 enabled
- **可关可留**（无赛题依赖，留着无害）：`KEEP_CLEAR`、`YIELD_SIGN`、`REROUTING`、`REFERENCE_LINE_END`（⚠️ 赛题5 可能想禁 REF_END 提前减速，但官方无该配置开关，属代码层）
- 其余场景（VALET/PULL_OVER/PARK_AND_GO 等）不触发，无需处理

---

## 3. 9 赛题纯配置方案（官方 + 插件）

> 通用：改配置 → `aem profile use default` → 重启/测试（**不编译**，除非装插件首次编译）

### 赛题 0：定速巡航（≤11 km/h ≈ 3.06 m/s）
```text
# planning.conf
--default_cruise_speed=3.0        # 或现场标准，留 0.05~0.2 裕量
```
⚠️ **冲突风险**：`default_cruise_speed` 是全局参数，调低会影响其他赛题速度。需现场确认定速巡航赛题是否独立配置/场景下发巡航速度。

### 赛题 1：减速带通行（≤1.5 m/s）
```text
# 方案A（推荐，用官方已有 gflag 机制）planning.conf：
--speed_bump_speed_limit=1.2      # 官方默认3.0，实车≤1.5 → 留裕量取1.2~1.4
# 方案B（限速插件）scene_speed_limit conf：
speed_bump_enabled: true
speed_bump_limit_speed: 1.2
speed_bump_forward_buffer: 3.0    # 提前限速距离（实车可调大些更早减速）
speed_bump_backward_buffer: 1.0
```
⚠️ 两个机制并存会取最小限速，**只配一个**最稳。

### 赛题 2：人行道（有人）停车 1~1.5 m
```text
# crosswalk conf:
stop_distance: 1.25               # 区间中值；实车停车偏移需现场标定（评测≈sd+偏移0.2~0.8）
expand_s_distance: 4.0            # 行人完全走完才放行
stop_timeout: 30.0                # 等待行人时间
```
⚠️ **最大风险点**：官方代码的 speed_decider 对**静止行人**用 `min_stop_distance_obstacle=6m` 建 STOP，会覆盖 crosswalk 的 1.25m → 车可能停 6m（仿真曾踩坑，靠代码 `pedestrian_stop_distance` gflag 修复，官方没有）。**需现场验证**；若发生，纯配置无解（该 gflag 不存在），只能接受或兜底改代码。

### 赛题 3：红绿灯停车 1~1.5 m
```text
# traffic_light conf:
stop_distance: 1.25
```

### 赛题 4：停止标志停车 1~1.5 m（完全停止）
```text
# stop_sign conf:
stop_distance: 1.25               # STOP_SIGN_UNPROTECTED 场景保证完全停止
```

### 赛题 5：障碍物停车避让（≥2 m）
```text
# 无需修改！min_stop_distance_obstacle=6m 天然达标
```

### 赛题 6：动态障碍物跟随
```text
# speed_decider conf:
follow_min_time_sec: 1.0          # 官方2.0 → 缩短前车起步后恢复迟滞（仿真 9.4s 教训）
stop_follow_distance: 2.0         # 保持官方
# 平稳性不够再调 piecewise_jerk_speed 的 jerk_weight/acc_weight
```

### 赛题 7：人行道路口减速（无人）≤1.5 m/s —— 需限速插件
```text
# scene_speed_limit conf:
crosswalk_enabled: true
crosswalk_limit_speed: 1.2
crosswalk_forward_buffer: 5.0
crosswalk_backward_buffer: 2.0
```

### 赛题 8：交通灯路口减速 ≤1.5 m/s —— 需限速插件
```text
# scene_speed_limit conf:
traffic_light_enabled: true
traffic_light_limit_speed: 1.2
traffic_light_forward_buffer: 8.0
traffic_light_backward_buffer: 2.0
```

---

## 4. 与仿真魔改代码的差异（实车必须注意）

| 本地仿真魔改 | 官方有没有 | 实车影响 |
|------------|:---:|---------|
| `pedestrian_stop_distance` gflag（行人1.75m） | ❌ | 赛题2 有 6m 覆盖风险；profile 不能照搬（未知 flag 报错） |
| `destination_check_distance=25` + 禁 REF_END | ❌ | 赛题5 终点计分策略不同；官方 profile 只有 `destination_check_distance=4.0` |
| 护栏 UNKNOWN_UNMOVABLE 过滤 | ❌ | 实车护栏闪烁 STOP 风险（需现场观察） |
| path_decider 行人 continue | ❌ | 行人 STOP 位置可靠性（感知噪声） |
| 无人人行道/绿灯 AddSpeedLimit(4.5) | ❌（由插件替代） | 赛题7/8 靠插件 |
| 绿灯/黑灯跳过 | 部分 | 官方绿灯逻辑以实车为准 |
| `planning_upper_speed_limit=20` 等提速参数 | ✅ 通用 gflag | 可沿用但要复核值 |

**结论**：实车**必须从官方默认 profile 起步**（不能照搬仿真 profile），按上表逐项配置。赛题 2/5 是纯配置不确定点，现场优先验证。

---

## 5. 现场验证顺序建议（30 分钟）

1. 连车 → `check_car.sh` → `backup_car.sh`
2. 确认限速插件是否在车（`ls ~/apollo*/modules/planning/traffic_rules/scene_speed_limit`）
3. 按"先限速类(1/7/8 插件) → 停车类(2/3/4 stop_distance) → 巡航(0) → 障碍类(5/6)"配配置
4. 每改一组 `apply_config.sh` 生效 → 测试 → 看评分
5. **优先验证赛题 2**（6m 覆盖风险最大）和**赛题 5**（终点计分）

## 6. 需要现场确认的开放问题

- [ ] 实车是否已预装/预编译限速插件？（决定 7/8 是纯配置还是需装插件+编译）
- [ ] 定速巡航是否独立配置巡航速度？（全局 `default_cruise_speed` 会互相影响）
- [ ] 赛题 2 静止行人是否触发 speed_decider 6m 覆盖？（决定是否需兜底）
- [ ] 实车评分细则（以现场手册为准，可能与仿真不同）
