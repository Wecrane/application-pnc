# TrafficRule 配置文件全参数深度分析（9 个规则）

> 工程：Apollo 11.0 EDU（application-pnc，官方原始版本 + 少量定制注释）
> 分析日期：2026-08-01
> 范围：`modules/planning/traffic_rules/` 下 9 个规则的 `conf/default_conf.pb.txt` 每一行参数
> 方法：proto 默认值对比 → grep 定位 `.cc/.h` → 阅读代码逻辑 → 行为影响分析
> **本分析为纯分析，未修改任何代码/配置**

---

## 0. 规则启用总览（traffic_rule_config.pb.txt）

主配置文件：`modules/planning/planning_component/conf/traffic_rule_config.pb.txt`

共启用 9 个规则，按执行顺序（ScenarioManager 注册顺序）：

| # | name | type | 对应 conf |
|---|------|------|-----------|
| 1 | BACKSIDE_VEHICLE | BacksideVehicle | `backside_vehicle/conf/default_conf.pb.txt` |
| 2 | CROSSWALK | Crosswalk | `crosswalk/conf/default_conf.pb.txt` |
| 3 | DESTINATION | Destination | `destination/conf/default_conf.pb.txt` |
| 4 | KEEP_CLEAR | KeepClear | `keepclear/conf/default_conf.pb.txt` |
| 5 | REFERENCE_LINE_END | ReferenceLineEnd | `reference_line_end/conf/default_conf.pb.txt` |
| 6 | REROUTING | Rerouting | `rerouting/conf/default_conf.pb.txt` |
| 7 | STOP_SIGN | StopSign | `stop_sign/conf/default_conf.pb.txt` |
| 8 | TRAFFIC_LIGHT | TrafficLight | `traffic_light/conf/default_conf.pb.txt` |
| 9 | YIELD_SIGN | YieldSign | `yield_sign/conf/default_conf.pb.txt` |

> 注：`traffic_rules/speed_setting`（SpeedSetting）存在但**未在列表中启用**，且无 conf 文件（依赖外部 SpeedCommand，用于动态调速）。
>
> 配置生效链路：`TrafficRule::LoadConfig<T>(&config_)`（各规则 `Init()` 中调用）→ 从 `<规则名>.proto` 定义的 message 加载 `conf/default_conf.pb.txt`。**profile 层（`profiles/default/modules/planning/`）当前只有 `planning_component/conf/planning.conf`，无 traffic_rules 覆盖层 → 运行期实际读取的是源码 `modules/planning/traffic_rules/<规则>/conf/default_conf.pb.txt`。**

---

## 1. CROSSWALK 人行道（赛题重点 ⭐⭐⭐）

配置文件：`modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt`
proto 定义：`modules/planning/traffic_rules/crosswalk/proto/crosswalk.proto`（`CrosswalkConfig`）
实现代码：`modules/planning/traffic_rules/crosswalk/crosswalk.cc`（`Crosswalk::MakeDecisions` / `Crosswalk::CheckStopForObstacle`）

```text
stop_distance: 1.0
max_stop_deceleration: 4.0
min_pass_s_distance: 1.0
expand_s_distance: 2.0
stop_strict_l_distance: 5.0
stop_loose_l_distance: 5.0
stop_timeout: 10.0
```

### 1.1 stop_distance：1.0（proto 默认 1.0）

- **代码位置**：`crosswalk.cc` `MakeDecisions()` → `util::BuildStopDecision(virtual_obstacle_id, crosswalk_overlap->start_s, config_.stop_distance(), STOP_REASON_CROSSWALK, ...)`
- **作用**：停车安全距离。`BuildStopDecision` 在 `planning_base/common/util/common.cc` 中：`const double stop_s = stop_line_s - stop_distance;` 即**停车点 = 人行道停止线 s 值（crosswalk start_s）往前回退 stop_distance**，stop 决策的 `distance_s = -stop_distance`。虚拟停车墙创建在 `start_s` 处。
- **对车辆行为影响**：车头最终停在**人行道停止线之前 stop_distance 米**处。值越大停得越早（越保守）。
- **赛题相关**：赛题要求"人行道停车 1.5–2.0m（有人/无人）"。当前 1.0m **不满足赛题**，需调大至 1.5–2.0m 区间。

### 1.2 max_stop_deceleration：4.0（proto 默认 4.0）

- **代码位置**：`crosswalk.cc` `CheckStopForObstacle()`：`if (stop_deceleration >= config_.max_stop_deceleration()) { if (obstacle_l_distance > config_.stop_strict_l_distance()) { stop = false; } }`；`stop_deceleration` 由 `util::GetADCStopDeceleration()`（`planning_base/common/util/util.cc`）计算：`v²/(2×距停止线距离)`（低速时返回 0）。
- **作用**：若按当前车速在停止线前刹停所需减速度 ≥ 该值（说明"来不及停了"），则**放弃停车**；当行人横向距离 > strict_l 时直接忽略（放行）。
- **对车辆行为影响**：值越大，越"愿意"尝试急刹停车（容忍更大减速度）；值越小越容易闯人行道（放弃停车直接通过）。4.0 是舒适性上限（约 0.4g）。**注意：只有横向距离 > stop_strict_l_distance 时才会因减速过大而放弃，否则仍会停车。**

### 1.3 min_pass_s_distance：1.0（proto 默认 1.0）

- **代码位置**：`crosswalk.cc` `MakeDecisions()`：`if (adc_front_edge_s - crosswalk_overlap->end_s > config_.min_pass_s_distance())` → 视为"已通过该人行道"，跳过（并清除 crosswalk_id/stop_time 状态）。
- **作用**：判定"车头已驶过人行道末端 + 该距离"即可认为通过，不再触发停车。
- **对车辆行为影响**：值越大，判定"通过"越晚（需要车头越过停止线更远才解除），可能延长停车等待；值越小越早放行。

### 1.4 expand_s_distance：2.0（proto 默认 2.0）

- **代码位置**：`crosswalk.cc` `CheckStopForObstacle()`：`const Polygon2d crosswalk_exp_poly = crosswalk_ptr->polygon().ExpandByDistance(config_.expand_s_distance());` 然后用 `IsPointIn(point)` 判断行人/自行车是否在"膨胀后的人行道多边形"内。
- **作用**：把人行道多边形向外膨胀 2.0m，扩大行人/自行车检测范围（含两侧人行道边区域）。
- **对车辆行为影响**：值越大，检测范围越大，越容易检测到行人而停车；值越小越容易漏检（行人贴近但不在膨胀区内则直接忽略）。

### 1.5 stop_strict_l_distance：5.0（**proto 默认 4.0，当前值被改大 ⚠️**）

- **代码位置**：`crosswalk.cc` `CheckStopForObstacle()`：
  - `obstacle_l_distance <= config_.stop_strict_l_distance()` 时：若在道路上且 s 在车头前 → **必须停车**（on_road 无条件停）；不在道路上则看路径交叉/是否朝自车移动。
  - 且该值也是"因减速过大而放弃停车"的门槛：只有 `obstacle_l_distance > stop_strict_l_distance` 时急刹可放弃。
- **作用**：行人横向距离 ≤ 该值 → 进入"严格停车区"，大概率要求停车。
- **对车辆行为影响**：当前 5.0 > 默认 4.0，**判定更严格**（更多横向位置的行人会被要求停车）。赛题若需"无人行道 ≤5m/s 通过"策略，此参数影响检测灵敏度。

### 1.6 stop_loose_l_distance：5.0（proto 默认 5.0）

- **代码位置**：`crosswalk.cc` `CheckStopForObstacle()`：`if (obstacle_l_distance >= config_.stop_loose_l_distance()) { if (is_path_cross) { stop = true; } }`；两者之间（strict < l < loose）用历史决策。
- **作用**：行人横向距离 ≥ 该值 → 视为"足够远"，**只有路径预测交叉才停车**，否则忽略。
- **对车辆行为影响**：值越大，"足够远"的门槛越高，更倾向于停车；值越小越宽松。注意当前 strict=loose=5.0，中间"历史决策"区间不存在。

### 1.7 stop_timeout：10.0（proto 默认 10.0）

- **代码位置**：`crosswalk.cc` `MakeDecisions()`：对**静止行人/自行车**（速度 ≤ 0.3m/s、不在车道上、距人行道 ≤ start_watch_timer_distance）启动看门计时器，`if (stop_time >= config_.stop_timeout()) { stop = false; }`。
- **作用**：行人静止等待超时阈值。超过 10s 后认为行人"不会动"，**解除停车、继续通过**。
- **对车辆行为影响**：值越小越早"放弃等待"直接通过（对赛题"无人/静止"场景有利）；值越大等待越久。这是"人行道停车"场景**无人**判定与放行的关键计时参数。

### 1.8 start_watch_timer_distance：40.0（**conf 中未写，取 proto 默认 40.0**）

- **代码位置**：`crosswalk.cc` `MakeDecisions()`：`if (stop && !is_on_lane && crosswalk_overlap->start_s - adc_front_edge_s <= config_.start_watch_timer_distance())` → 才启动静止行人计时器。
- **作用**：距人行道停止线 40m 内才启动"静止行人等待计时"。
- **对车辆行为影响**：距人行道很远时不计时（避免过早开始等待计时）；进入 40m 内开始计时。

### 1.9 人行道限速检查（赛题相关 ⚠️）

- **结论**：`crosswalk.cc` **不存在** `SetSpeedLimit` / `limit_speed` / `speed_limit` 相关代码（已 grep `modules/planning/traffic_rules/crosswalk/**` 确认 0 命中）。人行道规则**只会停车，不会限速**。
- **赛题影响**：赛题"无人人行道 ≤5m/s"、"绿灯路口 ≤5m/s"在当前 TrafficRule 配置/代码中**无实现字段**。要实现需新增代码（如在 `Crosswalk::ApplyRule` 中调用 `reference_line_info->SetSpeedLimit(...)` 或 `SetCruiseSpeed`），不能仅靠改配置。
- 现有可复用限速机制：减速带 `FLAGS_speed_bump_speed_limit`（见 §10）。

---

## 2. TRAFFIC_LIGHT 红绿灯（赛题重点 ⭐⭐⭐）

配置文件：`modules/planning/traffic_rules/traffic_light/conf/default_conf.pb.txt`
proto 定义：`modules/planning/traffic_rules/traffic_light/proto/traffic_light.proto`（`TrafficLightConfig`）
实现代码：`modules/planning/traffic_rules/traffic_light/traffic_light.cc`（`TrafficLight::MakeDecisions`）

```text
enabled: true
stop_distance: 1.0
max_stop_deceleration: 4.0
```

### 2.1 enabled：true（proto 默认 true）

- **代码位置**：`traffic_light.cc` `MakeDecisions()` 开头：`if (!config_.enabled()) { return; }`
- **作用**：总开关。false 则整个红绿灯规则不执行（直接闯灯）。
- **对车辆行为影响**：false = 无视红绿灯。

### 2.2 stop_distance：1.0（proto 默认 1.0）

- **代码位置**：`traffic_light.cc` `MakeDecisions()` → `util::BuildStopDecision(virtual_obstacle_id, traffic_light_overlap.start_s, config_.stop_distance(), STOP_REASON_SIGNAL, ...)`
- **作用**：停车安全距离。停车点 = 红绿灯停止线 s（`signal_overlap.start_s`）回退 stop_distance（`stop_s = stop_line_s - stop_distance`，`distance_s = -stop_distance`）。
- **对车辆行为影响**：车头停在**红灯停止线前 stop_distance 米**。赛题要求"红绿灯停车 1.5–2.0m"，当前 1.0m **不满足**，需调大。
- **与代码定制点**：`traffic_light.cc` 中有一处 `// mayaochang add` 注释的定制逻辑：`if (signal_color == GREEN || signal_color == BLACK) { continue; }`（绿灯/黑灯直接跳过，不建停车墙）——即红灯/黄灯/未知才停车。

### 2.3 max_stop_deceleration：4.0（proto 默认 4.0）

- **代码位置**：`traffic_light.cc` `MakeDecisions()`：`if (stop_deceleration > config_.max_stop_deceleration()) { AWARN << "stop_deceleration too big to achieve.  SKIP red light"; continue; }`；`stop_deceleration = util::GetADCStopDeceleration(vehicle_state, adc_front_edge_s, traffic_light_overlap.start_s)`（`v²/(2×距停止线距离)`）。
- **作用**：若红灯前刹停所需减速度 > 该值 → **放弃停车直接闯灯**（已来不及安全刹停）。
- **对车辆行为影响**：值越大越"硬刹停"，值越小越容易闯红灯。4.0 = 0.4g 舒适上限。赛题若要求严格停灯，可适当调大；但过大会导致急刹，需权衡。

### 2.4 红绿灯限速检查（赛题相关 ⚠️）

- **结论**：`traffic_light.cc` **无任何限速逻辑**（无 `SetSpeedLimit`）。绿灯路口限速 ≤5m/s 需新增代码，配置无法实现。

---

## 3. STOP_SIGN 停止标志（赛题重点 ⭐⭐⭐）

配置文件：`modules/planning/traffic_rules/stop_sign/conf/default_conf.pb.txt`
proto 定义：`modules/planning/traffic_rules/stop_sign/proto/stop_sign.proto`（`StopSignConfig`）
实现代码：`modules/planning/traffic_rules/stop_sign/stop_sign.cc`（`StopSign::MakeDecisions`）

```text
enabled: true
stop_distance: 1.0
```

### 3.1 enabled：true（proto 默认 true）

- **代码位置**：`stop_sign.cc` `MakeDecisions()`：`if (!config_.enabled()) { return; }`
- **作用**：总开关。
- **对车辆行为影响**：false = 无视停止标志。

### 3.2 stop_distance：1.0（proto 默认 1.0）

- **代码位置**：`stop_sign.cc` `MakeDecisions()` → `util::BuildStopDecision(virtual_obstacle_id, stop_sign_overlap.start_s, config_.stop_distance(), STOP_REASON_STOP_SIGN, wait_for_obstacle_ids, ...)`；`wait_for_obstacle_ids` 来自 `stop_sign_status.wait_for_obstacle_id()`（由 stop sign scenario 阶段设置，用于"让行"逻辑）。
- **作用**：停车点 = 停止标志停止线 s 回退 stop_distance。
- **对车辆行为影响**：车头停在停止牌线前 stop_distance 米。赛题要求"停止标志停车 1.5–2.0m"，当前 1.0m **不满足**，需调大。
- **注意**：实际"停稳 + 观察 + 蠕行"的节奏由 `STOP_SIGN` scenario（stop_sign_unprotected）驱动，本规则只负责建虚拟停车墙；`max_valid_stop_distance` 在 scenario 配置中（`scenarios/traffic_light_protected/conf/scenario_conf.pb.txt` 等，为 2.0）。

### 3.3 停止标志限速检查

- **结论**：`stop_sign.cc` **无限速逻辑**。停止标志处无"限速通过"机制。

---

## 4. YIELD_SIGN 让行标志

配置文件：`modules/planning/traffic_rules/yield_sign/conf/default_conf.pb.txt`
proto 定义：`modules/planning/traffic_rules/yield_sign/proto/yield_sign.proto`（`YieldSignConfig`）
实现代码：`modules/planning/traffic_rules/yield_sign/yield_sign.cc`（`YieldSign::MakeDecisions`）

```text
enabled: true
stop_distance: 1.0
```

### 4.1 enabled：true（proto 默认 true）

- **代码位置**：`yield_sign.cc` `MakeDecisions()`：`if (!config_.enabled()) { return; }`
- **作用**：总开关。
- **对车辆行为影响**：false = 无视让行标志。

### 4.2 stop_distance：1.0（proto 默认 1.0）

- **代码位置**：`yield_sign.cc` `MakeDecisions()` → `util::BuildStopDecision(virtual_obstacle_id, yield_sign_overlap.start_s, config_.stop_distance(), STOP_REASON_YIELD_SIGN, wait_for_obstacle_ids, ...)`
- **作用**：停车点 = 让行标志线 s 回退 stop_distance。
- **对车辆行为影响**：车头停在让行线前 stop_distance 米。让行后能否通过由 scenario/stage（`done_yield_sign_overlap_id`）与 `wait_for_obstacle_id` 决定。

---

## 5. BACKSIDE_VEHICLE 后方来车

配置文件：`modules/planning/traffic_rules/backside_vehicle/conf/default_conf.pb.txt`
proto 定义：`modules/planning/traffic_rules/backside_vehicle/proto/backside_vehicle.proto`（`BacksideVehicleConfig`）
实现代码：`modules/planning/traffic_rules/backside_vehicle/backside_vehicle.cc`（`BacksideVehicle::MakeLaneKeepingObstacleDecision`）

```text
backside_lane_width: 4.0
```

### 5.1 backside_lane_width：4.0（proto 默认 4.0）

- **代码位置**：`backside_vehicle.cc` `MakeLaneKeepingObstacleDecision()`：`const double lane_boundary = config_.backside_lane_width(); if (obstacle->PerceptionSLBoundary().start_l() > lane_boundary || obstacle->PerceptionSLBoundary().end_l() < -lane_boundary) { continue; }`
- **作用**：用于判定"自车后方的障碍车是否在自车横向投影范围 [−4.0, 4.0] 内"。在范围内的后方车辆**不忽略**（保留下方决策，防止变道/跟车时被无视造成碰撞）；范围外的后方车可忽略。
- **对车辆行为影响**：值越大，被"认真对待"的后方车辆横向范围越宽（更保守）；值越小越宽松。

---

## 6. DESTINATION 到达终点

配置文件：`modules/planning/traffic_rules/destination/conf/default_conf.pb.txt`
proto 定义：`modules/planning/traffic_rules/destination/proto/destination.proto`（`DestinationConfig`）
实现代码：`modules/planning/traffic_rules/destination/destination.cc`（`Destination::MakeDecisions`）

```text
stop_distance: 0.2
```

### 6.1 stop_distance：0.2（**proto 默认 0.5，当前值被改小 ⚠️**）

- **代码位置**：`destination.cc` `MakeDecisions()` 两处：
  1. 靠边停车（pull-over）分支：`stop_line_s = pull_over_sl.s() + front_edge_to_center + config_.stop_distance()`；
  2. 常规终点分支：`dest_lane_s = fmax(0.0, routing_end->s() - FLAGS_virtual_stop_wall_length - config_.stop_distance())`，再 `BuildStopDecision(stop_wall_id, routing_end->id(), dest_lane_s, config_.stop_distance(), STOP_REASON_DESTINATION, ...)`。
- **作用**：终点停车墙相对路由终点往回退 0.2m，停车点再往前 `stop_distance` 余量。`frame->is_near_destination()` 触发（由 gflag `destination_check_distance`，profile 中 =4.0）。
- **对车辆行为影响**：0.2m 意味着车几乎贴着路由终点停。值过小可能导致停车点越过终点/被 `reference_line_end` 的 PATH_END 墙先拦（代码里有对应 AWARN 警告）。赛题一般无需改动。

---

## 7. KEEP_CLEAR 禁停区/路口

配置文件：`modules/planning/traffic_rules/keepclear/conf/default_conf.pb.txt`
proto 定义：`modules/planning/traffic_rules/keepclear/proto/keepclear.proto`（`KeepClearConfig`）
实现代码：`modules/planning/traffic_rules/keepclear/keep_clear.cc`（`KeepClear::ApplyRule` / `BuildKeepClearObstacle`）

```text
enable_keep_clear_zone: true
enable_junction: true
min_pass_s_distance: 2.0
align_with_traffic_sign_tolerance: 4.5
```

### 7.1 enable_keep_clear_zone：true（proto 默认 true）

- **代码位置**：`keep_clear.cc` `ApplyRule()`：`if (config_.enable_keep_clear_zone()) { ... 遍历 clear_area_overlaps() ... }`
- **作用**：是否对地图 `keep-clear-zone`（禁停区/网格线区）生成虚拟障碍，禁止在其中停车。
- **对车辆行为影响**：true 时车不会停在禁停区内（如路口网格线）。

### 7.2 enable_junction：true（proto 默认 true）

- **代码位置**：`keep_clear.cc` `ApplyRule()`：`if (config_.enable_junction()) { ... 遍历 FirstEncounteredOverlaps()，对 PNC_JUNCTION 生成禁停障碍 ... }`
- **作用**：是否对 `pnc_junction`（路口）同时做禁停检查。
- **对车辆行为影响**：true 时路口区域也被禁停（避免停在路口中央）。

### 7.3 min_pass_s_distance：2.0（proto 默认 2.0）

- **代码位置**：`keep_clear.cc` `BuildKeepClearObstacle()`：`if (adc_front_edge_s - keep_clear_start_s > config_.min_pass_s_distance()) { return false; }`
- **作用**：车头越过禁停区起点超过 2.0m 后，认为已"通过"禁停区，不再生成禁停障碍。
- **对车辆行为影响**：值越大，车需要越深入禁停区才解除禁停障碍（更保守）。

### 7.4 align_with_traffic_sign_tolerance：4.5（proto 默认 4.5）

- **代码位置**：`keep_clear.cc` `ApplyRule()`：当 `pnc_junction` 起点与红绿灯/停止牌/人行道重叠起点之差 ≤ 4.5m 时，把 pnc_junction 起点**对齐到**该交通设施起点（先灯、后牌、再人行道）。
- **作用**：让路口的禁停区边界与交通灯/牌停止线对齐，避免禁停区与停止线错位导致逻辑冲突。
- **对车辆行为影响**：影响路口禁停区与停止线的对齐精度，间接影响停车位置一致性。

---

## 8. REFERENCE_LINE_END 参考线末端

配置文件：`modules/planning/traffic_rules/reference_line_end/conf/default_conf.pb.txt`
proto 定义：`modules/planning/traffic_rules/reference_line_end/proto/reference_line_end.proto`（`ReferenceLineEndConfig`）
实现代码：`modules/planning/traffic_rules/reference_line_end/reference_line_end.cc`（`ReferenceLineEnd::ApplyRule`）

```text
stop_distance: 0.5
min_reference_line_remain_length: 50.0
```

### 8.1 stop_distance：0.5（proto 默认 0.5）

- **代码位置**：`reference_line_end.cc` `ApplyRule()`：`const double stop_line_s = obstacle_start_s - config_.stop_distance();`（`obstacle_start_s = reference_line.Length() - 2 * FLAGS_virtual_stop_wall_length`），并 `stop_decision->set_distance_s(-config_.stop_distance());`
- **作用**：参考线末端虚拟停车墙再往前回退 0.5m 作为停车点。
- **对车辆行为影响**：车停在参考线末端前 0.5m，防止冲出参考线/地图边界。此墙的 `STOP_REASON_DESTINATION` 可能先于真正的 Destination 规则触发（代码中有相关 AWARN 提示）。

### 8.2 min_reference_line_remain_length：50.0（proto 默认 50.0）

- **代码位置**：`reference_line_end.cc` `ApplyRule()`：`double remain_s = reference_line.Length() - AdcSlBoundary().end_s(); if (remain_s > config_.min_reference_line_remain_length()) { return Status::OK(); }`
- **作用**：剩余参考线长度 > 50m 时不建末端停车墙；≤ 50m 才建。
- **对车辆行为影响**：值越大越早建墙（提前 50m 就开始建末端停车墙）；值越小越晚。50m 是安全提前量。

---

## 9. REROUTING 重新规划

配置文件：`modules/planning/traffic_rules/rerouting/conf/default_conf.pb.txt`
proto 定义：`modules/planning/traffic_rules/rerouting/proto/rerouting.proto`（`ReroutingConfig`）
实现代码：`modules/planning/traffic_rules/rerouting/rerouting.cc`（`Rerouting::ChangeLaneFailRerouting`）

```text
cooldown_time: 3.0
prepare_rerouting_time: 2.0
```

### 9.1 cooldown_time：3.0（proto 默认 3.0）

- **代码位置**：`rerouting.cc` `ChangeLaneFailRerouting()`：`if (rerouting->has_last_rerouting_time() && (current_time - rerouting->last_rerouting_time() < config_.cooldown_time())) { return true; }`
- **作用**：两次重规划请求的最小间隔（冷却时间）。3s 内不重复发 rerouting 请求。
- **对车辆行为影响**：值越小重规划越频繁（可能抖动）；值越大越"迟钝"。赛题一般无需改动。

### 9.2 prepare_rerouting_time：2.0（proto 默认 2.0）

- **代码位置**：`rerouting.cc` `ChangeLaneFailRerouting()`：`const double prepare_distance = speed * prepare_rerouting_time; if (sl_point.s() > adc_s + prepare_distance) { return true; }`
- **作用**：变道失败场景中，若"当前通道终点"距自车超过 `车速 × 2.0s`，说明还能开一会儿，暂不重规划。
- **对车辆行为影响**：值越大越晚触发重规划（留给自车更多行驶余量）；值越小越早重规划。

---

## 10. 赛题相关：减速带限速与通行限速机制（补充）

### 10.1 减速带 ≤3m/s（赛题 1）

- **实现位置**：不在 TrafficRule 中，而在 gflag + `ReferenceLineInfo`：
  - gflag：`planning_base/gflags/planning_gflags.cc`：`DEFINE_double(speed_bump_speed_limit, 4.4704, ...)`（默认 10mph=4.47m/s）
  - 当前值：`profiles/default/modules/planning/planning_component/conf/planning.conf`：`--speed_bump_speed_limit=3`（**已配置为 3m/s**）
  - 使用处：`planning_base/common/reference_line_info.cc`：`for (speed_bump : map_path.speed_bump_overlaps()) { reference_line_.AddSpeedLimit(speed_bump.start_s - 1.0, speed_bump.end_s + 1.0, FLAGS_speed_bump_speed_limit); }` → 在减速带前后各扩展 1m 施加 3m/s 限速。
- **行为影响**：车辆过减速带区间（±1m）速度被限制到 ≤3m/s。

### 10.2 无人行道 ≤5m/s、绿灯路口 ≤5m/s（赛题 7/8）

- **结论**：**9 个 TrafficRule 配置及 crosswalk/traffic_light/stop_sign 代码中均不存在"人行道/路口通行限速"字段或逻辑**（已全量 grep `SetSpeedLimit|speed_limit` 于 `traffic_rules/**/*.cc`，0 命中于这些规则）。
- 现状：唯一可用的"区域限速"机制是上面的 `speed_bump_speed_limit`（仅对 speed_bump overlap 生效），**无法直接复用于人行道/路口**。
- 实现建议（仅分析，未实施）：需修改 `Crosswalk::ApplyRule` / `TrafficLight::MakeDecisions` 增加 `reference_line_info->SetSpeedLimit(start_s, end_s, 5.0)` 逻辑，或新增 TrafficRule 插件；纯配置无法满足。

---

## 11. 关键参数汇总表（赛题导向）

| 规则 | 参数 | 当前值 | proto 默认 | 是否偏离默认 | 赛题目标 | 结论 |
|------|------|--------|-----------|:---:|---------|------|
| CROSSWALK | `stop_distance` | 1.0 | 1.0 | 否 | 停车 1.5–2.0m | ⚠️ 需调大 |
| CROSSWALK | `stop_strict_l_distance` | **5.0** | 4.0 | ✅ 改大 | 检测灵敏度 | 已被人为改大 |
| CROSSWALK | `stop_timeout` | 10.0 | 10.0 | 否 | 静止行人放行 | 可调小加速放行 |
| CROSSWALK | `max_stop_deceleration` | 4.0 | 4.0 | 否 | 刹车容忍 | 一般不动 |
| TRAFFIC_LIGHT | `stop_distance` | 1.0 | 1.0 | 否 | 停车 1.5–2.0m | ⚠️ 需调大 |
| TRAFFIC_LIGHT | `max_stop_deceleration` | 4.0 | 4.0 | 否 | 刹车容忍 | 一般不动 |
| STOP_SIGN | `stop_distance` | 1.0 | 1.0 | 否 | 停车 1.5–2.0m | ⚠️ 需调大 |
| YIELD_SIGN | `stop_distance` | 1.0 | 1.0 | 否 | — | 一般不动 |
| BACKSIDE_VEHICLE | `backside_lane_width` | 4.0 | 4.0 | 否 | — | 一般不动 |
| DESTINATION | `stop_distance` | **0.2** | 0.5 | ✅ 改小 | 终点停车 | 已被人为改小 |
| KEEP_CLEAR | 4 项 | — | — | 否 | — | 一般不动 |
| REFERENCE_LINE_END | `stop_distance`/`min_remain` | 0.5/50.0 | 0.5/50.0 | 否 | — | 一般不动 |
| REROUTING | `cooldown`/`prepare` | 3.0/2.0 | 3.0/2.0 | 否 | — | 一般不动 |
| （gflag） | `speed_bump_speed_limit` | **3** | 4.4704 | ✅ 改小 | 减速带 ≤3m/s | profile 已配 ✅ |
| （gflag） | `default_cruise_speed` | 11.18 | 5.0 | ✅ 改大 | 巡航 | profile 已配 |
| （gflag） | `planning_upper_speed_limit` | 20.0 | 31.3 | ✅ 改小 | 全局上限 | profile 已配 |

**最关键参数（3 个）**：
1. `crosswalk.stop_distance`（1.0 → 需 1.5–2.0）— 人行道停车距离（有人/无人场景均相关）
2. `traffic_light.stop_distance`（1.0 → 需 1.5–2.0）— 红灯停车距离
3. `stop_sign.stop_distance`（1.0 → 需 1.5–2.0）— 停止牌停车距离

**重要缺口**：无人行道 ≤5m/s、绿灯路口 ≤5m/s 在 TrafficRule 中**无对应参数/代码**，需改 C++ 源码实现，改配置无效。

---

## 附录 A：代码位置索引

| 逻辑 | 文件 |
|------|------|
| 停车墙构建（stop_distance 应用） | `modules/planning/planning_base/common/util/common.cc` `BuildStopDecision`（`stop_s = stop_line_s - stop_distance`） |
| 刹停减速度计算 | `modules/planning/planning_base/common/util/util.cc` `GetADCStopDeceleration`（`v²/(2d)`） |
| 减速带限速施加 | `modules/planning/planning_base/common/reference_line_info.cc`（`AddSpeedLimit(start_s-1, end_s+1, FLAGS_speed_bump_speed_limit)`） |
| 规则基类配置加载 | `modules/planning/planning_base/common/traffic_rule.cc` `TrafficRule::LoadConfig` |
| 相关 gflags | `modules/planning/planning_base/gflags/planning_gflags.cc`（`speed_bump_speed_limit`=4.4704、`destination_check_distance`=5.0、`virtual_stop_wall_length`=0.1、`destination_obstacle_id`="DEST"、`default_cruise_speed`=5.0、`planning_upper_speed_limit`=31.3） |

## 附录 B：profile 层当前生效配置（profiles/default/modules/planning/planning_component/conf/planning.conf）

```text
--planning_upper_speed_limit=20.00
--default_cruise_speed=11.18
--destination_check_distance=4.0
--speed_bump_speed_limit=3
（其余 TrafficRule 参数均来自源码 conf/default_conf.pb.txt）
```
