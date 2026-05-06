# 场景插件（Scenarios）详解

Planning 模块共 17 个场景插件，每个场景处理一种特定的驾驶用例。

---

## 1. LaneFollowScenario — 车道保持场景（默认场景）

**路径**：`modules/planning/scenarios/lane_follow/`

**功能**：默认的自动驾驶场景。车辆沿着路由线路行驶，遇到障碍物在当前车道内绕行或借道相邻车道绕行，根据 routing 信息换道行驶。对停止标志、让行标志、人行道、减速带等根据交通规则进行减速或停止让行。

**切入条件（IsTransferable）**：
1. Frame 规划命令中存在车道跟随命令
2. Frame 的参考线信息不为空
3. 其他场景为空（默认 fallback）

**Stage**：
- `LANE_FOLLOW`：在 reference_line_info 中寻找可驾驶线路，遍历所有参考线调用 `PlanOnReferenceLine` 规划

> 来源：`modules/planning/scenarios/lane_follow/README_cn.md`

---

## 2. PullOverScenario — 靠边停车场景

**路径**：`modules/planning/scenarios/pull_over/`

**功能**：当车辆到达终点附近时自动切入靠边停车。

**切入条件**：
1. 当前 command 为 `lane_follow_command`
2. 参考线信息不为空
3. `FLAGS_enable_pull_over_at_destination` = true
4. 主车不处于变道状态
5. 主车距离目标点满足靠边停车距离阈值
6. 不处于 overlap
7. 最右侧车道允许靠边停车

**Stages**：
| Stage | 描述 |
|-------|------|
| `PULL_OVER_APPROACH` | 靠近靠边停车点，使用 task 列表进行规划 |
| `PULL_OVER_RETRY_APPROACH_PARKING` | 直接靠边失败后重试接近 |
| `PULL_OVER_RETRY_PARKING` | 执行 openspace 轨迹规划完成停车 |

> 来源：`modules/planning/scenarios/pull_over/README_cn.md`

---

## 3. BareIntersectionUnprotectedScenario — 无保护裸露路口

**路径**：`modules/planning/scenarios/bare_intersection_unprotected/`

**功能**：处理既没有停止标志也没有交通灯的路口。车辆在路口前一段距离范围内切换到此场景。

**切入条件**：
- 当前 command 为 `lane_follow_command`
- 参考线不为空
- 主车距离 `pnc_junction_overlap` 小于 `start_bare_intersection_scenario_distance`

**Stages**：
| Stage | 描述 |
|-------|------|
| `BARE_INTERSECTION_UNPROTECTED_APPROACH` | 减速接近路口，设置限速，检查路权 |
| `BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE` | 通过路口 |

> 来源：`modules/planning/scenarios/bare_intersection_unprotected/README_cn.md`

---

## 4. TrafficLightProtectedScenario — 有保护交通灯路口

**路径**：`modules/planning/scenarios/traffic_light_protected/`

**功能**：处理有明确交通灯指示的路口（前行、左转、右转都有明确指示）。

---

## 5. TrafficLightUnprotectedLeftTurnScenario — 无保护左转

**路径**：`modules/planning/scenarios/traffic_light_unprotected_left_turn/`

**功能**：交通灯路口左转时仍可能有对向车辆通过，需要增加减速慢行（Creep）阶段观察对向车辆并让行。

---

## 6. TrafficLightUnprotectedRightTurnScenario — 无保护右转

**路径**：`modules/planning/scenarios/traffic_light_unprotected_right_turn/`

**功能**：右转时需要缓行并观察红绿灯情况，在安全的前提下右转。

---

## 7. StopSignUnprotectedScenario — 无保护停止标志路口

**路径**：`modules/planning/scenarios/stop_sign_unprotected/`

**功能**：处理双向/四向停止标志路口，车辆在通过前需观察路口来往车辆，通行密度较小时才通过。

---

## 8. YieldSignScenario — 让行标志路口

**路径**：`modules/planning/scenarios/yield_sign/`

**功能**：遇到让行标志时，对向有车辆时需要先让对向车辆先行。

---

## 9. EmergencyStopScenario — 紧急停车

**路径**：`modules/planning/scenarios/emergency_stop/`

**功能**：触发紧急停车时在当前车道内尽快安全停止。

---

## 10. EmergencyPullOverScenario — 紧急靠边停车

**路径**：`modules/planning/scenarios/emergency_pull_over/`

**功能**：收到 `PULL_OVER` 命令时，就近找到合适位置在当前车道内停车，保证行驶过程安全。

---

## 11. ValetParkingScenario — 泊车入库

**路径**：`modules/planning/scenarios/valet_parking/`

**功能**：当 routing 中包含 `parking_id` 信息时，将车辆泊入指定停车位。

---

## 12. ValetParkingParkScenario — 泊车停车

**路径**：`modules/planning/scenarios/valet_parking_park/`

**功能**：配合泊车入库场景，完成最终停车操作。

---

## 13. ParkAndGoScenario — 停车后起步

**路径**：`modules/planning/scenarios/park_and_go/`

**功能**：停车后重新起步行驶的场景处理。

---

## 14. LargeCurvatureScenario — 大曲率弯道

**路径**：`modules/planning/scenarios/large_curvature/`

**功能**：处理大曲率弯道路段，可能需要特殊的速度和路径规划策略。

---

## 15. FreeSpaceScenario — 自由空间行驶

**路径**：`modules/planning/scenarios/free_space/`

**功能**：处理无明确车道线的自由空间（如停车场内部道路）。

---

## 16. LaneFollowParkScenario — 车道保持停车

**路径**：`modules/planning/scenarios/lane_follow_park/`

**功能**：结合车道保持和停车功能。

---

## 17. SquareScenario — 广场场景

**路径**：`modules/planning/scenarios/square/`

**功能**：处理广场等开放区域的行驶。

---

## 场景开发流程

1. 在 `modules/planning/scenarios/` 下新建目录
2. 创建 Scenario 类和 Stage 类
3. 定义配置 proto 和 default_conf.pb.txt
4. 配置 pipeline.pb.txt（定义 Stage 顺序和每个 Stage 的 Task 列表）
5. 在 `planning_component/conf/scenario_config.pb.txt` 中注册

> 来源：各场景子模块 README_cn.md
