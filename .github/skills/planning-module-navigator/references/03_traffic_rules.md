# 交通规则插件（TrafficRules）详解

TrafficRules 是 Planning 模块中作用于**所有场景**的通用交通规则插件，并行执行，每个规则独立生成交通决策（如停车、让行、限速）。

---

## 1. Crosswalk — 人行横道避让

**路径**：`modules/planning/traffic_rules/crosswalk/`

**功能**：在人行横道处生成虚拟障碍物，礼让行人优先通行。

**模块流程**：
1. 遍历参考线上的人行横道信息，若自车车头已越过则忽略
2. 遍历障碍物信息，满足以下条件需停车让行：
   - 属于行人和非机动车类型
   - 在人行横道区域范围内
   - 障碍物要横穿自车 path 或在自车前方道路范围内
   - 自车停车所需减速度小于最大允许减速度
3. 在需要停车的人行横道处生成虚拟障碍物

**关键参数**：
| 配置项 | 说明 |
|--------|------|
| `stop_distance` | 虚拟障碍物距人行横道的距离 |
| `max_stop_deceleration` | 判断停车时允许的最大减速度 |
| `min_pass_s_distance` | 判断自车是否越过人行横道的距离 |
| `stop_strict_l_distance` | 障碍物 l 方向小于此阈值时停车 |
| `stop_loose_l_distance` | 障碍物 l 方向大于此阈值时若横穿 path 则停车 |
| `stop_timeout` | 障碍物静止超过此阈值不再让行 |

**配置注册**：在 `traffic_rule_config.pb.txt` 中添加 `type: "Crosswalk"`

> 来源：`modules/planning/traffic_rules/crosswalk/README_cn.md`

---

## 2. StopSign — 停止标志决策

**路径**：`modules/planning/traffic_rules/stop_sign/`

**功能**：对地图中的停止标记生成停车交通决策。

**触发条件**：
- 主车前方参考线存在停止标记
- 停止标记未在 injector 中标记为 done

**关键参数**：
| 配置项 | 说明 |
|--------|------|
| `enabled` | 是否使能 |
| `stop_distance` | 停止线前停车距离 |

> 来源：`modules/planning/traffic_rules/stop_sign/README_cn.md`

---

## 3. YieldSign — 让行标志决策

**路径**：`modules/planning/traffic_rules/yield_sign/`

**功能**：对地图中的让行标记生成让行交通决策。

**触发条件**：
- 主车前方参考线存在让行标记
- 让行标记未在 injector 中标记为 done

**关键参数**：
| 配置项 | 说明 |
|--------|------|
| `enabled` | 是否使能 |
| `stop_distance` | 停止线前停车距离 |

> 来源：`modules/planning/traffic_rules/yield_sign/README_cn.md`

---

## 4. Destination — 终点停车决策

**路径**：`modules/planning/traffic_rules/destination/`

**功能**：根据全局路由的终点位置生成规划终点的停车决策墙。如果是 pull over 场景则生成 pull over 停车决策墙。

**核心逻辑（MakeDecisions）**：
- 根据路由终点在 SL 坐标系计算 S 距离
- 若终点超出参考线长度则输出警告但仍生成停车决策
- 根据终点位置减去安全距离和虚拟墙距离生成停车决策
- 若终点是 pull over 场景终点则生成 pull over 停车墙
- 若自车 S 距离大于终点 S 距离则不生成

> 来源：`modules/planning/traffic_rules/destination/README_cn.md`

---

## 5. ReferenceLineEnd — 参考线末端停车

**路径**：`modules/planning/traffic_rules/reference_line_end/`

**功能**：在参考线末端生成虚拟障碍物停止墙。当自车距参考线末端在阈值内时，在末端前 `virtual_stop_wall_length` 距离生成停车决策。

**触发条件**：自车到参考线末端剩余距离 < `min_reference_line_remain_length`

**关键参数**：
| 配置项 | 说明 |
|--------|------|
| `min_reference_line_remain_length` | 触发距离阈值 |
| `stop_distance` | 虚拟障碍物前停车距离 |

> 来源：`modules/planning/traffic_rules/reference_line_end/README_cn.md`

---

## 6. KeepClear — 禁停区域

**路径**：`modules/planning/traffic_rules/keepclear/`

**功能**：在禁停区域生成虚拟静止障碍物，避免自车进入禁停区域。

**模块流程**：
- 遍历参考线上的禁停区域，若自车已进入则继续行驶
- 若未进入则在禁停区域起始位置生成虚拟静止障碍物
- 开启路口禁停开关时，将路口区域视为禁停区域（非蠕行状态下）

> 来源：`modules/planning/traffic_rules/keepclear/README_cn.md`

---

## 7. Rerouting — 换道失败重路由

**路径**：`modules/planning/traffic_rules/rerouting/`

**功能**：在换道未成功时触发重新路由。

**触发条件（ChangeLaneFailRerouting）**：
- 当前车道不是直行车道
- 车辆在当前参考线车道内
- 当前车道通路没有出口
- 上次重路由已过一定时间
- 主车距离当前通路很近

> 来源：`modules/planning/traffic_rules/rerouting/README_cn.md`

---

## 8. SpeedSetting — 巡航速度动态调整

**路径**：`modules/planning/traffic_rules/speed_setting/`

**功能**：实时处理修改巡航速度的外部命令（`apollo::external_command::SpeedCommand`），允许动态调整巡航速度。

**处理逻辑（ApplyRule）**：
- 仅在收到新的 SpeedCommand 时处理
- 收到导航命令 → 使用新导航命令的基础巡航速度
- 收到 SpeedCommand → 按其设置巡航速度
- 收到恢复命令 → 恢复原始设定巡航速度
- 收到比例命令 → 将上次速度乘以比例值

> 来源：`modules/planning/traffic_rules/speed_setting/README_cn.md`

---

## 9. BacksideVehicle — 后方来车处理

**路径**：`modules/planning/traffic_rules/backside_vehicle/`

**功能**：处理后方来车的交通规则。

---

## 10. TrafficLight — 交通灯规则

**路径**：`modules/planning/traffic_rules/traffic_light/`

**功能**：处理交通灯相关的通用交通规则决策。

---

## TrafficRule 开发流程

1. 在 `modules/planning/traffic_rules/` 下新建目录
2. 创建 TrafficRule 类（实现 `ApplyRule` 接口）
3. 定义配置 proto 和 default_conf.pb.txt
4. 配置 plugins.xml
5. 在 `planning_component/conf/traffic_rule_config.pb.txt` 中注册：
   ```
   rule { name: "XXX" type: "XxxClassName" }
   ```

> 来源：各 TrafficRule 子模块 README_cn.md
