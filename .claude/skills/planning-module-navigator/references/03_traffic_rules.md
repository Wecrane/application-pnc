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

---

## 6. KeepClear — 禁止停车区域

**路径**：`modules/planning/traffic_rules/keepclear/`

**功能**：标记路口等禁止停车区域，确保车辆不在这些区域停车。

---

## 7. Rerouting — 换道失败重路由

**路径**：`modules/planning/traffic_rules/rerouting/`

**功能**：当车辆换道行驶失败时，触发重路由请求，重新规划全局路径。

---

## 8. SpeedSetting — 动态巡航速度

**路径**：`modules/planning/traffic_rules/speed_setting/`

**功能**：根据道路类型、车道类型、限速标志等信息动态调整巡航速度。

---

## 9. BacksideVehicle — 后方来车处理

**路径**：`modules/planning/traffic_rules/backside_vehicle/`

**功能**：处理后方来车情况，对换道等决策产生影响。

---

## 10. TrafficLight — 交通灯规则

**路径**：`modules/planning/traffic_rules/traffic_light/`

**功能**：处理交通灯信号，生成对应的停车决策。

---

## TrafficRule 开发模板

新建 TrafficRule 插件的最小结构：

```cpp
class MyTrafficRule : public TrafficRule {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;
  common::Status MakeDecisions(Frame* frame,
                               ReferenceLineInfo* reference_line_info) override;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::MyTrafficRule,
                                     apollo::planning::TrafficRule)
```

配置注册（`traffic_rule_config.pb.txt`）：
```
rule {
  name: "MY_RULE"
  type: "MyTrafficRule"
}
```

> 来源：各子模块 README_cn.md
