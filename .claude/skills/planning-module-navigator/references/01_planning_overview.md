# Planning 模块总体架构

## 模块介绍

`planning` 是 planning 模块的主要流程和入口 package，包含 planning 模块的整体架构和流程。planning 模块根据上游模块输入的感知周围环境信息、地图定位导航信息以及全局路径信息，为自动驾驶车辆规划出一条运动轨迹（包含坐标、速度、加速度、jerk 加加速度、时间等信息），然后将这些信息传递给控制模块。

> 来源：`modules/planning/planning_component/README_cn.md`

## 目录结构

```shell
modules/planning/
├── planning_component/        # Planning 组件类和程序启动及配置
│   ├── conf/                  # 配置文件（planning.conf, scenario_config.pb.txt, traffic_rule_config.pb.txt）
│   ├── dag/                   # DAG 文件
│   └── docs/                  # 文档和图片
├── planning_interface_base/   # 插件父类接口（Scenario/Task/TrafficRule/Planner）
├── planning_base/             # 基础数据结构和算法库
├── planner/                   # 规划器子类插件
├── pnc_map/                   # PnC 地图，生成参考线
├── scenarios/                 # 场景插件（17个）
├── tasks/                     # 任务插件（30+个）
└── traffic_rules/             # 交通规则插件（10个）
```

> 来源：`modules/planning/planning_component/README_cn.md`

## 双层状态机机制

Planning 模块从 Apollo 3.5 开始使用双层状态机的场景机制：
- **Top Layer（Scenario 状态机）**：ScenarioManager 根据周围环境和地图信息决定切换到哪个场景
- **Bottom Layer（Stage 状态机）**：场景内按 Stage 顺序执行，所有 Stage 执行完毕后当前 Scenario 也执行完毕

### 场景切换逻辑

每个 Scenario 实现 `IsTransferable()` 判断是否可从其他场景切入，`ExitCondition` 判断当前场景是否完成可退出。

> 来源：`modules/planning/planning_component/README_cn.md`

## 插件类型与加载机制

| 插件类型 | 接口基类 | 作用范围 | 配置加载文件 |
|----------|---------|---------|------------|
| Scenario | `Scenario` | 全局切换，同时只有一个生效 | `planning_component/conf/scenario_config.pb.txt` |
| TrafficRule | `TrafficRule` | 所有场景通用，并行执行 | `planning_component/conf/traffic_rule_config.pb.txt` |
| Task | `Task` | 在 Stage 中按顺序依次执行 | 各场景 `conf/pipeline.pb.txt` |
| Planner | `Planner` | 全局规划器，执行核心轨迹规划 | `planning_component/conf/planning.conf` |

### 配置加载方式

**加载 TrafficRule 插件**：在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 中添加：
```
rule {
  name: "CROSSWALK"
  type: "Crosswalk"
}
```

**加载 Scenario 插件**：在 `modules/planning/planning_component/conf/scenario_config.pb.txt` 中添加：
```
scenario {
  name: "LANE_FOLLOW"
  type: "LaneFollowScenario"
}
```

**加载 Task 插件**：在各 Scenario 的 `conf/pipeline.pb.txt` 的对应 Stage 中添加：
```
task {
  name: "LANE_FOLLOW_PATH"
  type: "LaneFollowPath"
}
```

> 来源：各子模块 README_cn.md

## 关键概念

### ReferenceLine（参考线）
由 `pnc_map` 模块根据 planning 导航命令或地图信息生成，作为 planning 局部路径规划的路线参考。每条参考线关联 `ReferenceLineInfo`，包含车道信息、障碍物投影、路径/速度数据等。

### Frame
每帧 planning 的核心数据结构，包含：
- 自车状态（位置、速度、航向）
- 参考线列表
- 障碍物列表
- 规划上下文（起始点、目标点）

### Path + Speed = Trajectory
Planning 将规划分为：
1. **Path（路径）**：确定车辆在 SL 坐标系下的横向运动
2. **Speed（速度）**：确定车辆沿路径的速度剖面
3. 二者组合生成完整的 **Trajectory（轨迹）**

> 来源：`modules/planning/planning_component/README_cn.md`、`modules/planning/tasks/lane_borrow_path_generic/README_cn.md`

## Planning 主入口

```bash
# 单模块启动 Planning
mainboard -d /apollo/modules/planning/planning_component/dag/planning.dag
```
