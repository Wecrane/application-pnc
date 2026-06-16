# 06 - Planning 模块完整参考

> 来源：`apollo_docs_md/框架设计/软件核心/包管理工具/软件包文档/Apollo_Core/planning/`
> 共 **72 个** Planning 子模块文档，按体系分类整理

---

## Planning 模块概述

Planning（规划）模块是 Apollo 自动驾驶系统的核心模块之一。它根据上游模块（感知、预测、定位）输出的环境信息、地图信息和全局路径，为自动驾驶车辆规划出一条运动轨迹（包含坐标、速度、加速度、jerk、时间等），然后将轨迹传递给 Control（控制）模块执行。

### 模块目录结构

```
modules/planning/
├── planning_component/        # 主入口 & 启动配置
├── planning_interface_base/   # 插件父类接口（Scenario/Task/TrafficRule/Planner）
├── planning_base/             # 基础数据结构和算法库
├── planner/                   # 规划器子类插件（4种）
├── pnc_map/                   # PnC 地图，生成参考线
├── scenarios/                 # 场景插件（17个）
├── tasks/                     # 任务插件（25+个）
└── traffic_rules/             # 交通规则插件（10个）
```

### 双层状态机机制

```
Top Layer（Scenario）: ScenarioManager 决定当前场景
    └── Bottom Layer（Stage）: 场景内 Stage 按序执行
        └── 每个 Stage 包含多个 Task，依次执行
```

---

## 📋 Planning 模块文档索引

### 🔷 核心文档（必读）

| 文档 | 内容 | 路径 |
|------|------|------|
| **planning.md** | Planning 模块总览、架构、框架流程、场景机制 | `planning/planning.md` |
| **planning-base.md** | 基础数据结构和算法库 | `planning/planning-base.md` |
| **planning-lane-follow-map.md** | LaneFollow 场景使用的地图 | `planning/planning-lane-follow-map.md` |
| **routing.md** | Routing 路由模块说明 | `planning/routing.md` |
| **storytelling.md** | StoryTelling 模块说明 | `planning/storytelling.md` |

---

### 🔷 Planner 规划器（4个）

| 规划器 | 说明 | 路径 |
|--------|------|------|
| PublicRoadPlanner | **默认规划器**，基于高精地图 | `planning/planning-planner-public-road.md` |
| NaviPlanner | 基于实时相对地图 | `planning/planning-planner-navi.md` |
| LatticePlanner | 基于网格算法 | `planning/planning-planner-lattice.md` |
| RTKReplayPlanner | 基于录制轨迹回放 | `planning/planning-planner-rtk.md` |

---

### 🔷 Scenario 场景插件（17个）

每个场景对应一种驾驶场景，互斥运行。

#### 基础场景

| 场景 | 说明 | 路径 |
|------|------|------|
| **LaneFollow** | 🏠 **默认场景**，沿车道线行驶、绕行障碍物、变道 | `planning/planning-scenario-lane-follow.md` |
| **PullOver** | 靠边停车（到达目的地） | `planning/planning-scenario-pull-over.md` |
| **ParkAndGo** | 靠边停车后起步出发 | `planning/planning-scenario-park-and-go.md` |
| **EmergencyStop** | 紧急停车（立即刹停） | `planning/planning-scenario-emergency-stop.md` |
| **EmergencyPullOver** | 紧急靠边停车 | `planning/planning-scenario-emergency-pull-over.md` |

#### 路口场景

| 场景 | 说明 | 路径 |
|------|------|------|
| TrafficLightProtected | 有保护交通灯路口（直行/左转/右转） | `planning/planning-scenario-traffic-light-protected.md` |
| TrafficLightUnprotectedLeftTurn | 无保护交通灯左转（需观察让行） | `planning/planning-scenario-traffic-light-unprotected-left-turn.md` |
| TrafficLightUnprotectedRightTurn | 无保护交通灯右转 | `planning/planning-scenario-traffic-light-unprotected-right-turn.md` |
| StopSignUnprotected | 无保护停止标志路口 | `planning/planning-scenario-stop-sign-unprotected.md` |
| YieldSign | 让行标志路口 | `planning/planning-scenario-yield-sign.md` |
| BareIntersectionUnprotected | 无保护裸露路口（无信号灯/标志） | `planning/planning-scenario-bare-intersection-unprotected.md` |

#### 特殊场景

| 场景 | 说明 | 路径 |
|------|------|------|
| ValetParking | 自主泊车入库 | `planning/planning-scenario-valet-parking.md` |
| ValetParkingPark | 泊车入库执行 | `planning/planning-scenario-valet-parking-park.md` |
| FreeSpace | 自由空间规划 | `planning/planning-scenario-free-space.md` |
| LargeCurvature | 大曲率弯道 | `planning/planning-scenario-large-curvature.md` |
| LaneFollowPark | LaneFollow 泊车 | `planning/planning-scenario-lane-follow-park.md` |
| Square | 广场/开放区域 | `planning/planning-scenario-square.md` |

---

### 🔷 TrafficRule 交通规则插件（10个）

交通规则在所有场景中并行生效，用于生成交通决策约束。

| 交通规则 | 说明 | 路径 |
|----------|------|------|
| **Crosswalk** | 🚶 人行横道：检测行人，生成虚拟障碍物让行 | `planning/planning-traffic-rules-crosswalk.md` |
| **StopSign** | 🛑 停止标志：生成停止决策 | `planning/planning-traffic-rules-stop-sign.md` |
| **TrafficLight** | 🚦 交通灯：根据灯色决策 | `planning/planning-traffic-rules-traffic-light.md` |
| **YieldSign** | ⚠️ 让行标志：让行决策 | `planning/planning-traffic-rules-yield-sign.md` |
| **SpeedSetting** | 🚗 速度限制设置 | `planning/planning-traffic-rules-speed-setting.md` |
| **BacksideVehicle** | 🚙 后方来车处理 | `planning/planning-traffic-rules-backside-vehicle.md` |
| **Destination** | 🏁 目的地处理 | `planning/planning-traffic-rules-destination.md` |
| **KeepClear** | 📐 保持畅通区域 | `planning/planning-traffic-rules-keepclear.md` |
| **ReferenceLineEnd** | 🛣️ 参考线终点处理 | `planning/planning-traffic-rules-reference-line-end.md` |
| **Rerouting** | 🔄 重新路由请求 | `planning/planning-traffic-rules-rerouting.md` |

---

### 🔷 Task 任务插件（25个）

Task 在 Stage 中按序执行，分为路径规划和速度规划两类。

#### 路径规划 Task

| Task | 说明 | 路径 |
|------|------|------|
| LaneFollowPath | 车道跟随路径 | `planning/planning-task-lane-follow-path.md` |
| LaneChangePath | 变道路径 | `planning/planning-task-lane-change-path.md` |
| LaneChangePathGeneric | 通用变道路径 | `planning/planning-task-lane-change-path-generic.md` |
| LaneBorrowPath | 借道路径 | `planning/planning-task-lane-borrow-path.md` |
| LaneBorrowPathGeneric | 通用借道路径 | `planning/planning-task-lane-borrow-path-generic.md` |
| FallbackPath | 降级路径 | `planning/planning-task-fallback-path.md` |
| PathDecider | 路径决策器 | `planning/planning-task-path-decider.md` |
| PathReferenceDecider | 路径参考决策器 | `planning/planning-task-path-reference-decider.md` |

#### 速度规划 Task

| Task | 说明 | 路径 |
|------|------|------|
| FastStopTrajectoryFallback | 快速停止轨迹降级 | `planning/planning-task-fast-stop-trajectory-fallback.md` |

#### OpenSpace Task（自主泊车/自由空间）

| Task | 说明 | 路径 |
|------|------|------|
| OpenSpaceFallbackDecider | 降级决策 | `planning/planning-task-open-space-fallback-decider.md` |
| OpenSpacePreStopDecider | 预停止决策 | `planning/planning-task-open-space-pre-stop-decider.md` |
| OpenSpaceRoiDecider | ROI 区域决策 | `planning/planning-task-open-space-roi-decider.md` |
| OpenSpaceTrajectoryPartition | 轨迹分段 | `planning/planning-task-open-space-trajectory-partition.md` |
| OpenSpaceTrajectoryProvider | 轨迹提供 | `planning/planning-task-open-space-trajectory-provider.md` |

> 📂 完整 Task 列表（25个）：`planning/planning-task-*.md`，更多 Task 请直接在 `apollo_docs_md/框架设计/软件核心/包管理工具/软件包文档/Apollo_Core/planning/` 目录下检索。

---

### 🔷 ExternalCommand 外部命令（10个）

处理外部系统（如 Dreamview、云端）发来的命令。

| 命令 | 说明 | 路径 |
|------|------|------|
| LaneFollow | 车道跟随命令 | `planning/external-command-lane-follow.md` |
| FreeSpace | 自由空间命令 | `planning/external-command-free-space.md` |
| PathFollow | 路径跟随命令 | `planning/external-command-path-follow.md` |
| Speed | 速度命令 | `planning/external-command-speed.md` |
| Action | 动作命令 | `planning/external-command-action.md` |
| Chassis | 底盘命令 | `planning/external-command-chassis.md` |
| ValetParking | 泊车命令 | `planning/external-command-valet-parking.md` |
| ZoneCover | 区域覆盖命令 | `planning/external-command-zone-cover.md` |
| Process | 流程命令 | `planning/external-command-process.md` |
| ProcessorBase | 命令处理器基类 | `planning/external-command-processor-base.md` |

---

## 比赛开发指南

### 比赛规则约束

本次比赛 **只允许修改 Planning 模块**，不允许修改其他模块（Perception、Prediction、Control、Localization 等）。

### 你可以修改的内容

| 内容 | 位置 | 说明 |
|------|------|------|
| Scenario 插件 | `modules/planning/scenarios/` | 新增/修改场景 |
| Task 插件 | `modules/planning/tasks/` | 新增/修改任务 |
| TrafficRule 插件 | `modules/planning/traffic_rules/` | 新增/修改交通规则 |
| 配置参数 | `profiles/default/modules/planning/` | 调整参数 |
| Planner | `modules/planning/planner/` | 修改规划器 |

### 开发流程

```
1. 分析赛题 → 确定需要新增/修改的插件类型
2. 创建插件类（继承对应基类）
3. 注册到配置文件（scenario_config / traffic_rule_config / pipeline.pb.txt）
4. 编译验证：buildtool build -p modules/planning/ -j15 && aem profile use default
5. SimControl 仿真测试
6. 调参优化 → 打包提交
```

### 关键注意事项

- **编译后恢复 profile**：`aem profile use default`
- **下载 planning 源码**：`buildtool install planning*`
- **提交包格式**：`tar -zcvf 提交包.tar.gz modules/planning/ profiles/default`

---

## 文档快速查找

### 按功能查找

| 你要做什么 | 看哪个文档 |
|-----------|-----------|
| 了解 Planning 整体架构 | `planning.md` |
| 沿道路正常行驶 | `planning-scenario-lane-follow.md` |
| 靠边停车 | `planning-scenario-pull-over.md` |
| 处理交通灯路口 | `planning-scenario-traffic-light-protected.md` |
| 处理停止标志 | `planning-scenario-stop-sign-unprotected.md` |
| 处理让行标志 | `planning-scenario-yield-sign.md` |
| 处理人行横道 | `planning-traffic-rules-crosswalk.md` |
| 设置速度限制 | `planning-traffic-rules-speed-setting.md` |
| 紧急停车 | `planning-scenario-emergency-stop.md` |
| 泊车入库 | `planning-scenario-valet-parking.md` |
| 变道/借道 | `planning-task-lane-change-path.md` / `lane-borrow-path.md` |
| 无保护路口 | `planning-scenario-bare-intersection-unprotected.md` |
| 选择规划器 | `planning-planner-public-road.md` 等 |

### 按竞赛场景查找

| 竞赛场景 | 建议查阅 |
|----------|---------|
| 人行道避让 | `crosswalk` + `lane-follow` |
| 借道绕行 | `lane-borrow-path` + `lane-change-path` |
| 特殊车辆绕行 | `lane-follow` 障碍物绕行 |
| 施工区域减速 | `speed-setting` 限速 |
| 断头路 | `destination` + `reference-line-end` |
| 自主泊车 | `valet-parking` + `open-space-*` |
| 交通标志停止线 | `stop-sign` |
| 红绿灯路口减速 | `traffic-light` + `speed-setting` |
| 减速带通行 | `speed-setting` 限速 |
| 左转待转 | `traffic-light-unprotected-left-turn` |

---

## 完整文档路径

所有 72 个 Planning 文档位于：
```
apollo_docs_md/框架设计/软件核心/包管理工具/软件包文档/Apollo_Core/planning/
```

需要查看具体某文档时直接用文件名搜索即可，如 `planning-traffic-rules-crosswalk.md`。
