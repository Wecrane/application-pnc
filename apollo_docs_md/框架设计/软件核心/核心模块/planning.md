---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_collection_2planning_2README__cn.html
category: 框架设计 > 软件核心 > 核心模块 > planning
---

# README_cn

# module-planning

## 模块概述

`planning` 模块包含planning模块的整体架构和流程。planning模块根据上游模块输入的感知周围环境信息，地图定位导航信息，以及全局路径信息，为自动驾驶车辆规划出一条运动轨迹（包含坐标，速度，加速度，jerk加加速度，时间等信息），然后将这些信息传递给控制模块。

## 输入输出

### 输入

Planning模块需要获取外部环境信息，车辆自身信息进行轨迹规划，以下是planning的外部输入信息：

| Channel 名 | 类型 | 描述 |
| --- | --- | --- |
| /apollo/prediction | apollo::prediction::PredictionObstacles | 障碍物预测信息，可通过 modules/planning/planning_component/dag/planning.dag 启动文件修改channel名 |
| /apollo/perception/traffic_light | apollo::perception::TrafficLight | perception模块输出的交通灯感知信息，包含交通灯亮起的颜色，id等信息 |
| /apollo/localization/pose | apollo::localization::LocalizationEstimate | 定位信息，可通过 modules/planning/planning_component/dag/planning.dag 配置文件修改channel名 |
| /apollo/canbus/chassis | apollo::canbus::Chassis | canbus模块输出的车辆底盘信息，包含底盘速度，油门，刹车，档位，灯光等状态， modules/planning/planning_component/dag/planning.dag 配置文件修改channel名 |

此外，planning模块还需要外部输入的导航命令信息，用户首先向external_command发送导航命令请求，external_command再将这些命令进行处理后转发给planning模块。下面介绍用户可以发送的几种导航命令：

| Channel 名 | 类型 | 描述 |
| --- | --- | --- |
| /apollo/external_command/lane_follow | apollo::external_command::LaneFollowCommand | 基于高精地图导航的命令，给定终点的位置或朝向，从当前车辆位置导航到目标终点位置 |
| /apollo/external_command/valet_parking | apollo::external_command::ValetParkingCommand | 从当前位置导航泊车到停车位上 |
| /apollo/external_command/action | apollo::external_command::ActionCommand | HMI发送的流程操作命令 |

### 输出

| Channel 名 | 类型 | 描述 |
| --- | --- | --- |
| /apollo/planning | apollo::planning::ADCTrajectory | 输出规划轨迹，包含轨迹点，速度和时间等信息 |
| /apollo/planning/command_status | apollo::external_command::CommandStatus | 导航命令的执行状态 |
| /apollo/external_command/lane_follow | apollo::external_command::LaneFollowCommand | 在道路被阻塞，换道失败超时时，发送重新路由的申请 |

## 参数

### 配置

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/planning_component/conf/planning_config.pb.txt | apollo::planning::PlanningConfig | planning组件的配置文件 |
| modules/planning/planning_component/conf/public_road_planner_config.pb.txt | apollo::planning::PlannerOpenSpaceConfig | PublicRoadPlanner的配置文件 |
| modules/planning/planning_component/conf/traffic_rule_config.pb.txt | apollo::planning::TrafficRulesPipeline | 支持的traffic rules列表的配置文件 |
| modules/planning/planning_component/conf/discrete_points_smoother_config.pb.txt | apollo::planning::ReferenceLineSmootherConfig | 参考线使用离散点平滑时的配置文件 |
| modules/planning/planning_component/conf/qp_spline_smoother_config.pb.txt | apollo::planning::ReferenceLineSmootherConfig | 参考线使用五次多项式平滑时的配置文件 |
| modules/planning/planning_component/conf/spiral_smoother_config.pb.txt | apollo::planning::ReferenceLineSmootherConfig | 参考线使用五次螺旋线平滑时的配置文件 |
| modules/planning/planning_component/conf/planner_open_space_config.pb.txt | apollo::planning::PlannerOpenSpaceConfig | 开放空间规划算法的配置文件 |

### Flags

| 文件路径 | 说明 |
| --- | --- |
| modules/planning/planning_component/conf/planning.conf | planning模块的flag配置文件 |

## 包列表

| 文件路径 | 说明 |
| --- | --- |
| planning | planning组件包，负责启动planning模块，串联planning运行流程 |
| planning-interface-base | 包含planning插件父类接口 |
| planning-base | 包含planning模块基础数据结构和算法库 |
| planning-open-space | 包含开放空间规划算法 |
| planning-planner-lattice | LatticePlanner插件包，适用于高速公路等简单场景 |
| planning-planner-navi | Navi Planner插件包，适用于高速公路等简单场景 |
| planning-planner-rtk | Rtk Panner插件包，根据事先录制好的轨迹进行循迹行驶 |
| planning-planner-public-road | PublicRoad Planner插件包，适用于城市道路的场景 |
| planning-lane-follow-map | LaneFollowMap插件包，基于高精地图生成参考线 |
| planning-scenario-bare-intersection-unprotected | 无保护路口场景插件包，处理没有交通灯的交叉路口场景 |
| planning-scenario-emergency-pull-over | 紧急靠边停车场景插件包，接收紧急停靠的命令并就近靠边停车 |
| planning-scenario-emergency-stop | 紧急停车场景插件包，接收紧急停车的命令并在当前车道停车 |
| planning-scenario-lane-follow | 沿车道线行驶场景插件包，是默认的行驶场景 |
| planning-scenario-park-and-go | 停车后重新行驶场景插件包，在车辆停靠在路边非车道线范围内，重新行驶时时会先进入此场景，将车辆行驶到车道线上 |
| planning-scenario-pull-over | 靠边停车场景插件包，在车辆接近终点并且终点靠边停车使能后进入此场景 |
| planning-scenario-stop-sign-unprotected | 停车标志场景插件包，在行驶过程中遇到停止标志时进入此场景 |
| planning-scenario-traffic-light-protected | 有保护交通灯场景插件包，在路口有保护的交通标志时进入此场景 |
| planning-scenario-traffic-light-unprotected-left-turn | 无保护交通灯左转场景插件包，在路口无保护的交通灯左转时进入此场景 |
| planning-scenario-traffic-light-unprotected-right-turn | 无保护交通灯右转场景插件包，在路口无保护的交通灯右转时进入此场景 |
| planning-scenario-valet-parking | 在停车位停车场景插件包，在导航命令中指定了停车位并且车辆接近停车位时进入此场景 |
| planning-scenario-yield-sign | 让行标志场景插件包，在行驶时遇到让行标志时进入此场景 |
| planning-task-fallback-path | 备用路径任务插件包，在沿道路行驶时无法规划出合理的路径时，此任务产生备用路径，沿当前车道线行驶并在障碍物前停车 |
| planning-task-lane-borrow-path | 借道路径任务插件包，在沿道路行驶时当前车道被占用时生成借道路径 |
| planning-task-lane-change-path | 换到路径任务插件包，在沿道路行驶时根据路由线路需要换道时生成换道路径 |
| planning-task-lane-follow-path | 沿当前车道任务插件包，在沿道路行驶时正常沿当前车道行驶时生成路径 |
| planning-task-open-space-fallback-decider | 开放空间行驶备用轨迹任务插件包，检测开放空间下轨迹是否与障碍物发生碰撞并做出相应策略 |
| planning-task-open-space-pre-stop-decider | 开放空间行驶预先停车任务插件包，执行泊车与靠边停车任务时，生成在公共道路上的停车点，车辆停在停车点后，会转入开放空间算法 |
| planning-task-open-space-roi-decider | 开放空间行驶可行区域生成任务插件包，在开放空间算法中生成可行驶边界 |
| planning-task-open-space-trajectory-partition | 开放空间行驶轨迹分割任务插件包，在开放空间将轨迹按前进后退分割成多段，并调整速度规划 |
| planning-task-open-space-trajectory-provider | 开放空间轨迹生成任务插件包，在指定规划边界与目标点位姿的情况下，调用开放空间算法，生成相应轨迹 |
| planning-task-path-decider | 路径决策任务插件包，对行驶路径进行决策 |
| planning-task-path-reference-decider | 路径合理性决策任务插件包，对推理生成的路径的合理性进行决策，用于后续求解 |
| planning-task-path-time-heuristic | 速度规划初解任务插件包，基于动态规划算法，在非凸的ST空间做粗的速度规划 |
| planning-task-piecewise-jerk-speed | 线性速度规划优化解任务插件包，基于二次规划在凸ST空间进行速度规划 |
| planning-task-piecewise-jerk-speed-nonlinear | 非线性速度规划优化解任务插件包，基于非线性规划在凸ST空间进行速度规划 |
| planning-task-pull-over-path | 靠边停车路径生成任务插件包，用于生成靠边停车路径 |
| planning-task-reuse-path | 复用路径任务插件包，在没有碰撞的情况下，复用上一帧的路径 |
| planning-task-rss-decider | RSS决策器任务插件包，用于判断planning给出的规划是否安全 |
| planning-task-rule-based-stop-decider | 停车策略任务插件包，用于产生基于规则的停车策略 |
| planning-task-speed-bounds-decider | 速度边界生成任务插件包，用于产生速度规划的ST可行驶区间 |
| planning-task-speed-decider | 速度规划决策任务插件包，用于根据规划的粗速度曲线产生对障碍物的纵向决策 |
| planning-task-st-bounds-decider | st边界决策器任务插件包，生成st边界 |
| planning-traffic-rules-backside-vehicle | 后向车辆规则插件包，生成对后向车辆的速度决策 |
| planning-traffic-rules-crosswalk | 人行道规则插件包，对前方人行道生成是否停车的决策 |
| planning-traffic-rules-destination | 终点规则插件包，在终点附近生成是否停车的决策 |
| planning-traffic-rules-keepclear | 禁行区域规则插件包，在禁行区域生成速度决策 |
| planning-traffic-rules-reference-line-end | 参考线终点规则插件包，在参考线终点附近生成是否停车的决策 |
| planning-traffic-rules-rerouting | 重新规划规则插件包，在规划失败的情况下决策是否要重新规划 |
| planning-traffic-rules-stop-sign | 停止标志插件包，在停止标志附近决策是否要停车 |
| planning-traffic-rules-traffic-light | 交通灯规则插件包，在交通灯附近决策是否要停车 |
| planning-traffic-rules-yield-sign | 让行规则插件包，在让行标志附近生成速度决策 |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。