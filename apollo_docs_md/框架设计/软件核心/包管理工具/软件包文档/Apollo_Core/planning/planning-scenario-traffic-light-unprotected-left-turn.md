---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2scenarios_2traffic__light__unprotected__left__turn_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-scenario-traffic-light-unprotected-left-turn
---

# README_cn

# planning-scenario-traffic-light-unprotected-left-turn

## 简介

`TrafficLightUnprotectedLeftTurnScenario` 是没有路权保护的红绿灯左转场景。在该场景下，主车在左转车道线上

## 目录结构

modules/planning/scenarios/traffic_light_unprotected_left_turn/
├── BUILD
├── conf
│   ├── pipeline.pb.txt
│   ├── scenario_conf.pb.txt
│   └── traffic_light_unprotected_left_turn_intersection_cruise
├── context.h
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── traffic_light_unprotected_left_turn.proto
├── README_cn.md
├── stage_approach.cc
├── stage_approach.h
├── stage_approach_test.cc
├── stage_creep.cc
├── stage_creep.h
├── stage_creep_test.cc
├── stage_intersection_cruise.cc
├── stage_intersection_cruise.h
├── traffic_light_unprotected_left_turn_scenario.cc
├── traffic_light_unprotected_left_turn_scenario.h
└── traffic_light_unprotected_left_turn_scenario_test.cc
fragment

## 模块

### TrafficLightUnprotectedLeftTurnScenario插件

[apollo::planning::TrafficLightUnprotectedLeftTurnScenario](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1TrafficLightUnprotectedLeftTurnScenario.html)

#### 场景切入条件

1. 车辆前方参考线上的第一个overlap是交通灯类型的overlap
1. 距离停止标记overlap的距离小于start_traffic_light_scenario_distance
1. 前方overlap的信号灯存在红灯
1. 参考线overlap的车道类型为左转车道

#### 阶段

| 阶段名 | 类型 | 描述 |
| --- | --- | --- |
| TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_APPROACH | apollo::planning::TrafficLightUnprotectedLeftTurnStageApproach | 在红绿灯停止线前停车阶段 |
| TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP | apollo::planning::TrafficLightUnprotectedLeftTurnStageCreep | 绿灯后跛行观察路口来车 |
| TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE | apollo::planning::TrafficLightUnprotectedLeftTurnStageIntersectionCruise | 快速穿过红绿灯路口阶段 |

#### 配置

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/scenarios/traffic_light_unprotected_left_turn/conf/scenario_conf.pb.txt | apollo::planning::ScenarioTrafficLightUnprotectedLeftTurnConfig | 场景的配置文件 |
| modules/planning/scenarios/traffic_light_unprotected_left_turn/conf/pipeline.pb.txt | apollo::planning::ScenarioPipeline | 场景的流水线文件 |
| modules/planning/planning_component/conf/planning_config.pb.txt | apollo::planning::PlanningConfig | planning组件的配置文件 |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。