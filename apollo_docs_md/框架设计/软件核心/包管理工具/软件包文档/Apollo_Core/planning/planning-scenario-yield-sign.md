---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2scenarios_2yield__sign_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-scenario-yield-sign
---

# README_cn

# planning-scenario-yield-sign

## 简介

`YieldSignScenario`场景可以在有让行标记的场景减速观望，然后慢速通过。

## 目录结构

modules/planning/scenarios/yield_sign/
├── BUILD
├── conf
│   ├── pipeline.pb.txt
│   └── scenario_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── yield_sign.proto
├── README_cn.md
├── stage_approach.cc
├── stage_approach.h
├── stage_approach_test.cc
├── stage_creep.cc
├── stage_creep.h
├── stage_creep_test.cc
├── yield_sign_scenario.cc
├── yield_sign_scenario.h
└── yield_sign_scenario_test.cc
fragment

## 模块

### YieldSignScenario插件

[apollo::planning::YieldSignScenario](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1YieldSignScenario.html)

#### 场景切入条件

1. 车辆前方参考线上的第一个overlap是停止标记类型的overlap
1. 距离停止标记overlap的距离小于start_stop_sign_scenario_distance

#### 阶段

| 阶段名 | 类型 | 描述 |
| --- | --- | --- |
| YIELD_SIGN_APPROACH | apollo::planning::YieldSignStageApproach | 停让行标记前停车避让运动障碍物 |
| YIELD_SIGN_CREEP | apollo::planning::YieldSignStageCreep | 跛行通过让行区 |

#### 配置

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/scenarios/yield_sign/conf/scenario_conf.pb.txt | apollo::planning::ScenarioYieldSignConfig | 场景的配置文件 |
| modules/planning/scenarios/yield_sign/conf/pipeline.pb.txt | apollo::planning::ScenarioPipeline | 场景的流水线文件 |
| modules/planning/planning_component/conf/planning_config.pb.txt | apollo::planning::PlanningConfig | planning组件的配置文件 |

#### Flags

| 文件路径 | 说明 |
| --- | --- |
| modules/planning/planning_component/conf/planning.conf | planning模块的flag配置文件 |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。