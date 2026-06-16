---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2scenarios_2stop__sign__unprotected_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-scenario-stop-sign-unprotected
---

# README_cn

# planning-scenario-stop-sign-unprotected

## 简介

`StopSignUnprotectedScenario`场景可以在高精地图中有停止标记的路口时停车，观望周边车辆，等待周围车辆驶离后跛行，再快速通过路口。

## 目录结构

modules/planning/scenarios/stop_sign_unprotected/
├── BUILD
├── conf
│   ├── pipeline.pb.txt
│   ├── scenario_conf.pb.txt
│   └── stop_sign_unprotected_intersection_cruise
├── context.h
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── stop_sign_unprotected.proto
├── README_cn.md
├── stage_creep.cc
├── stage_creep.h
├── stage_creep_test.cc
├── stage_intersection_cruise.cc
├── stage_intersection_cruise.h
├── stage_pre_stop.cc
├── stage_pre_stop.h
├── stage_pre_stop_test.cc
├── stage_stop.cc
├── stage_stop.h
├── stage_stop_test.cc
├── stop_sign_unprotected_scenario.cc
├── stop_sign_unprotected_scenario.h
└── stop_sign_unprotected_scenario_test.cc
fragment

## 模块

### StopSignUnprotectedScenario插件

[apollo::planning::StopSignUnprotectedScenario](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1StopSignUnprotectedScenario.html)

#### 场景切入条件

1. 车辆前方参考线上的第一个overlap是停止标记类型的overlap
1. 距离停止标记overlap的距离小于start_stop_sign_scenario_distance

#### 阶段

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| STOP_SIGN_UNPROTECTED_PRE_STOP | apollo::planning::StopSignUnprotectedStagePreStop | 停止标记前停车阶段 |
| STOP_SIGN_UNPROTECTED_STOP | apollo::planning::StopSignUnprotectedStageStop | 停车观望阶段，等待周围车辆远离继续行驶 |
| STOP_SIGN_UNPROTECTED_CREEP | apollo::planning::StopSignUnprotectedStageCreep | 跛行阶段 |
| STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE | apollo::planning::StopSignUnprotectedStageIntersectionCruise | 快速通过路口阶段 |

#### 配置

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/scenarios/stop_sign_unprotected/conf/scenario_conf.pb.txt | apollo::planning::ScenarioStopSignUnprotectedConfig | 场景的配置文件 |
| modules/planning/scenarios/stop_sign_unprotected/conf/pipeline.pb.txt | apollo::planning::ScenarioPipeline | 场景的流水线文件 |
| modules/planning/planning_component/conf/planning_config.pb.txt | apollo::planning::PlanningConfig | planning组件的配置文件 |

#### Flags

| 文件路径 | 说明 |
| --- | --- |
| modules/planning/planning_component/conf/planning.conf | planning模块的flag配置文件 |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。