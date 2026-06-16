---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2tasks_2open__space__fallback__decider_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-task-open-space-fallback-decider
---

# README_cn

# planning-task-open-space-fallback-decider

## 简介

`OpenSpaceFallbackDecider`检测开放空间下轨迹是否与障碍物发生碰撞并做出相应策略。

### BuildPredictedEnvironment

根据预测输入的障碍物信息，构建出不同时间间隔的障碍物边界，将障碍物信息输入到`predicted_bounding_rectangles`中。

### IsCollisionFreeTrajectory

判断自车轨迹与障碍物轨迹是否没有碰撞，当返回为false，根据碰撞点位置，生成匀减速停车轨迹，使车辆停车。

### IsCollisionFreeEgoBox

判断车辆扩展后的box与静态障碍物是否有碰撞，当返回为false，车辆急刹并置fallback_flag为true，从而在task `OpenSpaceTrajectoryProvider`中重新规划轨迹。

## 目录结构

modules/planning/tasks/open_space_fallback_decider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── open_space_fallback_decider.cc
├── open_space_fallback_decider.h
├── open_space_fallback_decider_test.cc
├── plugins.xml
├── proto
│   ├── BUILD
│   └── open_space_fallback_decider.proto
└── README_cn.md
fragment

## 模块

### OpenSpaceFallbackDecider插件

[apollo::planning::OpenSpaceFallbackDecider](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1OpenSpaceFallbackDecider.html)

#### 配置文件&配置项

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/tasks/open_space_fallback_decider/conf/default_conf.pb.txt | apollo::planning::OpenSpaceFallBackDeciderConfig | OpenSpaceFallbackDecider 的配置文件 |
| modules/common/data/vehicle_param.pb.txt | apollo::common::VehicleConfig | 车辆底盘配置文件 |

#### Flags

| 文件路径 | 说明 |
| --- | --- |
| modules/planning/planning_component/conf/planning.conf | planning模块的flag配置文件 |

#### 使用方式

##### 配置加载 OpenSpaceFallbackDecider Task 插件

在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`OpenSpaceFallbackDecider`插件的scenarios xxxx中增加相应的配置，配置参数中 `name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可， `type` 是task的类名称，即 OpenSpaceFallbackDecider。 

task {
name: "OPEN_SPACE_FALLBACK_DECIDER"
type: "OpenSpaceFallbackDecider"
}
fragment

##### 配置 OpenSpaceFallbackDecider 参数

在`modules/planning/tasks/open_space_fallback_decider/conf/default_conf.pb.txt`中，对`OpenSpaceFallbackDecider`插件的参数进行配置。 在`modules/planning/planning_component/conf/planning.conf`中，对作用在`OpenSpaceFallbackDecider`插件的gflag参数进行配置  

##### 启动planning

mainboard -d modules/planning/planning_component/dag/planning.dag
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。