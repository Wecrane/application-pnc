---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2scenarios_2free__space_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-scenario-free-space
---

# README_cn

# planning-scenario-free-space

## 简介

`FreeSpaceScenario` 用于实现在指定可行驶区域边界与目标位姿条件下，通过openspace算法，生成相应的轨迹。

### 场景切入条件

1. `planning_command`有相应的自定义命令。
1. 自车处于静止状态。
1. `planning_command`中包含可行驶区域边界且边界点以逆时针顺序发布，且可行驶区域边界包含自车起点与终点。
1. 不可行驶区域边界的边界点以顺时针顺序发布且不可行驶区域边界不可包含自车起点与终点。

### 阶段.

| 阶段名 | 类型 | 描述 |
| --- | --- | --- |
| STAGE_FREE_SPACE | apollo::planning::StageFreeSpace | 生成在指定区域内驶向指定位姿的轨迹 |

`STAGE_FREE_SPACE`生成通向目标点的可行路径。该stage需满足不可行驶区域在可行驶区域边界内，同时区域边界约束可转为`Ax>b`形式。

## 目录结构

modules/planning/scenarios/free_space/
├── BUILD
├── conf
│   ├── pipeline.pb.txt
│   ├── scenario_conf.pb.txt
│   └── stage_free_space
│       ├── open_space_trajectory_partition.pb.txt
│       └── open_space_trajectory_provider.pb.txt
├── cyberfile.xml
├── free_space_scenario.cc
├── free_space_scenario.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── free_space_scenario.proto
├── README_cn.md
├── stage_free_space.cc
└── stage_free_space.h
fragment

## 模块

### FreeSpaceScenario插件

[apollo::planning::FreeSpaceScenario](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1FreeSpaceScenario.html)

#### 配置

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/scenarios/free_space/conf/scenario_conf.pb.txt | apollo::planning::ScenarioFreeSpaceConfig | 场景的配置文件 |
| modules/planning/scenarios/free_space/conf/pipeline.pb.txt | apollo::planning::ScenarioPipeline | 场景的流水线文件 |
| modules/planning/scenarios/free_space/conf/stage_free_space/open_space_trajectory_partition.pb.txt | apollo::planning::OpenSpaceTrajectoryPartitionConfig | stage_free_space stage 中 open_space_trajectory_partition task 配置文件 |
| modules/planning/scenarios/free_space/conf/stage_free_space/open_space_trajectory_provider.pb.txt | apollo::planning::OpenSpaceTrajectoryProviderConfig | stage_free_space stage 中open_space_trajectory_provider task 配置文件 |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。