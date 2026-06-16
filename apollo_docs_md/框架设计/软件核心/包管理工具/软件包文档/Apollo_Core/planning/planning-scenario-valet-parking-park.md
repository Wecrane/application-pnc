---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2scenarios_2valet__parking__park_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-scenario-valet-parking-park
---

# README_cn

# planning-scenario-valet-parking-park

## 简介

`ValetParkingParkScenario`可以在停车区域泊入指定的车位。

## 目录结构

modules/planning/scenarios/valet_parking_park/
├── BUILD
├── conf
│   ├── pipeline.pb.txt
│   ├── scenario_conf.pb.txt
│   ├── valet_parking_approaching_parking_spot_park
│   │   └── open_space_pre_stop_decider.pb.txt
│   ├── valet_parking_parking_park
│   │   ├── open_space_fallback_decider_park.pb.txt
│   │   ├── open_space_path_planning.pb.txt
│   │   ├── open_space_replan_decider.pb.txt
│   │   ├── open_space_roi_decider.pb.txt
│   │   ├── open_space_trajectory_optimizer.pb.txt
│   │   └── open_space_trajectory_post_process.pb.txt
│   └── valet_parking_retry_park
│       ├── open_space_fallback_decider_park.pb.txt
│       ├── open_space_path_planning.pb.txt
│       ├── open_space_replan_decider.pb.txt
│       ├── open_space_roi_decider.pb.txt
│       ├── open_space_trajectory_optimizer.pb.txt
│       └── open_space_trajectory_post_process.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── valet_parking_park.proto
├── README_cn.md
├── stage_approaching_parking_spot_park.cc
├── stage_approaching_parking_spot_park.h
├── stage_parking_park.cc
├── stage_parking_park.h
├── stage_parking_retry_park.cc
├── stage_parking_retry_park.h
├── valet_parking_scenario.cc
└── valet_parking_scenario.h
fragment

## 模块

### ValetParkingParkScenario插件

[apollo::planning::ValetParkingParkScenario](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1ValetParkingParkScenario.html)

#### 场景切入条件

1. planning command里存在泊车命令
1. 距离泊车点距离parking_spot_range_to_start以内

#### 阶段

| 阶段名 | 类型 | 描述 |
| --- | --- | --- |
| VALET_PARKING_APPROACHING_PARKING_SPOT_PARK | apollo::planning::StageApproachingParkingSpotPark | 引导车辆沿主路行驶到泊车位 |
| VALET_PARKING_PARKING_PARK | apollo::planning::StageParkingPark | 泊入车位 |
| VALET_PARKING_RETRY_PARK | apollo::planning::StageParkingRetryPark | 第一次泊入车位精度较低时调整位姿 |

#### 配置

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/scenarios/valet_parking_park/conf/scenario_conf.pb.txt | apollo::planning::ScenarioValetParkingConfig | 场景的配置文件 |
| modules/planning/scenarios/valet_parking_park/conf/pipeline.pb.txt | apollo::planning::ScenarioPipeline | 场景的流水线文件 |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。