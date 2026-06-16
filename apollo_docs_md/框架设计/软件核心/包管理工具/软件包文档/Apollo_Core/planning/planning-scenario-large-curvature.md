---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2scenarios_2large__curvature_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-scenario-large-curvature
---

# README_cn

# planning-scenario-large-curvature

## 简介

`LargeCurvatureScenario`在大曲率道路上生成通行轨迹，解决主路算法在大曲率道路上的顿挫问题。

## 目录结构

modules/planning/scenarios/large_curvature/
├── BUILD
├── conf
│   ├── large_curvature
│   │   ├── open_space_fallback_decider_park.pb.txt
│   │   ├── open_space_path_planning.pb.txt
│   │   ├── open_space_replan_decider.pb.txt
│   │   ├── open_space_roi_decider_park.pb.txt
│   │   ├── open_space_rt_optimizer.pb.txt
│   │   ├── open_space_trajectory_optimizer.pb.txt
│   │   └── open_space_trajectory_post_process.pb.txt
│   ├── pipeline.pb.txt
│   └── scenario_conf.pb.txt
├── cyberfile.xml
├── large_curvature_scenario.cc
├── large_curvature_scenario.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── large_curvature_scenario.proto
├── README_cn.md
├── stage_large_curvature.cc
└── stage_large_curvature.h
fragment

## 模块

### LargeCurvatureScenario插件

[apollo::planning::LargeCurvatureScenario](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1LargeCurvatureScenario.html)

#### 场景切入条件

1. 参考线自车前3米处曲率大于min_curvature
1. 自车前方参考线长度小于10米

#### 阶段

| 阶段名 | 类型 | 描述 |
| --- | --- | --- |
| StageLargeCurvature | apollo::planning::StageLargeCurvature | 规划出通过大曲率道路的轨迹 |

#### 配置

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/scenarios/large_curvature/conf/scenario_conf.pb.txt | apollo::planning::ScenarioLargeCurvatureConfig | 场景的配置文件 |
| modules/planning/scenarios/large_curvature/conf/pipeline.pb.txt | apollo::planning::ScenarioPipeline | 场景的流水线文件 |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。