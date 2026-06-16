---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2task__manager_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > task_manager > task-manager
---

# README

# task-manager

## Introduction

Task manager generates a series of tasks, including path planning, obstacle avoidance, traffic rule compliance, etc., based on the input map data and routing instructions (start and end positions). These tasks will be scheduled and executed in the form of Cyber concatenation to ensure that the vehicle is able to make correct driving decisions based on the real-time traffic conditions while traveling.

## Directory Structure

modules/task_manager
├── BUILD
├── common
├── conf
├── cyberfile.xml
├── cycle_routing_manager.cc
├── cycle_routing_manager.h
├── dag
├── parking_routing_manager.cc
├── parking_routing_manager.h
├── proto
├── README.md
├── task_manager_component.cc
└── task_manager_component.h
fragment

#### Input

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::task_manager::Task | task |
| msg | apollo::localization::LocalizationEstimate | localization |
| msg | apollo::planning::PlanningCommand | planning command |
| msg | planning::ADCTrajectory | planning trajectory |

#### Output

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::external_command::LaneFollowCommand | lane follow command |

## configs

| file path | type / struct | Description |
| --- | --- | --- |
| modules/task_manager/conf/task_manager_config.pb.txt | apollo::task_manager::TaskManagerConfig | task manager config |
| modules/task_manager/conf/task_manager.conf | gflags | gflags config |

## Flags

| flagfile | type | Description |
| --- | --- | --- |
| modules/task_manager/common/task_manager_gflags.cc | cc | task manager flags define |
| modules/task_manager/common/task_manager_gflags.h | h | task manager flags header |

#### How to Launch

cyber_launch start modules/task_manager/launch/task_manager.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。