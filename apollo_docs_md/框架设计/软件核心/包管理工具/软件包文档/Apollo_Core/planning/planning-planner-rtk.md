---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2planners_2rtk_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-planner-rtk
---

# README_cn

# planning-planner-rtk

## 介绍

`planning-planner-rtk` 是包含RTKReplayPlanner的插件包，RTKReplayPlanner是基于录制的轨迹进行循迹的Planner，需要事先录制好设定的轨迹来规划行车路线。

## 目录结构

modules/planning/planner/rtk
├── rtk
├── testdata                    // 测试数据
├── rtk_replay_planner.h        // RTKReplayPlanner头文件
├── rtk_replay_planner.cc       // RTKReplayPlanner源文件
├── rtk_replay_planner_test.cc  // 单元测试文件
├── BUILD                       // 构建规则文件
├── cyberfile.xml               // 包管理配置文件
├── plugins.xml                 // 插件描述说明文件
└── README_cn.md                // 说明文档
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。