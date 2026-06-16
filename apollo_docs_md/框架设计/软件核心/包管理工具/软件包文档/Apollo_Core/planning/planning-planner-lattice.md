---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2planners_2lattice_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-planner-lattice
---

# README_cn

# planning-planner-lattice

## 介绍

`planning-planner-lattice` 是包含LatticePlanner的插件包，LatticePlanner适用于在高速公路等简单场景下的规划。

## 目录结构

modules/planning/planner/lattice
├── lattice
├── behavior                // 行为决策库
├── trajectory_generation   // 轨迹生成库
├── lattice_planner.h       // LatticePlanner头文件
├── lattice_planner.cc      // LatticePlanner源文件
├── BUILD                   // 构建规则文件
├── cyberfile.xml           // 包管理配置文件
├── plugins.xml             // 插件描述说明文件
└── README_cn.md            // 说明文档
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。