---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2planning__base_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-base
---

# README_cn

# planning

## 介绍

`planning-base` 是planning模块的基础数据结构和算法库，它包含了模块运行流程中公共的数据结构类，以及底层算法函数。

## 目录结构

modules/planning/planning_base
├── planning_base
├── common                  // 公共算法库
├── gflags                  // gflag参数配置
├── learning_based          // 基于学习算法相关库
├── math                    // 基础数学库
├── open_space              // open_space相关算法库
├── proto                   // 公共（全局）参数配置结构定义
├── reference_line          // 参考线以及参考线处理类
├── testdata                // 单元测试数据
├── tools                   // 工具类
├── BUILD                   // 构建规则文件
├── cyberfile.xml           // 包管理配置文件
└── README_cn.md            // 说明文档
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。