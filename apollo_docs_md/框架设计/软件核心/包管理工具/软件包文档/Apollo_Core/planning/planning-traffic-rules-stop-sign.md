---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2traffic__rules_2stop__sign_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-traffic-rules-stop-sign
---

# README_cn

# planning-traffic-rules-stop-sign

## 简介

`StopSign`任务用于对于地图中的停止标记生成停车的交通决策 

### 模块流程

产生停止标记决策主要在StopSign::MakeDecisions函数中，产生停止标记决策的条件包括：

- 主车前方的参考线存在停止标记
- 停止标记在injector_中没有标记为done 

## 目录结构

modules/planning/traffic_rules/stop_sign/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── stop_sign.proto
├── README_cn.md
├── stop_sign.cc
└── stop_sign.h
fragment

## 模块

### StopSign插件

[apollo::planning::StopSign](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1StopSign.html)

#### 配置文件&配置项

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/traffic_rules/stop_sign/conf/default_conf.pb.txt | apollo::planning::StopSignConfig | StopSign的默认配置文件 |

#### 模块参数说明

算法参数配置定义于modules/planning/tasks/stop_sign/proto/stop_sign.proto

| enabled | 是否使能 |
| --- | --- |
| stop_distance | 在停止线前停车距离 |

#### 使用方式

##### 配置加载 StopSign 插件

在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 增加`StopSign`插件的配置，配置参数中`name` 表示rule的名称，这个由用户自定义，表达清楚是哪个rule即可，`type` 是rule的类名称，即`StopSign`。

rule {
name: "STOP_SIGN"
type: "StopSign"
}
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。