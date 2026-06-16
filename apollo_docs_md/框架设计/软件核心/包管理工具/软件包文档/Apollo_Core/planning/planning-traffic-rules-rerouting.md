---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2traffic__rules_2rerouting_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-traffic-rules-rerouting
---

# README_cn

# planning-traffic-rules-rerouting

## 简介

`Rerouting`任务用于换道没有成功时重路由 

### 模块流程

产生重路由的决策主要在Rerouting::ChangeLaneFailRerouting()函数中，产生重路由的条件包括：

- 当前车道不是直行车道
- 车辆在当前参考线车道内
- 当前车道通路没有出口
- 上一次产生重路由不久
- 主车距离当前通路很近 

## 目录结构

modules/planning/traffic_rules/rerouting
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── rerouting.proto
├── README_cn.md
├── rerouting.cc
└── rerouting.h
fragment

## 模块

### Rerouting插件

[apollo::planning::Rerouting](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1Rerouting.html)

#### 配置文件&配置项

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/traffic_rules/rerouting/conf/default_conf.pb.txt | apollo::planning::ReroutingConfig | Rerouting的默认配置文件 |

#### 模块参数说明

算法参数配置定义于modules/planning/tasks/rerouting/proto/rerouting.proto

| cooldown_time | 两次产生重路由间隔时间 |
| --- | --- |
| prepare_rerouting_time | 准备重路由时间 |

#### 使用方式

##### 配置加载 Rerouting 插件

在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 增加`Rerouting`插件的配置，配置参数中`name` 表示rule的名称，这个由用户自定义，表达清楚是哪个rule即可，`type` 是rule的类名称，即`Rerouting`。

rule {
name: "REROUTING"
type: "Rerouting"
}
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。