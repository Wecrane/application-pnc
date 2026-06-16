---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2traffic__rules_2backside__vehicle_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-traffic-rules-backside-vehicle
---

# README_cn

# planning-traffic_rules-backside-vhicle

## 简介

`BacksideVehicle`用于产生后方来车是否忽略的决策, 决策结果保存在 reference_line_info 的 path_decision 中

#### 模块流程

遍历所有障碍物, 根据其位置信息, 判断是否产生忽略的决策, 条件包括:

- 不能忽略自车前方或侧方的障碍物
- 忽略从自车正后方过来的障碍物

## 目录结构

modules/planning/traffic_rules/backside_vehicle/
├── BUILD                       // 构建规则文件
├── README_cn.md                // 说明文档
├── backside_vehicle.cc         // 源文件
├── backside_vehicle.h          // 头文件
├── conf                        // 参数配置文件夹
│   └── default_conf.pb.txt
├── cyberfile.xml               // 包管理配置文件
├── plugins.xml                 // 插件规则文件
└── proto                       // 配置定义文件夹
├── BUILD
└── backside_vehicle.proto
fragment

## 模块

### BacksideVehicle插件

[apollo::planning::BacksideVehicle](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1BacksideVehicle.html)

#### 配置文件 & 配置项

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/traffic_rules/backside_vehicle/conf/default_conf.pb.txt | apollo::planning::BacksideVehicleConfig | traffic rule的默认配置文件 |

#### 使用方式

在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 增加 `BacksideVehicle` 插件的配置

rule {
name: "BACKSIDE_VEHICLE"
type: "BacksideVehicle"
}
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。