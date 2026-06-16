---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2tasks_2rule__based__stop__decider_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-task-rule-based-stop-decider
---

# README_cn

# planning-task-rule-based-stop-decider

## 简介

`RuleBasedStopDecider`任务用于产生基于规则的停车策略 

### 模块流程

1. 借对向车道超车时，检查是否存在感知盲区，如果有则产生停车决策 ++
   StopOnSidePass(frame, reference_line_info);
 fragment 
换道时接近换道终点没有成功换道产生停车决策 ++
 if (config_.enable_lane_change_urgency_checking()) {
   CheckLaneChangeUrgency(frame);
 }

路径终点产生停车决策 ++
    AddPathEndStop(frame, reference_line_info);

## 目录结构

modules/planning/tasks/rule_based_stop_decider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── rule_based_stop_decider.proto
├── README_cn.md
├── rule_based_stop_decider.cc
└── rule_based_stop_decider.h
fragment

## 模块

### RuleBasedStopDecider 插件

[apollo::planning::RuleBasedStopDecider](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1RuleBasedStopDecider.html)

#### 配置文件&配置项

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/tasks/rule_based_stop_decider/conf/default_conf.pb.txt | apollo::planning::RuleBasedStopDeciderConfig | RuleBasedStopDecider的默认配置文件 |
| modules/planning/planning_component/conf/planning_config.pb.txt | apollo::planning::PlanningConfig | planning组件的配置文件 |
| modules/common/data/vehicle_param.pb.txt | apollo::common::VehicleConfig | 车辆底盘配置文件 |

#### Flags

| 文件路径 | 说明 |
| --- | --- |
| modules/planning/planning_component/conf/planning.conf | planning模块的flag配置文件 |

#### 模块参数说明

算法参数配置定义于modules/planning/tasks/rule_based_stop_decider/proto/rule_based_stop_decider.proto

| max_adc_stop_speed | 判断主车停车速度 |
| --- | --- |
| max_valid_stop_distance | 停车距离 |
| search_beam_length | 盲区搜索距离 |
| search_beam_radius_intensity | 盲区搜索密度向 |
| search_range | 盲区范围 |
| is_block_angle_threshold | 判断盲区阈值 |
| approach_distance_for_lane_change | 到换道终点距离 |
| urgent_distance_for_lane_change | 换道终点前停车距离 |
| enable_lane_change_urgency_checking | 是否产生换道停止决策 |
| short_path_length_threshold | 路径长度阈值 |

#### 使用方式

##### 配置加载 RuleBasedStopDecider Task 插件

```
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`RuleBasedStopDecider`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`RuleBasedStopDecider`。
```
task {
  name: "RULE_BASED_STOP_DECIDER"
  type: "RuleBasedStopDecider"
}
  ```

```

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。