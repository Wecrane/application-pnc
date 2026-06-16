---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2tasks_2speed__bounds__decider_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-task-speed-bounds-decider
---

# README_cn

# planning-task-speed-bounds-decider

## 简介

`SpeedBoundsDecider`任务用于产生速度规划的ST可行驶区间 

### 模块流程

1. 将障碍物预测轨迹投影到ST空间，根据决策构建ST边界，如果没有决策保留障碍物上下两个边界。 ++
 if (boundary_mapper.ComputeSTBoundary(path_decision).code() ==
     ErrorCode::PLANNING_ERROR) {
 const std::string msg = "Mapping obstacle failed.";
 AERROR << msg;
 return Status(ErrorCode::PLANNING_ERROR, msg);
 }
AERROR#define AERRORDefinition log.h:44
 fragment 
根据道路限速，障碍物nudge限速，产生限速曲线 ++
 if (!speed_limit_decider
          .GetSpeedLimits(path_decision->obstacles(), &speed_limit)
          .ok()) {
 const std::string msg = "Getting speed limits failed!";
 AERROR << msg;
 return Status(ErrorCode::PLANNING_ERROR, msg);
 }

## 目录结构

modules/planning/tasks/speed_bounds_decider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── speed_bounds_decider.proto
├── README_cn.md
├── speed_bounds_decider.cc
├── speed_bounds_decider.h
├── speed_limit_decider.cc
├── speed_limit_decider.h
├── st_boundary_mapper.cc
├── st_boundary_mapper.h
└── st_boundary_mapper_test.cc
fragment

## 模块

### SpeedBoundsDecider 插件

[apollo::planning::SpeedBoundsDecider](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1SpeedBoundsDecider.html)

#### 配置文件&配置项

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/tasks/speed_bounds_decider/conf/default_conf.pb.txt | apollo::planning::SpeedBoundsDeciderConfig | SpeedBoundsDecider的默认配置文件 |
| modules/common/data/vehicle_param.pb.txt | apollo::common::VehicleConfig | 车辆底盘配置文件 |

#### Flags

| 文件路径 | 说明 |
| --- | --- |
| modules/planning/planning_component/conf/planning.conf | planning模块的flag配置文件 |

#### 模块参数说明

算法参数配置定义于modules/planning/tasks/rule_based_stop_decider/proto/rule_based_stop_decider.proto

| total_time | ST边界总时长 |
| --- | --- |
| max_centric_acceleration_limit | 最大向心加速度 |
| minimal_kappa | 最小曲率 |
| point_extension | ST投影S方向拓展长度 |
| lowest_speed | 最低限速 |
| collision_safety_range | nudge安全距离，小于安全距离会降低速度 |
| static_obs_nudge_speed_ratio | 静态障碍物nudge限速比例 |
| dynamic_obs_nudge_speed_ratio | 动态障碍物nudge限速比例 |
| enable_nudge_slowdown | 是否nudge时减速 |
| lane_change_obstacle_nudge_l_buffer | 换道时nudge缓冲 |
| max_trajectory_len | 轨迹最长距离 |

#### 命令行参数/gflags

`modules/planning/planning_base/common/planning_gflags.cc`中定义了用到的命令行参数，planning.conf中定义命令行参数值。 

| flag | 描述 |
| --- | --- |
| FLAGS_trajectory_check_collision_time_step | 检查碰撞时间步长 |

#### 使用方式

##### 配置加载 SpeedBoundsDecider Task 插件

```
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`SpeedBoundsDecider`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`SpeedBoundsDecider`。
```
task {
  name: "SPEED_BOUNDS_PRIORI_DECIDER"
  type: "SpeedBoundsDecider"
}
  ```

```

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。