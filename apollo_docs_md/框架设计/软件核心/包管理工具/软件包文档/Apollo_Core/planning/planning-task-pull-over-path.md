---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2tasks_2pull__over__path_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-task-pull-over-path
---

# README_cn

# planning-task-pull-over-path

## 简介

`PullOverPath`任务用于生成靠边停车路径 

### 模块流程

1. 如果已经有成功生成的路径，或者参考线是换道的参考线，则跳过任务 ++
 if (!reference_line_info->path_data().Empty() ||
     reference_line_info->IsChangeLanePath()) {
 return Status::OK();
 }
 fragment 
计算路径规划的SL起始点状态 ++
  GetStartPointSLState();

构造路径边界 ++
 if (!DecidePathBounds(&candidate_path_boundaries)) {
 return Status::OK();
 }

初始化路径边界  ++
 if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_, &path_bound, init_sl_state_)) {
 AERROR << "Failed to initialize path boundaries.";
 return false;
}
AERROR#define AERRORDefinition log.h:44

从道路边界获得路径边界  ++
if (!GetBoundaryFromRoads(*reference_line_info_, &path_bound)) {
 AERROR << "Failed to decide a rough boundary based on road boundary.";
 return false;
  }

从道路边界获得路径边界 ++
 if (!GetBoundaryFromRoads(*reference_line_info_, &path_bound)) {
 AERROR << "Failed to decide a rough boundary based on road boundary.";
 return false;
   }

- 从车道边界获得路径边界  ++
UpdatePullOverBoundaryByLaneBoundary(is_pull_over_right, &path_bound);
 fragment 
- 将静态障碍物加入路径边界  ++
if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
        *reference_line_info_, init_sl_state_, &path_bound,
        &blocking_obstacle_id)) {
 AERROR << "Failed to decide fine tune the boundaries after "
 "taking into consideration all static obstacles.";
 return false;
}
 fragment 
- 如果没有指定靠边停车位置，搜索靠边停车位置 ++
 if (!SearchPullOverPosition(path_bound, &pull_over_configuration)) {
 AERROR << "Failed to find a proper pull-over position.";
 return false;
     }
 fragment 
规划路径 ++
 if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
 return Status::OK();
 }

评估路径是否有效 ++
 if (AssessPath(&candidate_path_data,
                reference_line_info->mutable_path_data())) {
 ADEBUG << "pull-over path success";
 }
ADEBUG#define ADEBUGDefinition log.h:41

## 目录结构

modules/planning/tasks/pull_over_path/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── pull_over_path.proto
├── pull_over_path.cc
├── pull_over_path.h
└── README_cn.md
fragment

## 模块

### PullOverPath插件

[apollo::planning::PullOverPath](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1PullOverPath.html)

#### 配置文件&配置项

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/tasks/pull_over_path/conf/default_conf.pb.txt | apollo::planning::PullOverPathConfig | PullOverPath的默认配置文件 |

#### 模块参数说明

算法参数配置定义于modules/planning/tasks/pull_over_path/proto/pull_over_path.proto

| path_optimizer_config | 路径规划器参数 |
| --- | --- |
| pull_over_road_edge_buffer | 靠边停车离道路边界距离 |
| pull_over_weight | 靠边停车权重（权重越大，曲率越大） |
| pull_over_direction | 靠边停车方向 |
| pull_over_position | 靠边停车位置类型 |

#### 命令行参数/gflags

`modules/planning/planning_base/common/planning_gflags.cc`中定义了用到的命令行参数，planning.conf中定义命令行参数值。 

| flag | 描述 |
| --- | --- |
| FLAGS_obstacle_lon_start_buffer | 障碍物起始位置buffer |
| FLAGS_obstacle_lon_end_buffer | 障碍物结束位置buffer |
| FLAGS_num_extra_tail_bound_point | 路径阻塞额外添加点数量 |

#### 使用方式

##### 配置加载 PullOverPath Task 插件

```
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`PullOverPath`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`PullOverPath`。
```
task {
  name: "PULL_OVER_PATH"
  type: "PullOverPath"
}
  ```

```

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。