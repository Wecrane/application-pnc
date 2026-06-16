---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2tasks_2reuse__path_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-task-reuse-path
---

# README_cn

# planning-task-pull-over-path

## 简介

`ReusePath`任务用于生成重用路径。重用路径指的是如果上一帧的路径在当前帧没有和障碍物发生碰撞，重用上一帧的路径作为当前路径 

### 模块流程

1. 判断路径是否重用 ++
 if (!IsPathReusable(frame, reference_line_info)) {
   path_reusable_ = false;
 return Status::OK();
 }
 fragment 
触发重规划不能重用路径
路径发生碰撞不能重用路径
上一帧速度规划失败不能重用路径

可以重用，剪裁上一帧路径作为当前帧路径 

++
if
(!TrimHistoryPath(frame, reference_line_info)) {
path_reusable_ =
false
;
return
Status::OK();
}
fragment

## 目录结构

modules/planning/tasks/reuse_path/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── reuse_path.proto
├── README_cn.md
├── reuse_path.cc
└── reuse_path.h
fragment

## 模块

### ReusePath插件

[apollo::planning::ReusePath](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1ReusePath.html)

#### 配置文件&配置项

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/planning/tasks/reuse_path/conf/default_conf.pb.txt | apollo::planning::ReusePathConfig | ReusePath的默认配置文件 |

#### 模块参数说明

算法参数配置定义于modules/planning/tasks/reuse_path/proto/reuse_path.proto

| enable_reuse_path_in_lane_follow | 非换道场景是否启用路径重用 |
| --- | --- |
| short_path_threshold | 最短路径长度 |

#### 命令行参数/gflags

`modules/planning/planning_base/common/planning_gflags.cc`中定义了用到的命令行参数，planning.conf中定义命令行参数值。 

| flag | 描述 |
| --- | --- |
| FLAGS_path_bounds_decider_resolution | 路径规划采样点距离 |
| FLAGS_num_extra_tail_bound_point | 路径尾部额外增加采样点数量 |

#### 使用方式

##### 配置加载 ReusePath Task 插件

```
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`ReusePath`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`ReusePath`。
```
task {
  name: "REUSE_PATH"
  type: "ReusePath"
}
  ```

```

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。