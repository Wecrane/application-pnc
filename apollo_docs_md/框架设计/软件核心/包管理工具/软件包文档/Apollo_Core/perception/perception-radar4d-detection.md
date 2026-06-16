---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2radar4d__detection_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-radar4d-detection
---

# README

# Introduction

The radar4d detection module receives the 4d radar driver message, which includes the radar point cloud and the tracked objects (optional). The original tracked objected are not used. Instead, obstacles are detected and tracked through the radar point cloud, then sent to the multi-sensor fusion module.

# Directory Structure

radar4d_detection
├── BUILD     // bazel build file
├── README.md
├── app       // main process
├── conf      // dag config file
├── cyberfile.xml  // package description file
├── dag
├── data      // lib's configuration file
├── interface
├── launch
├── lib       // algorithm library
├── proto     // proto file
├── radar4d_detection_component.cc  // component
└── radar4d_detection_component.h
fragment

# Input and Output

## Input

| Input channel | Type | Description |
| --- | --- | --- |
| /apollo/sensor/oculii/PointCloud2 | OculiiPointCloud | 4d radar drive message |

>Note: Radar4d data from driver. The default trigger channel is `/apollo/sensor/oculii/PointCloud2`. The detailed input channel information is in `modules/perception/radar4d_detection/dag/radar4d_detection.dag` file.

## Output

| Output channel | Type | Description |
| --- | --- | --- |
| /perception/inner/PrefusedObjects | onboard::SensorFrameMessage | frame contains object detection |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/radar4d_detection/conf/radar4d_component_config.pb.txt` file.

# How to run

In most cases, the radar4d detection module needs to work with the multi-sensor fusion module. If you want to debug the radar4d detection module separately and view the detected obstacle information, you can combine the `msg_adapter` module.

Run the command as follows. 

cyber_launch start modules/perception/launch/perception_radar4d.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。