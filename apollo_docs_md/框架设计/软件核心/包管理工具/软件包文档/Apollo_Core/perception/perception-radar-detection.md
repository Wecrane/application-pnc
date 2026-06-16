---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2radar__detection_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-radar-detection
---

# README

# perception-radar-detection

## Introduction

The radar detection module detects and tracks obstacles through the radar sensor, then sends them to the multi-sensor fusion module.

The radar detection module receives the radar drive message, which is already include the objects detected and tracked. Even so, the radar module has re-tracked the objects.

## Directory Structure

radar_detection
├── BUILD     // bazel build file
├── README.md
├── app       // main process
├── common    // utils
├── conf      // dag config file
├── cyberfile.xml  // package description file
├── dag
├── data      // lib's configuration file
├── interface
├── launch
├── lib       // algorithm library
├── proto     // proto file
├── radar_detection_component.cc  // component
└── radar_detection_component.h
fragment

## Modules

### RadarDetectionComponent

[apollo::perception::radar::RadarDetectionComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1radar_1_1RadarDetectionComponent.html)

#### Input

| Input channel | Type | Description |
| --- | --- | --- |
| /apollo/sensor/radar/front | apollo::drivers::ContiRadar | radar drive message |

>Note: Radar data from driver. The default trigger channel is `/apollo/sensor/radar/front`. The detailed input channel information is in `modules/perception/radar_detection/dag/radar_detection_front.dag` file.

#### Output

| Output channel | Type | Description |
| --- | --- | --- |
| /perception/inner/PrefusedObjects | apollo::perception::onboard::SensorFrameMessage | frame contains object detection |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/radar_detection/conf/front_radar_detection_config.pb.txt` file.

#### How to run

In most cases, the radar detection module needs to work with the multi-sensor fusion module. If you want to debug the radar module separately and view the detected obstacle information, you can combine the `msg_adapter` module.

Run the command as follows.

cyber_launch start modules/perception/launch/perception_radar.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。