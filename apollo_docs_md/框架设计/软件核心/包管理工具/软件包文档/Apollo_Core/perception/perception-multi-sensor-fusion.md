---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2multi__sensor__fusion_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-multi-sensor-fusion
---

# README

# perception-multi-sensor-fusion

## Introduction

The multi-sensor fusion module fuses the output results of Lidar, Camera, and Radar multiple sensors to make the detection results more reliable.

It uses post-processing technology, the algorithm used is probabilistic fusion.

## Directory Structure

multi_sensor_fusion
├── BUILD      // bazel build file
├── README.md
├── base       // base data structure
├── common     // filter and utils
├── conf       // dag config file
├── cyberfile.xml  // package description file
├── dag
├── data        // lib's configuration file
├── fusion      // fusion function
├── interface
├── multi_sensor_fusion_component.cc  // component
├── multi_sensor_fusion_component.h
└── proto       // proto file
fragment

## Input and Output

### MultiSensorFusionComponent

[apollo::perception::fusion::MultiSensorFusionComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1fusion_1_1MultiSensorFusionComponent.html)

#### Input

| Input channel | Type | Description |
| --- | --- | --- |
| /perception/inner/PrefusedObjects | apollo::perception::onboard::SensorFrameMessage | frame contains object detection |

>Note: The input channel is structure type data. The default trigger channel is `/perception/inner/PrefusedObjects`. The detailed input channel information is in `modules/perception/multi_sensor_fusion/dag/multi_sensor_fusion.dag` file. By default, the upstream components of the messages received by the component include `lidar_detection_tracking`, `camera_tracking`, `radar_detection`.

#### Output

| Output channel | Type | Description |
| --- | --- | --- |
| /apollo/perception/obstacles | apollo::perception::PerceptionObstacles | detection results after fusion |

>Note: The output channel is proto type data. The detailed output channel information is in `modules/perception/multi_sensor_fusion/conf/multi_sensor_fusion_config.pb.txt` file.

#### How to run

The multi-sensor fusion module does not support running alone, it needs to run together with lidar, camera and radar detection modules.

You can use the following command to start the whole perception function, including lidar, camera, and radar target detection, and output their results after fusion.

cyber_launch start modules/perception/launch/perception_all.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。