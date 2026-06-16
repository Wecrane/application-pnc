---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2lane__detection_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-lane-detection
---

# README

# perception-lane-detection

## Introduction

The lane detection module detects lane lines through the camera sensor. Lane lines can be used as an aid to other modules.

## Directory Structure

lane_detection
├── BUILD       // bazel build file
├── README.md
├── app         // main process
├── common      // common functions
├── conf        // dag config file
├── cyberfile.xml  // package description file
├── dag
├── data        // lib's configuration file
├── interface
├── lane_detection_component.cc   // component
├── lane_detection_component.h
├── launch
├── lib        // algorithm library
└── proto      // proto file
fragment

## Modules

### LaneDetectionComponent

[apollo::perception::onboard::LaneDetectionComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1onboard_1_1LaneDetectionComponent.html)

#### Input

| Channel | Type | Description |
| --- | --- | --- |
| /apollo/sensor/camera/front_6mm/image | apollo::drivers::Image | camera drive message |
| /apollo/sensor/camera/front_12mm/image | apollo::drivers::Image | camera drive message |
| /apollo/perception/motion_service | apollo::perception::onboard::LaneDetectionComponent::MotionServiceMsgType | motion service message |

#### Output

| Channel | Type | Description |
| --- | --- | --- |
| /perception/lanes | apollo::perception::PerceptionLanes | lane line |

#### How to run

You can start the lane detection module with the following command.

cyber_launch start modules/perception/launch/perception_lane.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。