---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2motion__service_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-motion-service
---

# README

# perception-motion-service

## Introduction

The motion service module provides current motion estimation.

## Directory Structure

motion_service
├── BUILD      // bazel build file
├── README.md
├── conf       // dag config file
├── cyberfile.xml  // package description file
├── dag
├── launch
├── motion     // main process
├── motion_service_component.cc   // component
├── motion_service_component.h
└── proto      // proto file
fragment

## Modules

### MotionServiceComponent

[apollo::perception::camera::MotionServiceComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1camera_1_1MotionServiceComponent.html)

#### Input

| Channel | Type | Description | Name |
| --- | --- | --- | --- |
| /apollo/sensor/camera/front_6mm/image | apollo::perception::camera::ImageMsgType | camera drive message | msg |
| /apollo/localization/pose | apollo::perception::camera::LocalizationMsgType | localization message | msg |

#### Output

| Channel | Type | Description | Name |
| --- | --- | --- | --- |
| /apollo/perception/motion_service | apollo::perception::MotionService | motion service message | msg |

#### How to run

You can start the lane detection module with the following command.

cyber_launch start modules/perception/motion_service/launch/motion_service.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。