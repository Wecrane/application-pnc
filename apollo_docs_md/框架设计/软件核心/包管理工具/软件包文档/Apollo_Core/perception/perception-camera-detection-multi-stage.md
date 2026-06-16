---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2camera__detection__multi__stage_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-camera-detection-multi-stage
---

# README

# perception-camera-detection-multi-stage

## Introduction

The camera 2D object detection module is a multitask model developed based on early `yolo`, which can simultaneously output dozens of dimensional information such as `2d` ,`3d`, and vehicle turn signals. This module completes operations such as image data preprocessing, detection, result post-processing and so on.

## Directory Structure

├── camera_detection_multi_stage // camera detect multi stage module
├── conf            // module configuration files
├── dag             // dag files
├── data            // model params
├── detector        // main part for detector
│   ├── yolo        // YoloObstacleDetector
│   └── ...
├── interface       // function interface folder
├── proto           // proto files
├── camera_detection_multi_stage_component.cc // component interface
├── camera_detection_multi_stage_component.h
├── cyberfile.xml   // package management profile
├── README.md
└── BUILD
fragment

## Modules

### CameraDetectionMultiStageComponent

[apollo::perception::camera::CameraDetectionMultiStageComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1camera_1_1CameraDetectionMultiStageComponent.html)

#### Input

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::drivers::Image | camera sensor image |

#### Output

| Name | Type | Description |
| --- | --- | --- |
| frame | apollo::perception::onboard::CameraFrame | camera frame message |

#### How to Launch

cyber_launch start modules/perception/launch/perception_camera_multi_stage.launch
fragment

## Reference

1. [YOLO: Real-Time Object Detection](https://pjreddie.com/darknet/yolo/)
1. [3D Bounding Box Estimation Using Deep Learning and Geometry](https://arxiv.org/abs/1612.00496)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。