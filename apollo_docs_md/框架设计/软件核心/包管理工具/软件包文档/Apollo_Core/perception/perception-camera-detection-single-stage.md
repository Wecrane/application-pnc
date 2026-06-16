---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2camera__detection__single__stage_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-camera-detection-single-stage
---

# README

# perception-camera-detection-single-stage

## Introduction

The camera 3D object detection module includes two models: `caddn` and `smoke`, which can simultaneously output `2d` and `3d` information at the same time. This module completes operations such as image data preprocessing, detection, and result postprocessing. This module can directly transfer the results to `camera_tracking` component.

## Directory Structure

├── camera_detection_single_stage // camera detect 3d module
├── conf            // module configuration files
├── dag             // dag files
├── data            // model params
├── detector        // main part for 3d detector
│   ├── SMOKE       // SmokeObstacleDetector
│   └── ...
├── interface       // function interface folder
├── proto           // proto files
├── camera_detection_single_stage_component.cc // component interface
├── camera_detection_single_stage_component.h
├── cyberfile.xml   // package management profile
├── README.md
└── BUILD
fragment

## Modules

### CameraDetectionSingleStageComponent

[apollo::perception::camera::CameraDetectionSingleStageComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1camera_1_1CameraDetectionSingleStageComponent.html)

#### Input

| Name | Type | Description | Input channal |
| --- | --- | --- | --- |
| msg | apollo::drivers::Image | camera sensor image | /apollo/sensor/camera/front_6mm/image |

>Note: Enter the data type defined by proto. The default trigger camera channel is `/apollo/sensor/camera/front_6mm/image`. The detailed input channel information is in `modules/perception/camera_detection_single_stage/dag/camera_detection_single_stage.dag` file.

#### Output

| Name | Type | Description | Output channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::CameraFrame | camera frame message | /perception/inner/Detection |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/camera_detection_multi_stage/conf/camera_detection_multi_stage_yolox3d_config.pb.txt` file.

#### How to Launch

cyber_launch start modules/perception/launch/perception_camera_single_stage.launch
fragment

## Reference

1. [SMOKE: Single-Stage Monocular 3D Object Detection via Keypoint Estimation](https://arxiv.org/pdf/2002.10111.pdf)
1. [CADDN](https://arxiv.org/abs/2103.01100)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。