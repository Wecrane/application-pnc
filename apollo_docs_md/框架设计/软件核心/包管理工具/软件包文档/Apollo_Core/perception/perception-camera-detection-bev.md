---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2camera__detection__bev_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-camera-detection-bev
---

# README

# perception-camera-detection-bev

## Introduction

The BEV object detection module is a multi camera `transformer` model trained using `Nuscenes` datasets under the `paddlepaddle` framework, which can complete the inference of multiple camera data to obtain obstacle targets.

## Directory Structure

├── camera_detection_bev // camera bev detector module
├── detector         // main part for detector
│   ├── petr         // BEVObstacleDetector
│   └── ...
├── interface        // function interface folder
├── proto            // proto files
├── camera_detection_bev_component.cc // component interface
├── camera_detection_bev_component.h
├── cyberfile.xml    // package management profile
├── README.md
└── BUILD
fragment

## Modules

### CameraDetectionBevComponent

[apollo::perception::camera::CameraDetectionBevComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1camera_1_1CameraDetectionBevComponent.html)

#### Input

| Name | Type | Description | Input channel |
| --- | --- | --- | --- |
| msg | apollo::drivers::Image | camera sensor image | /apollo/sensor/camera/CAM_BACK/image   /apollo/sensor/camera/CAM_FRONT/image   apollo/sensor/camera/CAM_FRONT_RIGHT/image   /apollo/sensor/camera/CAM_FRONT_LEFT/image   /apollo/sensor/camera/CAM_BACK_LEFT/image  /apollo/sensor/camera/CAM_BACK_RIGHT/image |

>Note: Enter the data type defined by proto. This component triggers the proc function based on the main camera sensor data. The trigger camera channel is `/apollo/sensor/camera/CAM_BACK/image`. The detailed input channel information is in `modules/perception/camera_detection_bev/conf /camera_detection_bev_config.pb.txt` file.

#### Output

| Name | Type | Description | Output channal |
| --- | --- | --- | --- |
| frame | apollo::perception::Obstacles | camera frame message | /apollo/perception/obstacles |

>Note: If the receiving data type is a custom class or structure, the new component and the upstream and downstream components must be started in the same process to send and receive messages normally. If the received message is of the data type defined by proto, the upstream and downstream components will be started in different processes and can receive the message normally.

#### How to Launch

mainboard -d modules/perception/camera_detection_bev/dag/camera_detection_bev.dag
fragment

## Reference

1. [PETR: Position Embedding Transformation for Multi-View 3D Object Detection](https://arxiv.org/abs/2203.05625)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。