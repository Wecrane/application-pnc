---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2camera__location__refinement_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-camera-location-refinement
---

# README

# perception-camera-location-refinement

## Introduction

Post process the obstacles detected in the previous stage. It mainly calculates the relative position and height of the tuning target and the camera by fitting the ground plane in the camera plane

## Directory Structure

├── camera_location_refinement  // object location refinement module
├── conf                    // module configuration files
├── dag                     // dag files
├── data                    // model params
├── interface               // function interface folder
├── proto                   // proto files
├── location_refiner        // implementation of algorithm
├── camera_location_refinement_component.cc // component interface
├── camera_location_refinement_component.h
├── cyberfile.xml           // package management profile
├── README.md
└── BUILD
fragment

## Modules

### CameraLocationRefinementComponent

[apollo::perception::camera::CameraLocationRefinementComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1camera_1_1CameraLocationRefinementComponent.html)

#### Input

| Name | Type | Description | Input channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::CameraFrame | camera frame message | /perception/inner/location_estimation |

>Note: The input channel is structure type data. The default trigger channel is `/perception/inner/location_estimation`. The detailed input channel information is in `modules/perception/camera_location_refinement/dag/camera_location_refinement.dag` file. By default, the upstream components of the messages received by the component include `camera_location_estimation`.

#### Output

| Name | Type | Description | Output channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::CameraFrame | camera frame message | /perception/inner/location_refinement |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/camera_location_refinement/conf/camera_location_refinement_config.pb.txt` file.

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。