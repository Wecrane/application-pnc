---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2traffic__light__region__proposal_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-traffic-light-region-proposal
---

# README

# perception-traffic-light-region-proposal

## Introduction

This module is used to query positioning and map signal light information, select a camera for detecting traffic lights according to the projection of the signal light on the image plane, and finally save the camera selection result.

## Directory Structure

├── traffic_light_region_proposal // trafficlight region proposal module
├── conf            // module configuration files
├── dag             // dag files
├── interface       // function interface folder
├── launch          // launch files
├── preprocessor    // main part for preprocess
├── proto           // proto files
├── traffic_light_region_proposal_component.cc // component interface
├── traffic_light_region_proposal_component.h
├── cyberfile.xml   // package management profile
├── README.md
└── BUILD
fragment

## Modules

### TrafficLightsPerceptionComponent

apollo::perception::onboard::TrafficLightsPerceptionComponent

#### Input

| Name | Type | Description | Input channal |
| --- | --- | --- | --- |
| msg | apollo::drivers::Image | camera sensor image | /apollo/sensor/camera/front_6mm/image   /apollo/sensor/camera/front_6mm/image |
| hd-map | apollo::perception::map::HDMapInput | HD map | - |

>Note: Image data from driver. The default trigger channel include `/apollo/sensor/camera/front_6mm/image` and `/apollo/sensor/camera/front_12mm/image`. The detailed input channel information is in `modules/perception/traffic_light_region_proposal/conf/traffic_light_region_proposal_config.pb.txt` file.

#### Output

| Name | Type | Description | Output channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::TrafficDetectMessage | trafficlight message | /perception/inner/Detection |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/traffic_inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process.

#### How to Launch

cyber_launch start modules/perception/traffic_light_region_proposal/launch/traffic_light_region_proposal.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。