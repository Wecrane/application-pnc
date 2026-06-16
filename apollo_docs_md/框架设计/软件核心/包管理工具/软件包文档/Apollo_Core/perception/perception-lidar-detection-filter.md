---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2lidar__detection__filter_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-lidar-detection-filter
---

# README

# perception-lidar-detection-filter

## Introduction

Filter foreground and background objects based on obejcts attributes, lane lines, ROI, etc.

## Structure

├── lidar_detection_filter
├── conf                // component config file
├── dag                 // component dag file
├── data                // object filter config files
├── object_filter_bank          // object filter code
│   ├── object_filter_bank.cc   // object filter inference
│   ├── object_filter_bank.h
│   ├── background_filter    // filter algorithm based on lane
│   └── roi_boundary_filter  // filter algorithm based on roi boundary
│   └── strategy_filter      // merge strategy based on expand-big include small
├── interface       // definition of BaseObjectFilter
├── proto           // definition of data structure
├── lidar_detection_filter_component.cc // component inference
├── lidar_detection_filter_component.h
├── cyberfile.xml   // package configs
├── README.md
└── BUILD
fragment

## Modules

### LidarDetectionFilterComponent

[apollo::perception::lidar::LidarDetectionFilterComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1lidar_1_1LidarDetectionFilterComponent.html)

#### Input

| Name | Type | Description | Input channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::LidarFrameMessage | LidarFrame‘s segmented_objects | /perception/lidar/detection |

>Note: The input channel is structure type data. The default trigger channel is `/perception/lidar/detection`. The detailed input channel information is in `modules/perception/lidar_detection_filter/dag/lidar_detection_filter.dag` file. By default, the upstream components of the messages received by the component include `lidar_detection`.

#### Output

| Name | Type | Description | Output channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::LidarFrameMessage | put the preserved objects into LidarFrame‘s segmented_objects | /perception/lidar/detection_filter |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/lidar_detection_filter/conf/lidar_detection_filter_config.pb.txt` file.

#### How to use

1. Add vehicle params configs to `modules/perception/data/params`，keep frame_id and sensor_name consistent, start the tf

cyber_launch start modules/transform/launch/static_transform.launch
fragment
1. Select the lidar model，download the model files and put them into `modules/perception/data/models` using `amodel` tool.

amodel install https://xxx.zip
fragment
1. Modify the `modules/perception/lidar_detection_filter/dag/lidar_detection_filter.dag`

- config_file_path: path of config path
- reader channel: the name of input channel

1. Modify `modules/perception/lidar_detection_filter/conf/lidar_detection_filter_config.pb.txt`

- output_channel_name: the name of output channel
- use_object_filter_bank: whether use object filter
- config_path: the path of config
- config_file: the name of config

1. start the lidar component

cyber_launch start modules/perception/launch/perception_lidar.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。