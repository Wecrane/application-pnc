---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2lidar__tracking_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-lidar-tracking
---

# README

# perception-lidar-tracking

## Introduction

The tracking module is to track the trajectory of obstacles, update their motion status and geometric shape, and assign tracking id.

## Directory Structure

├── lidar_tracking         // tracking module
├── classifier         // classfier
├── conf               // configuration folder
├── dag                // module startup file
├── data               // module configuration parameters
├── launch             // launch file
├── interface          // interface
├── proto              // tracking module configuration proto file
├── tracker            // tracking functions
├── lidar_tracking_component.cc // component entrance
├── lidar_tracking_component.h
├── cyberfile.xml      // package management file
├── README.md          // readme file
└── BUILD              // compile file
fragment

## Modules

### LidarTrackingComponent

[apollo::perception::lidar::LidarTrackingComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1lidar_1_1LidarTrackingComponent.html)

#### Input

| Name | Type | Description | Input channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::LidarFrameMessage | lidar frame message | /perception/lidar/detection_filter |

>Note: The input channel is structure type data. The default trigger channel is `/perception/lidar/detection_filter`. The detailed input channel information is in `modules/perception/lidar_tracking/dag/lidar_tracking.dag` file. By default, the upstream components of the messages received by the component include `lidar_detection_filter`.

#### Output

| Name | Type | Description | Output channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::SensorFrameMessage | sensor frame message | /perception/inner/PrefusedObjects |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/lidar_tracking/conf/lidar_tracking_config.pb.txt` file.

SensorFrameMessage is used in multi sensor fusion.

#### How to Launch

1. Add vehicle parameter configuration file to `modules/perception/data/params`, corresponding frame_id and sensor_name, launch transform

cyber_launch start modules/transform/launch/static_transform.launch
fragment
1. Modify `modules/perception/launch/perception_lidar.launch`

- select the dag file to start, use `modules/perception/lidar_tracking/dag/lidar_tracking.dag` here
- modify msg_adapter. It is used to wrap messages sent by other steps as `/apollo/perception/obstacles`, this can be used for individual debugging. Modify relevant channel configurations in `modules/perception/data/flag/perception_common.flag`

1. Modify parameter of `modules/perception/lidar_tracking/conf/lidar_tracking_config.pb.txt`

- output_channel_name: output channel name
- multi_target_tracker_param: multi traget tracking parameters
name: method name
config_path: configuration path
config_file: configuration file name

- fusion_classifier_param: classification parameters
name: method name
config_path: configuration folder
config_file: configuration file name

1. Launch tracking component

cyber_launch start modules/perception/lidar_tracking/launch/lidar_tracking.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。