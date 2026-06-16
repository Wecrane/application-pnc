---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2lidar__detection_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-lidar-detection
---

# README

# perception-lidar-detection

## Introduction

Perform 3D object detection based on the point cloud, and output the position, size, and orientation of the detected objects. Apollo provides 4 lidar detection models: centerpoint, maskpillars, pointpillars, cnnseg.

## Structure

├── lidar_detection
├── conf            // component config file
├── dag             // component dag file
├── data            // model config files
├── detector        // lidar model
│   ├── center_point_detection   // CenterPoint model code
│   ├── cnn_segmentation         // CNNSeg model code
│   ├── mask_pillars_detection   // MaskPillars model code
│   └── point_pillars_detection  // PointPillars model code
├── interface       // definition of BaseLidarDetector
├── proto           // definition of data structure
├── lidar_detection_component.cc // component inference
├── lidar_detection_component.h
├── cyberfile.xml   // package configs
├── README.md
└── BUILD
fragment

## Modules

### LidarDetectionComponent

[apollo::perception::lidar::LidarDetectionComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1lidar_1_1LidarDetectionComponent.html)

#### Input

| Name | Type | Description | Input channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::LidarFrameMessage | lidar frame message | perception/lidar/pointcloud_ground_detection |

>Note: The input channel is structure type data. The default trigger channel is `/perception/lidar/pointcloud_ground_detection`. The detailed input channel information is in `modules/perception/lidar_detection/dag/lidar_detection.dag` file. By default, the upstream components of the messages received by the component include `pointcloud_ground_detection`.

#### Output

| Name | Type | Description | Output channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::LidarFrameMessage | LidarFrame's segmented_objects | /perception/lidar/detection |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/lidar_detection/conf/lidar_detection_config.pb.txt` file.

#### How to use

1. Add vehicle params configs to `modules/perception/data/params`，keep frame_id and sensor_name consistent, start the tf

cyber_launch start modules/transform/launch/static_transform.launch
fragment
1. Select the lidar model，download the model files and put them into `modules/perception/data/models` using `amodel` tool.

amodel install https://xxx.zip
fragment
1. Modify the `modules/perception/lidar_detection/dag/lidar_detection.dag`

- config_file_path: path of config path
- reader channel: the name of input channel

1. Modify `modules/perception/lidar_detection/conf/lidar_detection_config.pb.txt`

- output_channel_name: the name of output channel
- sensor_name: sensor name
- use_object_builder: whether use object builder
- plugin_param
name： the name of lidar model
config_path: the path of model config
config_file: the name of model config

1. start the lidar component

cyber_launch start modules/perception/launch/perception_lidar.launch
fragment

## Reference

1. [Center-based 3D Object Detection and Tracking](https://arxiv.org/abs/2006.11275)
1. [PointPillars: Fast Encoders for Object Detection from Point Clouds](https://arxiv.org/abs/1812.05784)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。