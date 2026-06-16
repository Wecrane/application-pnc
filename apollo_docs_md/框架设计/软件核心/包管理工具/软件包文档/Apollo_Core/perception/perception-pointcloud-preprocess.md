---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2pointcloud__preprocess_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-pointcloud-preprocess
---

# README

# perception-pointcloud-preprocess

## Introduction

The point cloud preprocessing module preprocesses the point cloud data output by the driver. Deleting nan value points, points that are too far away, points scanned onto the self vehicle, and point that are too high.

## Directory Structure

├── pointcloud_preprocess  // point cloud preprocess component
├── conf               // configuration folder
├── dag                // module startup file
├── data               // module configuration parameters
├── launch             // launch file
├── interface          // preprocess interface
├── proto              // preprocess module configuration proto file
├── preprocessor       // preprocess method
├── pointcloud_preprocess_component.cc // component entrance
├── pointcloud_preprocess_component.h
├── cyberfile.xml      // package management file
├── README.md          // readme file
└── BUILD              // compile file
fragment

## Modules

### PointCloudPreprocessComponent

[apollo::perception::lidar::PointCloudPreprocessComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1lidar_1_1PointCloudPreprocessComponent.html)

#### Input

| Name | Type | Description | Input channal |
| --- | --- | --- | --- |
| msg | apollo::drivers::PointCloud | point cloud message | /apollo/sensor/velodyne64/compensator/PointCloud2 |

Point cloud data from driver: If there is one lidar, output point cloud after motion compensation. If there are multiple lidars, concatenate the point clouds into one frame after motion compensation. The default trigger channel is `/apollo/sensor/velodyne64/compensator/PointCloud2`. The detailed input channel information is in `modules/perception/pointcloud_preprocess/dag/pointcloud_preprocess.dag` file.

#### Output

| Name | Type | Description | Output channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::LidarFrameMessage | lidar frame message | /perception/lidar/pointcloud_preprocess |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/pointcloud_preprocess/conf/pointcloud_preprocess_config.pb.txt` file.

#### How to Launch

1. Add vehicle parameter configuration file to `modules/perception/data/params`, corresponding frame_id and sensor_name, launch transform

cyber_launch start modules/transform/launch/static_transform.launch
fragment
1. `Modify modules/perception/launch/perception_lidar.launch`

- select the dag file to start, use `modules/perception/pointcloud_preprocess/dag/pointcloud_preprocess.dag` here
- modify msg_adapter. It is used to wrap messages sent by other steps as `/apollo/perception/obstacles`, this can be used for individual debugging. Modify relevant channel configurations in `modules/perception/data/flag/perception_common.flag`

1. Modify parameters of `modules/perception/pointcloud_preprocess/conf/pointcloud_preprocess_config.pb.txt`

- sensor_name: sensor name
- lidar_query_tf_offset: tf time offset
- output_channel_name: output channel name
- plugin_param: plugin parameters
name: method name
config_path: configuration file path
config_file: configuration file name

1. Launch point cloud preprocess component

cyber_launch start modules/perception/pointcloud_preprocess/launch/pointcloud_preprocess.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。