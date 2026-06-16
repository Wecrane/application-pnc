---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2pointcloud__motion_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-pointcloud-motion
---

# README

# Module Name

pointcloud_motion

# Introduction

The pointcloud motion module assigns a motion label to each point for pointcloud.

# Directory Structure

├── pointcloud_motion   // pointcloud_motion component
├── conf               // configuration folder
├── dag                // module dag file
├── data               // module configuration parameters
├── interface          // interface of parser
├── launch             // launch file
├── parser             // parser method
├── proto              // pointcloud motion module configuration proto file
├── BUILD              // build file
├── cyberfile.xml      // package management file
├── pointcloud_motion_component.cc  // component entrance
├── pointcloud_motion_component.h
└── README
fragment

# Module Input and Output

## Input

| Name | Type | Description |
| --- | --- | --- |
| msg | onboard::LidarFrameMessage | lidar frame message |

## Output

| Name | Type | Description |
| --- | --- | --- |
| frame | onboard::LidarFrameMessage | lidar frame message |

# How to Launch

1. Add vehicle parameter configuration file to modules/perception/data/params in profile, corresponding frame_id and sensor_name, launch transform cyber_launch start /apollo/modules/transform/launch/static_transform.launch
 fragment 
1. Modify [modules/perception/launch/perception_lidar.launch](https://apollo.baidu.com/docs/apollo/latest/perception__lidar_8launch.html)

- select the dag file to start, add `pointcloud_motion.dag` to [perception_lidar.launch](https://apollo.baidu.com/docs/apollo/latest/perception__lidar_8launch.html)
- modify msg_adapter. It is used to wrap messages sent by other steps as /apollo/perception/obstacles, this can be used for individual debugging. Modify relevant channel configurations in [modules/perception/data/flag/perception_common.flag](https://apollo.baidu.com/docs/apollo/latest/perception__common_8flag.html)

1. Modify parameters of [modules/perception/pointcloud_motion/conf/pointcloud_motion.pb.txt](https://apollo.baidu.com/docs/apollo/latest/pointcloud__motion_8pb_8txt.html) in profile

- output_channel_name: output channel name
- plugin_param: plugin parameters
name: method name
config_path: configuration folder
config_file: configuration file name

1. Launch pointcloud perception cyber_launch start /apollo/modules/perception/launch/perception_lidar.launch
 fragment 

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。