---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2lidar__output_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-lidar-output
---

# README

# perception-lidar-output

## Introduction

Convert the SensorFrameMessage to PerceptionObstacles.

## Structure

├── lidar_output
├── conf         // component config file
├── dag          // component dag file
├── proto        // definition of data structure
├── lidar_output_component.cc // component inference
├── lidar_output_component.h
├── cyberfile.xml   // package configs
├── README.md
└── BUILD
fragment

## Modules

### LidarOutputComponent

[apollo::perception::lidar::LidarOutputComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1lidar_1_1LidarOutputComponent.html)

#### Input

| Name | Type | Description | Input channal |
| --- | --- | --- | --- |
| message | apollo::perception::onboard::SensorFrameMessage | sensor frame message | /perception/inner/PrefusedObjects |

>Note: The input channel is structure type data. The default trigger channel is `/perception/inner/PrefusedObjects`. The detailed input channel information is in `modules/perception/lidar_output/dag/lidar_output.dag` file.

#### Output

| Name | Type | Description | Output channal |
| --- | --- | --- | --- |
| out_message | apollo::perception::onboard::PerceptionObstacles | lidar output message | /apollo/perception/obstacles |

>Note: The output channel is proto type data. The upstream and downstream components will be started in different processes and can receive the message normally. The detailed output channel information is in `modules/perception/lidar_output/conf/lidar_output_config.pb.txt` file.

#### How to use

1. Add vehicle params configs to `modules/perception/data/params`，keep frame_id and sensor_name consistent, start the tf

cyber_launch start modules/transform/launch/static_transform.launch
fragment
1. Select the lidar model，download the model files and put them into `modules/perception/data/models` using `amodel` tool.

amodel install https://xxx.zip
fragment
1. Modify `modules/perception/lidar_output/dag/lidar_output.dag`

- config_file_path: path of config path
- reader channel: the name of input channel

1. Modify `modules/perception/lidar_output/conf/lidar_output_config.pb.txt`

- output_channel_name: the name of output channel

1. start the lidar component

mainboard -d modules/perception/lidar_output/dag/lidar_output.dag
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。