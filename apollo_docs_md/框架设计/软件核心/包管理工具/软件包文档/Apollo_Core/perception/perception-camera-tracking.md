---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2camera__tracking_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-camera-tracking
---

# README

# perception-camera-tracking

## Introduction

The function of camera tracking module is to process the camera detection obstacles by using the omt obstacle tracker, and output the tracked obstacles for following multi sensor fusion.

## directory structure

├── camera_tracking     // camera object tracking module
├── base            // definition of tracked_target, frame_list, etc.
├── common          // common used functions
├── conf            // module configuration files
├── dag             // dag files
├── data            // tracker configuration files
├── feature_extract // feature extractor for camera tracking
├── interface       // function interface files
├── proto           // proto files
├── tracking        // omt obstacle tracker
├── camera_tracking_component.cc
├── camera_tracking_component.h // component interface
├── cyberfile.xml   // package management profile
├── README.md
└── BUILD
fragment

## modules

### CameraTrackingComponent

[apollo::perception::camera::CameraTrackingComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1perception_1_1camera_1_1CameraTrackingComponent.html)

#### input

| Name | Type | Description | Input channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::CameraFrame | camera detection frame | /perception/inner/location_refinement |

>Note: The input channel is structure type data. The default trigger channel is `/perception/inner/location_refinement`. The detailed input channel information is in `modules/perception/camera_tracking/dag/camera_tracking.dag` file. By default, the upstream components of the messages received by the component include `camera_location_refinement` and `camera_detection_single_stage`.

#### output

| Name | Type | Description | Output channal |
| --- | --- | --- | --- |
| frame | apollo::perception::onboard::SensorFrameMessage | camera tracked obstacles | /perception/inner/PrefusedObjects |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/camera_tracking/conf/camera_tracking_config.pb.txt` file.

#### how to launch module

1. modify the file: `modules/perception/camera_tracking/dag/camera_tracking.dag`

- config_file_path: the path of module config files
- reader channel: the channel name of input message

1. modify the file: `modules/perception/camera_tracking/conf/camera_tracking_config.pb.txt`

- output_channel_name: the channel name of output message
- image_width and image height: the size of input images
- gpu_id: the gpu id
- plugin_param.name: the name of used camera tracker
- plugin_param.config_path: the config path of used camera tracker
- plugin_param.config_file: the config name of used camera tracker

1. run `mainboard -d modules/perception/camera_tracking/dag/camera_tracking.dag`

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。