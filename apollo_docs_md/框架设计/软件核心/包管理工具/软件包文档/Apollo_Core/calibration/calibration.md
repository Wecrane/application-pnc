---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2calibration_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > calibration > calibration
---

# README

# calibration

## Introduction

Apollo currently supports the following vehicle models by default. You can select different vehicle models in HMI mode and run the Apollo autonomous driving module by playing records.

- kitti_140 - KITTI dataset collection vehicle
- mkz_121 - vehicle for test
- mkz_example - vehicle for simulation
- mkz_lgsvl_321 - vehicle for lgsvl simulation
- nuscenes_165 - nuScenes dataset collection vehicle

## Directory Structure

modules/calibration/
├── BUILD
├── cyberfile.xml
├── data
│   ├── kitti_140
│   ├── mkz_121
│   ├── mkz_example
│   ├── mkz_lgsvl_321
│   └── nuscenes_165
└── README.md
fragment

## Naming rules

`vehicle model` = `vehicle name` + `_` + `lidar_num` + `camera_num` + `radar_num`. For example 

kitti_140

kitti // vehicle name
1     // 1 lidar
4     // 4 camera
0     // 0 radar
fragment

## Configuration

Different autonomous vehicles have different sensor types, numbers, and installation locations, also have different dynamic parameters.

For a well functioning vehicle, generally you need the following configs.

mkz_121
├── camera_params
├── gnss_params           # Params for GNSS
├── lidar_params          # Params for lidar
├── novatel_localization_extrinsics.yaml
├── perception_conf
├── perception_dag
├── radar_params
├── transform_conf
├── vehicle_param.pb.txt   # Instance of apollo.common.VehicleParam
└── vehicle_params
fragment
> 
> ‍Need add control params like "calibration_table.pb.txt" 
> 

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。