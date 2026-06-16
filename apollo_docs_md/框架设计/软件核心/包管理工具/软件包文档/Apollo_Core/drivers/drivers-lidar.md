---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2drivers_2lidar_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > drivers > drivers-lidar
---

# README

# drivers-lidar

## Introduction

This module consists of two parts, the first part is the LIDAR driver, which reads the LIDAR data and parses it when it starts. The second part is the LIDAR parser, which receives the raw radar data published by the driver and publishes the converted point cloud data.

## Directory Structure

modules/drivers/lidar/
├── BUILD
├── common                      // common function and data structure
├── conf
├── cyberfile.xml
├── dag
├── hesai                       // hesai lidar drivers
├── launch
├── lidar_driver_component.cc
├── lidar_driver_component.h
├── lidar_robosense
├── proto
├── README.md
├── robosense                   // robosense lidar drivers
└── velodyne                    // velodyne lidar drivers
fragment

## Modules

### LidarDriverComponent

[apollo::drivers::lidar::LidarDriverComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1drivers_1_1lidar_1_1LidarDriverComponent.html)

#### Input

| Name | Type | Description |
| --- | --- | --- |
| socket | scan raw packet buffer | raw packet |

#### Output

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::drivers::velodyne::VelodyneScan | scan raw output |
| msg | apollo::drivers::PointCloud | raw pointcloud |
| msg | apollo::drivers::PointCloud | compensation pointcloud |

#### configs

| file path | type / struct | Description |
| --- | --- | --- |
| modules/drivers/lidar/lidar_config.pb.txt | apollo::drivers::lidar::config | lidar config |

#### How to Launch

cyber_launch start modules/drivers/lidar/launch/driver.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。