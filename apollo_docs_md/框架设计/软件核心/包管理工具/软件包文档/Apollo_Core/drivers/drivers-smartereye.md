---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2drivers_2smartereye_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > drivers > drivers-smartereye
---

# README

# drivers-smartereye

## Introduction

This package is responsible for receiving the native camera image of smartereye device inputed from upd socket and parsing it to send it to the perception-camera-detecetion package for subsequent processing.

## Directory Structure

modules/drivers/smartereye/
├── BUILD
├── compress_component.cc
├── compress_component.h
├── conf
├── cyberfile.xml
├── dag
├── drivers-smartereye.BUILD
├── launch
├── proto
├── README.md
├── smartereye_component.cc
├── smartereye_component.h
├── smartereye_device.cc
├── smartereye_device.h
├── smartereye_handler.cc
└── smartereye_handler.h
fragment

## Modules

### SmartereyeComponent

[apollo::drivers::smartereye::SmartereyeComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1drivers_1_1smartereye_1_1SmartereyeComponent.html)

#### Input

The package will received image is consistent with the user configuration information, only then it starts to parse the data received from the udp socket and send the message.

#### Output

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::drivers::smartereye::Image | parsed smartereye image |
| msg | apollo::drivers::smartereye::SmartereyeObstacles | obstacles detected by device |
| msg | apollo::drivers::smartereye::SmartereyeLanemark | lane detected by device |

#### configs

| file path | type / struct | Description |
| --- | --- | --- |
| modules/drivers/smartereye/conf/smartereye.pb.txt | apollo::drivers::smartereye::config::Config | smartereye config |

#### How to Launch

cyber_launch start modules/drivers/smartereye/launch/smartereye.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。