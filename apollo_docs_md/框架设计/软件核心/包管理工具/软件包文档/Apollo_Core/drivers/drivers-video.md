---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2drivers_2video_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > drivers > drivers-video
---

# README

# drivers-video

## Introduction

This package is responsible for receiving the native camera image from upd socket and compressing it in h265 encoding.

## Directory Structure

modules/drivers/video/
├── BUILD
├── conf
├── cyberfile.xml
├── dag
├── driver.cc
├── driver.h
├── input.h
├── launch
├── proto
├── README.md
├── socket_input.cc
├── socket_input.h
├── tools                       // tool of converting h265 encoding video to jpeg
├── video_driver_component.cc
└── video_driver_component.h
fragment

## Modules

### SmartereyeComponent

[apollo::drivers::video::CompCameraH265Compressed](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1drivers_1_1video_1_1CompCameraH265Compressed.html)

#### Input

The package will received image is consistent with the user configuration information, only then it starts to parse the data received from the udp socket and compress the image in h265 encoding. In the end send the message and write it to the file specified.

#### Output

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::drivers::CompressedImage | image in h265 encoding |

#### configs

| file path | type / struct | Description |
| --- | --- | --- |
| modules/drivers/video/conf/video_front_6mm.pb.txt | apollo::drivers::video::config::CameraH265Config | video config |

#### How to Launch

cyber_launch start modules/drivers/video/launch/video.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。