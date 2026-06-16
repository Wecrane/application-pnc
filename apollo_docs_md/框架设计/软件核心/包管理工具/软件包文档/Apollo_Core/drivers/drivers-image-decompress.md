---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2drivers_2tools_2image__decompress_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > drivers > drivers-image-decompress
---

# README

# drivers-image-decompress

## Introduction

This package is responsible for decompressing the image in jpeg encoding to rgb format.

## Directory Structure

modules/drivers/tools/image_decompress
|-- BUILD
|-- README.md
|-- conf
|-- cyberfile.xml
|-- dag
|-- drivers-image-decompress.BUILD
|-- image_decompress.cc
|-- image_decompress.h
|-- image_decompress_test.cc
|-- launch
`-- proto
fragment

## Modules

### ImageDecompressComponent

[apollo::image_decompress::ImageDecompressComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1image__decompress_1_1ImageDecompressComponent.html)

#### Input

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::drivers::CompressedImage | image in jpeg encoding |

#### Output

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::drivers::Image | image in RGB format |

#### configs

| file path | type / struct | Description |
| --- | --- | --- |
| modules/drivers/tools/image_decompress/conf/camera_front_6mm.pb.txt | apollo::image_decompress::Config | image decompress config |

#### How to Launch

cyber_launch start modules/drivers/video/launch/video.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。