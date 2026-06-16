---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2transform_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > transform > transform
---

# README

# transform

## Introduction

The transform module is used to read the transform configuration file to publish static transform relationships. This module can publish different static coordinate transformations by defining different configurations.

## Directory Structure

modules/transform
├── buffer.cc
├── buffer.h
├── buffer_interface.h
├── BUILD
├── conf
├── cyberfile.xml
├── dag
├── launch
├── proto
├── README.md
├── static_transform_component.cc
├── static_transform_component.h
├── static_transform_component_test.cc
├── transform_broadcaster.cc
└── transform_broadcaster.h
fragment

#### Input

None

#### Output

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::transform::Transform | tf msg |

## configs

| file path | type / struct | Description |
| --- | --- | --- |
| modules/transform/conf/static_transform_conf.pb.txt | apollo::static_transform::Conf | tf config |

#### How to Launch

cyber_launch start modules/transform/launch/static_transform.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。