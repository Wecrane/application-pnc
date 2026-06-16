---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2drivers_2gnss_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > drivers > drivers-gnss
---

# README

# drivers-gnss

## Introduction

The GNSS driver is mainly responsible for receiving and processing GNSS signals and passing them to the positioning module to realize high-precision positioning.

## Directory Structure

modules/drivers/gnss/
├── BUILD
├── conf
├── cyberfile.xml
├── dag
├── gnss_component.cc
├── gnss_component.h
├── launch
├── parser              // parser to parse different kinds of gnss data
├── proto
├── README.md
├── stream              // stream handler to process gnss data from different data streams
├── test
├── test_data
└── util                // common function
fragment

## Modules

### GnssDriverComponent

[apollo::drivers::gnss::GnssDriverComponent](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1drivers_1_1gnss_1_1GnssDriverComponent.html)

#### Input

| Name | Type | Description |
| --- | --- | --- |
| data stream | binary data from gnss stream | - |
| msg | apollo::canbus::Chassis | chassis message |

#### Output

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::drivers::gnss::RawData | gps bin raw data |
| msg | apollo::drivers::gnss::RawData | gps rtcm raw data |
| msg | apollo::drivers::gnss::RawData | gnss stream raw data |
| msg | apollo::drivers::gnss::StreamStatus | gnss stream status |

#### configs

| file path | type / struct | Description |
| --- | --- | --- |
| modules/drivers/gnss/gnss_conf.pb.txt | apollo::drivers::gnss::conf::Config | gnss config |

#### How to Launch

cyber_launch start modules/drivers/gnss/launch/gnss.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。