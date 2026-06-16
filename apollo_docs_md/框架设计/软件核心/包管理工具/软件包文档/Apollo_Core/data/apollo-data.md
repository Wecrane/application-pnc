---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2data_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > data > apollo-data
---

# README

# apollo-data

## Introduction

Package apollo-data contains a tool named smart recorder, which is targeting to reduce the recording data size. Instead of recording all the topics all the time, it selectively records by the following way.

1. All the small topics all the time. These include "/apollo/localization/pose", "/apollo/canbus/chassis" and so on, which are samll in size.
1. Large topics only in specified scenarios. These include sensor data include all "PointCloud" and "Camera" topics, which are large in size. The specific scenarios are configurable, as well as the time range for recording when the specified scenario occurs.

## Directory Structure

modules/data/
├── BUILD
├── cyberfile.xml
├── proto
├── README.md
└── tools           // smart recorder
fragment

## How to use

1. Build apollo in source env or install `apollo-scripts` and `apollo-data` package in package management env
1. python3 /apollo/scripts/record_message.py –help

## How to add new scenarios

1. Configure the new scenario in [conf/smart_recorder_config.pb.txt](https://apollo.baidu.com/docs/apollo/latest/smart__recorder__config_8pb_8txt.html), including time range, name, description and etc.
1. Add new class inherit from base class "TriggerBase", and implement interface "Pull"
1. Add the instance of the new class into triggers pool inside function "ProcessRecord::InitTriggers"

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。