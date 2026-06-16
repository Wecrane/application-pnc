---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2drivers_2canbus_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > drivers > drivers-canbus
---

# README

# drivers-camera

## Introduction

The driver-canbus module mainly provides an interface for obtaining CAN bus data from the chassis and provides native information to the canbus module.

## Directory Structure

modules/drivers/canbus/
├── BUILD
├── can_client                      // can client implementation
│   ├── can_client_factory.cc
│   ├── can_client_factory.h
│   ├── can_client_factory_test.cc
│   ├── can_client.h
│   ├── can_client_tool.cc
│   ├── esd
│   ├── fake
│   ├── hermes_can
│   └── socket
├── can_comm                        // can receiver implementation
│   ├── can_receiver.h
│   ├── can_receiver_test.cc
│   ├── can_sender.h
│   ├── can_sender_test.cc
│   ├── message_manager.h
│   ├── message_manager_test.cc
│   ├── protocol_data.h
│   └── protocol_data_test.cc
├── common                          // common data structure
│   ├── byte.cc
│   ├── byte.h
│   ├── byte_test.cc
│   └── canbus_consts.h
├── cyberfile.xml
├── proto
├── README.md
├── sensor_canbus.h
├── sensor_gflags.cc
└── sensor_gflags.h
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。