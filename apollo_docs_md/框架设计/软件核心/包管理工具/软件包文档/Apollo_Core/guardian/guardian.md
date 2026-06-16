---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2guardian_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > guardian > guardian
---

# README

# v2x

## Introduction

The Guardian module in Apollo is a safety module that monitors the status of the autonomous driving system. When the module fails, it actively cuts off the control command output and triggers the brakes. This module is a bit like a fuse, with a fallback mechanism. It is triggered by two main conditions: the interval between messages reporting the module status exceeds kSecondsTillTimeout (2.5 seconds), or there is a safety_mode_trigger_time field in the reported status message. In safety mode, the vehicle stops immediately if the emergency brake is detected or an obstacle is detected by the ultrasound. In addition, the Guardian module is a timing module with a frequency of 10ms, so it will increase the delay of the control command by a maximum of 10ms.

## Directory Structure

modules/guardian/
├── BUILD
├── conf
├── cyberfile.xml
├── dag
├── guardian_component.cc
├── guardian_component.h
├── launch
├── proto
└── README.md
fragment

#### Input

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::canbus::Chassis | chassis msg |
| msg | apollo::control::ControlCommand | ControlCommand msg |
| msg | apollo::monitor::SystemStatus | SystemStatus msg |

#### Output

| Name | Type | Description |
| --- | --- | --- |
| msg | apollo::guardian::GuardianCommand | GuardianCommand msg |

## configs

| file path | type / struct | Description |
| --- | --- | --- |
| modules/guardian/conf/guardian_conf.pb.txt | apollo::guardian::GuardianConf | - |

#### How to Launch

cyber_launch start modules/guardian/launch/guardian.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。