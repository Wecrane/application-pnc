---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2monitor_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > monitor > monitor
---

# README

# Monitor

## Introduction

This module contains system level software such as code to check hardware status and monitor system health. In Apollo 5.5, the monitor module now performs the following checks among others:

1. Status of running modules
1. Monitoring data integrity
1. Monitoring data frequency
1. Monitoring system health (e.g. CPU, memory, disk usage, etc)
1. Generating end-to-end latency stats report

Note: You can configure the modules that you would like to monitor for the first 3 capabilities mentioned above.
fragment

## Directory Structure

modules/monitor/
├── BUILD
├── common              // common function
├── cyberfile.xml
├── dag
├── hardware            // hardware monitor implementation
├── launch
├── monitor.cc
├── monitor.h
├── README.md
└── software            // software monitor implementation
fragment

## Hardware Monitors

Hardware related monitoring, e.g. CAN card / GPS status health check. Check results are reported back to HMI.

## Software Monitors

### Process Monitor

It checks if a process is running or not. Config it with apollo::monitor::ProcessConf proto, which works similar to 

ps aux | grep <keyword1> | grep <keyword2> | ...
fragment

### Topic Monitor

It checks if a given topic is updated normally. Config it with apollo::monitor::TopicConf proto.

### Summary Monitor

It summarizes all other specific monitor's results to a simple conclusion such as OK, WARN, ERROR or FATAL.

## How to Launch

cyber_launch start modules/monitor/launch/monitor.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。