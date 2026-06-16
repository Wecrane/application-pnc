---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2external__command_2command__processor_2chassis__command__processor_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > external-command-chassis
---

# README_cn

# external-command-chassis

## 简介

external-command-chassis 插件主要功能是对底盘Chassis内部的接口命令进行封装处理，用户可以通过自定义的HMI接口向底盘发送相关控制命令；实现方式主要通过Service的通讯方式，通过Client端发送园区场景下的底盘Chassis外部请求控制指令（例如"LEFT_TURN"和"雨刷请求等），Service对Client端请求命令进行处理后交付PNC模块进行执行；该模块同时对Chassis命令执行后的状态进行监控上报。

@subsubsection autotoc_md3525 OnCommand
该函数的主要目的是透传Client端的Request请求，该函数作为Service的主要功能实现函数，<tt>command</tt>为Client端Request，Client端的创建函数在<tt>external_command_wrapper_demo</tt>中的<tt>Init()</tt>函数中，Client端的Request在<tt>external_command_wrapper_demo</tt>中的<tt>SendVehicleSignalCommand()</tt>和<tt>SendCustomChassisCommand()</tt>函数中写入；<tt>status</tt>为Service端的Response状态，<tt>OnCommand</tt>函数主要将Client端的请求写入<tt>/apollo/canbus/chassis_control</tt>channel中，后续由PNC模块对请求进行响应。

@subsubsection autotoc_md3526 GetCommandStatus
该函数的调用主要是在<tt>external_command_process_component.cc</tt>中的<tt>Init()</tt>函数中，该函数的主要作用是在Client发送完毕请求后，判断底盘Chassis命令执行后的状态进行监控上报，主要监控底盘的转向灯、远近光灯、喇叭以及双闪灯的状态。

@subsection autotoc_md3527 文件组织结构及说明

@icode{shell} 
command_processor/command_processor/chassis_command_processor/
├── conf/                                                 // 控制器配置参数文件
├── BUILD                                                 // 规则构建文件
├── cyberfile.xml                                         // 插件包管理配置文件
├── chassis_command_processor.cc                          // 任务器实现文件
├── chassis_command_processor.h                           // 任务器实现文件
├── plugins.xml                                           // 插件配置文件
└── README_cn.md                                          // 说明文档
@endicode 

@subsection autotoc_md3528 模块输入输出与配置

@subsubsection autotoc_md3529 external-command-chassis插件

@paragraph autotoc_md3530 输入

<table class="markdownTable">
  <tr class="markdownTableHead">    <th class="markdownTableHeadNone"> service 名   

Request类型   

Response类型   

描述

`/apollo/external_command/chassis` 

`apollo::external_command::ChassisCommand` 

`apollo::external_command::CommandStatus` 

自定义底盘命令（园区场景下的左转和雨刷等命令）   

#### 输入

| Channel名称 | 类型 | 描述 |
| --- | --- | --- |
| /apollo/canbus/chassis | apollo::canbus::Chassis | 车辆底盘信息（车辆实际转向角，车辆速度） |

#### 输出

| Channel名称 | 类型 | 描述 |
| --- | --- | --- |
| /apollo/canbus/chassis_control | apollo::external_command::ChassisCommand | 车辆的控制指令：刹车、油门、EPB手刹指令 |

#### 配置文件

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/external_command/command_processor/chassis_command_processor/config.pb.txt | apollo::external_command::CommandProcessorConfig | external-command-chassis的输入、输出channel配置文件 |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。