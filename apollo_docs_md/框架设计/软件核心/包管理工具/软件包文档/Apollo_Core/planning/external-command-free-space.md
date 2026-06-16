---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2external__command_2command__processor_2free__space__command__processor_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > external-command-free-space
---

# README_cn

# external-command-free-space

## 简介

external-command-free-space 插件主要功能是外部接口命令进行封装处理，用户可以通过自定义的接口向PNC模块发送相关控制命令，处理后由PNC模块执行相关的Command，实现方式主要通过Service的通讯方式，通过Client端发送PNC外部请求控制指令，Service对Client端请求命令进行处理后交付PNC模块进行执行。

### Convert

该函数暂未做处理，在`external-command-free-space`中未做任何处理。

### ProcessSpecialCommand

该函数的主要调用在`motion_command_processor_base.cc`文件中的`OnCommand`函数中，`OnCommand`函数的主要目的是透传Client端的Request请求，该函数作为Service的主要功能实现函数，`command`为Client端Request，Client端的创建函数在`external_command_wrapper_demo`中的`Init()`函数中，`ProcessSpecialCommand`函数主要将Client端接收的请求透传给`planning_command`，`planning_command`后续写入`apollo::planning::PlanningCommand`channel中由PNC模块执行相关请求。

## 文件组织结构及说明

command_processor/command_processor/free_space_command_processor/
├── conf/                                                 // 控制器配置参数文件
├── BUILD                                                 // 规则构建文件
├── cyberfile.xml                                         // 插件包管理配置文件
├── free_space_command_processor.cc                       // 任务器实现文件
├── free_space_command_processor.h                        // 任务器实现文件
├── plugins.xml                                           // 插件配置文件
└── README_cn.md                                          // 说明文档
fragment

## 模块输入输出与配置

### external-command-chassis插件

#### 输入

| service 名 | Request类型 | Response类型 | 描述 |
| --- | --- | --- | --- |
| /apollo/external_command/free_space | apollo::external_command::FreeSpaceCommand | apollo::external_command::CommandStatus | 开阔场景下车辆的Command请求 |

#### 输入

| Channel名称 | 类型 | 描述 |
| --- | --- | --- |
| /apollo/planning/command_status | apollo::external_command::CommandStatus | 车辆Planning模块请求的实际状态 |

#### 配置文件

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/external_command/command_processor/free_space_command_processor/config.pb.txt | apollo::external_command::CommandProcessorConfig | free_space_command_processor的输入、输出channel配置文件 |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。