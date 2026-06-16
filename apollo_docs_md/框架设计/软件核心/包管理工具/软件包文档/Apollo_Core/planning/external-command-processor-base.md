---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2external__command_2command__processor_2command__processor__base_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > external-command-processor-base
---

# README_cn

# external-command-processor-base

## 介绍

`external-command-processor-base`是外部命令处理基类package，包含基类的接口定义和实现。

![](https://apollo.baidu.com/docs/apollo/latest/modules/external_command/command_processor/command_processor_base/docs/images/class.png)

### CommandProcessorBase

基类`apollo::external_command::CommandProcessorBase`主要包含以下实现内容：

1. 定义了外部命令处理类的接口：  ++
virtual bool GetCommandStatus(int64_t command_id, CommandStatus* status) const = 0;
 fragment  通过这个函数可以实时获取命令当前的执行状态，正在进行中，完成或者发生错误。
1. 创建了apollo::cyber::Node，方便每个子类订阅输入的topic时使用。
1. 实现从配置文件读取配置参数的方法：  ++
const CommandProcessorConfig& GetProcessorConfig() const;
 fragment 

### MotionCommandProcessorBase

`apollo::external_command::MotionCommandProcessorBase`是包含运动命令处理的模板基类，它主要包含以下实现内容：

1. 重写Init初始化函数，初始化导航命令相关的`apollo::cyber::Reader`，`apollo::cyber::Writer`或`apollo::cyber::Service`。
1. 实现了基类函数GetCommandStatus，通过订阅`apollo::external_command::CommandStatus`，并匹配command_id，获取命令执行的状态。
1. 定义纯虚函数Convert，将输入的导航命令转换成`apollo::routing::RoutingRequest`。这是因为如果外部命令如果是点到点的沿道路行驶，需要先搜索routing线路。而routing模块搜索的输入格式是RoutingRequest，所以需要先将外部命令转换成RoutingRequest。如果外部命令中不包含点到点沿道路行驶的命令（如指定线路行驶），则不需要转换。  ++
virtual bool Convert(const std::shared_ptr<T>& command, std::shared_ptr<apollo::routing::RoutingRequest>& routing_request) const = 0;
 fragment 
1. 实现了处理外部命令的接口，接收到外部命令请求时，首先转换其包含的路由线路请求；如果有路由线路请求，调用routing模块搜索；最后对包含的特殊命令进行处理，这一部分在子类中进行实现。  ++
void OnCommand(const std::shared_ptr<T>& command, std::shared_ptr<CommandStatus>& status);
 fragment 
1. 定义了处理特殊命令的接口，用于子类对自身的命令进行特殊处理。  ++
virtual bool ProcessSpecialCommand(const std::shared_ptr<T>& command, const std::shared_ptr<apollo::planning::PlanningCommand>& planning_command) const = 0;
 fragment 

## 目录结构

modules/external_command/command_processor/command_processor_base/
├── command_processor_base
├── docs                             // 说明文档
├── proto                            // 公共（全局）参数配置结构定义
├── BUILD                            // 构建规则文件
├── cyberfile.xml                    // 包管理配置文件
├── command_processor_base.cc        // 外部命令处理基类源码
├── command_processor_base.h         // 外部命令处理基类头文件
├── motion_command_processor_base.h  // 运动命令处理基类头文件
└── README_cn.md                     // 说明文档
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。