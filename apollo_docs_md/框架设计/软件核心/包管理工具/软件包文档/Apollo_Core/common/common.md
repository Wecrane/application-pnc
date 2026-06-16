---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2common_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > common > common
---

# README_cn

# 通用模块

本模块中的代码不是针对某一模块的。
fragment

## adapters

不同的模块间能通过topic彼此通信。在`adapter_flags`中定义了大量的topic名称。
fragment

## configs/data

可以在这里对车辆进行配置。
fragment

## filters

实现了一些滤波器类，包括低通数字滤波器、均值滤波器等。
fragment

## kv_db

用于存储系统范围参数的轻量级“键-值”数据库。
fragment

## latency_recorder

可以记录时延。
fragment

## math

包含许多有用的数学库。
fragment

## monitor_log

定义日志记录系统。
fragment

## proto

定义多个项目范围的序列化数据。
fragment

## status

用于确定某些函数是否成功执行，
否则提供有用的错误消息。
fragment

## util

包含带有注册的工厂设计模式的实现，
一些字符串解析函数，以及一些解析工具用来解析序列化数据。
fragment

## vehicle_state

该类指定车辆的当前状态（例如位置、速度、航向等）。
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。