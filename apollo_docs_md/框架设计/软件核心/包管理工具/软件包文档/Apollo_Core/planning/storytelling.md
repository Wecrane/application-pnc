---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2storytelling_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > storytelling
---

# README_cn

# Storytelling

## 简介

Storytelling是一个全局性的高级场景管理器，用于协调跨模块的行动。为了在城市道路上安全驾驶自动驾驶汽车，需要复杂的规划场景来确保安全驾驶。这些复杂的场景可能涉及不同的模块以确保适当的机动。为了避免对这种场景采用基于序列的方法，创建了一个新的独立场景管理器，“Storytelling”模块。该模块创建复杂场景的Story，这些Story会触发多个模块的行动。根据一些预定义的规则，该模块创建一个或多个Story，并发布到“/apollo/storytelling”通道。该模块的主要优点是微调驾驶体验，并将复杂的场景隔离成Story，这些Story可以被其他模块（如规划、控制等）订阅。

## 模块

### Storytelling

#### 输入

| Channel 名 | 类型 | 描述 |
| --- | --- | --- |
| /apollo/planning | apollo::planning::ADCTrajectory | planning输出的规划轨迹 |

#### 输出

| Channel 名 | 类型 | 描述 |
| --- | --- | --- |
| /apollo/storytelling | apollo::storytelling::Stories | Storytelling发布的Story |

#### 配置

| 文件路径 | 类型/结构 | 说明 |
| --- | --- | --- |
| modules/storytelling/conf/storytelling_conf.pb.txt | apollo::storytelling::StorytellingConfig | Storytelling的配置文件 |

#### 使用方式

##### 使用 mainboard 启动

mainboard -d modules/storytelling/dag/storytelling.dag
fragment

##### 使用 cyber_launch 启动

```shell cyber_launch start [modules/storytelling/launch/storytelling.launch](https://apollo.baidu.com/docs/apollo/latest/storytelling_8launch.html)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。