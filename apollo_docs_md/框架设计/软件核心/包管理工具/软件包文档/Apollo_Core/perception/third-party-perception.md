---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2third__party__perception_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > third-party-perception
---

# README

# Third Party Perception

## Introduction

In Apollo 2.5, the third_party_perception module incorporates third-party sensors like Mobileye and Conti/Delphi Radar output with simple fusion and creates a similar perception output produced as obstacle/lane detection information as defined in [Perception Obstacles Interface](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/proto/perception_obstacle.proto). This module was only intend to serve for the Prediction/Planning/Control algorithm in real vehicle before perception modules fully ready before 2.5. We recommend using 'modules/perception' instead for your own test purpose after Apollo 2.5 officially released.

## Input

The perception module inputs are:

- Radar data (ROS topic _/apollo/sensor/conti_radar_ or _/apollo/sensor/delphi_esr_ )
- Mobileye data (ROS topic _/apollo/sensor/mobileye_)

## Output

The perception module outputs are:

- The 3D obstacle tracks with the heading, velocity and classification information (ROS topic _/apollo/perception/obstacles_)
- The lane marker information with fitted curve parameter, spatial information(l0,r0, etc) as well as semantic information (lane type) (ROS topic _/apollo/perception/obstacles_)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。