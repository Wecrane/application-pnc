---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2barrier__recognition_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > perception > perception-barrier-recognition
---

# README

# perception-barrier-recognition

## Introduction

This module is used to recognize the status of barrier gate nearby. The status includes opened, closed, opening and closing.

## Directory Structure

├── barrier_recognition // barrier recognition module
├── conf            // module configuration files
├── dag             // dag files
├── interface       // function interface folder
├── launch          // launch files
├── detector        // main part for recognition
├── proto           // proto files
├── tracke          // part for tracker
├── barrier_recognition_component.cc // component interface
├── barrier_recognition_component.h
├── cyberfile.xml   // package management profile
├── README.md
└── BUILD
fragment

#### How to Launch

cyber_launch start modules/perception/barrier_recognition/launch/barrier_recognition.launch
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。