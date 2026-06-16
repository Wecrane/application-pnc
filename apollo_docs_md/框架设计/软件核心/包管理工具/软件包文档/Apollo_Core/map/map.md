---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2map_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > map > map
---

# README

# map

## Introduction

Map module is an important part of Apollo, which is mainly used to load map data in OpenStreetMap format and provides rich map operation APIs, including map drawing, positioning, search and other functions. Through the Apollo Map module, the vehicle can obtain real-time road information, traffic conditions, point-of-interest information, etc., so as to carry out intelligent path planning, navigation and other operations.

## Directory Structure

modules/map/
├── BUILD
├── cyberfile.xml
├── data                // Map data, including vector maps, satellite maps and so on.
├── hdmap               // API implementations of high-precision maps.
├── pnc_map             // API implementations for the planning control module.
├── README.md
├── relative_map        // API implementations for relative map.
├── testdata
└── tools               // tools for processing map data and performing map-related operations.
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。