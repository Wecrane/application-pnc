---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2routing_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > routing
---

# README_cn

# 路由

## 介绍

路由模块根据请求生成高级导航信息。

路由模块依赖于路由拓扑文件，通常称为Apollo中的routing_map.*。路由地图可以通过命令来生成。 

bash scripts/generate_routing_topo_graph.sh
fragment

## 输入

- 地图数据
- 路由请求（开始和结束位置） 

## 输出

- 路由导航信息

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。