---
title: 包管理如何下载planning所有模块源码
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_x95_x85_xE9_x9A_x9C_xE6_x8E_x92_xE6_x9F_xA5_2_xE5_xAE_x89_xE8_xA3_x85_xE7_xBC_x96_b2fa1d0bf824e4cbf66708be88c99a08.html
category: 故障排查 > 安装编译 > 包管理如何下载planning所有模块源码
---

# 包管理如何下载planning所有模块源码

### 问题描述

在使用buildtool install planning时，只会将planning_component模块源码下载到本地，如何将planning相关所有模块全部下载？

### 解决方案

可通过通配符"*"下载planning相关所有模块源码 

buildtool install planning*
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。