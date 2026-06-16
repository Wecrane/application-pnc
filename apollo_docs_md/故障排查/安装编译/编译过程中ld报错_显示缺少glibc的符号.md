---
title: 编译过程中ld报错 显示缺少glibc的符号
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_x95_x85_xE9_x9A_x9C_xE6_x8E_x92_xE6_x9F_xA5_2_xE5_xAE_x89_xE8_xA3_x85_xE7_xBC_x96_2d1b5d521eaecb41ed6cd04213f544d5.html
category: 故障排查 > 安装编译 > 编译过程中ld报错 显示缺少glibc的符号
---

# 编译过程中ld报错 显示缺少glibc的符号

### 问题描述:

编译报错: collect2: error: ld returned 1 exit status，并且出现glibc符号

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/安装编译/images/build_error_ld.png)

### 问题原因:

在arm架构下的nvidia-docker-runtime，会将宿主机的和gpu相关的一些动态库挂载进容器内。 目前apollo适配的ubuntu系统版本是20.04，对应glibc版本是2.31，当用户使用22.04的ubuntu时，对应的glibc版本是2.33+，容器内缺少这些符号，导致链接这些动态库的时候ld报错。

### 解决方案:

arm架构下，目前apollo支持的版本是ubuntu 20.04，建议用户使用这一版本的ubuntu x86则没有这个问题

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。