---
title: 安装及编译问题FAQ–docker
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_207__FAQ_xE6_x95_x85_xE9_x9A_x9C_xE6_x31c4aab8f16244a4f3f147ad8bc079c8.html
category: 赛事总汇 > 07_FAQ故障排查 > 安装及编译问题FAQ--docker
---

# 安装及编译问题FAQ–docker

# 安装及编译问题FAQ–docker

> 
> ‍作者: 宇新 | 发布时间: 2023-10-23 14:24 | 链接: https://apollo.baidu.com/community/article/1156 
> 

## 安装docker

开发者在使用apollo提供的脚本安装docker时，可能会遇到以下问题

# 常见问题问题一：

安装docker时，脚本运行完apt-get update后提示仓库无效或不含release文件错误： 

+ sudo -E sh -c apt-get update -qq >/dev/null
E: 仓库 "file:/cdrom lunar Release" 不再含有 Release 文件
Failed to restart docker.service: Unit docker.service not found
fragment

## 问题原因：

造成这种现象的原因是由于开发者安装的系统的apt源配置有问题，导致apt无法更新软件包信息

## 解决方法：

切换到可用apt源即可，例如阿里源，将文件/etc/apt/sources.list的内容替换成阿里源即可。

注：替换源的时候请确保系统版本与apt源相匹配，例如：

deb http://mirrors.aliyun.com/ubuntu/ focal main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ focal main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ focal-security main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ focal-security main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ focal-updates main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ focal-updates main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ focal-backports main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ focal-backports main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ focal-proposed main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ focal-proposed main restricted universe multiverse
fragment

 是20.04的apt源，18.04和22.04可以百度对应版本的apt源

# 常见问题2:

如若出现无法安装docker的情况，可以参考以下方法解决：

在使用脚本安装Docker时遇到了无法安装的问题，出现这种情况的原因是什么？

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/6b458daad73cebc9abbd16465eeaf3c2.png)

## 问题原因:

出现这种问题的原因通常是网络问题，可能是由于下载所需的依赖文件时出现了错误或下载速度缓慢导致的。

## 解决方式：

可以尝试通过以下命令来解决这个问题：

sudo apt install docker.io
fragment

 这个命令可以通过APT包管理器安装Docker，即使您在使用脚本时出现了问题，也可以使用这个命令手动安装Docker。

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。