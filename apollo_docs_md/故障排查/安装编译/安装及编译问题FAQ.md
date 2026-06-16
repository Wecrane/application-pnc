---
title: 安装及编译问题FAQ
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_x95_x85_xE9_x9A_x9C_xE6_x8E_x92_xE6_x9F_xA5_2_xE5_xAE_x89_xE8_xA3_x85_xE7_xBC_x96_95cfb671d65907a3072f06be0923b717.html
category: 故障排查 > 安装编译 > 安装及编译问题FAQ
---

# 安装及编译问题FAQ

## 环境相关

| 问题方向 | 使用相关指令 | 相关问题链接 |
| --- | --- | --- |
| 安装docker相关 | bash docker_install.sh | https://apollo.baidu.com/community/article/1156 |
| aem 工具、Apollo下载&Apollo启动相关 | sudo apt install apollo-neo-env-manager-dev aem start aem -h | https://apollo.baidu.com/community/article/1157 |
| 拉取工程目录 | git clone | https://apollo.baidu.com/community/article/1158 |
| dreamview相关 | aem bootstrap start aem bootstrap start --plus | https://apollo.baidu.com/community/article/1159 |
| buildtool相关 | buildtool build buildtool -h buildtool install buildtool create | https://apollo.baidu.com/community/article/1227 |
| 编译相关 | buildtool build buildtool build -p modules/ apollo.sh build | https://apollo.baidu.com/community/article/1160 |

## 感知相关

| 问题方向 | 相关问题链接 |
| --- | --- |
| 如何安装GPU驱动【ubuntu系统】 | https://apollo.baidu.com/community/article/1181 |
| GPU驱动问题FAQ | https://apollo.baidu.com/community/article/1228 |
| 40系列显卡新镜像支持 | https://apollo.baidu.com/community/article/1212 |
| 40系列显卡使用Apollo指南 | https://apollo.baidu.com/community/article/1201 |
| 9.0 模型安装指南 | https://apollo.baidu.com/community/article/1236 |

由于使用的硬件、安装的ubuntu系统的镜像以及系统配置不一致等原因，开发者在上述流程中可能会遇到各种问题，我们总结了这个过程一些常见的问题以及原因和解决方案，方便各开发者进行排查，顺利跑通上述流程

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。