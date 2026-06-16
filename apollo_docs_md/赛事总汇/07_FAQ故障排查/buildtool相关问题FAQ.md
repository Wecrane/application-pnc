---
title: buildtool相关问题FAQ
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_207__FAQ_xE6_x95_x85_xE9_x9A_x9C_xE6_x421fade9e0a8991f0f2bc29513895bdd.html
category: 赛事总汇 > 07_FAQ故障排查 > buildtool相关问题FAQ
---

# buildtool相关问题FAQ

# buildtool相关问题FAQ

> 
> ‍作者: 宇新 | 发布时间: 2023-12-18 17:03 | 链接: https://apollo.baidu.com/community/article/1227 
> 

| 问题序号 | 问题 | 相关截图 | 解决方法 |
| --- | --- | --- | --- |
| 1 | buildtool: command not found |  | 可能因为删除宿主机的.aem文件夹导致 需在宿主机的工作目录中输入以下指令：<br>::清理容器<br>aem remove #重新启动容器<br>aem start |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。