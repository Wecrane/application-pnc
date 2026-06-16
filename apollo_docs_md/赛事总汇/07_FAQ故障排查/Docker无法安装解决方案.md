---
title: Docker无法安装解决方案
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_207__FAQ_xE6_x95_x85_xE9_x9A_x9C_xE6_x1c0994d75c8ba0b89c83156ce71b5478.html
category: 赛事总汇 > 07_FAQ故障排查 > Docker无法安装解决方案
---

# Docker无法安装解决方案

# Docker无法安装解决方案

> 
> ‍作者: 宇新 | 发布时间: 2023-05-09 22:41 | 链接: https://apollo.baidu.com/community/article/1058 
> 

问: 在使用脚本安装Docker时遇到了无法安装的问题，出现这种情况的原因是什么？

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/6b458daad73cebc9abbd16465eeaf3c2.png)

原因: 出现这种问题的原因通常是网络问题，可能是由于下载所需的依赖文件时出现了错误或下载速度缓慢导致的。

解决方式： 可以尝试通过以下命令来解决这个问题：

sudo apt install docker.io
fragment

 这个命令可以通过APT包管理器安装Docker，即使您在使用脚本时出现了问题，也可以使用这个命令手动安装Docker。

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。