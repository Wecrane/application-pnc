---
title: buildtool build -p core 报错指南
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_207__FAQ_xE6_x95_x85_xE9_x9A_x9C_xE6_x5a85755ddf1d3d423801954d536a0ded.html
category: 赛事总汇 > 07_FAQ故障排查 > buildtool build -p core 报错指南
---

# buildtool build -p core 报错指南

# buildtool build -p core 报错指南

> 
> ‍作者: 宇新 | 发布时间: 2024-05-07 17:42 | 链接: https://apollo.baidu.com/community/article/1265 
> 

# 问题

| 问题 | 截图 | 修复方式 |
| --- | --- | --- |
| 下载超时；终端出现 connect timed out |  | cd /apollo_workspace/ wget https://apollo-system.cdn.bcebos.com/bazel_deps/cache.tar.gz sudo rm -rf .cache tar -xzvf cache.tar.gz 【注意】需要在容器中执行 执行完成后，再执行指令 buildtool build -p core |
| 当前工作目录下存在一个或者多个工作目录 |  | 例：  我们需要将多的工作目录删除： 输入指令： sudo rm -rf application-pnc |
| 文件夹没权限 | 复制在 permission denied 前的路径<br>sudo chown 用户名:用户名 -R 路径 例：sudo chown 用户名:用户名 -R /apollo_workspace/.cache/ 执行完成后，再执行指令 buildtool build -p core |  |
| 文件夹没权限 |  | 工作空间下tools/bazel.rc权限存在问题，删除即可 sudo rm -rf /apollo_workspace/.apollo.bazelrc |
| 文件夹没权限 |  | 工作空间下.bazelrc权限存在问题，删除即可 sudo rm -f .bazelrc |
| 文件夹没权限 |  | .cache的权限错了，说明之前用过非root用户编译过代码，但后来用root用户进入的容器，建议不要切权限 |
| 网络不稳定导致 |  | 再执行指令 buildtool build -p core |

| 工程目录挂载错误 | ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/e745a05ea70efb991ca11305f24a334d.jpg)|在容器中输入：

#退出容器<br>`exit`

#清理容器<br>`aem remove`

#找到工程目录后，再使用<br>`aem start_cpu`

`aem enter` | |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。