---
title: 安装及编译问题FAQ–拉取工程目录
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_207__FAQ_xE6_x95_x85_xE9_x9A_x9C_xE6_x44c678fd2c247217fdb0d55ef5c80833.html
category: 赛事总汇 > 07_FAQ故障排查 > 安装及编译问题FAQ--拉取工程目录
---

# 安装及编译问题FAQ–拉取工程目录

# 安装及编译问题FAQ–拉取工程目录

> 
> ‍作者: 宇新 | 发布时间: 2023-12-18 17:35 | 链接: https://apollo.baidu.com/community/article/1158 
> 

| 序号 |  | 举例 | 修改方式 |
| --- | --- | --- | --- |
| 1 | 方案一： 使用gitee clone | git clone https://github.com/ApolloAuto/application-pnc.git | 将github改成gitee<br>git clone https://gitee.com/ApolloAuto/application-pnc.git |
| 2 | 方案二： 使用镜像网站 | git clone https://github.com/ApolloAuto/application-pnc.git | 将**github.com**改成**kkgithub.com** git clone https://kkgithub.com/ApolloAuto/application-pnc.git |
| 3 | 方案三： 使用镜像网站 | git clone https://github.com/ApolloAuto/application-pnc.git | 在github前添加gitclone.com/ git clone https://gitclone.com/github.com/ApolloAuto/application-pnc.git |

## 方案一：

举例： 

git clone https://github.com/ApolloAuto/application-pnc.git
fragment

## 方案二：

举例： 

git clone https://github.com/ApolloAuto/application-pnc.git
fragment

 如出现无法访问等问题，可以将**github.com**改成**kkgithub.com**

git clone https://kkgithub.com/ApolloAuto/application-pnc.git
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。