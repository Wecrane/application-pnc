---
title: 安装及编译问题FAQ——dreamview相关
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_207__FAQ_xE6_x95_x85_xE9_x9A_x9C_xE6_xf225f4571d555519ddb463c261824f40.html
category: 赛事总汇 > 07_FAQ故障排查 > 安装及编译问题FAQ——dreamview相关
---

# 安装及编译问题FAQ——dreamview相关

# 安装及编译问题FAQ——dreamview相关

> 
> ‍作者: 宇新 | 发布时间: 2024-05-13 11:33 | 链接: https://apollo.baidu.com/community/article/1159 
> 

| 截图 | 问题分析 | 解决方法 |
| --- | --- | --- |
|  | 部分文件夹没权限 当出现permission denied的时候，就需要给相应的文件夹赋予权限了 / 如图所示： 在permission denied 前为/opt/apollo/neo/data/log/dreamview_plus.log路径没权限 / sudo chown 用户名:用户名 -R 路径 / 如图所示 sudo chown 用户名:用户名 -R /opt/apollo/neo/data/log/ |  |
|  | 一般这个情况是，电脑性能问题导致的，如果输入指令后出现这个情况，是因为后台有些服务没有打开，所以会有个报错，一般需要等个几十秒 | 重复执行几次启动指令： aem bootstrap start –plus |
|  |  |  |
|  | 存在部分依赖没完全下载 | 重新执行遍： buildtool build -p core |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。