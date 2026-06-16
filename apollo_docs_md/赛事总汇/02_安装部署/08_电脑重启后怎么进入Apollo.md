---
title: 08_电脑重启后怎么进入Apollo
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_202___xE5_xAE_x89_xE8_xA3_x85_xE9_x83_034b96d72e07bf075ade53fbb8455aaf.html
category: 赛事总汇 > 02_安装部署 > 08_电脑重启后怎么进入Apollo
---

# 08_电脑重启后怎么进入Apollo

# 电脑重启后怎么进入Apollo？

> 
> ‍作者: 宇新 | 发布时间: 2025-04-08 11:35 | 链接: https://apollo.baidu.com/community/article/1286 
> 

### 1. 进入Apollo环境

打开终端方法：ctrl + alt + t

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/e178b273718acc363a178e1e91ffec65.png)

cd application-pnc
aem enter
fragment

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/d253e2d33ba86f1591a847345668e18e.png)

### 2. 启动dreamview可视化界面&模式

aem bootstrap start --plus
fragment

如若没有http://localhost:8888/的显示，请认真查看下图的书写方式。 ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/a69df7a6f1600d9086454676d7c517e0.png)

**进入可视化界面**

**方法一：**

将鼠标移动至（[http://localhost:8888/](http://localhost:8888/)），按住ctrl + 鼠标左键打开网页

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/f3f6abf21d8b0012e12acdb28a065503.png)

**方法二：**

打开浏览器输入（[http://localhost:8888/](http://localhost:8888/)）进入dremview中。

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。