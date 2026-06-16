---
title: mainboard和cyber_lanuch报错 this process will be terminated due to duplicated node 模块名 please ensure that each node has a unique name
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_x95_x85_xE9_x9A_x9C_xE6_x8E_x92_xE6_x9F_xA5_2_xE5_xB7_xA5_xE5_x85_xB7_xE4_xBD_xBF_db8f5379daf29a249493e2fe5d1783d2.html
category: 故障排查 > 工具使用 > mainboard和cyber_lanuch > mainboard和cyber_lanuch报错 this process will be terminated due to duplicated node 模块名 please ensure that each node has a unique name
---

# mainboard和cyber_lanuch报错 this process will be terminated due to duplicated node 模块名 please ensure that each node has a unique name

### 问题描述:

使用mainboard或cyber_lanuch启动模块报错：this process will be terminated due to duplicated node[模块名], please ensure that each node has a unique name.

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/工具使用/mainboard和cyber_lanuch/images/mainboard_error.png)

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/工具使用/mainboard和cyber_lanuch/images/cyber_lanuch_error.png)

### 问题原因:

模块已启动，再次启动失败

### 解决方案:

可通过ps查询当前启动了哪些模块，使用kill停止已启动的模块后再重新启动

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/工具使用/mainboard和cyber_lanuch/images/ps_mainboard.png)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。