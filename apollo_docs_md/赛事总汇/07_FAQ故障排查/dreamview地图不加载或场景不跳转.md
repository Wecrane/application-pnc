---
title: dreamview地图不加载或场景不跳转
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_207__FAQ_xE6_x95_x85_xE9_x9A_x9C_xE6_x4030bee0d0470392127f351ef009c833.html
category: 赛事总汇 > 07_FAQ故障排查 > dreamview地图不加载或场景不跳转
---

# dreamview地图不加载或场景不跳转

# dreamview地图不加载或场景不跳转

> 
> ‍作者: 宇新 | 发布时间: 2024-04-24 18:25 | 链接: https://apollo.baidu.com/community/article/1266 
> 

| 问题描述 | 解决方式 |
| --- | --- |
| 在dreamview中，若出现地图不加载或者点击场景不跳转的情况，可以参考如下方式修复 | 如果场景不跳转或者地图不加载：<br>cd ~/.apollo/resources/dynamic_models/models rm -r * cd /apollo_workspace aem bootstrap restart --plus 切换到 scenario_sim 下切换场景，如果切换不成功，则说明没地图 地图获取请查看：https://apollo.baidu.com/community/article/1128 使用指令查看有无下载地图：ls data/map_data/ 下载完地图后需要重启 dreamview：aem bootstrap restart --plus 启动完成后切换操作为 sim control，打开相应的地图（xh_2024_contest）查看是否能加载 |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。