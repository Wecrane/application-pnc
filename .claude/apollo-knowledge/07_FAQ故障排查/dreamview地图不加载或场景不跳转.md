# dreamview地图不加载或场景不跳转

> 作者: 宇新 | 发布时间: 2024-04-24 18:25 | 链接: https://apollo.baidu.com/community/article/1266

| 问题描述 | 解决方式 |
| --- | --- |
| 在dreamview中，若出现地图不加载或者点击场景不跳转的情况，可以参考如下方式修复 | 如果场景不跳转或者地图不加载：<br>`cd ~/.apollo/resources/dynamic_models/models`<br>`rm -r *`<br>`cd /apollo_workspace`<br>`aem bootstrap restart --plus`<br>切换到 scenario_sim 下切换场景，如果切换不成功，则说明没地图<br>地图获取请查看：[https://apollo.baidu.com/community/article/1128](https://apollo.baidu.com/community/article/1128)<br>使用指令查看有无下载地图：`ls data/map_data/`<br>下载完地图后需要重启 dreamview：`aem bootstrap restart --plus`<br>启动完成后切换操作为 sim control，打开相应的地图（xh_2024_contest）查看是否能加载 |
