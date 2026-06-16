---
title: 编译报错 server terminated abruptly error code 14 error message socket closed
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_x95_x85_xE9_x9A_x9C_xE6_x8E_x92_xE6_x9F_xA5_2_xE5_xAE_x89_xE8_xA3_x85_xE7_xBC_x96_cb4eebc0da798e6fdebe4a024b4fd1eb.html
category: 故障排查 > 安装编译 > 编译报错 server terminated abruptly error code 14 error message socket closed
---

# 编译报错 server terminated abruptly error code 14 error message socket closed

### 问题描述:

编译报错: [Server](https://apollo.baidu.com/docs/apollo/latest/classServer.html) terminated abruptly (error code: 14, error message: 'Socket closed')

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/安装编译/images/build_error_socket_closed.png)

### 问题原因:

硬件资源耗尽导致OOM

### 解决方案:

#### 9.0 源码

修改脚本来控制编译使用的硬件资源: scripts/apollo_base.sh中746和748行

--jobs=${count} --local_ram_resources=HOST_RAM*0.7
改为
--jobs=2 --local_ram_resources=HOST_RAM*0.5
fragment

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/安装编译/images/build_socket_closed.png)

#### 9.0 包管理 和 10.0

在编译时可以使用-j和-m参数控制使用的线程数量以及内存百分比，例如buildtool build -j 4 -m 0.5 为使用4个线程，总内存的50用量进行编译

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/安装编译/images/build_command_show.png)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。