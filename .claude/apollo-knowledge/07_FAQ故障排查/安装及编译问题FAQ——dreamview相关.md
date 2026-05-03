# 安装及编译问题FAQ——dreamview相关

> 作者: 宇新 | 发布时间: 2024-05-13 11:33 | 链接: https://apollo.baidu.com/community/article/1159

| 截图 | 问题分析 | 解决方法 |
| --- | --- | --- |
| ![](../images/02_安装部署/8d7f59a92ee50b93c0ac697e38dc5624.jpg) | `部分文件夹没权限 当出现permission denied的时候，就需要给相应的文件夹赋予权限了 / 如图所示： 在permission denied 前为/opt/apollo/neo/data/log/dreamview_plus.log路径没权限 / sudo chown 用户名:用户名 -R 路径 / 如图所示 sudo chown 用户名:用户名 -R /opt/apollo/neo/data/log/` |  |
| ![](../images/02_安装部署/06eb48fcafb285a6f42b17920c48f3fc.jpg) | 一般这个情况是，电脑性能问题导致的，如果输入指令后出现这个情况，是因为后台有些服务没有打开，所以会有个报错，一般需要等个几十秒 | 重复执行几次启动指令： aem bootstrap start --plus |
|  |  |  |
| ![](../images/02_安装部署/b1eb4d3ecf0e25150c77fddbae6aaf5c.jpg) | 存在部分依赖没完全下载 | 重新执行遍： buildtool build -p core |
