# buildtool相关问题FAQ

> 作者: 宇新 | 发布时间: 2023-12-18 17:03 | 链接: https://apollo.baidu.com/community/article/1227

| 问题序号 | 问题 | 相关截图 | 解决方法 |
| --- | --- | --- | --- |
| 1 | `buildtool: command not found` | ![](../images/02_安装部署/767ec7426ff19e3be58409157d8f107d.png) | 可能因为删除宿主机的.aem文件夹导致<br>需在宿主机的工作目录中输入以下指令：<br>#清理容器<br>`aem remove`<br>#重新启动容器<br>`aem start` |
