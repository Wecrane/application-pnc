---
title: 编译报错 cannot find bazel
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_x95_x85_xE9_x9A_x9C_xE6_x8E_x92_xE6_x9F_xA5_2_xE5_xAE_x89_xE8_xA3_x85_xE7_xBC_x96_b09d8a3899b8e3502018c7c1a753d389.html
category: 故障排查 > 安装编译 > 编译报错 cannot find bazel
---

# 编译报错 cannot find bazel

### 问题描述:

源码环境，用户使用sudo bash docker/scripts/dev_into.sh进入容器后执行编译，报错cannot find bazel. Please install bazel first.

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/安装编译/images/cannot_find_bazel.png)

### 问题原因:

进入容器使用了sudo,导致权限混乱

### 解决方案:

执行启动容器和进入容器命令都不要添加sudo

**注:** 以当前用户执行启动容器和进入容器命令报错: dial unix /var/run/dcker.sock: connect: permission denied.需要将当前用户加入docker用户组或给docker.sock赋777权限

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。