---
title: aem enter报错 error retrieving current directory getcwd cannot access parent directories no such file or directory
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_x95_x85_xE9_x9A_x9C_xE6_x8E_x92_xE6_x9F_xA5_2_xE5_xB7_xA5_xE5_x85_xB7_xE4_xBD_xBF_e691c92cc380fdecbdcda0134c8de50f.html
category: 故障排查 > 工具使用 > aem > aem enter报错 error retrieving current directory getcwd cannot access parent directories no such file or directory
---

# aem enter报错 error retrieving current directory getcwd cannot access parent directories no such file or directory

### 问题描述:

执行aem enter报错: error retrieving current directory: getcwd: cannot access parent directories: no such file or directory

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/工具使用/aem/images/start_container_mount_failed1.png)

### 问题原因:

用户删除了宿主机的挂载文件夹，并且没有执行aem remove清除旧容器，导致容器错误

### 解决方案:

执行aem remove删除历史容器后再启动新的容器

### 相关文档:

- [步骤六：删除工程](https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE5_xAE_x89_xE8_xA3_x85_xE6_x8C_x87_xE5_x8D_x97_2_xE5_x8C_x85_xE7_xAE_xA1_xE7_x90_x86_410bb1324792103828eeacd86377c551.html)
- [aem快速入门](https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_xA1_x86_xE6_x9E_xB6_xE8_xAE_xBE_xE8_xAE_xA1_2_xE5_x91_xBD_xE4_xBB_xA4_xE8_xA1_x8C_xE5_xB7_xA5_xE5_x85_xB7_2aem.html)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。