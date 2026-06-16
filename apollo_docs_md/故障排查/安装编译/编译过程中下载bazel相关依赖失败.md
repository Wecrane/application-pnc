---
title: 编译过程中下载bazel相关依赖失败
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_x95_x85_xE9_x9A_x9C_xE6_x8E_x92_xE6_x9F_xA5_2_xE5_xAE_x89_xE8_xA3_x85_xE7_xBC_x96_545be5719b296b98c4e51dcd2eb4a9d1.html
category: 故障排查 > 安装编译 > 编译过程中下载bazel相关依赖失败
---

# 编译过程中下载bazel相关依赖失败

### 问题描述:

编译过程中下载bazel相关依赖失败downloading [[https://github.com/bazelbuild/.....](https://github.com/bazelbuild/.....). connect time out

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/安装编译/images/build_download_error.png)

### 问题原因:

网络问题导致下载bazel依赖失败

### 解决方案:

目前已提供预下载缓存，用户可根据自身需求下载对应缓存文件

- [Apollo 8.0预下载缓存](https://apollo-system.cdn.bcebos.com/bazel_deps/3.7.1/cache.tar.gz)
- [Apollo 9.0 x86预下载缓存](https://apollo-system.cdn.bcebos.com/bazel_deps/5.2.1/cache.tar.gz)
- [Apollo 9.0 arm预下载缓存](https://apollo-system.cdn.bcebos.com/bazel_deps/arm-5.2.1/cache.tar.gz)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。