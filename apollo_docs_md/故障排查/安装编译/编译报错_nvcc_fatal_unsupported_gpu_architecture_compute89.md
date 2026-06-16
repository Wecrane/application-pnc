---
title: 编译报错 nvcc fatal unsupported gpu architecture compute89
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_x95_x85_xE9_x9A_x9C_xE6_x8E_x92_xE6_x9F_xA5_2_xE5_xAE_x89_xE8_xA3_x85_xE7_xBC_x96_8ef3fad96a58a11ab9f51894ddfbad38.html
category: 故障排查 > 安装编译 > 编译报错 nvcc fatal unsupported gpu architecture compute89
---

# 编译报错 nvcc fatal unsupported gpu architecture compute89

### 问题描述:

编译报错: nvcc fatal : Unsupported gpu architecture 'compute_89'

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/安装编译/images/build_unsupported_compute_89.png)

### 问题原因:

这种情况一般是用户是用户的显卡型号是40系，部署9.0 apollo-core的工程出现的问题。在x86架构下，apollo分为以下几个版本： apollo-core：即apollo开源版本，对应的是github上的application-core仓库，其中9.0的版本只支持30系及30系及以下的显卡型号，不支持40系显卡。 apollo-universe：即x86架构下的apollo通用园区版，园区版支持40系及40系以下的显卡型号，如果您使用的是40系显卡，请使用通用园区版的工程，具体部署方式以及显卡驱动要求可以参考园区版本的安装文档。

### 解决方案:

如果必须需要使用9.0，可以使用apollo通用园区版。如果可以使用更新的版本，请使用Apollo 10.0，默认支持40系显卡。

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。