---
title: camera的driver launch文件运行时报出segemation fault
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_x95_x85_xE9_x9A_x9C_xE6_x8E_x92_xE6_x9F_xA5_2_xE6_xA8_xA1_xE5_x9D_x97_xE8_xB0_x83_cdc407f6cae05c27e5074211076fbf0c.html
category: 故障排查 > 模块调试 > 相机驱动 > camera的driver launch文件运行时报出segemation fault
---

# camera的driver launch文件运行时报出segemation fault

### 问题描述:

camera的driver launch文件运行时报出segemation fault

### 问题原因:

相机分辨率和默认配置不匹配

### 解决方案:

修改配置文件/apollo/modules/drivers/camera/conf/camera_front_6mm.pb.txt和/apollo/modules/drivers/camera/conf/camera_front_12mm.pb.txt

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/模块调试/相机驱动/images/camera_conf.png)

**注意：** 目前apollo支持部分相机分辨率，4k分辨率(3840×2160)暂不支持

常见支持相机分辨率如下

- 640×480
- 1280×720
- 1920×1080
- 2560×1920

支持相机分辨率计算方式如下

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/模块调试/相机驱动/images/camera_max_size.png)

![](https://apollo.baidu.com/docs/apollo/latest/docs/故障排查/模块调试/相机驱动/images/camera_usage.png)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。