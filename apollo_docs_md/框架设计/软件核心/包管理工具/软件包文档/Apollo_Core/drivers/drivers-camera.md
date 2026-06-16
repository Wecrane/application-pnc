---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2drivers_2camera_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > drivers > drivers-camera
---

# README_cn

## Camera

camera包是基于V4L USB相机设备实现封装，提供图像采集及发布的功能。本驱动中使用了一台长焦相机和一台短焦相机。

### Output channels

- /apollo/sensor/camera/front_12mm/image
- /apollo/sensor/camera/front_6mm/image
- /apollo/sensor/camera/front_fisheye/image
- /apollo/sensor/camera/left_fisheye/image
- /apollo/sensor/camera/right_fisheye/image
- /apollo/sensor/camera/rear_6mm/image

### 启动camera驱动

**请先修改并确认launch文件中的参数与实际车辆相对应** 

# in docker
bash /apollo/scripts/camera.sh
# or
cd /apollo && cyber_launch start modules/drivers/camera/launch/camera.launch
fragment

### 启动camera + video compression驱动

**请先修改并确认launch文件中的参数与实际车辆相对应** 

# in docker
bash /apollo/scripts/camera_and_video.sh
# or
cd /apollo && cyber_launch start modules/drivers/camera/launch/camera_and_video.launch

### 常见问题
1. 如果出现报错“sh: 1: v4l2-ctl: not found”，需要安装v4l2库。
fragment

 bash sudo apt-get install v4l-utils ```

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。