---
title: Add Panel
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE5_xB7_xA5_xE5_x85_xB7_xE4_xBD_xBF_xE7_x94_xA8_2Dreamview_09_2Add_01Panel.html
category: 工具使用 > Dreamview+ > Add Panel
---

# Add Panel

## 面板简介

面板主要包含 Console、Module Delay、Vehicle Visualization、Camera View、Point Cloud、Vehicle Dashboard。

- Console/控制台：实时展示系统日志和调试信息，供调试使用。
- Module Delay/模块延时：实时展示核心 Module channel 的延时情况。
- Vehicle Visualization/车辆可视化：车辆可视化，展示自动驾驶车辆的位置，地图数据和自动驾驶系统相关信息。
- Camera View/相机视图：相机视图面板展示相机数据。
- Point Cloud/点云：点云视图面板显示点云数据。
- Vehicle Dashboard/车辆仪表盘：车辆仪表盘面板，展示车辆当前状态，包括车速、加减速踏板开度、方向盘转角、档位等信息。
- PnC Monitor/PNC 监控：检测车辆运行过程中规划控算法的相关数据信息。例如：Planning 和 [Control](https://apollo.baidu.com/docs/apollo/latest/classControl.html) 算法的输出，当前车辆位置、速度、加速度、方向盘转角等等。
- Components/监控组件：用于展示常用组件状态的面板。

## 面板操作

### 新增面板并自定义位置

您可以通过点击左侧面板名称添加面板，还可以通过长按鼠标拖拽添加面板。本小节以添加 **Vehicle Visualization/车辆可视化** 面板为例介绍如何新增面板并自定义面板位置。

1. 在左侧导航栏点击 **Add Panel/添加面板**。

1. 选择需要新增的面板类型 **Vehicle Visualization/车辆可视化**，拖动至右侧主操作区的期望位置，如下图蓝色半透明区域所示：

1. 对新增面板做需要的配置，此处以修改视角为例。将 **View/视角** 设置为 **Map/地图** 视角。
1. 鼠标移动到面板边缘，拖动修改面板大小，添加设置完成。

### 删除面板

在面板右上方设置图标中，点击 **Remove Panel/移除**。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_6ae746c.png)

### 拖动移动位置

拖动面板窗口栏可以调整面板位置。

### 调整高/宽度

调整面板面框位置可以调整面板的高度和宽度。

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。