---
title: Dreamview功能介绍
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_205___xE5_xB7_xA5_xE7_xA8_x8B_xE6_xA1_9867587ffc82685c9ce9f6c8aca58580.html
category: 赛事总汇 > 05_工程框架与工具 > Dreamview功能介绍
---

# Dreamview功能介绍

# Dreamview功能介绍

> 
> ‍作者: 宇新 | 发布时间: 2023-05-11 16:08 | 链接: https://apollo.baidu.com/community/article/1063 
> 

## 功能简介

DreamView 是一个 web 应用程序，提供如下的功能：

可视化显示当前自动驾驶车辆模块的输出信息。例如：规划路径、车辆定位、车架信息等。 为使用者提供人机交互接口以监测车辆硬件状态，对模块进行开关操作，启动自动驾驶车辆等。 提供调试工具。例如 PnC 监视器可以高效的跟踪模块输出的问题。 

## 界面布局和特性

该应用程序的界面被划分为多个区域：标题、侧边栏、主视图和工具视图。

### 标题

标题包含 6 个下拉列表，可以像下述图片所示进行操作：

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/840d34940e9f33d85d5a41aadc02c7a8.png)

> 
> ‍注意：导航模块是在Apollo 2.5版本引入的满足低成本测试的特性。在该模式下，Baidu或Google地图展现的是车辆的绝对位置，而主视图中展现的是车辆的相对位置。 
> 

### 侧边栏和工具视图

侧边栏控制着显示在工具视图中的模块。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/fe239ea3c1fec6425ecca6da10f9a829.png)

### Tasks

在 DreamView 中，您可以操作的 tasks 有：

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/3f0d4391dc89ed3581be4250eb9c6670.png)

**Quick Start**: 当前选择的模式支持的指令。通常情况下，**Setup**: 开启所有模块。**Reset all**: 关闭所有模块。**Start Auto**: 开始车辆的自动驾驶。 **Others**: 工具经常使用的开关和按钮。 **Module Delay**: 从模块中输出的两次事件的时间延迟。 **Console**: 从 Apollo 平台输出的监视器信息。 

### Module Controller

监视硬件状态和对模块进行开关操作。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/bd96a65ba27b113effd980712cc5419a.png)

### Layer Menu

显式控制各个元素是否显示的开关。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/b9719d11a2f8949c8a08452fb2fce4d7.png)

### Route Editing

在向 Routing 模块发送寻路信息请求前，可以编辑路径信息的可视化工具。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/39cce487f5f0999d833975e1d2eb6250.png)

### Data Recorder

将问题报告给 rosbag 中的 drive event 的界面。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/1281608fd2c9cc197b5cab6f5ad912c0.png)

### Default Routing

预先定义的路径或者路径点，该路径点称为兴趣点（POI）。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/56a69ea5327f1a522e545a47537978c9.png)

如果打开了路径编辑模式，路径点可被显式的在地图上添加。

如果关闭了路径编辑模式，点击一个期望的POI会向服务器发送一次寻路请求。如果只选择了一个点，则寻路请求的起点是自动驾驶车辆的当前点。否则寻路请求的起点是选择路径点中的第一个点。

查看Map目录下的 [default_end_way_point.txt](https://apollo.baidu.com/docs/apollo/latest/default__end__way__point_8txt.html) 文件可以编译POI信息。例如，如果选择的地图模式为“Demo”，则在modules/map/data/demo目录下可以查看对应的 default_end_way_point.txt 文件。

### 主视图

主视图在 web 页面中以动画的方式展示 3D 计算机图形。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/82da638b8e3efaf48581b6a4aeecb185.png)

下表列举了主视图中各个元素：

| Visual Element | Depiction Explanation |
| --- | --- |
|  | 自动驾驶车辆。 |
|  | 车轮转动的比率。 左右转向灯的状态。 |
|  | 交通信号灯状态。 |
|  | 驾驶状态： AUTO， DISENGAGED， MANUAL 等。 |
|  | 行驶速度 km/h。 加速速率/刹车速率。 |
|  | 红色粗线条表示建议的寻路路径。 |
|  | 轻微移动物体决策—橙色表示应该避开的区域。 |
|  | 绿色的粗曲线条带表示规划的轨迹。 |

### 障碍物

| Visual Element | Depiction Explanation |
| --- | --- |
|  | 车辆障碍物。 |
|  | 行人障碍物。 |
|  | 自行车障碍物。 |
|  | 未知障碍物。 |
|  | 速度方向显示了移动物体的方向，长度随速度按照比率变化。 |
|  | 白色箭头显示了障碍物的移动方向。 |
|  | 黄色文字表示： 障碍物的跟踪 ID， 自动驾驶车辆和障碍物的距离及障碍物速度。 |
|  | 线条显示了障碍物的预测移动轨迹，线条标记为和障碍物同一个颜色。 |

#### Planning决策

##### 决策栅栏区

决策栅栏区显示了Planning模块对车辆障碍物做出的决策。每种类型的决策会表示为不同的颜色和图标，如下图所示：

| Visual Element | Depiction Explanation |
| --- | --- |
|  | **停止**：表示物体主要的停止原因。 |
|  | **停止**：表示物体的停止原因。 |
|  | **跟车**：物体。 |
|  | **让行**：物体决策—点状的线条连接了各个物体。 |
|  | **超车**：物体决策—点状的线条连接了各个物体。 |

线路变更是一个特殊的决策，因此不显示决策栅栏区，而是将路线变更的图标显示在车辆上。

| Visual Element | Depiction Explanation |
| --- | --- |
|  | 变更到左车道。 |
|  | 变更到右车道。 |

在优先通行的规则下，当在交叉路口的停车标志处做出让行决策时，被让行的物体在头顶会显示让行图标。

| Visual Element | Depiction Explanation |
| --- | --- |
|  | 停止标志处的让行物体。 |

### Planning决策-停止原因

如果显示了停止决策栅栏区，则停止原因展示在停止图标的右侧。可能的停止原因和对应的图标为：

| Visual Element | Depiction Explanation |
| --- | --- |
|  | 前方道路侧边区域。 |
|  | 前方人行道。 |
|  | 到达目的地。 |
|  | 紧急停车。 |
|  | 自动驾驶模式未准备好。 |
|  | 障碍物阻塞道路。 |
|  | 前方行人穿越。 |
|  | 黄/红信号灯。 |
|  | 前方有车辆。 |
|  | 前方停止标志。 |
|  | 前方让行标志。 |

#### 视图

可以在主视图中展示多种从 **Layer Menu **选择的视图模式：

| Visual Element | Point of View |
| --- | --- |
|  | 默认视图 |
|  | 近距离视图 |
|  | 俯瞰视图 |
|  | 地图 放大/缩小：滚动鼠标滚轮或使用两根手指滑动 移动：按下右键并拖拽或或使用三根手指滑动 |

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。