# Dreamview功能介绍

> 作者: 宇新 | 发布时间: 2023-05-11 16:08 | 链接: https://apollo.baidu.com/community/article/1063

## 功能简介

DreamView 是一个 web 应用程序，提供如下的功能：

可视化显示当前自动驾驶车辆模块的输出信息。例如：规划路径、车辆定位、车架信息等。
为使用者提供人机交互接口以监测车辆硬件状态，对模块进行开关操作，启动自动驾驶车辆等。
提供调试工具。例如 PnC 监视器可以高效的跟踪模块输出的问题。
## 界面布局和特性

该应用程序的界面被划分为多个区域：标题、侧边栏、主视图和工具视图。

### 标题

标题包含 6 个下拉列表，可以像下述图片所示进行操作：

![](../images/06_工程框架/840d34940e9f33d85d5a41aadc02c7a8.png)

> 注意：导航模块是在Apollo 2.5版本引入的满足低成本测试的特性。在该模式下，Baidu或Google地图展现的是车辆的绝对位置，而主视图中展现的是车辆的相对位置。

### 侧边栏和工具视图

侧边栏控制着显示在工具视图中的模块。

![](../images/06_工程框架/fe239ea3c1fec6425ecca6da10f9a829.png)

### Tasks

在 DreamView 中，您可以操作的 tasks 有：

![](../images/06_工程框架/3f0d4391dc89ed3581be4250eb9c6670.png)

**Quick Start**: 当前选择的模式支持的指令。通常情况下，**Setup**: 开启所有模块。**Reset all**: 关闭所有模块。**Start Auto**: 开始车辆的自动驾驶。
**Others**: 工具经常使用的开关和按钮。
**Module Delay**: 从模块中输出的两次事件的时间延迟。
**Console**: 从 Apollo 平台输出的监视器信息。
### Module Controller

监视硬件状态和对模块进行开关操作。

![](../images/06_工程框架/bd96a65ba27b113effd980712cc5419a.png)

### Layer Menu

显式控制各个元素是否显示的开关。

![](../images/06_工程框架/b9719d11a2f8949c8a08452fb2fce4d7.png)

### Route Editing

在向 Routing 模块发送寻路信息请求前，可以编辑路径信息的可视化工具。

![](../images/06_工程框架/39cce487f5f0999d833975e1d2eb6250.png)

### Data Recorder

将问题报告给 rosbag 中的 drive event 的界面。

![](../images/06_工程框架/1281608fd2c9cc197b5cab6f5ad912c0.png)

### Default Routing

预先定义的路径或者路径点，该路径点称为兴趣点（POI）。

![](../images/06_工程框架/56a69ea5327f1a522e545a47537978c9.png)

如果打开了路径编辑模式，路径点可被显式的在地图上添加。

如果关闭了路径编辑模式，点击一个期望的POI会向服务器发送一次寻路请求。如果只选择了一个点，则寻路请求的起点是自动驾驶车辆的当前点。否则寻路请求的起点是选择路径点中的第一个点。

查看Map目录下的 default_end_way_point.txt 文件可以编译POI信息。例如，如果选择的地图模式为“Demo”，则在modules/map/data/demo目录下可以查看对应的 default_end_way_point.txt 文件。

### 主视图

主视图在 web 页面中以动画的方式展示 3D 计算机图形。

![](../images/06_工程框架/82da638b8e3efaf48581b6a4aeecb185.png)

下表列举了主视图中各个元素：

| Visual Element | Depiction Explanation |
| --- | --- |
| ![](../images/06_工程框架/72c26b26425a2df850debebb0d5b82a4.png) | 自动驾驶车辆。 |
|  ![](../images/06_工程框架/0b510a1e86845bd13b74fd080210a0ee.png) | 车轮转动的比率。 左右转向灯的状态。 |
| ![](../images/06_工程框架/b7dd3b2dbd9aa86c9cfa8beabdb5f667.png) | 交通信号灯状态。 |
| ![](../images/06_工程框架/70576eb1566d1496c804965ff7122156.png) | 驾驶状态： AUTO， DISENGAGED， MANUAL 等。 |
| ![](../images/06_工程框架/7636aa52ae394587713222f1a1141d24.png) | 行驶速度 km/h。 加速速率/刹车速率。 |
| ![](../images/06_工程框架/f1a8f8e6f3108ab485a3101ef58a9976.png) | 红色粗线条表示建议的寻路路径。 |
| ![](../images/06_工程框架/767ccf7440139455433c9aa27f18ee14.png)  | 轻微移动物体决策—橙色表示应该避开的区域。 |
| ![](../images/06_工程框架/250ace668a6b08d7c16061572f815be5.png) | 绿色的粗曲线条带表示规划的轨迹。 |

### 障碍物

| Visual Element | Depiction Explanation |
| --- | --- |
| ![](../images/06_工程框架/849595cd733992be022ea6dadf21a9c5.png) | 车辆障碍物。 |
| ![](../images/06_工程框架/1db968cd6d3c6b5ec0a857dc535c7de8.png) | 行人障碍物。 |
| ![](../images/06_工程框架/7850bac5c9d271eda1d2501cb8462056.png)  | 自行车障碍物。 |
| ![](../images/06_工程框架/497c9b0f0ddbb036da06fb99924de766.png) | 未知障碍物。 |
| ![](../images/06_工程框架/40d6463fe9374492cf9fe19d853a019b.png) | 速度方向显示了移动物体的方向，长度随速度按照比率变化。 |
| ![](../images/06_工程框架/6f03b551ffb01b25f2eed26dc655573b.png) | 白色箭头显示了障碍物的移动方向。 |
| ![](../images/06_工程框架/fae711a4c79ee506b4ce6d8dba006fb1.png) | 黄色文字表示： 障碍物的跟踪 ID， 自动驾驶车辆和障碍物的距离及障碍物速度。 |
| ![](../images/06_工程框架/5cfda03700eff08aebc377781e17c354.png)  | 线条显示了障碍物的预测移动轨迹，线条标记为和障碍物同一个颜色。 |

#### Planning决策

##### 决策栅栏区

决策栅栏区显示了Planning模块对车辆障碍物做出的决策。每种类型的决策会表示为不同的颜色和图标，如下图所示：

| Visual Element | Depiction Explanation |
| --- | --- |
| ![](../images/06_工程框架/69b26d5b679cf78c17d1fead6b4ed1d1.png) | **停止**：表示物体主要的停止原因。 |
| ![](../images/06_工程框架/e549c33024a5a940ee1eabdc92fac5f2.png) | **停止**：表示物体的停止原因。 |
| ![](../images/06_工程框架/3cf78f207f25864b588235a5b32979b3.png) | **跟车**：物体。 |
| ![](../images/06_工程框架/eae2f69432c4cdd945a439ccbff4b3ef.png) | **让行**：物体决策—点状的线条连接了各个物体。 |
| ![](../images/06_工程框架/cc764847b3ec69fcbf6c3e61e3af4385.png) | **超车**：物体决策—点状的线条连接了各个物体。 |

线路变更是一个特殊的决策，因此不显示决策栅栏区，而是将路线变更的图标显示在车辆上。

| Visual Element | Depiction Explanation |
| --- | --- |
| ![](../images/06_工程框架/7301b595b195bfe13420ad95afa47999.png) | 变更到左车道。 |
| ![](../images/06_工程框架/27f260dcd10a886b52b800288dfaec33.png) | 变更到右车道。 |

在优先通行的规则下，当在交叉路口的停车标志处做出让行决策时，被让行的物体在头顶会显示让行图标。

| Visual Element | Depiction Explanation |
| --- | --- |
| ![](../images/06_工程框架/24ec31c08f6133ad860cbfb9096d2c16.png) | 停止标志处的让行物体。 |

### Planning决策-停止原因

如果显示了停止决策栅栏区，则停止原因展示在停止图标的右侧。可能的停止原因和对应的图标为：

| Visual Element | Depiction Explanation |
| --- | --- |
| ![](../images/06_工程框架/20aae81231756677b17d19587db4b2ac.png) | 前方道路侧边区域。 |
| ![](../images/06_工程框架/fc398e933c0e2c64ef56dc1d6bd996db.png) | 前方人行道。 |
| ![](../images/06_工程框架/35782f09046747068fe32bfba387999b.png) | 到达目的地。 |
| ![](../images/06_工程框架/1c81ac2deab932a6bd731e9ee326e9ab.png) | 紧急停车。 |
| ![](../images/06_工程框架/7d6de8f41e2d8a4e269e6a1e66488679.png) | 自动驾驶模式未准备好。 |
| ![](../images/06_工程框架/dda8b7934e33bed2fa490f1ac967657b.png) | 障碍物阻塞道路。 |
| ![](../images/06_工程框架/d98523d0b90e2480954b75641ae519a6.png) | 前方行人穿越。 |
| ![](../images/06_工程框架/8513c252bd41474f7a16bcba4a955470.png) | 黄/红信号灯。 |
| ![](../images/06_工程框架/1e6d48d370893243b2d69f356b734101.png) | 前方有车辆。 |
| ![](../images/06_工程框架/734e75db1f7bb80092aaf504782c5a58.png) | 前方停止标志。 |
| ![](../images/06_工程框架/eaec6cf41cbd1ba61309ef09fa977e3b.png) | 前方让行标志。 |

#### 视图

可以在主视图中展示多种从 **Layer Menu **选择的视图模式：

| Visual Element | Point of View |
| --- | --- |
| ![](../images/06_工程框架/138b5b58caf1e83a2a0fc1a686bf749d.png) | **默认视图** |
| ![](../images/06_工程框架/db542fe72d11a105fd6f026cfb63552a.png) | **近距离视图** |
| ![](../images/06_工程框架/6e91b44bc0cf0dd0cbb87183aae081e3.png) | **俯瞰视图** |
| ![](../images/06_工程框架/e7dd9840e4c00819addc1c1f1ddd65dd.png) | **地图** 放大/缩小：滚动鼠标滚轮或使用两根手指滑动 移动：按下右键并拖拽或或使用三根手指滑动 |
