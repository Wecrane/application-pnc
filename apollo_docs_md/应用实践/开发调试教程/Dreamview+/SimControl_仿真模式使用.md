---
title: SimControl 仿真模式使用
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE5_xBA_x94_xE7_x94_xA8_xE5_xAE_x9E_xE8_xB7_xB5_2_xE5_xBC_x80_xE5_x8F_x91_xE8_xB0_x83_4c064346c5f0f4fa3ade131e35350af4.html
category: 应用实践 > 开发调试教程 > Dreamview+ > SimControl 仿真模式使用
---

# SimControl 仿真模式使用

## 步骤一：启动 Dreamview+

您可以通过包管理或源码方式启动 Dreamview+，请您选择一种方式按照相应命令启动 Dreamview+。

### 1. 启动 Dreamview+

#### 方式一：包管理方式

通过包管理方式进入 docker 环境中，在 docker 环境中执行以下命令启动 Dreamview+：

aem bootstrap start --plus
fragment
> 
> ‍注意：
> 
> 如果您想要停止 Dreamview+，请输入aem bootstrap stop --plus，
> 如果您想要重启 Dreamview+，请输入aem bootstrap restart --plus。 
> 
> 

#### 方式二：源码方式

通过源码方式进入 docker 环境，在 docker 环境中执行以下命令启动 Dreamview+：

bash scripts/bootstrap.sh start_plus
fragment
> 
> ‍注意：
> 
> 如果您想要停止 Dreamview+，请输入bash scripts/bootstrap.sh stop_plus，
> 如果您想要重启 Dreamview+，请输入bash scripts/bootstrap.sh restart_plus。 
> 
> 

### 2. 打开 Dreamview+

启动成功后，在浏览器输⼊ `localhost:8888` ⽹址打开 Dreamview+ 界面。

当出现如下界面，表示 Dreamview+ 启动成功了。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_8455c10.png)

点击左下角 **个人中心** > **设置** > **全局设置** ，可以选择界面语言类型。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ce0ce76.png)

## 步骤二：使用 SimControl 仿真自动驾驶场景

1. 在 模式 设置中选择 PNC 模式 ，并启动 Planning 模块，选择 Sim_Control 操作模式，高精地图选择 Sunnyvale Big Loop ，车辆选择 MKZ Example 。

1. 点击 车辆可视化 面板的 路由编辑 功能，进入车辆路由设置界面。

1. 分别设置起点和轨迹点（最后一个轨迹点为终点），设置完成后点击保存编辑。

1. 回到主界面后，点击左下角启动按钮，即可看到车辆开始在仿真环境中运行。

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。