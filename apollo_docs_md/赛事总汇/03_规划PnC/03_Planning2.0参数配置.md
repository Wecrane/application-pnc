---
title: 03_Planning2.0参数配置
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_203___xE8_xA7_x84_xE5_x88_x92PnC_203__b6545aba37a4661f14c713a6a4b7e665.html
category: 赛事总汇 > 03_规划PnC > 03_Planning2.0参数配置
---

# 03_Planning2.0参数配置

# 理解Planing2.0 参数配置

> 
> ‍作者: 宇新 | 发布时间: 2023-08-13 17:43 | 链接: https://apollo.baidu.com/community/article/1115 
> 

# planning2.0配置参数新开发模式

Apollo提供了丰富的功能和业务逻辑，涵盖了多种自动驾驶场景和应用。许多常见的使用场景在Apollo中已经得到满足，因此对于某些应用来说，无需进行额外的开发工作，仅通过调整规划模块的配置参数即可实现所需功能。

当apollo的现有功能和业务逻辑完全适用于您的使用场景时，例如，您只希望点到点行驶调整巡航速度，或者过转弯时期望速度更低，您可以不需要开发代码，只通过调整规划模块配置参数实现场景的功能需求。

但是planning中的配置参数量较大，入门调试时难度较高，用户想要修改的功能对应的参数不直观，并且不易快速定位需要修改的参数位置。

**针对这些问题，对配置参数目录进行了以下调整：**

> 
> ‍SUCCESS 将参数分成全局变量和局部变量，全局变量是多个算法或插件中共同使用的参数；局部变量是专属于某个算法或插件的参数。如果用户需要调整某个插件的参数，直接在插件的目录中查找。 添加对常用功能使用到的参数的说明文档，方便用户调试时查询。 
> 

对参数目录进行重新梳理和作用范围的划分，有以下优点：

> 
> ‍INFO 参数的目录跟随作用范围和功能，这样对参数的定位更清晰和直观。 
> 

**planning2.0对配置参数新增功能：**

> 
> ‍SUCCESS 用户新增的插件所使用的参数，可以跟随插件进行发布和管理。 优化代码跳转功能 同时引入了一个profile的配置参数插件，用户可以使用这个profile插件，将配置参数放在该目录中，在profiles目录下，用户都可以定义一到多份配置参数，并可以使用aem工具对查看和管理配置参数。 
> 

## planning2.0相关云实验

**实验一：Apollo规划之减速带配置仿真实践**

[https://apollo.baidu.com/community/course/23](https://apollo.baidu.com/community/course/23)

**实验二：Apollo规划之交通灯场景仿真调试**

[https://apollo.baidu.com/community/course/9](https://apollo.baidu.com/community/course/9)

**实验三：Apollo规划之借道绕行仿真调试**

[https://apollo.baidu.com/community/course/7](https://apollo.baidu.com/community/course/7)

# 1、配置参数综述

在Apollo的planning2.0中，我们对配置参数进行了重要的更新，以提高系统的灵活性和适用性。以下是更新的主要内容：

> 
> ‍INFO 配置参数拆分：我们对1.0版本的配置参数进行了细分，将原本的配置参数拆分成了两大类：全局参数（planning_base）和局部参数（scenarios、tasks、traffic_rules）。全局参数包含了规划模块的基本参数设置，而局部参数则针对不同的场景和任务进行了特定的配置。 多份配置参数切换：为了适应不同的应用需求，我们引入了多份配置参数切换方式。用户都可以定义一到多份配置参数，以满足不同的场景和任务要求。 
> 

planning参数配置分为全局参数和局部参数，全局参数是在planning基本流程和基础算法中使用的参数，在planning_base文件夹中定义和配置，包含gflag和protobuf两种文件配置方式；局部参数是在planning的插件中定义和配置的，它只作用于所属的插件。

## 配置参数分类如下：

经过修改后的配置参数文件目录可以分为：全局配置参数和局部配置参数。无论是全局还局部配置参数，它们的配置目录都可以分为两类：一类是采用proto文件进行定义，用于定义配置参数的数据结构；另一类是采用conf文件进行配置，用于包含实际的配置参数。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/99_其他/e2cf7b41f26f36a61782d85a2d98ef84.png)

> 
> ‍INFO planning参数配置的结构如下： 
> 

### 全局配置参数：

| 全局配置参数 | 文件目录结构 | 文件 |  |
| --- | --- | --- | --- |
| planning_base | proto | planning_config.proto | planing全局配置参数定义 |
|  |  | traffic_rule_config.proto | traffic_rule生效列表配置定义 |
|  |  | 其他基础算法配置参数.proto | 其他算法配置参数定义 |
|  | conf | planning.conf | 全局flag变量 |
|  |  | planning_config.pb.txt | planning全局配置参数 |
|  |  | traffic_rule_config.pb.txt | 选择traffic_rule生成列表 |
|  |  | 其他基础算法配置参数.pb.txt | 其他算法配置参数定义 |
|  | common | planning_gflags.h | 全局flag变量定义 |
|  | scenario_base | proto/scenario_pipeline.proto | scenaro的pipeline定义 |

### 局部配置参数scenarios：

| 局部配置参数 | 场景 | 文件目录结构 |  |  |  |
| --- | --- | --- | --- | --- | --- |
| scenarios | park_and_go | proto | park_and_go.proto | Scenario:park_and_go scenario的配置参数定义 |  |
|  |  | conf | pipeline.pb.txt | Scenario Pipeline :park_and_go scenario的PiPeline参数配置 |  |
|  |  |  | scenario_conf.pb.txt | Scenario :park_and_go scenario的配置参数 |  |
|  |  |  | park_and_go_adjust Stage目录： park_and_go just stage的task配置参数 | open_space_roi_decider.pb.txt | Task：open_space_roi_decider task的配置参数 |
|  |  |  |  | open_space_trajectory_provider.pb.txt | Task：open_space_trajectory_provider的配置参数 |
|  |  |  | park_and_go_check Stage目录：park_and_go_check stage的task配置参数 | open_space_roi_decider.pb.txt | Task：open_space_roi_decider task的配置参数 |
|  | lane_follow | proto |  |  |  |
|  |  | conf |  |  |  |
|  | bare_intersection_unprotected | proto |  |  |  |
|  |  | conf |  |  |  |
|  | … |  |  |  |  |

### 局部配置参数tasks：

| 局部配置参数 | 场景 | 文件目录结构 |  |  |
| --- | --- | --- | --- | --- |
| tasks | lane_borrow_path task默认配置 | proto | lane_borrow_path.proto | lane_borrow_path task的配置参数定义 |
|  |  | conf | default_conf.pb.txt | lane_borrow_path task的配置参数 |
|  | open_space_roi_decider task默认配置 | proto |  |  |
|  |  | conf |  |  |
|  | path_decider task默认配置 | proto |  |  |
|  |  | conf |  |  |
|  | … |  |  |  |

### 局部配置参数traffic_rules：

| 局部配置参数 | 场景 |  |  |  |
| --- | --- | --- | --- | --- |
| traffic_rules | baskside_vehicle task默认配置 | proto | backside_vihicle.proto | backside_vihicle的配置参数定义 |
|  |  | conf | default_conf.pb.txt | backside_vihicle的配置参数 |
|  | crosswalk task默认配置 | proto |  |  |
|  |  | conf |  |  |
|  | keepclear task默认配置 | proto |  |  |
|  |  | conf |  |  |
|  | … |  |  |  |

# 2、Planing_base参数配置

> 
> ‍SUCCESS Planing_base:全局配置参数 
> 

## 2.1 概述

全局配置参数，是指会参与规划上所有内容的参数。这些参数对整个规划过程起着全局性的影响，它们决定了规划模块的基本行为和策略。全局配置参数在整个规划系统中是通用的，不会因为不同的场景或任务而变化，因此它们适用于所有的规划任务。

对应的配置文件在“modules/planning/planning_base/conf/planning.conf”中：

navi模式

对应的配置文件在“modules/planning/planning_base/conf/planning_navi.conf”中：

## 2.2 配置流程

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/99_其他/d00b5cf71adb0cc6c7f9e7ae8b00f160.png)

开发者如果对配置参数有特定的修改要求，只需输入同步指令，系统将自动将全局配置参数复制到profile的default目录中，然后就可以在profile目录上轻松修改配置参数。

buildtool profile config init --package planning --profile=default
fragment

 使用default目录这份配置，执行以下命令：

aem profile use default
fragment

profile插件目录结构

profiles/default/modules/planning/planning_base/
|-- conf
|
|-- discrete_points_smoother_config.pb.txt
|
|-- planner_open_space_config.pb.txt
|
|-- planning.conf
|
|-- planning_config.pb.txt
|
|-- planning_config_navi.pb.txt
|
|-- planning_navi.conf
|
|-- planning_semantic_map_config.pb.txt
|
|-- qp_spline_smoother_config.pb.txt
|
|-- spiral_smoother_config.pb.txt
|
`-- traffic_rule_config.pb.txt
|-- dag
|-- launch
`-- testdata
fragment

## 2.3 实践

### 全局配置参数对车辆规划的影响

在这个实践案例中，我们将通过调整不同的配置参数，探索全局配置参数对车辆速度规划、减速带处理以及绕行距离规划的影响。我们将使用planning2.0来进行参数配置和规划，并通过Apollo的dreamview来观察车辆规划的效果。

**实践内容：**

> 
> ‍INFO 全局速度配置实践： 目标：调整全局配置参数，使得车辆的最高行驶速度不超过10m/s。 方法：在planning2.0中修改全局速度配置参数。 结果：在dreamview上实时查看车辆规划的速度情况，验证是否符合要求。 减速带配置实践： 目标：调整全局配置参数，使得车辆通过减速带时的最高行驶速度不超过4m/s。 方法：在planning2.0中设定不同的减速带配置参数。 结果：通过dreamview实时观察车辆的规划路径和速度情况，验证是否满足要求。 绕行距离配置实践： 目标：调整全局配置参数，使得车辆在遇到障碍物时与障碍物保持1.0m的距离。 方法：在planning2.0中改变绕行距离配置参数。 结果：在dreamview上实时查看车辆绕行的路径和与障碍物的距离，验证是否符合要求。 
> 

**实践目的：**

> 
> ‍SUCCESS 探索全局配置参数对车辆规划的影响。将关注全局速度配置、减速带配置以及绕行距离配置，分别观察车辆规划速度、减速带处理和绕行路径规划的效果。 
> 

**实践流程：**

> 
> ‍INFO 默认配置参数规划效果 启动dreamview并观察默认配置参数下车辆速度规划、减速带处理以及绕行距离规划的仿真效果 使用planning2.0仿真：https://apollo.baidu.com/community/article/1116
> 全局速度配置实践： a. 找到planning2.0中全局速度配置的相关文件。 b. 根据要求，调整全局速度配置参数，确保车辆的最高行驶速度不超过10m/s。 c. 保存修改后的配置参数，并重新启动planning 模块。 d. 运行车辆规划模块，并在dreamview上观察车辆规划速度的变化。
> 减速带配置实践： a. 找到planning2.0中减速带配置的相关文件。 b. 设定不同的减速带配置参数，确保车辆通过减速带时的最高行驶速度不超过4m/s。 c. 保存修改后的配置参数，并重新编译planning2.0模块。 d. 运行车辆规划模块，并在dreamview上实时观察车辆遇到减速带时的通行速度和规划路径。
> 绕行距离配置实践： a. 寻找planning2.0中绕行距离配置的相关文件。 b. 改变绕行距离配置参数，确保车辆在遇到障碍物时与障碍物保持1.0m的距离。 c. 保存修改后的配置参数，并重新编译planning2.0模块。 d. 运行车辆规划模块，并在dreamview上实时查看车辆绕行的路径和与障碍物的距离。 
> 

#### 2.3.1.1默认配置参数规划效果

### 全局速度配置实践：

**相关云实验：Apollo规划之减速带配置仿真实践**

[https://apollo.baidu.com/community/course/23](https://apollo.baidu.com/community/course/23)

启动dreamview后，选取这三个场景进行仿真依次进行仿真。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/99_其他/b5889c5654b1721b8825f08ce1b8e4d4.png)

#### 2.3.1.2全局速度配置实践：

在全局配置参数中找到该全局速度的配置参数：

–default_cruise_speed=1.5 将配置参数的值修改为：

–default_cruise_speed=10.0 修改之后保存代码后进行仿真。

**实验现象：**

|  | 原始 | 修改后 |
| --- | --- | --- |
| 全局速度配置实践 |  |  |

#### 2.3.1.3减速带配置实践：

### 减速带配置实践：

**相关云实验：Apollo规划之减速带配置仿真实践**

[https://apollo.baidu.com/community/course/23](https://apollo.baidu.com/community/course/23)

将speed_bump_speed_limit 参数添加至全局配置参数中。

–speed_bump_speed_limit=1.5 将配置参数的值修改为：

–speed_bump_speed_limit=4.0 修改之后保存代码后进行仿真。

**实验现象：**

|  | 原始 | 修改后 |
| --- | --- | --- |
| 减速带配置实践 |  |  |

#### 2.3.1.4绕行距离配置实践：

### 绕行距离配置实践：

**planning2.0相关云实验：Apollo规划之借道绕行仿真调试**

[https://apollo.baidu.com/community/course/7](https://apollo.baidu.com/community/course/7)

全局配置参数中找到减速带配置参数

–obstacle_lat_buffer=0.3 将配置参数的值修改为：

–obstacle_lat_buffer=1.0 修改之后保存代码后进行仿真

|  | 原始 | 修改后 |
| --- | --- | --- |
| 绕行距离配置实践 |  |  |

# 3、traffic rules参数配置

> 
> ‍SUCCESS traffic rules:局部配置参数 
> 

## 3.1 概述

traffic rules实现了在所有场景下都会生效的规则，用户可以根据自己的需求，决定启用哪些traffic rules，对应的配置文件在“modules/planning/planning_base/conf/traffic_rule_config.pb.txt”中：

rule {
name: “BACKSIDE_VEHICLE”
type: “BacksideVehicle”
}
rule {
name: “CROSSWALK”
type: “Crosswalk”
}
rule {
name: “DESTINATION”
type: “Destination”
}
rule {
name: “KEEP_CLEAR”
type: “KeepClear”
}
rule {
name: “STOP_SIGN”
type: “StopSign”
}
fragment
> 
> ‍INFO 
> 

rule中的”name”为场景的别名，用户可以自定义，使用大写字母命名；”type”是对应场景插件类的类名，省略namespace apollo::planning。 对于每一个交通规则，还有其自己的交通规则参数，位于交通规则插件的conf目录内。比如人行道规则参数位于

modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt
fragment
stop_distance: 1.0
max_stop_deceleration: 4.0
min_pass_s_distance: 1.0
expand_s_distance: 2.0
stop_strict_l_distance: 4.0
stop_loose_l_distance: 5.0
start_watch_timer_distance:10
stop_timeout: 10.0
fragment

## 3.2 配置流程

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/99_其他/6a69ae7f9a84f4969d8abafd241f63ed.png)

开发者如果对配置参数有特定的修改要求，只需输入同步指令，系统将自动将全局配置参数复制到profile的default目录中，然后就可以在profile目录上轻松修改配置参数。

buildtool profile config init \
--package planning-traffic-rules-traffic-light \
--profile=default
fragment

 查看profile插件目录结构：

tree profiles/default/modules/planning/traffic_rules/
fragment

目录结构：

profiles/default/modules/planning/traffic_rules/
`-- traffic_light
`-- conf
`-- default_conf.pb.txt
fragment

## 3.3 实践案例

### 3.3.1Traffic Rules配置参数对车辆规划的影响：

在这个实践案例中，我们将通过调整不同的配置参数，探索Traffic Rules配置参数对车辆遇到红绿灯时的停车距离的影响。我们将使用planning2.0来进行参数配置和规划，并通过Apollo的Dreamview来观察车辆规划的效果。

**实践内容：**

> 
> ‍INFO 局部配置参数交通灯场景配置实践 目标：通过调整Traffic Rules配置参数，使得主车在监测到前方为红灯时，能够准确停车在距离停止线2米至2.5米的位置，不得超过停止线。当交通灯变为绿色后，主车能够顺利通过红绿灯路口。 方法：在planning2.0中找到Traffic Rules配置的相关文件，并调整配置参数，以满足红绿灯场景的要求。 结果：在Apollo的Dreamview仿真环境中，观察主车在红绿灯场景下的行驶情况，验证是否能够在红灯时停车在合适位置，并在绿灯后通过红绿灯路口。 
> 

**实践目的：**

> 
> ‍SUCCESS 探索Traffic Rules配置参数对车辆规划的影响。观察车辆在不同情况下的停车行为，并探索如何优化交通规则处理模块，以实现更精确和安全的车辆规划。 
> 

**实践流程：**

> 
> ‍INFO 默认配置参数规划效果 启动dreamview并观察默认配置参数下车辆遇到交通灯的仿真效果。 使用planning2.0仿真：https://apollo.baidu.com/community/article/1116
> 红绿灯场景实践： a. 找到planning2.0中Traffic Rules配置的相关文件。 b. 根据要求，调整Traffic Rules配置参数，确保车辆停车在距离停止线2米至2.5米的位置停车。 c. 保存修改后的配置参数，并重新启动planning模块。 d. 运行车辆规划模块，并在dreamview上观察车辆遇到红绿灯场景的停车变化。
> 
> 

#### 3.3.1.1 红绿灯场景实践配置参数修改

### 红绿灯场景配置实践：

**planning2.0相关云实验：Apollo规划之交通灯场景仿真调试**

[https://apollo.baidu.com/community/course/9](https://apollo.baidu.com/community/course/9)

在Traffic Rules参数中找到交通灯停止距离的配置参数

stop_distance: 1.0
fragment

将默认配置参数修改成

stop_distance: 2.0
fragment

 **仿真对比：**

|  | 原始 | 修改后 |
| --- | --- | --- |
| 绕行距离配置实践 |  |  |

# 4、planning2.0配置文件新功能

在Apollo的planning2.0中，为了解决不同场景下可能需要对同一模块使用不同配置参数的问题，我们引入了profiles目录，并提供了使用aem工具来查看和使用profile配置文件的功能。

在profiles目录下，用户可以定义一到多份配置参数，以适应不同的应用场景。通过使用aem命令，我们可以轻松地查看已有的profile，并在不同场景下切换使用不同的配置参数。这为我们快速切换配置提供了便利，以验证不同参数对系统性能和效果的影响，从而优化规划模块的表现。

**profile工作原理如下：**

在planning2.0中，我们将配置参数分为了两大类，分别为全局配置参数（组件）和局部配置参数（插件）。

在读取配置参数时，其优先级如下：

组件：profile > 默认，后续会改为工作空间源码 > profile > 默认

插件：工作空间源码 > profile > 默认

**关于profile：**

> 
> ‍SUCCESS 在默认情况下，profile目录是空的，不包含任何配置参数，profile软链会优先使用编译产出后的配置参数。想将其他配置参数添加到profile目录中，需要输入同步指令，将配置参数复制到profile目录下。 通过执行同步指令，将在profile目录中看到与默认配置参数相同的文件。另外，如果希望切换不同的配置参数，可以通过输入切换指令来实现，这会改变软链的指向，从而选择不同的配置文件。 
> 

**关于工作空间源码：**

> 
> ‍INFO 这里所谓的“工作空间源码”指的是工作目录下的modules文件夹中包含了该份代码的配置参数。这意味着系统会优先读取工作目录中的源码配置参数，以确保在配置参数的选择过程中具有更高的灵活性和个性化定制性。通过这一机制，我们可以更加灵活地适应不同的应用需求，从而提升系统的适应性和性能。 
> 

以下是aem指令用于查看和切换配置参数：

buildtool profile config init
# 查看已有的profile
aem profile list
# 使用名字叫default的这份配置
aem profile use default
fragment

 在profiles目录下，用户可以定义一到多份配置参数。这样的好处在于

> 
> ‍SUCCESS **场景适配性**：通过根据不同的场景设置不同的配置参数，我们可以轻松地适应各种应用场景。不同的任务、环境或需求可能需要不同的配置，这样的灵活性能够提高系统的适应性和性能表现。 **参数保存和复用**：在调参过程中，我们可以将认为比较好的参数保存在一个文件夹中。这样，我们不仅能够方便地复用之前优化过的参数，还能够在之后的调参过程中避免遗忘或丢失重要的设置，提高了工作效率。 **快速回滚**：通过保存各个版本的配置文件，如果在后续调整参数时发现新的配置不如之前的好，我们可以轻松地将代码切换回之前保存的较为完美的配置文件。这样能够迅速回滚到之前的状态，避免不必要的错误或性能下降。 **团队协作**：当多个团队成员共同参与项目时，可以为每个成员创建独立的配置文件，便于个人调试和优化，而不会干扰其他人的工作。这种方式有助于提高团队协作效率，各成员可以专注于自己的任务而不用担心影响他人。 
> 

## 4.1多份配置参数切换方式

在Apollo的planning2.0中，我们引入了多份配置参数切换方式，使得根据不同的场景使用不同的配置成为可能，以适应各种应用需求。下面将详细介绍如何设置参数目录：

### 4.1.1参数设置目录设置

首先，进入docker工作空间，在/apollo_workspace路径下输入以下配置参数指令，将配置参数复制到profile插件目录中。

> 
> ‍INFO 注意：默认情况下，配置参数使用的是 
> 

示例指令，查看已有的profile配置指令：

aem profile list
fragment

 profile插件的配置文件目录

default demo_1 demo_2 在这个例子中，通过执行以下指令，我们可以查看当前使用的配置为"default"这份配置文件，并查看profiles目录的结构：

tree profiles
fragment

profile目录结构：

profiles/
├── current -> default  # 软链
├── default
├── demo_1
└── demo_2
fragment

 通过使用以下命令，我们可以轻松切换到名为"demo_1"的配置参数：

# 使用名字叫 demo_1 的这份配置
aem profile use demo_1
fragment
> 
> ‍INFO 这样的做法十分便捷，不需要手动处理所有配置文件，只需关注需要修改的部分，系统会智能地处理其余配置，确保我们能够快速调整参数并保持系统正常运行。这样的配置管理方式有效地提高了工作效率和代码的可维护性，让调参过程更加简洁和灵活。 
> 

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。