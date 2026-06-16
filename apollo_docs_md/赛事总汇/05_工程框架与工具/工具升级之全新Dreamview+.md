---
title: 工具升级之全新Dreamview+
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_205___xE5_xB7_xA5_xE7_xA8_x8B_xE6_xA1_1f2deb4984443f17ac2d130c30a49c30.html
category: 赛事总汇 > 05_工程框架与工具 > 工具升级之全新Dreamview+
---

# 工具升级之全新Dreamview+

# 工具升级之全新Dreamview+

> 
> ‍作者: 宇新 | 发布时间: 2023-11-23 17:32 | 链接: https://apollo.baidu.com/community/article/1206 
> 

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/99a9cacb8965c1e0ed829fcbba3fa8a9.png)

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/d583b805c84fe020d646b9704c8edd88.png) ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/b4ab46d99e89ee30b05864549160e48b.png)

# <strong>1 Dreamview+是什么？</strong>

## 1.1 Dreamview+简介

Dreamview+是Apollo框架下的可视化调试工具，通过它，开发者能够直观地查看算法的输出结果，对算法进行调试。

对于感知开发者，可以通过Dreamview+查看感知原始的摄像头图像和点云数据，以及算法对障碍物的识别和输出结果。

对于PnC开发者，可以通过Dreamview+查看规划控制的相关输出，包括规划路径、规划线以及自车的行驶状态等。

## 1.2 Dreamview+升级点

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/bbc3fb0e847e1a8d524c8371848939af.png) ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/884216fe2f124a68ab0692e0ee38a2ad.png)

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/d57f7ad9015624bd959a2a278c3cb3a2.png)

## 1.3 Dreamview+功能一览

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/34e0022a9b8cce6213008f61ed4ae5ec.png)

# <strong>2 Dreamview+怎么用？</strong>

## 2.1 安装/启动Dreamview+

本地安装：[https://apollo.baidu.com/docs/apollo/latest/md_docs_2installation__instructions_2essential__software__installation__guide__cn.html](https://apollo.baidu.com/docs/apollo/latest/md_docs_2installation__instructions_2essential__software__installation__guide__cn.html)

线上体验：云实验《定速巡航场景仿真调试》、《借道绕行仿真调试》

## 2.2 插件同步与资源下载

步骤1：从浏览器中打开[https://apollo.baidu.com/workspace](https://apollo.baidu.com/workspace)，进入Apollo Studio云端工作台

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/205a9f7145def23b370f5316af6ef6a3.png)

步骤2：点击“个人中心”，打开“我的服务”

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/c9f574b6be3cc99e5db8d649551d3447.png)

步骤3：选择“仿真”，在“插件安装”中点击“生成”，选择Apollo版本后点击“确定”

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/c9f574b6be3cc99e5db8d649551d3447.png)

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/168c7e962b238a7b9237bd77364cf61e.png)

步骤4：选择“一键复制”，之后在docker环境中运行该指令，插件同步（Dreamview+的登陆）就完成了

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/343c6d3daa486bd05e9f89b6ecc83800.png)

## 2.3 可视化播放PnC数据包

步骤1：选择PnC模式/PnC Mode，选择播包操作/Records Operation，在资源中选择PnC数据包，可以选择默认的PnC数据包，点击底部栏的播放/Play

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/cfae8404400cee37b47c15e7f1cfce78.png)

步骤2（可选）：可调整面板布局以便于查看需要的内容

## 2.4 可视化播放感知数据包

步骤1：选择感知模式/Perception Mode，选择播包操作/Records Operation

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/46eddac65a7d7d2a870d434dc182a544.png)

步骤2：在资源中选择感知数据包，可以选择默认的感知数据包，点击底部栏的播放/Play

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/f7987ccb41f69522488f77c6cd71a4ba.png)

步骤3（可选）：可调整面板布局以便于查看需要的内容

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/ebed53fc44f95b092cdaa78e9369d20b.png)

## 2.5 PnC场景仿真

步骤1：选择PnC模式/PnC Mode，选择场景仿真操作/Scenario_Sim Operation（此时Viz界面有数据则成功切换），打开Modules中的Prediction和Planning

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/070196a2d53e968988003494f9bf7d78.png)

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/e45bfe6828938104c595b753a247f4d0.png)

步骤2：在资源中找到场景集列表，选择一个场景，点击底部栏的运行/Run，即可查看结果

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/56eb7c09c17861cf44e5233b008d1da4.png)

步骤3（可选）：可调整面板布局以便于查看需要的内容

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/f584f247bb19807b227f31b11e69f56f.png)

## 2.6 PnC自由仿真

步骤1：选择PnC模式/PnC Mode，选择自由仿真操作/SimControl Operation

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/9bef4e4d12ff0f81dd23eeec2e8b419c.png)

步骤2：打开Routing Editing，绘制途径点-终点，或是直接绘制终点，返回主页，点击底部栏的运行/Run，即可查看结果

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/049ba5e10b222010abfd9562c5357baf.png)

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/78a06bd2d9ac352f2ed1b457af49809f.png)

步骤3（可选）：可调整面板布局以便于查看需要的内容，比如调整PnCMonitor面板

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/f584f247bb19807b227f31b11e69f56f.png)

## 2.7 自定义面板布局

改变当前子面板大小：鼠标移动至面板中间，待光标变为双向箭头，按住鼠标左键拖动，改变面板布局 ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/a465b7d8ddc45c4f51919a88f3ca1917.png)

改变面板位置：鼠标移至面板顶部栏，待光标变为手，按住鼠标左键拖动，改变面板位置 ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/bd3a8b8ccc4942c0d51b1024fcf07c4f.png)

删除面板、全屏、向左拆分、向右拆分：点击面板顶部栏的设置按钮，选择对应功能即可 ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/ae3d20b0c637c5f92cfc019804e7a78a.png)

添加新面板：左侧找到添加面板Add Panel按钮，在列表中选择想要添加的类型，按住拖动到右侧期望的位置，添加完成 ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/b8bff42388826d2d04cdbb6f0b32a8d6.png)

## 2.8 中英版本切换

打开左下角“我的”-“设置”，在通用设置中选择语言即可

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/b2fa21d3bb16c275b0831fe2f53de415.png)

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/86d4f6cff69428a042cd6057e2e4a1c1.png)

# 3 Dreamview+架构详解

## 3.1 旧版Dreamview架构

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/805c1f397f5150da008d1b8525c69883.png)

回看一下Dreamview的架构模型，这套架构在易用性和快速开发方面具有明显优势，然而，这种架构也面临着一些问题。

**请求-响应模式通讯冗余：** Dreamview的架构采用的是传统Web架构，这在处理自动驾驶这类高度复杂且要求实时性的业务时可能显得不够适应。在这种场景下，传统的请求-响应模式可能无法满足对高效实时数据处理和流式传输的需求。 

## **二次开发导致代码分叉：**当系统进行二次开发以适应特定需求时，如果偏离了Apollo Dreamview的官方版本，可能会遇到集成后续更新成本过高，甚至无法集成的问题，难以适应不断变化的工具链和服务需求。

全新架构的设计**重点**：在原有大框架下提升前后端通信效率以及系统的扩展性和动态配置能力。

## 3.2 新版Dreamview+架构

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/1504ad5ad1e8ba22c48e593e19f0bb07.png)

架构设计旨在满足业务需求的演变，在开始详细了解dreamview+架构，我们思考这样一个需求：

> 
> ‍INFO 如何实现Cyber RT中的特定channel的时序数据可视化？ 
> 

| 版本 | Dreamview | Dreamview+ |
| --- | --- | --- |
| 步骤1 | 前端添加request接口和定时器不断获取后端数据 | 0代码 |
| 步骤2 | 后端添加response接口并注册对应reader的channel |  |
| 步骤3 | reader中添加对应回调函数获取数据传递到前端 |  |

![](https://apollo.baidu.com/docs/apollo/images/06_工程框架/f62f9c57141e601fd44656688ed959ee.gif)

**注：当前功能目前仍处于内测阶段，会随9.0正式版发布。*

这一能力的实现背后驱动力正是Dreamview+中全新设计的 — DV Protocol，接下来我将详细介绍下DV Protocol。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/843cde122298b452c096355f95324640.png)

### 3.2.1 DV Protocol

**是什么?** DV Protocol 是一个专为Dreamview+设计的标准通信协议，旨在通过一系列定义良好的规则和结构来优化前后端之间的数据交换。 **为什么？**

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/805c1f397f5150da008d1b8525c69883.png)

在原版Dreamview中，我们面临着一个明显的架构问题：WebSocket 客户端数据的处理逻辑与 MobX 状态管理库的交互过于紧密。这种耦合导致了**状态管理层**和**数据传输层**之间的职责界限变得模糊，从而造成了以下问题：

**逻辑复杂性违背关注点分离原则**：状态管理本应仅关注视图层状态的反应性更新，然而，它被迫参与处理来自 WebSocket 的原始数据流，这违背了关注点分离原则。 **数据与视图的紧密耦合导致的维护困难**：数据处理与视图状态间的紧密耦合，使得前后端逻辑交织，难以维护。 为了解决这些问题，DV Protocol 在Dreamview+中被引入，旨在分离数据传输逻辑与视图状态管理。通过引入元数据协商和数据流订阅管理等措施，我们能够实现一个解耦的架构：其中状态管理专注于视图的响应性，而数据传输则通过 DV Protocol 标准化，实现了逻辑的清晰分层和系统的高内聚低耦合，从而提高了数据处理效率和整体系统的可维护性。

###### 协议概要:

DV Protocol 是一个为 Apollo Dreamview+ 设计的通信协议，它利用了 WebSocket 的文本和二进制消息特性来实现客户端和服务端之间的实时通信。这个协议定义了一系列操作（action），以指导消息的处理和数据流的管理。下面，我们将详细解释这个协议的工作原理和关键组成部分。

###### 通信基础

消息类型：DV Protocol 通过 WebSocket 发送的每个消息都包含一个 action 字段，该字段是一个操作标识，用于标识消息的类型和预期的操作。 文本消息：以 JSON 对象的形式发送，action 字段必须存在。 二进制消息：通过标准proto序列化包装实际业务数据；通过Metadata实现数据的反序列化。

{
"action": "subscribe",
"type": "subscribe",
"data": {
"name": "subscribe",
"source": "dreamview",
"info": {
"websocketName": "map",
"dataFrequencyMs": 100
},
"sourceType": "websocktSubscribe",
"targetType": "module",
"requestId": "subscribe"
}
}
fragment

**DV Protocol 操作的类型**

请求消息相关操作 请求消息类型 request：客户端发起请求，期望获得一些数据或触发某个动作。 订阅消息类型 subscribe：客户端表明希望开始接收特定数据流的更新。 取消订阅消息类型 unsubscribe：客户端表明希望停止接收特定数据流的更新。 响应消息相关操作 响应消息类型 response：对客户端请求的直接响应。 流消息类型 stream：从服务器到客户端的数据流消息，通常包含实时数据。 元数据相关操作 元数据消息类型 metadata：服务器发送给客户端的消息，描述了服务器可以提供的数据流及其相关元信息，帮助客户端理解如何订阅和解释这些数据流。 元数据增量更新消息类型 join：服务器发送元数据的增量更新，例如添加了新的数据流。 元数据局部清理消息类型 leave：服务器通知客户端某部分元数据已经不再有效。 **通信模式**

DV Protocol 基于 WebSocket 协议，封装了三种主要的通信模式：

数据流订阅：客户端可以订阅特定的数据流，服务端会按照客户端的订阅，周期性地发送数据。

请求-响应：客户端发送请求给服务端，服务端处理后返回一个响应。

请求-响应流：这是请求-响应的一个变种，其中客户端的请求会触发服务端发送一个数据流，而不是单一的响应消息。

###### 订阅数据流通讯时序图

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/3e4bbdb13a4c538c65fb43a4c39ced14.png)

> 
> ‍INFO 数据流： 在 Dreamview+中，数据流是指一连串持续更新的数据序列，由后端系统生成并发送到HMI。这些数据通常包括自动驾驶车辆的实时状态信息。数据流是动态的，随时间变化而持续更新，旨在实时反映车辆的操作和环境情况。在系统建立前后端连接时，后端会向前端声明它能够提供的数据流类型，从而使前端能够基于这些信息决定订阅哪些特定的数据流，以便在不同的面板上显示和处理。通过这种方式，数据流成为连接后端处理逻辑和前端展示界面的关键桥梁，确保了信息的实时传递和精确展示。 预置的数据流包含： 核心数据流（Core Data Streams），例如 hmistatus、simulationworld、cyber；感知数据流（Perception Data Streams），例如camera、pointcloud、obstacle等。 
> 

###### 元数据（Metadata）

在Dreamview+中，Metadata 是一个核心数据结构，用于封装客户端和服务端之间的通信协议元信息。它包含了一系列元数据项，定义了服务端管理的数据流的结构和语义，使得客户端能够动态地解释和订阅这些数据流。

服务端 Metadata 基于配置自动注册流式数据并创建WebSocket监听。

data_handler_info  {
key: "apollo.dreamview.SimulationWorld",  value  {
data_name: "simworld",    msg_type: "apollo.dreamview.SimulationWorld",    websocket_info  {
websocket_name: "simworld",      websocket_pipe: "/simworld"
}
}
}
fragment

客户端基于Metadata实现动态订阅和处理服务端数据的能力。

export type MetadataItem = {
/** 数据流的名称。 */
dataName: StreamDataNames;
/** 通过该数组中的频道进行数据流通信。 */
channels: Channel[];
/** 表明是否存在具有唯一schema的不同频道。 */
differentForChannels: boolean;
/** protobuf schema的文件路径。 */
protoPath: string;
/** protobuf schema中定义的消息类型的描述符。 */
msgType: string;
/** WebSocket通信详细信息。 */
websocketInfo: {
/** WebSocket连接的指定名称。 */
websocketName: string;
/** 通过该通道或’管道’发送WebSocket消息。 */
websocketPipe: string;
};
};
fragment

 DV Protocol 为Dreamview+提供了一套完整的数据通信机制，不仅包括了前文提及的基本通信方式、通讯模式等，还涉及到以下重要的特性和组件： **元数据（Metadata）协商**：此机制使客户端能够与后端同步更新元数据。 **基于protobuf的动态数据流传输协议**：收敛、统一数据流通讯方式。 **独立客户端组件WebSocket Gateway**：该组件作为数据通信的关键接口，管理客户端与服务端之间的WebSocket连接，协调数据订阅与分发，确保数据传输的高效和可靠。

###### DV Protocol 优势

通信效率提升：通过标准化的通信协议和二进制数据传输，DV Protocol 提高了数据交换的速度和准确性。 动态数据管理：元数据的使用使得前端能够根据后端的实时变化调整用户界面，提高了系统的动态性和灵活性。 前后端紧密协作：元数据协商机制加强了前后端的协作，确保双方能够高效地同步数据和处理数据。 通过这些变更，Apollo Dreamview+ 的新架构提供了更加健壮和高效的前后端通信解决方案，以应对自动驾驶领域不断增长的数据处理需求。当然 DV Protocol 还需要随着技术的进步和用户需求的变化而持续发展和完善。期待社区开发者积极参与，通过分享你们的见解、经验和创新想法来不断完善DV Protocol。

> 
> ‍INFO 需求2 如何实现同时在追踪视角和俯瞰视角下观测主车周围环境与自动驾驶运行状态？ 
> 

要实现这个需求，就需要依赖Dreamview+架构另一项全新能力-面板管理器。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/8ec0ec5fad1f79e74986d8ded492091b.png)

**是什么？** 面板管理器是一个关键的前端组件，它负责管理和协调Dreamview+界面中的各个面板。每个面板可以视为一个独立的交互单元，负责展示特定的数据和信息。面板管理器使得这些面板可以独立地指定和订阅所需的数据流，而不依赖于全局状态。它提供了一个灵活且模块化的环境，允许面板根据特定需求订阅和接收数据，减少了面板之间的直接依赖关系。

> 
> ‍INFO 面板: 在Dreamview+架构中，面板（Panel）被定义为一个封装了时序数据和业务逻辑可插拔的模块化前端组件。每个面板能够独立运作，通过声明式或命令式的方式订阅数据流，确保了整个系统的灵活性与扩展性。同时，面板的设计使得用户二次开发能力与Dreamview+官方发布的新功能和更新无缝融合。这种集成能力使面板成为了一个至关重要的元素，它不仅满足当前用户的需求，同时也为系统未来的功能扩展搭建了桥梁。 
> 

目前Dreamview+已为开发者预置10类面板, 部分面板功能目前仍处于内测阶段，会随9.0正式版发布。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/62dbd80588ce51f002f934ecba3af2f2.png)

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/f95a9208745a65e53bd5b92cd73ca14f.png)

通过面板管理器，我们能够高效地复制和应用任何自动驾驶业务功能。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/5f9652f5f87dc228710212bb5ca14aeb.png)

面板管理器主要包含两个核心部分：

###### 面板服务接口（Panel Service Interface）

Dreamview+更新引入了一套标准化的面板服务接口，旨在简化和加速开发者在测试阶段和后续版本中自定义面板的实现过程。包括： **订阅初始化**（initSubscription）：为面板设置初始数据流。 **面板元数据管理**（panelMetaData, updateMetaData）：管理面板的基本信息和属性。 **面板操作功能**（splitPanel, closePanel）：面板的分割和关闭。 **全屏模式切换**（enterFullScreen, exitFullScreen）：允许面板在全屏和普通模式间切换。 **键盘事件处理**（setKeyDownHandlers, setKeyUpHandlers）：设置快捷键。 等等，在此不一一列举。

> 
> ‍INFO 通过这些标准化接口，我们要如何实现一个自定义面板？ 
> 

import React, { useEffect, useState } from 'react';
import Panel from '@dreamview/dreamview-core/src/components/panels/base/Panel';
import { usePanelContext } from '@dreamview/dreamview-core/src/components/panels/base/store/PanelStore';
import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';

function InternalCustomPanel() {
const panelContext = usePanelContext();
const { initSubscription } = panelContext;
const [data, setData] = useState();

// 初始化订阅并设置数据处理
useEffect(() => {
initSubscription({
[StreamDataNames.SIM_WORLD]: {
consumer: (simData) => {
setData(simData); // 更新数据状态
},
},
});
}, []);

// 根据数据渲染UI
return (
<div>
{/* 这里可以放置UI组件，例如显示数据的列表或图表 */}
</div>
);
}

function CustomPanel(props) {
const Compnent = useMemo(
() =>
// 使用Panel高阶组件封装InternalCustomPanel
Panel({
PanelComponent: InternalCustomPanel,
panelId: props.panelId,
subscribeInfo: [{ name: StreamDataNames.SIM_WORLD, needChannel: false }],
}),
[],
);
return <Compnent {...props} />;
}

export default React.memo(CustomPanel);
fragment

 上面的代码，我们的自定义面板成功订阅了 SimulationWorld 数据流，可以展示来自模拟世界的实时数据。这种数据集成增强了面板的功能性，使其成为一个强大的工具，既可以用于数据监控，也可以用于进行更复杂的数据分析或可视化展示。这样的自定义面板不仅提升了用户交互体验，也为进一步扩展和定制提供了灵活性。

###### 动态窗口容器

![](https://apollo.baidu.com/docs/apollo/images/06_工程框架/9a80e3fe1de71594490a14f9d13b749c.gif)

新架构中的动态窗口容器支持窗口的拉伸、拖拽和快速拆分，允许用户根据个人需求自由调整窗口布局，并在不同的面板间灵活展示数据。

面板管理器的优势：

模块化和独立性：面板管理器实现了面板的独立模块化，允许单独管理数据订阅和状态，减少面板间依赖，提升应用的可维护性和可扩展性。 灵活的数据流管理：支持面板声明式数据流订阅，提高数据管理灵活性，使面板根据功能和需求获取数据，不受全局状态限制。 提升开发效率和自定义能力：集成标准化服务接口，简化自定义面板开发，覆盖数据更新到用户界面控制，增强开发效率和个性化。 优化用户交互体验：动态窗口容器增强交互性，支持窗口拉伸、拖拽、快速拆分，使用户可根据偏好灵活调整界面布局。 通过这些改进，Apollo Dreamview+的面板管理器不仅提供了更加强大和灵活的前端开发框架，同时也为用户带来了更丰富和个性化的交互体验，有效地支持了自动驾驶领域对动态、高效数据处理的需求。

### 3.2.3 其它

除了上述提及的重大改进之外，Dreamview+还引入了许多其他新特性，由于篇幅限制，这里无法对每一项进行详尽的介绍。下面我简单介绍下：

**引导提示**：为新用户提供了直观的模式引导，使他们更快地熟悉操作界面，提升了用户体验和易用性。 **国际化支持**：国际化使得Apollo Dreamview能够服务于更广泛的全球用户群体，适应不同语言和地区的需求，拓宽了其应用范围，目前支持中/英切换。 **主题支持**：主题定制功能让用户能够根据个人喜好调整界面风格，提供更加个性化的用户体验。 我们期待大家下载并使用这个新版本的Dreamview+。希望这些新特性能够为您带来更加丰富和愉快的体验，同时也欢迎大家提供宝贵的反馈和建议。希望大家喜欢这个不断进步的平台，让我们一起见证Apollo Dreamview+的未来发展。

---

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/6e86034929b441120164911c72dd8fd0.png) ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/06_工程框架/22ff985968b351e0b92c77439f1d9751.png)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。