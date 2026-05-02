# 基于Apollo 10.0的线控底盘CAN协议适配

> 作者: 宇新 | 发布时间: 2025-05-23 11:48 | 链接: https://apollo.baidu.com/community/article/1297

> 本教程介绍了Apollo10.0系统与线控底盘适配的两种方法，其中重点介绍了如何根据Apollo的CAN接口协议对线控底盘进行改造，实现CAN协议适配的方法。

Apollo 10.0安装：[https://apollo.baidu.com/community/article/1128](https://apollo.baidu.com/community/article/1128)

获取application-core代码：
```bash
git clone https://github.com/ApolloAuto/application-core.git
```

# 一、线控底盘适配思路

## 1、自动驾驶实车构成

![](../images/08_硬件传感器/f3800a4a8427f86ec0d0a6400c3a7c1e.png)

根据Apollo10.0的结构图，我们可以了解到自动驾驶包括硬件层、软件层、应用层和云服务四大模块，要实现实车的自动驾驶，我们需要分别了解已经应用这四个模块。

硬件层：需要准备线控底盘、处理器以及传感器硬件，提供对应的硬件接口与Apollo系统适配；
软件层（核心）：通过Apollo的实时系统，运行自动驾驶的核心组件，包括地图引擎、定位、感知、预测、规划、控制、显示以及V2X等；
应用层：使用Apollo软件层的能力，组合开发出不同场景的应用，例如教育、矿卡、共享巴士等；
服务层：提供云服务等工具，实现底盘评测、外参标定、高精地图构建、仿真、模型训练等功能。
![](../images/08_硬件传感器/2f1d09551cb6e472bcd5336350a8947e.png)

实车的硬件构成方面，自动驾驶实车设备需要一般需要包含有上图的器件，包括：汽车、处理器、IMU、GPS接收器、毫米波雷达、激光雷达、摄像头。如果要自己搭建一个适配Apollo实车，需要保证这些元器件都能够与Apollo系统进行通讯。

由于传感器精密度高，学员难以自己制作，可以挑选已经完成适配的传感器使用。

**实车参考套件**

如果评估在硬件的适配方面有较大困难的，也可以采购参考套件：

联系人：王老师 17322204491

![](../images/08_硬件传感器/45b60c770a3e8a6f76de56070a32f54d.png)

## 2、Apollo系统与线控底盘的适配方法

**方法一：修改CANBUS代码**

Apollo中实际上默认提供了很多种类车型的CAN协议，包括林肯、gem，ge3，devkit等。可以参考这些车型的代码，根据已有的车辆CAN协议，编写代码。

![](../images/08_硬件传感器/db18798ce1c94b6dd6078f0f4c880759.png)

这个方式需要涉及修改Apollo中的Canbus、Canbus_Vehicle、common_msgs和Cantrol模块的代码，需要对Apollo这部分代码有较为完整的理解。实车套件使用的是这种方式。

参考文档：[https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEAhAILiAnABZxEgOzK1Y%2BQA51M3ROUnJBsbK2WZo83IAi4%2BQDZ1F2Wb1vJS5bPFvPg0CAAYXKT1QpQAzGIIpBClkgGYEdhSXDL1SdmwUvT0NUMxQClQgA](https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEAhAILiAnABZxEgOzK1Y%2BQA51M3ROUnJBsbK2WZo83IAi4%2BQDZ1F2Wb1vJS5bPFvPg0CAAYXKT1QpQAzGIIpBClkgGYEdhSXDL1SdmwUvT0NUMxQClQgA)

**方法二：修改线控底盘协议**

第二种方式是根据Apollo接口修改底盘，Apollo提供了Devkit车型的底盘CAN协议。将线控底盘的协议逻辑按照Devkit车型的CAN协议修改，就可以在不修改Apollo代码的情况下实现底盘的适配。

![](../images/08_硬件传感器/70a037415ccbdd9af22b832534f009fe.png)

我们在此次培训中主要介绍的是这种方式，有很多学校都有自己制作小车的经验，应该可以通过这次培训理解并掌握线控底盘适配的方法。

## 3、系统及工具准备

**系统：**Ubuntu20.04 （Ubuntu18.04或者22.04应该都可以）

**Apollo版本：**Apollo10.0 

**软件：**Vector CANdb++ (其他可以查看DBC文件的软件都行）、异或校验器

暂时无法在飞书文档外展示此内容

**硬件：**CAN收发模块 - canable 2.0

**其他：**apollo_demo.dbc

---

# 二、Apollo CAN接口介绍

## 1、CAN总线简介

CAN：Controller Area Network，控制局域网络，最早由德国 BOSCH(博世)开发，目前已经是国际标准(ISO 11898)，是当前应用最广泛的现场总线之一。

CAN 总线使用两根线来连接各个单元：CAN_H 和 CAN_L，CAN 控制器通过判断这两根线上的电位差来得到总线电平，CAN 总线电平分为显性电平和隐性电平两种。显性电平表示逻辑“0”，此时 CAN_H 电平比 CAN_L 高，分别为 3.5V 和 1.5V，电位差为 2V。隐形电平表示逻辑“1”，此时 CAN_H 和 CAN_L 电压都为 2.5V 左右，电位差为 0V。CAN 总线就通过显性和隐形电平的变化来将具体的数据发送出去，如图所示：

![](../images/08_硬件传感器/f284b4403f63e1e9083ad9742b7b9ba8.png)

Apollo CAN总线的速率默认设置为500k，发送周期为20ms。

## 2、DBC文件使用

DBC（Database CAN的缩写）是[CAN网络](https://zhida.zhihu.com/search?content_id=248615106&amp;content_type=Article&amp;match_order=1&amp;q=CAN%E7%BD%91%E7%BB%9C&amp;zhida_source=entity)数据库文件。这种文件格式由德国VECTOR公司制定，关于这种格式的了解可以参考规范文件《DBC File Format Document》，可以在附件中的链接获取。DBC文件可以定义CAN网络的节点（Network Node）、报文（Message）、信号（Signal），以及每个节点收发哪些报文，每个报文包含的信号，信号的位置等等。

DBC文件需要使用CAN DB++软件打开，下载地址：[https://www.vector.com/se/en/download/candb-31-sp5/](https://www.vector.com/se/en/download/candb-31-sp5/)，安装完成后，可以看到Apollo_demo.dbc文件图标变化，如下图所示：

![](../images/08_硬件传感器/207b9f1de254304456c790edee77ddb6.png)

双击即可打开DBC文件：

![](../images/08_硬件传感器/bde942e65c454f6ed21295823e2e35d0.png)

---

**重要概念：**

**Byte Order：**CAN数据的字节序包含两种——MOTOROLA（大端序）、INTEL（小端序）。

大端序中，高位字节存储在低地址位，低位字节存储在高地址端。例如，十六进制数值0x12345678在大端字节序中的存储形式为12 34 56 78。这种排列方式符合人类的阅读习惯，从左至右依次进入。
小端序中，低位字节存储在内存的低地址端，而高位字节存储在高地址端。例如，十六进制数值0x12345678在小端字节序中的存储形式为78 56 34 12。小端模式在数据类型转换时效率较高，因为不需要调整字节内容。
**Value Type：**值类型包含signed、unsigned、IEEE Float、IEEE Double

<!--br {mso-data-placement:same-cell;}--> td {white-space:nowrap;border:0.5pt solid #dee0e3;font-size:10pt;font-style:normal;font-weight:normal;vertical-align:middle;word-break:normal;word-wrap:normal;}

| 类型 | 描述 |
| --- | --- |
| Signed | 有符号整型数据，范围：-2^{信号长度-1}~2^{信号长度-1}-1 |
| Unsigned | 无符号整型数据，范围：0~2^{信号长度}-1 |
| IEEE Float | 32位IEEE浮点数据，范围3.4*10^-38~3.4*10^38（有效数字7位） |
| IEEE Double | 64位IEEE双精度型数据，范围：1.7*10^-308~1.7*10^308（有效数字15位） |

**Factor：**因数，即传输信号值与实际信号值之间的倍率

**Offset：**偏移量，实际信号值 = 因数 * 传输信号值 + 偏移量

**Unit：**单位

**Value Descriptions：**一般用于枚举类的定义，例如：

![](../images/08_硬件传感器/dc25e726669bcfd24c99564c2dd3f3e0.png)

---

**Layout视图：**使用LAYOUT视图，可以通过可视化的方式看清楚报文的字节排序，例如有几个信号，每个信号占用的bit位置和数量。

![](../images/08_硬件传感器/d0f1dbfe511f49f7c9c5f53f3f2dc0a6.png)

## 3、CAN报文解析示例

以油门控制指令（Throttle_Command）为例
![](../images/08_硬件传感器/cd8baac171c85f61772f5c64b490bb61.png)

例如需要发送以下控制指令：

**油门使能、加速度为5.2m/s²、踏板值为25.3%，速度目标值为18m/s。**

首先我们定义一个标准帧，Can_id为0x100，8个字节。

查看Throttle_en_ctrl的ValueDescriptions，可知有两个枚举类型，0x00为不使能，0x00为使能。
![](../images/08_硬件传感器/1244f5260c9c1c0fe509ad838664d1be.png)

**第一个字节**设置为**0x01**，使油门使能。

加速度值为5.2，由于Factor为0.01，所以CAN传输的数值为5.2 / 0.01 = 520 = 0x208。但是需要注意的是，Throttle_Acc的起始位是22，而不是16。也就是说数据向左移了6位。CAN的传输值应该修改为0x208 &lt;&lt; 6 = 0x8200。
![](../images/08_硬件传感器/75d4a5297d7463d6ace9975fb0ae4e07.png)

**第二、三字节设置为0x8200。**

油门踏板值为25.3%，由于Factor为0.1，所以CAN传输的数值为25.3 / 0.1 = 253 = 0x00FD。
**第四、五字节设置为0x00FD。**

速度目标值为18m/s，由于Facator为0.01，所以CAN传输的数值为18 / 0.01 = 1800 = 0x0708。但是需要注意Speed_Target的起始位是52，而不是48，数据需要左移4位。CAN的传输值修改为0x0708 &lt;&lt; 4 = 0x7080。
**第六、七字节设置为0x7080。**

最后为异或校验位，可以使用异或校验计算工具计算，得到异或校验值为0x8E
![](../images/08_硬件传感器/7724450f9060649492aba1ea921a428b.png)

所以CAN的报文为 01 82 00 00 FD 70 80 8E。

> 异或校验位用于快速检验传输的数据是否出现错误。在接收数据时可以检查是否8字节数据的异或校验结果是否为0来判断数据传输是否正确。

![](../images/08_硬件传感器/cf7a4d021b68dc4e29a49068081ebaad.png)

# 三、CAN接口适配测试

> 在示例中，我们将会创建一个新的车辆预设，并通过这个来使用devkit车型。

## 1、硬件连接拓扑

![](../images/08_硬件传感器/55084f2766445bb4b7011b1b3b8922c5.png)

## 2、SocketCAN启动及测试

CAN接收器连接到电脑后，需要输入指令打开CAN0端口：

```bash
sudo ip link set can0 up type can bitrate 500000
```
可以通过回环测试，验证CAN0是否启动正常启动：

```bash
# 在终端一中运行
candump can0
```
```bash
# 打开另外一个终端，运行
cansend can0 123#1122334455667788
```
![](../images/08_硬件传感器/1eb062682268ccef195c8912e191721d.png)

## 2、Apollo配置文件修改

（1）创建新的车辆预设

```bash
cd application-core/profiles
cp -r sample my_vehicle       # 名字可以修改
```
（2）更新canbus配置文件

进入application-core/

```bash
profiles/my_vehicle/modules/canbus/conf目录，
```

![](../images/08_硬件传感器/a5d0e5614b3b5262eae39f42faf2fa3f.png)

打开“canbus_conf.pb.txt”文件，并修改车型代码：DKIT  

```protobuf
vehicle_parameter {
  brand: DKIT
  # 修改车型代码
  max_enable_fail_attempt: 5
  driving_mode: COMPLETE_AUTO_DRIVE
}
can_card_parameter {
  brand: SOCKET_CAN_RAW
  type: PCI_CARD
  channel_id: CHANNEL_ID_ZERO
  # 注意这个CAN通道ID
  interface: NATIVE
}
enable_debug_mode: false
enable_receiver_log: false
enable_sender_log: false
```
打开“canbus.conf”文件，并修改第4-5行：

```
--flagfile=modules/common/data/global_flagfile.txt
--canbus_conf_file=modules/canbus/conf/canbus_conf.pb.txt
--load_vehicle_library=modules/canbus_vehicle/ch/libapollo_canbus_vehicle_devkit.so
--load_vehicle_class_name=DevkitVehicleFactory
--enable_chassis_detail_pub
--enable_chassis_detail_sender_pub
--chassis_debug_mode=true
--pad_msg_delay_interval=3
--control_period=0.01
--max_control_miss_num=10
--guardian_period=0.01
--max_guardian_miss_num=10
--use_control_cmd_check=true
--use_guardian_cmd_check=true
--noreceive_guardian
--estop_brake=30.0
```
## 3、CANBUS模块测试

Apollo的CANBUS模块会与线控底盘之间进行通讯，所以在启动CANBUS模块后，应该能够看到Apollo端向CAN接口。

（1）进入到apollo容器中

```bash
cd application-core/ &&
aem start
```
![](../images/08_硬件传感器/d280cebe4ee25aacc6e2f23b1a390c6b.png)

（2）运行DreamView+界面

![](../images/08_硬件传感器/d8b3ad16a83f1c77fad7335820b92d4a.png)

(3)选择车辆预设

![](../images/08_硬件传感器/91f89d72f4159ff7918d702ea28d6193.png)

(4)运行CANBUS模块：

![](../images/08_硬件传感器/9e7176640f7ce4464e94bcf4b22a484f.png)

(5)使用cyber_monitor工具查看/apollo/canbus/chassis通道以及/apollo/canbus_detail_sender数据

![](../images/08_硬件传感器/33b7ae174ecc57db98ca090a6e3b0709.png)

## 4、数据的接收及解析测试

使用终端命令向can0发送报文数据，测试can报文是否被Apollo系统接受并解析：

（1）在启动CANBUS模块的情况下，打开终端，输入以下指令：

```bash
cansend can0 500#01000000C8000000
```
（2）使用cyber_monitor工具查看/apollo/canbus/chassis_detail通道

![](../images/08_硬件传感器/dbe349387edc5482c5b34995ca50181c.png)

可以看到发送到CAN0的数据被正确解析，并且显示在通道中。

# 四、后续工作

线控底盘CAN协议适配完成后，后续还有一些可选的调优工作需要进行，可以根据实际需要去完成：

根据接口测试验证表格，检查CAN通讯逻辑
暂时无法在飞书文档外展示此内容

通过自动控制测评工具，标定底盘控制参数
暂时无法在飞书文档外展示此内容

**动力学标定：**[https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEAhAOziAgjMmqAHKoXiJAFjkAROQDEdisQE59YpepE6AbMoAM49eenKJ4i0p1bzFREHXXUAM1D5AgBGAmEwh2cEaPZhdXZ2KWSwhFIHUnkAZkxQClQgA](https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEAhAOziAgjMmqAHKoXiJAFjkAROQDEdisQE59YpepE6AbMoAM49eenKJ4i0p1bzFREHXXUAM1D5AgBGAmEwh2cEaPZhdXZ2KWSwhFIHUnkAZkxQClQgA)

**底盘标定：**[https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEAhAOziAgjMmqAHKoXiJAFjkAROQDEdisQE59YpepE6AbMoAM49eenKJ4i0p1bzFT5nAEZ1ADNwgGZ5KMjhC2FdYVILbAdsYRD2eRyLcOcEZ1JMUApUIA](https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEAhAOziAgjMmqAHKoXiJAFjkAROQDEdisQE59YpepE6AbMoAM49eenKJ4i0p1bzFT5nAEZ1ADNwgGZ5KMjhC2FdYVILbAdsYRD2eRyLcOcEZ1JMUApUIA)
