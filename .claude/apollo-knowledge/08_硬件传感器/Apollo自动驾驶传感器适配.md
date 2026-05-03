# Apollo自动驾驶传感器适配

> 作者: 宇新 | 发布时间: 2025-05-23 11:39 | 链接: https://apollo.baidu.com/community/article/1296

> 本教程介绍了Apollo10.0系统传感器适配的方法，其中重点介绍如何实现摄像头和激光雷达传感器的适配，以及摄像头和激光雷达的标定方法。

Apollo的官方文件中也提供了传感器的集成适配流程，可以跟此教程对照学习，对Apollo传感器适配有更全面的了解。地址：[https://apollo.baidu.com/community/Apollo-Homepage-Document](https://apollo.baidu.com/community/Apollo-Homepage-Document)

# 一、传感器适配思路

![](../images/08_硬件传感器/2f1d09551cb6e472bcd5336350a8947e.png)

汽车智驾系统的传感器一般包括IMU、GPS接收器、毫米波雷达、激光雷达、摄像头。本教程会基于Apollo实车赛的场景需求，以参考套件硬件平台为对象，介绍Apollo自动驾驶驾驶平台传感器适配方法。在第一节中，会通过硬件选型、电气适配、软件适配三个模块来简略讲解在Apollo中适配传感器的思路。

## 1、电气适配

除了驱动层外，还需要注意传感器的电气适配问题，包括是设备的供电、接口、通讯方式等。虽然电气的适配一般都比较简单，但仍然需要重点关注，否则容易对设备产生不可逆的损伤甚至危及生命安全。

**电压匹配**
需要检查小车输出电压是否与传感器相匹配，例如小车电池包输出电压为48V，而一般传感器输入电压为12V或者9-36V（宽压），这时就需要使用降压稳压模块将电压变换为12V，否则连接后会烧坏传感器设备。

**功率匹配**
在设备上机前需要考虑整机的功耗与供电之间的关系，例如小车的电池额定输出功率为150W，整车的用电设备额定功率为200W，这时可能设备可以正常开机，但到自动驾驶的高功耗场景，设备会突然关机甚至烧融电线或DC模块。

**接口匹配**
有些传感器的接口采用航空接口、或者车规级接口，需要特别注意接口的定义，防止错接导致设备烧坏。

**通讯接口匹配**
需要注意传感器的通讯接口是否匹配，例如一些毫米波雷达默认采用250K速率，而小车采用500K速率；或者传感器与底盘CAN协议出现相同的帧ID，需要考虑使用多通道CAN以规避数据冲突；而像是一些使用TCP或者UDP通讯的设备，需要特别注意设备IP和网段的匹配。

## 2、硬件选型

如果已经完成了apollo的安装，查看

`application-core/core/cyberfile.xml` 文件中的 drivers 部分，可以看到一些驱动的程序包文件。包括摄像头（camera）、CAN总线（canbus）、组合导航系统（gnss）、激光雷达（lidar）、毫米波雷达（radar）等。

```xml
<!--  drivers  -->
<depend repo_name="drivers-camera" type="binary">drivers-camera</depend>
<depend repo_name="drivers-canbus" type="binary">drivers-canbus</depend>
<depend repo_name="drivers-gnss" type="binary">drivers-gnss</depend>
<depend repo_name="drivers-image-decompress" type="binary">drivers-image-decompress</depend>
<depend repo_name="drivers-lidar" type="binary">drivers-lidar</depend>
<depend repo_name="drivers-radar" type="binary">drivers-radar</depend>
```
以激光雷达为例，我们通过buildtool工具下载driver-lidar驱动包，新建终端，进入apollo容器，并下载driver-lidar驱动包：

```bash
# 进入apollo项目目录
cd application-core
# 进入apollo容器
aem start
# 下载代码
buildtool install drivers-lidar
```
下载完成后，可以看到application-core/modules/drivers目录下出现了lidar程序包：

![](../images/08_硬件传感器/0c3619b3f4c2bfb311e018efd5bba650.png)

需要知道Apollo具体支持的设备硬件，还需要点击进入到驱动包内，例如我们进入到“lidar”目录下：

**Directory Structure  目录结构**

```bash
modules/drivers/lidar/
|-- BUILD
|-- README.md
|-- common
|-- compensator
|-- conf             // 配置文件
|-- cyberfile.xml
|-- dag              // dag文件
|-- fusion
|-- hslidar          // 禾赛驱动包
|-- launch
|-- lidar_driver_component.cc
|-- lidar_driver_component.h
|-- livox             // 览沃驱动包
|-- lslidar           // 镭神驱动包
|-- proto 
|-- rslidar           // 速腾驱动包
|-- seyond            // 图达通驱动包
|-- vanjeelidar       // 万集驱动包
`-- velodyne          // 威力登驱动包
```
可以看到Apollo已经提供了多种激光雷达的驱动包，具体的硬件型号可以进入到各个厂商驱动包中查看REAME.md文件进行查阅，这里就不再赘述。硬件选型的时候可以通过查看驱动包来了解哪些硬件是可以直接适配的，如果自己的硬件不在已提供的驱动包内，可能需要自行编写程序进行适配。

## 3、配置参数及启动路径

不同的传感器有不同的参数及路径。在Apollo中，一般参数配置是通过conf文件夹中的xx_config.pb.txt文件进行配置，而启动路径一般是在dag文件夹中的xxx.dag中进行配置。在下面的课程中会详细进行讲解。

![](../images/08_硬件传感器/1e6783868bca3f1c4464199be4215334.png)

此外，如果想要通过DreamView界面来正确启动传感器，还需要稍微修改一下DreamView界面的调用路径，在下面的课程中也会进行讲解。

![](../images/08_硬件传感器/53f81b578d5f9eb9f4f17112c4c11c8e.png)

# 三、摄像头适配

相机作为自动驾驶感知设备，具有非常出色的成像能力，其成像原理为光线通过镜头或者镜头组进入相机，相机成像元件将进入的环境光线转化为数字信号，数字信号通过影像运算芯片储存在存储设备中。相机数据相比较激光雷达点云数据是稠密的，有颜色信息的，在自动驾驶中应用广泛，比如红绿灯的识别、车道线的识别等等。

**1、摄像头硬件连接**

参考套件使用的是USB摄像头，相机应水平安装，俯仰角向下 0-2 度（向下倾斜小于 2 度，不能上仰），翻滚角误差 ±1 度（左右两侧的平齐程度），航向角误差 ±2 度，镜头保持清洁，避免影响图像采集。

电气连接方面，直接将摄像头的USB接口连接到工控机即可。

![](../images/08_硬件传感器/10448989d3a63c1068c6810ba7db029c.png)

**2、摄像头数据检测**

摄像头连接到了Linux系统后，会自动生成视频设备文件/dev/video，当只有一个视频设备的时候，其名称为/dev/video0。（部分版本的Ubuntu系统接入一个视频设备后分别会生成/dev/video0和/dev/video1两个文件）

查看视频设备文件：

```bash
ll /dev/video*
```
![](../images/08_硬件传感器/915d4ac2876245dce9c1e96c00fa2451.png)

也可以使用Ubuntu内置的一些拍照软件（例如：茄子），检测摄像头是否可以正常输出数据。

![](../images/08_硬件传感器/f4698a2f7128349d2cf53d89066a6d70.png)

**3、摄像头驱动参数配置**

以sample车型的配置文件为例，查看sample车型文件中的摄像头的dag文件application-core/

```bash
profiles/samples/modules/drivers/camer/dag/camera.dag
```

```
# Define all coms in DAG streaming.
module_config {
    module_library : "modules/drivers/camera/libcamera_component.so"
    components {
      class_name : "CameraComponent"
      config {
        name : "camera_front_6mm"
        config_file_path : "modules/drivers/camera/conf/camera_front_6mm.pb.txt"
      }
    }
    components {
      class_name : "CameraComponent"
      config {
        name : "camera_front_12mm"
        config_file_path : "modules/drivers/camera/conf/camera_front_12mm.pb.txt"
      }
    }
}
```

可以看到这里调用了两个摄像头组件，一个是6mm，一个是12mm。我们参考套件中只使用了一个摄像头，可以将12mm的组件删除，整理后如下：

```
# Define all coms in DAG streaming.
module_config {
    module_library : "modules/drivers/camera/libcamera_component.so"
    components {
      class_name : "CameraComponent"
      config {
        name : "camera_front_6mm"
        config_file_path : "modules/drivers/camera/conf/camera_front_6mm.pb.txt"
      }
    }
}
```

根据DAG文件内容描述，6mm摄像头的配置文件为

```bash
modules/drivers/camera/conf/camera_front_6mm.pb.txt
```

我们再确认一下参数配置文件的内容：

```
camera_dev: "/dev/video4"
frame_id: "camera_front_6mm"
pixel_format: "yuyv"
io_method: IO_METHOD_MMAP
width: 1920
height: 1080
frame_rate: 10
monochrome: false
brightness: -1
contrast: -1
saturation: -1
sharpness: -1
gain: -1
auto_focus: false
focus: -1
auto_exposure: true
exposure: 100
auto_white_balance: true
white_balance: 4000
bytes_per_pixel: 2
trigger_internal: 0
trigger_fps: 15
raw_channel_name: "/apollo/sensor/camera/front_6mm/raw/image"
channel_name: "/apollo/sensor/camera/front_6mm/image"
...
```
发现摄像头的设备文件默认为/dev/video4，我们对应实际设备的文件名修改为/dev/video0。

**5、验证相机模块**

进入Apollo容器并启动DreamView plus：

```bash
# 进入apollo项目目录
cd application-core
# 进入apollo容器
aem start
# 启动DV
aem bootstrap start --plus
```
进入DreamView后，先加载sample预设，再启动摄像头：

![](../images/08_硬件传感器/f46ba4852136cf10a3d224f048b2629f.png)
![](../images/08_硬件传感器/a276b2f50f0f271ff2e55cd395661b3a.png)

可以使用cyber_monitor工具查看摄像头的数据通道数据被激活：

```bash
cyber_monitor
```
![](../images/08_硬件传感器/28a53ee20dd6c657cf0d78689fe77fb6.png)

也可以在DreamView界面中添加一个**相机视图**面板来查看摄像头。

![](../images/08_硬件传感器/9664d0802d5c7081218bbc6f1723e9aa.png)

# 三、激光雷达适配

**1、激光雷达硬件连接**

激光雷达要牢靠固定安装在车顶部，水平放置：

**电源连接**
参考套件的激光雷达支持12-32V宽压输入，额定工作电压为24V：

![](../images/08_硬件传感器/8adb943e66c73658e6c009acdeaf44ca.png)

电源连接前需要使用万用表测量电压以及DC的正负极是否与传感器接口对应！

![](../images/08_硬件传感器/1d1d6a3b7e627ec16b6ad183ad2af7b0.png)

**通讯线束连接**
激光雷达的数据通过网线传输，可以将激光雷达的网线与工控主机连接：

![](../images/08_硬件传感器/1704805a59f4873d714a9d3c26c581b7.png)

首次拿到激光雷达时，由于激光雷达默认的IP可能与自动驾驶设备网络不匹配，需要使用官方上位机软件进行设置。部分上位机软件只支持Windows系统，所以先将网线连接到Windows电脑上。

**2、激光雷达设备参数设置**

参考套件激光雷达上位机为VanJeeView720.exe，双击打开软件：

![](https://p9xmuijapd.feishu.cn/space/api/box/stream/download/asynccode/?code=MDNhZGMzY2FlNWZlMjM5NGFjYmQwNmU3NmI4ZWI5MzJfS25OT2Q4YWx3eU93NkFuVXFScFllTmdLemVjeWhDSWpfVG9rZW46RVJ0OWJlbERQb2dlWld4ZkMzVmNHNVJmbkNiXzE3NDY2ODk5OTA6MTc0NjY5MzU5MF9WNA)

初次使用前需要确保电脑IP为192.168.2.88，并且软件没有被防火墙或第三方安全软件禁止，如下图示，将域网络、专用网络都勾选，点击允许访问，确保软件能正常使用：

![](../images/08_硬件传感器/e2f506a16265a57c735b467a7bf48369.png)

![](../images/08_硬件传感器/0bf19cc4067ca29074fd6eebd90d6062.png)

选择设备，点击WLR-720:

![](../images/08_硬件传感器/883072242874dd7deab514e8f7474fbe.png)
![](../images/08_硬件传感器/c7cac878c29373e82db0803b2f06b705.png)

点击“操作”->“配置”，可以看到激光雷达的网络参数：

![](../images/08_硬件传感器/718aaf49160c53240ef133b3c1db2363.png)

如果自动驾驶主机的网段有特殊要求，例如一定要在192.168.0.1网段，可以参考下图进行修改，需要注意修改网关以及目标IP，并且自动驾驶主机的IP要与目标IP一致才能接收到点云数据。点击“设置”即可应用设置。

![](../images/08_硬件传感器/4f6aad2ece361ccc5519394fb41f8113.png)

**3、激光雷达数据检测**

将激光雷达连接回自动驾驶主机上，检查是否接收到激光雷达数据。首先要检查一下电脑的网络配置是否正确：

![](../images/08_硬件传感器/36ea2326900ee4472d7324e65d05e1cc.png)

检测无误后，可以使用抓包工具查看激光雷达的数据包：

```bash
sudo tcpdump
```
![](../images/08_硬件传感器/ac9fa82a30182f69a6a0b1bce8e7342c.png)

当看到类似上图的数据包，代表激光雷达的点云数据包发送给自动驾驶主机了。

**4、激光雷达驱动参数配置**

以sample车型的配置文件为例，查看sample车型文件中lidar的dag文件：

`application-core/profiles/sample/modules/drivers/lidar/dag/lidar.dag`

```
# Define all coms in DAG streaming.
module_config {
    module_library : "modules/drivers/lidar/rslidar/librslidar_component.so"
    ##################################################
    #                   drivers                      #
    ##################################################
    components {
      class_name : "RslidarComponent"
      config {
        name : "robo_back_driver"
        config_file_path : "modules/drivers/lidar/conf/lidar16_back.conf"
      }
    }
}
###################################################
#                   fusion                        #
###################################################
module_config {
    module_library : "modules/drivers/lidar/fusion/libfusion_component.so"
    components {
      class_name : "PriSecFusionComponent"
      config {
        name : "lidar_fusion"
        config_file_path : "modules/drivers/lidar/conf/fusion_conf.pb.txt"
        readers {channel: "/apollo/sensor/lidar16/back/PointCloud2"}
      }
    }
}
###################################################
#                compensation                     #
###################################################
module_config {
    module_library : "modules/drivers/lidar/compensator/libcompensator_component.so"
    components {
        class_name : "CompensatorComponent"
        config {
          name : "compensator"
          config_file_path : "modules/drivers/lidar/conf/compensator.pb.txt"
          readers {channel: "/apollo/sensor/lidar16/fusion/PointCloud2"}
        }
    }
}
```

可以看出默认调用激光雷达驱动是速腾的激光雷达，而我们参考套件使用的是万集的，需要进行修改。Apollo中已经提供了不同厂商传感器的默认配置文件，可以到对应传感器的驱动包中查看。例如：

`application-core/modules/drivers/lidar/vanjeelidar/dag/vanjeelidar.dag`

```
module_config {
    module_library : "modules/drivers/lidar/vanjeelidar/libvanjeelidar_component.so"
    components {
        class_name : "VanjeelidarComponent"
        config {
          name : "VanjeeLiDAR_Driver"
          config_file_path : "/apollo/modules/drivers/lidar/vanjeelidar/conf/vanjeelidar.pb.txt"
        }
    }
}
```

将 `vanjeelidar.dag` 中的文件替换到 `lidar.dag` 文件中即可。查看第7行的描述可知，配置文件直接调用了 `/apollo/modules/drivers/lidar/vanjeelidar/conf/vanjeelidar.pb.txt` 文件作为参数文件。我们可以打开这个文件看一下：

```
config_base {
  scan_channel: "/apollo/sensor/vanjeelidar/up/Scan"
  point_cloud_channel: "/apollo/sensor/vanjeelidar/up/PointCloud2"
  frame_id: "vanjeelidar_up"
  # sample ONLINE_LIDAR, RAW_PACKET
  source_type: ONLINE_LIDAR
}
# sample: "vanjee_720_16,vanjee_720_32"
model: "vanjee_720_16"
connect_type: 1
host_msop_port: 3001
lidar_msop_port: 3333
host_address: "192.168.2.88"
lidar_address: "192.168.2.86"
publish_mode: 2
start_angle: 0
end_angle: 360
min_distance: 0.3
max_distance: 120
use_lidar_clock: false
dense_points: false
wait_for_difop: true
config_from_file: false
angle_path: "/usr/local/include/vanjee_driver/param/Vanjee_720_16.csv"
```

主要关注13-16行的设备端口和IP，由于我在上面修改了IP，这里也对应进行修改：

```
host_msop_port: 3001
lidar_msop_port: 3333
host_address: "192.168.0.88"
lidar_address: "192.168.0.86"
```
**5、Apollo激光雷达启动**

![](../images/08_硬件传感器/f46ba4852136cf10a3d224f048b2629f.png)
![](../images/08_硬件传感器/7047fbf1db2a9f5cb9cdf5f5930839c7.png)

**6、验证激光雷达模块**

使用cyber_monitor工具检查激光雷达通道数据：

```bash
cyber_monitor
```
![](../images/08_硬件传感器/5fb7feb3ae8c96f4ea9fecb87d454ea2.png)

也可以创建激光雷达点云视图面板查看点云数据，不过在没有定位的状态下，DreamView中无法渲染出点云数据。为了解决这个问题，可以使用场景仿真工具提供虚拟的定位数据。

切换到PnC模式中，并选择Sim_Control：
![](../images/08_硬件传感器/b673edfd11d8db44780357aeb1f37238.png)

即可查看点云状态：

![](../images/08_硬件传感器/ad9eb11badb496a3a1c210c0d9c58021.png)

# 四、后续工作

**1、组合导航系统适配**

Apollo提供了室内外的导航定位方案：

室外自动驾驶：通过组合导航系统提供的GPS和IMU信息来判断车辆的方向、位置、速度信息。并且采集RTK（差分定位信号）和激光雷达点云绘制高精地图，确定道路的拓扑信息

室内自动驾驶：在室内无法通过GPS信息实现定位，Apollo提供通过激光点云和IMU数据构建SLAM地图的方式完成车辆定位。

**2、传感器外参标定**

传感器安装在实车后，需要对传感器的外参进行标定，包括车辆中心（组合导航）到激光雷达的坐标变换，激光雷达到摄像头的坐标变换。通过传感器标定确定了车辆和传感器之间的坐标关系，才能正确得感知环境以及障碍物，并规划出可行的运动轨迹。

激光雷达标定：https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEAhAOziAgjMmqAHKoXiJAFjkAROQDEdisQE59YpepE6AbMoAM49eenKJ4i0p1bzFT5SBGcARmEAZnkLUmx1MIAzRPlksKlSeVTQgk1hTQtMUApUIA

摄像头标定：https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEAhAOziAgjMmqAHKoXiJAFjkAROQDEdisQE59YpepE6AbMoAM49eenKJ4i0p1bzFT4AMwIAZil2MIdghzDhXVJ1XWFSMIBGdmFndWF2ZwdnYIRddMxQClQgA

**3、感知通道修改**

Apollo中默认使用激光雷达的补偿后的数据进行感知，如果没有使用组合导航授时功能，需要将激光雷达感知模块所用的通道修改为无补偿的通道。

查看感知的dag启动文件 `application-core/profiles/sample/modules/perception/pointcloud_preprocess/dag/pointcloud_preprocess.dag`：

```
module_config {
  module_library : "modules/perception/pointcloud_preprocess/libpointcloud_preprocess_component.so"
  components {
    class_name : "PointCloudPreprocessComponent"
    config {
      name : "PointCloudPreprocess"
      config_file_path : "/apollo/modules/perception/pointcloud_preprocess/conf/pointcloud_preprocess_config.pb.txt"
      flag_file_path: "/apollo/modules/perception/data/flag/perception_common.flag"
      readers {
        channel: "/apollo/sensor/lidar16/compensator/PointCloud2"  # 感知读取数据的通道需要与lidar的输出通道一致
      }
    }
  }
}
```

第十行的默认通道是补偿后的激光雷达通道，需要修改为实际激光雷达点云的通道，例如：`/apollo/sensor/vanjeelidar/up/PointCloud2`

此外，DreamView中默认启动的激光雷达感知是旧版的命令，需要做一些修改：

打开hmi配置文件 `application-core/profiles/sample/modules/dreamview_plus/conf/hmi_modes/vehicle_test.pb.txt`

**默认的启动代码：**

```
modules {
  key: "LidarPerception"
  value: {
    start_command: "nohup cyber_launch start modules/perception/launch/perception_lidar.launch &"
    stop_command: "nohup cyber_launch stop modules/perception/launch/perception_lidar.launch &"
    process_monitor_config {
      command_keywords: "start modules/perception/launch/perception_lidar.launch"
    }
    required_for_safety: false
  }
}
```

**修改为以下的启动代码：**

```
cyber_modules {
  key: "LidarPerception"
  value: {
    dag_files: "/apollo/modules/perception/msg_adapter/dag/msg_adapter.dag"
    dag_files: "/apollo/modules/perception/pointcloud_preprocess/dag/pointcloud_preprocess.dag"
    dag_files: "/apollo/modules/perception/pointcloud_map_based_roi/dag/pointcloud_map_based_roi.dag"
    dag_files: "/apollo/modules/perception/pointcloud_ground_detection/dag/pointcloud_ground_detection.dag"
    dag_files: "/apollo/modules/perception/lidar_detection/dag/lidar_detection.dag"
    dag_files: "/apollo/modules/perception/lidar_segmentation/dag/lidar_segmentation.dag"
    dag_files: "/apollo/modules/perception/lidar_detection_filter/dag/lidar_detection_filter.dag"
    dag_files: "/apollo/modules/perception/lidar_tracking/dag/lidar_tracking.dag"
    required_for_safety: false
    process_group: "perception"
  }
}
```

![](../images/08_硬件传感器/9f8d8193b92b562f748c7a503638a9c5.png)
