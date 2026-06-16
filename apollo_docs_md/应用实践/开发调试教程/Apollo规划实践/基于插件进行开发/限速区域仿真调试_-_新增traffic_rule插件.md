---
title: 限速区域仿真调试 - 新增traffic rule插件
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE5_xBA_x94_xE7_x94_xA8_xE5_xAE_x9E_xE8_xB7_xB5_2_xE5_xBC_x80_xE5_x8F_x91_xE8_xB0_x83_8aec81ad38e956f26eddb80c70f045b6.html
category: 应用实践 > 开发调试教程 > Apollo规划实践 > 基于插件进行开发 > 限速区域仿真调试 - 新增traffic rule插件
---

# 限速区域仿真调试 - 新增traffic rule插件

## 实践内容

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_e4e2b74.png)

以车辆在行进过程中，驶入需限速的交汇路口为例，向大家介绍 Planning 模块插件机制，以及针对该交汇路口限速场景，教大家新建自定义插件，实现在指定区域的限速功能。新建一个 traffic rule 插件流程如下图所示，主要包括了配置插件相关文件、在`traffic_rule_config.pb.txt`中加入新增插件。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_4266c80.png)

Apollo9.0 中通过交通规则插件配置可以配置 planning 启用的交通规则插件，交通规则插件的所有业务逻辑对于所有场景均会生效，本实验将通过添加 traffic rule 插件，实现自动驾驶车辆在行驶到交汇路口限速区域时低于指定速度行驶。

## 实践目的

学习 traffic rule 新增插件流程，掌握开发 traffic rule 插件步骤，使学员具有新建自定义插件能力。在本实验中，你将学会：

- a. 指定位置新建插件`region_speed_limit`文件夹。
- b. 根据要求，配置相应插件文件与`traffic_rule_config.pb.txt`。
配置RegionSpeedLimit类代码文件以及相应 BUILD 文件，
配置参数文件以及 BUILD 文件，
配置插件参数文件default_conf.pb.txt，
配置cyberfile.xml，
配置plugins.xml，
traffic_rule_config.pb.txt中加入新增插件。

- c. 编译`region_speed_limit`插件。
- d. 运行车辆规划模块，并在 dreamview+ 上观察车辆驶入交汇路口的速度变化。

## 启动仿真环境

1. 在终端中输入以下指令，启动 dreamview+。
aem bootstrap start --plus
 fragment 

1. 运行成功后，点击左上角 dreamview+ 按钮。

1. 首次进入 Dreamview+ 界面，需要勾选 Accept the User Agreement and Privacy Policy**，点击 **Enter this mode 进入 Dreamview+。

1. 点击红框 Skip 跳过介绍。

1. 在 Mode/模式 中选择 PnC 模式。

1. 在 Operations/操作 中选择 **Sim Control/仿真**。

1. 在 Modules/模块 中打开 Planning 模块。

1. 在 Vehicle Visualization/车辆可视化 面板中点击 Routing Editing/路径编辑 进入路径编辑界面。

1. 点击左上角第一个红框选择车辆起始点，第二个红框选择途经点与终点。

1. 点击左下角红框开始仿真。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_8a83620.png)

1. 如图所示，红线为道路参考线，浅蓝色的轨迹是 planning 模块实时规划的轨迹。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_5760e6c.png)

## 全局配置实践实验流程

### 全局配置参数同步

输入全局配置参数同步指令，系统将自动将全局配置参数复制到 profile 的 default 目录中，然后就可以在 profile 目录上轻松修改配置参数。

buildtool profile config init --package planning --profile=default && aem profile use default
fragment

### 新建 traffic rule 插件实践

#### 新建插件文件

输入下述指令，在`modules/planning/traffic_rules/`下新建插件文件夹`region_speed_limit`：

buildtool create --template plugin \
--namespaces planning \
--includes "modules/common/status/status.h" \
"modules/planning/planning_base/traffic_rules_base/traffic_rule.h" \
"modules/planning/traffic_rules/region_speed_limit/proto/default_conf.pb.h" \
--dependencies planning:planning \
--build_dependencies "//modules/planning/planning_base:apollo_planning_planning_base" \
--base_class_name apollo::planning::TrafficRule \
--config_message_name RegionSpeedLimitConfig \
--config_file_name default_conf \
modules/planning/traffic_rules/region_speed_limit
fragment

输入下述指令修改配置文件名为`default_conf.pb.txt`：

mv modules/planning/traffic_rules/region_speed_limit/conf/region_speed_limit.pb.txt modules/planning/traffic_rules/region_speed_limit/conf/default_conf.pb.txt
fragment

#### 配置插件文件

1. 打开在线编辑器。
点击红框所示位置，打开在线编辑器，对插件文件进行编辑。

1. 编辑器配置。
从左到右依次按红框所示点击，选择/apollo_workspace/目录，点击 **OK**。

进入后下图红框所示文件夹即为新建插件模版。

region_speed_limit.h
region_speed_limit.h文件位置：

region_speed_limit.h文件内容：
#pragma once
 
 #include <memory>
 #include "cyber/plugin_manager/plugin_manager.h"
 
 /* 添加了相应的头文件*/
 #include "modules/common/status/status.h"
 #include "modules/planning/traffic_rules/region_speed_limit/proto/region_speed_limit.pb.h"
 #include "modules/planning/planning_base/traffic_rules_base/traffic_rule.h"
 
 namespace apollo {
 namespace planning {
 
 class RegionSpeedLimit : public TrafficRule {
     /* 声明成员函数*/
     public:
         bool Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) override;
         virtual ~RegionSpeedLimit() = default;
 
         common::Status ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info);
 
         void Reset() override {}
 
     private:
         RegionSpeedLimitConfig config_;
 };
 
 CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RegionSpeedLimit, apollo::planning::TrafficRule)
 
 }  // namespace planning
 }  // namespace apollo
 fragment region_speed_limit.cc
region_speed_limit.cc文件位置：

region_speed_limit.cc文件内容：
#include <memory>
 #include "modules/planning/traffic_rules/region_speed_limit/region_speed_limit.h"
 
 namespace apollo {
 namespace planning {
 
 /* 定义成员函数*/
 
 using apollo::common::Status;
 using apollo::hdmap::PathOverlap;
 
 bool RegionSpeedLimit::Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) {
     if (!TrafficRule::Init(name, injector)) {
         return false;
     }
     // Load the config this task.
     return TrafficRule::LoadConfig<RegionSpeedLimitConfig>(&config_);
 }
 
 Status RegionSpeedLimit::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
     ReferenceLine* reference_line = reference_line_info->mutable_reference_line();
     const std::vector<PathOverlap>& junction_overlaps
             = reference_line_info->reference_line().map_path().junction_overlaps();
     for (const auto& junction_overlap : junction_overlaps) {
         reference_line->AddSpeedLimit(
                 junction_overlap.start_s - config_.forward_buffer(),
                 junction_overlap.end_s + config_.backward_buffer(),
                 config_.limit_speed());
     }
     return Status::OK();
 }
 
 }  // namespace planning
 }  // namespace apollo
 fragment plugin_region_speed_limit_description.xml
plugin_region_speed_limit_description.xml文件位置：

plugin_region_speed_limit_description.xml文件内容
<library path="modules/planning/traffic_rules/region_speed_limit/libregion_speed_limit.so">
     <class type="apollo::planning::RegionSpeedLimit" base_class="apollo::planning::TrafficRule">    </class>
 </library>
 fragment cyberfile.xml
cyberfile.xml文件位置

cyberfile.xml文件内容
<package format="2">
   <name>region-speed-limit</name>
   <version>local</version>
   <description>
     This is a demo package
   </description>
 
   <maintainer email="sample@sample.com">Apollo Developer</maintainer>
   <license>Apache License 2.0</license>
   <url type="website">https://www.apollo.auto/</url>
   <url type="repository">https://github.com/ApolloAuto/apollo</url>
   <url type="bugtracker">https://github.com/ApolloAuto/apollo/issues</url>
 
   <type>module</type>
   <src_path>//modules/planning/traffic_rules/region_speed_limit</src_path>
   <builder>bazel</builder>
 
   <depend type="binary" repo_name="cyber">cyber</depend>
   <!-- add new dependency-->
   <depend type="binary" repo_name="planning">planning</depend>
 
   <depend>bazel-extend-tools</depend>
 </package>
 fragment BUILD
BUILD 文件位置

BUILD 文件内容
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")
 load("//tools:cpplint.bzl", "cpplint")
 
 package(default_visibility = ["//visibility:public"])
 
 filegroup(
     name = "region_speed_limit_files",
     srcs = glob([
         "conf/**",
     ]),
 )
 
 apollo_plugin(
     name = "libregion_speed_limit.so",
     srcs = [
         "region_speed_limit.cc",
     ],
     hdrs = [
         "region_speed_limit.h",
     ],
     description = ":plugin_region_speed_limit_description.xml",
     deps = [
         "//cyber",
         "//modules/planning/planning_base:apollo_planning_planning_base",
         "//modules/planning/traffic_rules/region_speed_limit/proto:region_speed_limit_cc_proto",
     ],
 )
 
 apollo_package()
 
 cpplint()
 fragment proto/BUILD
proto/BUILD 文件位置

proto/BUILD文件内容
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools/proto:proto.bzl", "proto_library")
 load("//tools:cpplint.bzl", "cpplint")
 
 package(default_visibility = ["//visibility:public"])
 
 proto_library(
     name = "region_speed_limit_proto",
     srcs = ["region_speed_limit.proto"],
 )
 
 apollo_package()
 
 cpplint()
 fragment proto/region_speed_limit.proto
proto/region_speed_limit.proto 文件位置

proto/region_speed_limit.proto 文件内容
syntax = "proto2";
 
 package apollo.planning;
 
 message RegionSpeedLimitConfig {
   // 声明RegionSpeedLimitConfig中的数据结构
   optional double forward_buffer = 1 [default = 3];
   optional double backward_buffer = 2 [default = 2];
   optional double limit_speed = 3 [default = 5];
 }
 fragment conf/default_conf.pb.txt
conf/default_conf.pb.txt文件位置

修改conf/default_conf.pb.txt文件内容如下：
forward_buffer: 3.0
backward_buffer: 2.0
limit_speed: 10.0
 fragment 
1. 配置插件流程文件。
配置文件位置

配置文件内容
rule {
  name: "BACKSIDE_VEHICLE"
  type: "BacksideVehicle"
}
rule {
  name: "CROSSWALK"
  type: "Crosswalk"
}
rule {
  name: "DESTINATION"
  type: "Destination"
}
rule {
  name: "KEEP_CLEAR"
  type: "KeepClear"
}
rule {
  name: "REFERENCE_LINE_END"
  type: "ReferenceLineEnd"
}
rule {
  name: "REROUTING"
  type: "Rerouting"
}
rule {
  name: "STOP_SIGN"
  type: "StopSign"
}
rule {
  name: "TRAFFIC_LIGHT"
  type: "TrafficLight"
}
rule {
  name: "YIELD_SIGN"
  type: "YieldSign"
}
rule {
  name: "REGION_SPEED_LIMIT"
  type: "RegionSpeedLimit"
}
 fragment 

#### 编译插件

输入如下命令进行编译：

buildtool build -p modules/planning/traffic_rules/region_speed_limit/
fragment

#### 重启 planning

点击 **Modules/模块** 中的 Planning 模块，关闭 Planning。而后再次点击 Planning 模块，打开 Planning。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_c59210d.png)

打开 **Routing Editing/路径编辑** 面板，设置起点终点：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_cdebe8b.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_c6a39be.png)

原始配置仿真：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_d24e1f6.png)

修改插件配置参数：

在`apollo_workspace`工作目录找到`modules/planning/traffic_rules/region_speed_limit/conf/default_conf.pb.txt`配置文件，调整经过路口速度。

1. 使用在线编辑工具修改modules/planning/traffic_rules/region_speed_limit/conf/default_conf.pb.txt配置文件，将limit_speed参数
limit_speed: 10.0
 fragment 将配置参数的值修改为：
limit_speed: 3.0
 fragment 修改之后保存代码，而后重新编译插件：
buildtool build -p modules/planning/traffic_rules/region_speed_limit/
 fragment 
1. 修改好代码参数并重新编译后，在 **Modules/模块** 中重启 Planning 模块（必须步骤）。
1. 重新调整插件参数，观察车辆减速效果与减速起止点。

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。