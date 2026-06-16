---
title: README_cn
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2planning_2traffic__rules_2speed__setting_2README__cn.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > planning > planning-traffic-rules-speed-setting
---

# README_cn

# planning-traffic-rules-speed-setting

## 介绍

`planning-traffic-rules-speed-setting` 是planning模块中用来实时处理修改巡航速度命令的插件，在车辆使用当前设定的巡航速度行驶时，用户可以通过`apollo::external_command::SpeedCommand`来修改当前的巡航速度（参考external-command-speed包的说明）。

对修改巡航速度的命令处理在ApplyRule函数中，函数的处理步骤有：

- **接受处理命令的判断**：
只有在接收到新的SpeedCommand时才进行处理。
如果接收到的是导航命令，使用新的导航命令作为基础巡航速度。
如果没有收到SpeedCommand命令，使用上次调整过之后的巡航速度作为本周期的巡航速度
如果收到SpeedCommand命令解析失败，使用上次的巡航速度
如果收到恢复巡航速度的SpeedCommand，使用ReferenceLineInfo中原始设置的巡航速度
如果收到设置巡航速度大小的SpeedCommand，使用SpeedCommand中的速度作为本周期的巡航速度
如果收到设置巡航速度比例的SpeedCommand，将上次的巡航速度乘以比例值之后，设置新的巡航速度

## 目录结构

modules/planning/traffic_rules/speed_setting/
├── speed_setting
├── conf                    // 参数配置文件
├── docs                    // 说明文档引用内容
├── BUILD                   // 构建规则文件
├── cyberfile.xml           // 包管理配置文件
├── speed_setting.cc        // 设置速度插件源码
├── speed_setting.h         // 设置速度插件头文件
└── README_cn.md            // 说明文档
fragment

## 插件

### SpeedSetting

[apollo::planning::SpeedSetting](https://apollo.baidu.com/docs/apollo/latest/classapollo_1_1planning_1_1SpeedSetting.html)

#### 使用方式

##### 添加包依赖

如果使用包管理的方式运行apollo，在包管理的cyber.xml文件中添加对插件的依赖： 

<depend repo_name="planning-traffic-rules-speed-setting" type="binary">planning-traffic-rules-speed-setting</depend>
fragment

##### 插件加载配置

所有traffic rule的插件都是在planning包中加载运行的，如果需要支持SpeedCommand命令的处理，在配置文件modules/planning/planning_component/conf/traffic_rule_config.pb.txt中添加以下配置： 

rule {
name: "SPEED_SETTING"
type: "SpeedSetting"
}
fragment

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。