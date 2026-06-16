---
title: 02_安装VScode开发环境
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_202___xE5_xAE_x89_xE8_xA3_x85_xE9_x83_6f0ade757642042eb3aba49647aef20d.html
category: 赛事总汇 > 02_安装部署 > 02_安装VScode开发环境
---

# 02_安装VScode开发环境

# 2023CCF-BDCI-百度Apollo星火自动驾驶大赛搭建VScode开发环境

> 
> ‍作者: 宇新 | 发布时间: 2023-10-07 22:30 | 链接: https://apollo.baidu.com/community/article/1137 
> 

## 前言：

Visual Studio Code是一个轻量级但功能强大的源代码编辑器，可在桌面上运行，适用于Windows、macOS和Linux。支持广泛的编程语言，并且可以安装扩展来添加额外的功能。它的一个关键特点是其集成开发环境（IDE），包括代码补全、调试和 Git 集成等功能。它还具有内置终端，允许开发人员直接在编辑器中运行命令，已经成为最受欢迎的代码编辑器之一。本文将介绍在VScode中搭建Apollo的开发环境的方法。

## 使用前提：

> 
> ‍INFO 已经正确安装了Apollo alpha（软件包方式） 
> 

## 安装方法：

> 
> ‍INFO 搭建VScode开发环境，主要包含了以下四个步骤： 安装Vscode环境 安装Vscode扩展 打开Apollo工程 
> 

### 步骤一、安装VSCode

1、在VS Code官网[https://code.visualstudio.com/Download](https://code.visualstudio.com/Download) 中下载适用于ubuntu操作系统的vscode安装程序。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/01_赛事竞赛/ef6f8805901748eb0ce08064ff9a5b5c.png)

2、下载完成后可以在【主文件夹】下的【下载】目录中找到刚才下载的VScode安装文件。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/01_赛事竞赛/73c1e7565972bc03806fadc6759f9064.png)

3、鼠标双击刚才下载的文件，点击安装，等待安装完成即可。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/01_赛事竞赛/6542e28815fe08fcd4737597dcac666d.png)

如果无法双击安装，也可以在终端窗口中通过命令行的方式去安装，安装命令如下所示：

sudo dpkg -i  下载包的名字.deb
fragment
> 
> ‍DANGER 新手请不要直接复制该条指令，务必将下载包的名字.deb改成下载的包名称； 
> 

4、安装完成后，可以在终端窗口中执行以下命令，即可打开VScode。

code . 或者也可以在ubuntu界面左下角启动台找到Visual Studio Code，双击打开即可。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/01_赛事竞赛/accf97eae4a7678d3baa09b04bddb311.png)

### 步骤二、安装Vscode扩展

Vscode可以通过安装扩展来便于开发者阅读和编辑代码，可以通过在VS Code左侧菜单栏中选择“扩展”来打开扩展面板，在搜索框中输入要按照的扩展名，在搜索结果中找到相应的扩展，单击“安装”即可。本文将安装以下两个扩展。

> 
> ‍INFO Chinese (Simplified) (简体中文) ：将vscode界面设置为中文 C/C++：用于编译C++代码的扩展。 
> 

1、安装Chinese (Simplified) (简体中文) 扩展，安装完成后，重新启动VScode，即可支持中文。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/01_赛事竞赛/b4df5e18f027d2e8876c792f3eafc4b1.png)

2、安装C/C++扩展。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/01_赛事竞赛/1ef907661e707bf15a08f13ba8085731.png)

当然也可以根据自己的喜好安装其他的扩展，这里不在赘述。

### 步骤三、打开Apollo工程

1、在VScode菜单栏中，选择【文件】> 【打开文件夹】选项，选择打开Apollo代码所在的文件夹，如下图所示，application-pnc为Apollo alpha的工程目录。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/01_赛事竞赛/08734aa901ebf81b93eea2ed350e3f6a.png)

2、在VScode菜单栏中，选择【终端】> 【新建终端】选项，如下图所示，屏幕的右下角区域即刻出现终端窗口，在该窗口中，我们可以执行对Apollo的操作，例如，执行planing模块的编译任务。

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/01_赛事竞赛/da3eb2820dfca4d2c8a169c24b1df851.png)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。