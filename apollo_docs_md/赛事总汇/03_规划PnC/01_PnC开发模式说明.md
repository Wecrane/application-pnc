---
title: 01_PnC开发模式说明
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_203___xE8_xA7_x84_xE5_x88_x92PnC_201__62eaf31bfd64c73fdfeb04421f9d845c.html
category: 赛事总汇 > 03_规划PnC > 01_PnC开发模式说明
---

# 01_PnC开发模式说明

# pnc开发模式说明

> 
> ‍作者: 宇新 | 发布时间: 2024-07-14 14:22 | 链接: https://apollo.baidu.com/community/article/1261 
> 

# 综述

在Apollo 9.0中，我们对规划模块进行了全面重构，引入了全新的插件扩展机制和分级参数配置机制，旨在提高系统的灵活性和开发效率。

> 
> ‍SUCCESS 为了保证各位开发者快速入门Apollo planning的开发模式，可先学习相关课程 【PnC基础(9.0版)】https://apollo.baidu.com/community/online-course/776 
> 

**planning模块综述：**

具体详见技术文档：https://apollo.baidu.com/docs/apollo/latest/md_collection_2planning_2README__cn.html **全新插件扩展机制**：我们将场景（scenario）、任务（task）和交通规则（traffic rules）进行插件化处理，使开发者能够更方便地开发和部署自己的插件。通过简化的配置流程，开发者可以轻松启动和运行这些插件。

具体详见技术文档：https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEAhAILiAnABZxEgOzK1Y%2BQA51M3ROUnJBsbK2WZoyUdkBhcXoAMhlwDFlARnUXZdzE9AGY%2BBBUEJQ1sHz1sKQRhdlcNdgAzaIA2LKUCUlcQggQNLNJSTFAKVCA **分级参数配置机制**：此机制明确区分了全局参数和局部参数。局部参数被嵌入到各自的插件中，实现独立管理，这极大地方便了开发者在进行参数查询和修改时的操作。

具体详见技术文档：https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFgoIH0adPgCY%2BADwCiAVnEAhAILiAnABZxEgOzK1Y%2BQA51M3ROUnJBsbK2WZoyUdkBhcXoAMhlwDFlARnUXZdzE9AGY%2BFRUCBCkQ1wQVYQIVEIA2JSUEXFJSFOwlADMVbAI9H3ypTFAKVCA 

# 目录结构

在Apollo 9.0中，我们设定了精心组织的工作目录结构以支持各种操作和开发需求，详细的目录结构如下：**工作目录（容器内显示/apollo_workspace）**

这个目录下包含了三个核心的子目录：

application-core
├── .aem
│   └── envroot
│       ├── apollo          # 会挂载到容器内的 /apollo 目录，【注】/apollo为配置文件生效目录
│       └── opt             # 会挂载到容器内的 /opt/ 目录，而 Apollo 的软件包会默认安装到 /opt/ 下，因此该目录可以起到缓存的作用
├── modules                 # 存放需要修改源码目录
└── profiles                # 新版配置目录
├── current -> default  # 当前启用的配置目录。修改完成后会在/apollo下生效
└── default             # 名为 default 的配置目录
fragment

 **.aem目录**：存放所有模块的源码。需要注意的是，此目录下的源码不支持修改，主要用于参考和系统运行。 **modules目录**：此目录用于存放可以修改的部分模块源码。开发者可以使用特定的命令从Apollo的软件库同步需要修改的源码到工作目录的modules目录，具体命令如下：。 

buildtool install <包名>
fragment

 **profiles目录**：这是一个新版配置参数目录，用于存放和管理配置参数。此目录结构如下：配置参数可以根据需要进行修改，修改后的配置将在/apollo目录下生效。 

├── current -> default  # 指向当前启用的配置目录，默认指向名为default的目录
├── 其他
└── default             # 默认配置目录，存放初始配置参数
fragment

# 全新插件扩展机制

在Apollo 9.0 中，planning base 为规划模块基座，负责接受和发布和其他模块的通信信息，生成参考线，提供基础数据结构和基础库的功能。双层状态机的第一层为 Scenario 插件，每个Scenario插件通常会包含其独有的Stage插件，每个 Stage 插件会从任务库中编排不同的任务(Task)来执行。

再此过程中，所有的场景（scenario）、任务（task）和交通规则（traffic rules）均被定义为插件，用户均可以使用自己所定制的插件以满足使用要求再或者通过修改Apollo planning 中自带的插件进行二次开发以满足要求；

## 一、自己定义场景（scenario）、任务（task）和交通规则（traffic rules）插件，以满足使用要求

具体可以查看技术文档&视频：

| 插件 | 文章名 | 文章链接 | 视频链接 | buildtool create指令使用方式 |
| --- | --- | --- | --- | --- |
| 交通规则（traffic rules） | 交汇路口减速慢行场景（traffic rules插件） | https://apollo.baidu.com/community/article/1253 | https://apollo.baidu.com/community/online-course/776 | https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_xA1_x86_xE6_x9E_xB6_xE8_xAE_xBE_xE8_xAE_xA1_2_xE5_x91_xBD_xE4_xBB_xA4_xE8_xA1_x8C_4c106ac967ec9830c44df172820112f9.html |
| 任务（task） | 交汇路口减速慢行场景（task插件） | https://apollo.baidu.com/community/article/1254 |  |  |
| 场景（scenario） | 左转待转场景 | https://apollo.baidu.com/community/article/1246 |  |  |

## 二、修改Apollo planning 中自带的插件进行二次开发以满足要求

使用指令

buildtool install <包名>
fragment

 例如：修改交通规则（traffic rules）的crosswalk插件

使用指令将（traffic rules）的crosswalk插件同步至工作目录下的modules目录

buildtool install planning-traffic-rules-crosswalk
fragment
> 
> ‍INFO 如需二次开发planning其他插件，相关包名请查阅相关planning综述文档：https://apollo.baidu.com/docs/apollo/latest/md_collection_2planning_2README__cn.html 
> 

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/04_规划PnC/fa8fab2bb06e45aafd0f7e93be324e69.png)

# 分级参数配置机制

## **配置参数调整的具体措施**：

参数被划分为全局变量和局部变量。全局变量用于多个算法或插件之间共享；局部变量则专属于特定的算法或插件。若需调整某个插件的参数，用户可直接在该插件的目录下进行查找和修改。 我们添加了一份涵盖常用功能参数的说明文档，以便用户在调试时能快速查找所需信息。 此外，我们引入了一个名为“profile”的配置参数插件，用户可以在此插件内放置配置参数。所有配置文件均存放于“profiles”目录下，用户可以在此目录中定义一份或多份配置文件，并可通过aem工具进行查看和管理。

**配置参数同步方式**

配置文件的管理和同步通过以下命令进行：

buildtool profile config init --package <包名> --profile=目录
fragment
# 例，将planning的全局配置参数同步至default目录
buildtool profile config init --package planning --profile=default
fragment

 使用default目录的配置参数

aem profile use default
fragment

 使用指令：ll profiles ，查看current的指向

├── current -> default  # 指向当前启用的配置目录，默认指向名为default的目录
fragment

 其他目录配置参数

aem profile use 其他
fragment

 使用指令：ll profiles ，查看current的指向

├── current -> 其他  # 指向当前启用的配置目录，默认指向名为其他的目录
fragment
> 
> ‍INFO 如需对planning其他插件进行参数调整，相关包名请查阅相关planning综述文档：https://apollo.baidu.com/docs/apollo/latest/md_collection_2planning_2README__cn.html 
> 

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/04_规划PnC/5e4a7241cc4f184af1a3e0cfc5b10792.png)

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。