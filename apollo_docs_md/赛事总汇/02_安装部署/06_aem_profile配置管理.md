---
title: 06_aem_profile配置管理
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_202___xE5_xAE_x89_xE8_xA3_x85_xE9_x83_4860325cecb9f9bb3b81874d8e71683f.html
category: 赛事总汇 > 02_安装部署 > 06_aem_profile配置管理
---

# 06_aem_profile配置管理

# aem profile 配置管理

> 
> ‍作者: 宇新 | 发布时间: 2024-05-10 | 链接: https://apollo.baidu.com/community/article/1271 
> 

`aem profile` 用于管理 Apollo 的配置参数目录，支持多份配置切换，方便调参和快速回滚。

---

## 1. 同步配置参数到 profile 目录

将某个包的配置文件同步到 `profiles/<目录名>/` 下：

buildtool profile config init --package <包名> --profile=<目录名>
fragment

 示例：将 planning 包的全部配置文件同步到 `profiles/default/`：

buildtool profile config init --package planning --profile=default
fragment
> 
> ‍注意： 自己新增的 planning 插件无法通过该命令同步，需手动复制配置文件到对应 profile 目录。 
> 

如需对 planning 其他插件调参，包名请查阅：[Apollo Planning 文档](https://apollo.baidu.com/docs/apollo/latest/md_collection_2planning_2README__cn.html)

![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/f09cf205d2de436e378862443b474a9c.png)

---

## 2. 使配置参数生效

启用某份配置：

aem profile use default
fragment

 查看当前 `current` 软链指向：

ll profiles/current
fragment

 ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/44745f0189f0536cd411988b2bccb7a6.png)

> 
> ‍重点： 如果本地下载了源码，可能导致配置参数不生效。出现此问题时，重新执行 aem profile use default 即可。 
> 

## </blockquote>

## 3. 配置参数不生效排查

**步骤 1：** 检查 profile 软链是否正确：

ll profiles/current
fragment

 ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/44745f0189f0536cd411988b2bccb7a6.png)

**步骤 2：** 检查 Apollo 内部配置是否指向 `current`（以 `planning_component` 为例）：

ll /apollo/modules/planning/planning_component/conf/
fragment

 ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/7a9c908c9ccab7ef1ba2ea55ad1c8525.png)

**步骤 3：** 如果指向不对，重新执行：

aem profile use default
fragment

 查看源码目录：

ll modules/planning/planning_component/
fragment

 ![](https://apollo.baidu.com/docs/apollo/latest/docs/赛事总汇/images/02_安装部署/0fbf6a104f72a2507b4d793c34e83097.png)

---

## 4. 多份配置切换

在 `profiles/` 目录下可以维护多份配置，适用于不同场景或对比调参。

目录结构示例： 

profiles/
├── current -> demo_1   # 软链，指向当前启用的配置
├── default
├── demo_1
└── demo_2
fragment

 `demo_1` 内部结构示例（只需保留需要修改的配置文件，其余由系统自动读取模块默认配置）： 

demo_1/
└── modules
└── planning
└── planning_base
└── conf
├── planning.conf
├── planning_config.pb.txt
└── ……
fragment

 切换配置命令：

# 查看已有的 profile
aem profile list

# 切换到指定配置
aem profile use default
aem profile use demo_1
fragment
> 
> ‍优势：
> **场景适配**：不同赛事场景使用不同配置，快速切换
> **参数复用**：保留调优后的参数，避免重复工作
> **快速回滚**：随时切回之前认为最好的配置 
> 
> 

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。