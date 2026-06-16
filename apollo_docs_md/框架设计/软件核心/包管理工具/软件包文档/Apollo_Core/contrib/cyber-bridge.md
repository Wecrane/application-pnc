---
title: README
source: https://apollo.baidu.com/docs/apollo/latest/md_modules_2contrib_2cyber__bridge_2README.html
category: 框架设计 > 软件核心 > 包管理工具 > 软件包文档 > Apollo Core > contrib > cyber-bridge
---

# README

# cyber-bridge

## Introduction

This is bridge that exposes custom TCP socket for accepting and transmitting Cyber messages.

## Directory Structure

modules/contrib/cyber_bridge/
├── bridge.cc
├── BUILD
├── client.cc
├── client.h
├── clients.cc
├── clients.h
├── cyber-bridge.BUILD
├── cyberfile.xml
├── LICENSE
├── node.cc
├── node.h
├── README.md
├── server.cc
└── server.h
fragment

## Building

Run the build inside docker.

cd /apollo && bash apollo.sh build contrib                      # in source env
cd /apollo_workspace && buildtool build -p modules/contrib      # in package management env
fragment

## Running

cd /apollo && ./bazel-bin/module/contrib/cyber_bridge/cyber_bridge  # in source env
cyber_bridge                                                        # in package management env
fragment

For extra logging:

GLOG_v=4 GLOG_logtostderr=1 ./bazel-bin/modules/contrib/cyber_bridge/cyber_bridge # in source env
GLOG_v=4 GLOG_logtostderr=1 cyber_bridge                                          # in package management env
fragment

Add extra `-port 9090` argument for custom port (9090 is default).

## Example

In one terminal launch `cyber_bridge`:

cd /apollo && ./bazel-bin/module/contrib/cyber_bridge/cyber_bridge  # in source env
cyber_bridge                                                        # in package management env
fragment

In another terminal launch example talker:

cd /apollo && ./bazel-bin/cyber/python/cyber_py3/examples/talker    # in source env
talker                                                              # in package management env
fragment

In one more terminal launch example listener:

cd /apollo && ./bazel-bin/cyber/python/cyber_py3/examples/listener  # in source env
listener                                                            # in package management env
fragment

Now you should observe talker and listener sending & receiving message with incrementing integer.

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。