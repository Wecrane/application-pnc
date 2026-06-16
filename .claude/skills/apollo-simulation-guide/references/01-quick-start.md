# 01 - 快速启动：从零到仿真运行

> 来源：`apollo_docs_md/赛事总汇/02_安装部署/` 系列文档

---

## 系统要求

- **OS**: Ubuntu 18.04 / 20.04 / 22.04（**不推荐 24.04**）
- **配置**: 最低 4 核 16G 内存
- **存储**: 建议 55G 以上
- **GPU**: **不需要**（PnC 仿真纯 CPU）

---

## 安装流程（按顺序执行）

### 步骤 1：安装 Ubuntu

参考 [Ubuntu 官方安装指南](https://documentation.ubuntu.com/desktop/en/latest/tutorial/install-ubuntu-desktop/)。

如果使用 18.04，从 [清华镜像](https://mirrors.tuna.tsinghua.edu.cn/ubuntu-releases/) 下载 `desktop-amd64.iso`。

安装后更新：
```bash
sudo apt-get update
sudo apt-get upgrade
```

### 步骤 2：安装 Docker Engine

Apollo 依赖 Docker 19.03+。使用 Apollo 提供的脚本安装：

```bash
wget http://apollo-pkg-beta.bj.bcebos.com/docker_install.sh
bash docker_install.sh
```

如果下载失败或过慢，使用阿里云镜像：
```bash
sudo rm -f /etc/apt/sources.list.d/docker.list
wget http://apollo-pkg-beta.bj.bcebos.com/docker_install.sh
wget http://apollo-pkg-beta.bj.bcebos.com/get_docker.sh
bash get_docker.sh --mirror Aliyun
bash docker_install.sh
```

### 步骤 3：安装 aem 工具

aem（Apollo Environment Manager）是 Apollo 的容器管理工具。

```bash
wget https://apollo-pkg-beta.bj.bcebos.com/aem_install.sh
bash aem_install.sh
```

验证安装：
```bash
aem --version
```

### 步骤 4：拉取 PnC 工程

```bash
# 创建工作目录
mkdir -p ~/application-pnc && cd ~/application-pnc

# 拉取工程
aem init
```

### 步骤 5：启动容器并进入

```bash
aem start     # 启动 Apollo 容器
aem enter     # 进入容器
```

进入容器后，提示符应变为 `in-dev-docker` 形式，工作目录为 `/apollo_workspace`。

### 步骤 6：编译

```bash
buildtool build -p core -j15
```

**建议执行两次**，第一次可能因缓存未建立而部分失败。

### 步骤 7：设置 Profile

```bash
buildtool profile config init --package planning --profile=default
aem profile use default
```

### 步骤 8：启动 Dreamview+

```bash
aem bootstrap start --plus
```

在宿主机浏览器访问 `http://localhost:8888`。

### 步骤 9：下载地图

```bash
buildtool map get demo
```

### 步骤 10：开始仿真

1. Dreamview 中选择 **Mode Settings** → **PnC 开发调试**
2. 点击 **Resource Manager** 下载需要的场景
3. 选择地图、车辆模型
4. 点击仿真开始按钮

---

## 可选步骤

### 安装 VS Code 开发环境

```bash
# 宿主机安装 VS Code，然后在容器内：
code /apollo_workspace
```

### 使用编译缓存（网络差时加速）

参考 `apollo_docs_md/赛事总汇/02_安装部署/03_赛事编译缓存.md`

### 下载 Planning 源码

```bash
# 在容器内执行，下载全部 planning 源码
buildtool install planning*

# 或只下载指定子模块
buildtool install planning-traffic-rules-crosswalk
```

---

## 电脑重启后恢复工作

```bash
cd ~/application-pnc
aem start
aem enter
# 在容器内：
buildtool build -p modules/planning/ -j15
aem profile use default
aem bootstrap start --plus
```

---

## 打包提交

### 只改了配置
```bash
tar -zcvf 提交包.tar.gz profiles/default
```

### 改了源码
```bash
tar -zcvf 提交包.tar.gz modules/planning/ profiles/default
```

---

## 参考

- 完整安装指南：`apollo_docs_md/赛事总汇/02_安装部署/01_Apollo_EDU版本Pnc工程安装指南.md`
- 安装总览：`apollo_docs_md/赛事总汇/02_安装部署/00_安装总览.md`
