# 预测模块打不开 — 修复与固化方案

> 时间：2026-08-01 | 来源：`~/下载/edu下prediction打不开.pdf` + prediction 日志分析

---

## 一、问题根因

- 现象：DreamView 中 prediction 模块打不开（启动即退出）
- 日志：`data/log/prediction.log.INFO.*` 停在 `Evaluator [2] is registered`（mlp 模型加载成功）后无输出 → **在加载 torch 模型时崩溃**
- 根因：prediction 所有 torch 模型代码都调用 `torch::cuda::is_available()`（共 17 个文件 25 处）。无 GPU 环境下该调用初始化 CUDA context 导致**段错误**，进程直接退出（无 glog 报错）
- PDF 临时方案：`export CUDA_VISIBLE_DEVICES=""` 让 CUDA 对进程不可见 → `is_available()` 返回 false → 走 CPU → 正常

## 二、固化方案 A（已实施 ✅ 无需编译）

**`~/.bashrc` 追加 `export CUDA_VISIBLE_DEVICES=""`**（已备份 `.bashrc.bak_20260801`）

- 宿主机 `/home/skye/.bashrc`（aem worklocal 模式与容器共享）
- 用户 `aem enter` 后自动生效 → 启动 Dreamview → prediction 子进程继承环境变量
- **已验证**：容器内 `bash -lc 'echo $CUDA_VISIBLE_DEVICES'` 输出 `[]`（空）✅
- 立即生效，无需编译，重启终端/容器后仍生效

## 三、固化方案 B（可选，彻底解决，需编译）

**修改 prediction 代码，把 `torch::cuda::is_available()` 的 CUDA 分支改为强制 CPU**。

涉及 17 个文件 25 处：

| 类别 | 文件 | 处数 |
|------|------|:---:|
| torch 模型（8 个） | `evaluator/model_manager/model/{multi_agent,semantic_lstm}_{vehicle,pedestrian}_torch_{cpu,gpu}/xxx_torch_model.cc` | 各 2 处 |
| evaluator（9 个） | `evaluator/vehicle/{cruise_mlp,junction_mlp,lane_scanning,multi_agent,semantic_lstm,jointly_prediction_planning,vectornet,junction_map}_evaluator.cc` + `evaluator/pedestrian/pedestrian_interaction_evaluator.cc` | 各 1 处 |

改动方式：把 `if (torch::cuda::is_available()) { device = torch::Device(torch::kCUDA); }` 的 device 固定为 `torch::kCPU`（或删除 CUDA 分支）。

编译：`buildtool build -p modules/prediction/ -j15 && aem profile use default`

## 四、相关配置现状

- 工作区 `modules/prediction/conf/prediction_conf.pb.txt`：已是 **CPU 配置**（`backend: CPU` + `xxxCpuTorch` 模型），随 git 提交
- 容器内 `/apollo/modules/prediction/conf/prediction_conf.pb.txt` 软链指向 `/opt/apollo/neo/share/...`（官方包，也是 CPU 配置）
- **配置已是 CPU，问题在代码的 CUDA 初始化** → 方案 A/B 针对代码层

## 五、结论

- **方案 A（~/.bashrc）已固化生效**，日常使用 prediction 可正常打开
- 若需彻底摆脱环境变量依赖（换环境也能用），实施方案 B（需编译 prediction）
