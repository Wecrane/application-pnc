# 03 限速插件 `scene_speed_limit` 全解析

> 来源：`shiche/hqev_speed_limit.tar.gz`（主办方提供）→ 解压于 `shiche/_extracted/`
> 这是实车赛**只调参完赛**的关键依据：把仿真赛需要改代码才能实现的赛题 7/8 限速变成纯配置。

---

## 1. 插件是什么

- 一个 **Apollo TrafficRule 插件**（`libscene_speed_limit.so`），包名 `scene-speed-limit`
- 作用：按配置对三类**地图元素**（人行道 crosswalk、信号灯 traffic_light、减速带 speed_bump）施加**区段限速**
- 与内置规则关系：**只限速，不干预停车/避让决策**（那些由 crosswalk/traffic_light/stop_sign 内置规则管）→ 可共存

## 2. 文件结构

```
modules/planning/traffic_rules/scene_speed_limit/
├── scene_speed_limit.h          # 类声明，CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN
├── scene_speed_limit.cc         # ApplyRule 实现（核心逻辑）
├── conf/default_conf.pb.txt     # 运行时配置（主办方打包：三组 enabled 全 false）
├── proto/scene_speed_limit.proto # SceneSpeedLimitConfig 消息
├── plugin_scene_speed_limit_description.xml  # 插件描述
├── cyberfile.xml                # 包描述（依赖 cyber/planning-interface-base）
└── BUILD                        # apollo_plugin 构建
```

## 3. 核心逻辑（scene_speed_limit.cc）

```cpp
Status SceneSpeedLimit::ApplyRule(Frame* frame, ReferenceLineInfo* reference_line_info) {
  ReferenceLine* reference_line = reference_line_info->mutable_reference_line();
  const auto& map_path = reference_line_info->reference_line().map_path();

  if (config_.crosswalk_enabled()) {
    for (const auto& overlap : map_path.crosswalk_overlaps()) {
      reference_line->AddSpeedLimit(
          overlap.start_s - config_.crosswalk_forward_buffer(),
          overlap.end_s + config_.crosswalk_backward_buffer(),
          config_.crosswalk_limit_speed());
    }
  }
  // traffic_light / speed_bump 同构 ...
}
```

**机制**：
- 每个规划周期执行一次
- 从 `map_path` 取三类 overlap（地图元素在参考线上的投影区间）
- `AddSpeedLimit(start_s - 前缓冲, end_s + 后缓冲, 限速值)` → 参考线限速
- 下游 `SpeedLimitDecider` 取**所有限速的最小值** → 与减速带 gflag、地图限速等并存时按最小生效

## 4. 配置字段（proto 默认值）

| 字段 | 默认值 | 含义 |
|------|:---:|------|
| `crosswalk_enabled` | true | 人行道限速开关 |
| `crosswalk_forward_buffer` | 5.0 | 人行道前缓冲 m（提前限速） |
| `crosswalk_backward_buffer` | 2.0 | 人行道后缓冲 m |
| `crosswalk_limit_speed` | 3.0 | 人行道限速 m/s |
| `traffic_light_enabled` | true | 信号灯限速开关 |
| `traffic_light_forward_buffer` | 8.0 | 信号灯前缓冲 m |
| `traffic_light_backward_buffer` | 2.0 | 信号灯后缓冲 m |
| `traffic_light_limit_speed` | 4.0 | 信号灯限速 m/s |
| `speed_bump_enabled` | true | 减速带限速开关 |
| `speed_bump_forward_buffer` | 3.0 | 减速带前缓冲 m |
| `speed_bump_backward_buffer` | 1.0 | 减速带后缓冲 m |
| `speed_bump_limit_speed` | 2.0 | 减速带限速 m/s |

**主办方默认配置**（default_conf.pb.txt）：
```text
crosswalk_enabled: false      # ← 未启用
crosswalk_forward_buffer: 1.0
crosswalk_backward_buffer: 1.0
crosswalk_limit_speed: 1.0
traffic_light_enabled: false  # ← 未启用
traffic_light_forward_buffer: 1.0
traffic_light_backward_buffer: 1.0
traffic_light_limit_speed: 1.0
speed_bump_enabled: false     # ← 未启用
speed_bump_forward_buffer: 1.0
speed_bump_backward_buffer: 1.0
speed_bump_limit_speed: 1.0
```
> ⚠️ 官方默认三组 **enabled=false**，需按赛题启用并配置限速值。

## 5. 与赛题的关系（实车方案）

| 赛题 | 要求 | 插件字段 | 建议值 | 说明 |
|------|------|---------|--------|------|
| 1 减速带 | ≤1.5 m/s（实车）/ ≤3（仿真） | `speed_bump_enabled=true`, `speed_bump_limit_speed` | 1.2~1.4 或 2.5~2.7 | 按现场手册标准取，留裕量 |
| 7 人行道路口减速 | ≤1.5 m/s | `crosswalk_enabled=true`, `crosswalk_limit_speed` | **1.2~1.4** | 仿真取值 4.5 是 ≤5 场景；实车 ≤1.5 应取 1.2~1.4 |
| 8 交通灯路口减速 | ≤1.5 m/s | `traffic_light_enabled=true`, `traffic_light_limit_speed` | **1.2~1.4** | 同上 |

**裕量原则**：限速值 = 赛题上限 − 0.1~0.3（控制超调）。实车控制超调比仿真大，裕量应更足。

## 6. 部署与生效

1. 安装插件包（把 `scene_speed_limit` 目录放到 `modules/planning/traffic_rules/` 下，按官方插件安装流程构建注册）
2. 注册到 TrafficRule 配置：`traffic_rule_config.pb.txt` 添加 `scene_speed_limit` 条目
3. 配置 `conf/default_conf.pb.txt`（或 profile 覆盖层）设置 enabled=true + 限速值
4. 生效：**重启 Planning 模块**（Dreamview+ 模块栏关开，或 `apply_config.sh planning`）——不用编译、不用 `aem profile use default`（⚠️ 仅**首次安装插件**需要编译一次，之后纯调参只重启模块）
5. 验证：`cat /apollo/modules/planning/traffic_rules/scene_speed_limit/conf/default_conf.pb.txt` 确认生效

## 7. 注意事项

- 限速区间带前/后缓冲，缓冲越大越早减速 → 实车建议缓冲适中（避免长距离慢速影响时间）
- 与内置 crosswalk 规则并存：插件限速不影响 crosswalk 的停车逻辑；两者独立
- 若实车**没有**这个插件：赛题 7/8 需像仿真一样改代码加 `AddSpeedLimit`（或确认是否有其他限速机制如 SpeedSetting/地图限速）
- 减速带还有 gflag `speed_bump_speed_limit` 机制（reference_line_info.cc 对 speed_bump overlap ±1m 限速）——两个机制可能叠加，取最小限速，注意不要互相打架（都配同一值最稳）
