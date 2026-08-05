# traffic_rules 模块分析报告

> 29 文件 / ~2,300 行 | 10 个交通规则检查器

## 功能概述

TrafficRule 是交通规则的抽象，通过 `TrafficDecider` 按 pipeline 配置顺序串联执行。每个规则在参考线上构建 **StopDecision**（停车决策），影响后续的路径和速度规划。

---

## 规则清单

| # | 规则 | 行数 | 核心功能 |
|---|------|------|----------|
| 1 | **crosswalk** | 495 | 人行横道停车：检测横道 + 行人/非机动车，构建虚拟障碍物 |
| 2 | **keepclear** | 306 | 路口禁停区：检测 keep_clear 区域，防止车辆停在路口内 |
| 3 | **traffic_light** | 241 | 红绿灯停车：检测红灯/黄灯，在停止线前停车 |
| 4 | **backside_vehicle** | 217 | 后方来车：检测后方快速接近的车辆 |
| 5 | **rerouting** | 207 | 重路由触发：检测是否需要发起重路由请求 |
| 6 | **destination** | 196 | 终点停车：在导航终点构建停车 |
| 7 | **yield_sign** | 168 | 让行标志停车：检测让行标志，减速/停车 |
| 8 | **reference_line_end** | 165 | 参考线终点：在参考线末端构建停车 |
| 9 | **stop_sign** | 159 | 停车标志：检测停车标志，构建停车 |
| 10 | **speed_setting** | 142 | 速度限制：设置道路速度上限 |

---

## 执行模型

```
TrafficDecider::Init()
└── 从 traffic_rule_config.pb.txt 加载 pipeline
    └── 按顺序创建 TrafficRule 实例 → rule_list_ (std::vector)

TrafficDecider::Execute(frame, reference_line_info)
├── for each rule in rule_list_:
│   ├── rule->Reset()
│   └── rule->ApplyRule(frame, reference_line_info)
│       └── 可能调用 BuildStopDecision() → 修改 reference_line_info
└── BuildPlanningTarget(reference_line_info)
    └── 汇总所有 StopDecision 为统一规划目标
```

## TrafficRule 接口

```cpp
class TrafficRule {
public:
    virtual bool Init(const TrafficRuleConfig& config) = 0;  // 初始化
    virtual void Reset() = 0;                                 // 重置状态
    virtual void ApplyRule(Frame* frame, 
                           ReferenceLineInfo* ref_line_info) = 0;  // 执行规则
protected:
    // 构建停车决策的辅助方法
    void BuildStopDecision(const std::string& reason, ...);
};
```

## 规则间依赖

规则按如下顺序执行（配置在 `traffic_rule_config.pb.txt` 中）：

```
destination → speed_setting → keepclear → crosswalk 
→ traffic_light → stop_sign → yield_sign 
→ reference_line_end → rerouting → backside_vehicle
```

先执行的规则优先级更高（先构建的 StopDecision 不会被后续规则覆盖）。
