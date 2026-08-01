# 赛题一：减速带场景 — 解决方案

> 时间：2026-08-01 | 赛题要求：主车通过减速带时速度 **≤ 3 m/s**，超速每 1m/s 每帧扣 2 分

---

## 一、机制研究结论（代码链路）

```
HD Map speed_bump_overlaps（6 个减速带）
  → ReferenceLineInfo::Init()  (reference_line_info.cc:123-127)
      对每个减速带施加限速区 [start_s-1.0, end_s+1.0]
      限速值 = FLAGS_speed_bump_speed_limit（gflag）
  → ReferenceLine::GetSpeedLimitFromS()  (reference_line.cc:919)
      限速区内的 s 返回该限速值
  → SpeedLimitDecider::GetSpeedLimits()  (speed_limit_decider.cc:45)
      读取参考线限速 + 曲率限速 + 绕障限速 → 取最小
  → SpeedBoundsDecider 构建 ST 边界
  → DP(PathTimeHeuristic) + QP(PiecewiseJerkSpeed) 生成不超限速的速度曲线
```

**核心参数**：`speed_bump_speed_limit`（gflag）
- 定义：`planning_gflags.cc:305`，默认 **4.4704 m/s（10mph）**
- 唯一生效位置：`reference_line_info.cc:127`
- **无专门 Scenario / TrafficRule**，纯 gflag 机制

## 二、地图解析结论（容器内解析 base_map.bin，关键发现）

| 项目 | 结果 |
|------|------|
| 减速带数量 | 6 个（SpeedBump_1 ~ 6） |
| 关联车道 | 8 条（Lane_1962/1963/1973/1974/1999/2000/2003/2004），**全部在主路线上** |
| 减速带车道限速 | **70 km/h（19.44 m/s）** ← 地图不帮忙限速！ |
| 全图 27 条 10km/h 车道 | 与减速带无关（其他区域） |

> 🔴 **关键结论**：减速带车道地图限速 70km/h，**没有任何地图辅助限速**，
> 减速带限速**完全依赖 `speed_bump_speed_limit` gflag**。若该 gflag 未生效
> （默认 4.47 m/s），主车将以约 4.5m/s 通过减速带 → **必超速扣分**。

## 三、配置修改（default profile）

文件：`profiles/default/modules/planning/planning_component/conf/planning.conf`

| 参数 | 原值 | 新值 | 理由 |
|------|------|------|------|
| `speed_bump_speed_limit` | 3 | **2.5** | 3.0 贴边，控制跟踪超调可能到 3.0~3.3；2.5 留 0.5 裕量确保实际 ≤3 |

- 修改方式：gflags，**无需编译**
- 生效：`aem profile use default` + 重启 Dreamview

## 四、生效与验证步骤（用户执行）

```bash
# 1. 确认 /apollo 下 planning.conf 软链指向 profile（关键！）
grep speed_bump_speed_limit /apollo/modules/planning/planning_component/conf/planning.conf
#    应输出 --speed_bump_speed_limit=2.5
#    若显示注释掉或默认，说明软链指向源码，需重新：
aem profile use default

# 2. 重启 Dreamview
aem bootstrap stop && aem bootstrap start --plus

# 3. 跑仿真，观察通过减速带时的实际车速（应 ≤3 m/s）
```

## 五、风险与回退

| 风险 | 说明 | 缓解 |
|------|------|------|
| 配置不生效 | /apollo 软链指向源码 planning.conf（源码里该行被注释） | 执行 `aem profile use default`；或在源码 `modules/planning/planning_component/conf/planning.conf` 同步取消注释 |
| 2.5 过慢 | 通过减速带体验慢，但不影响评分（只罚超速） | 若实测 2.5 太慢且速度余量足够，可回调到 2.7 |
| 减速带前急减速 | 巡航 16.667 → 2.5 需提前减速（DP max_deceleration=-4.0，约 33m 提前量） | DP/QP 自动处理，一般平顺 |

## 六、结论

- 减速带赛题是**纯配置方案**，核心参数 `speed_bump_speed_limit` 已设为 **2.5 m/s**
- 唯一风险是**配置未生效**（软链问题），务必按第四节验证
- 提交内容：`profiles/default/.../planning.conf`（`speed_bump_speed_limit: 3 → 2.5`）
