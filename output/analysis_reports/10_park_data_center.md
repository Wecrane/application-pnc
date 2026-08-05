# park_data_center 模块分析报告

> 5 文件 / ~472 行 | 泊车数据中心

## 功能概述

`park_data_center` 是泊车场景的 **专用数据中心**，使用单例模式管理跨帧的障碍物绕行（Nudge）信息。

---

## 核心类

### ParkDataCenter

| 文件 | 行数 | 职责 |
|------|------|------|
| `park_data_center.h` | 58 | 单例接口 + 历史队列定义 |
| `park_data_center.cc` | 81 | 实现 |
| `park_data_center_test.cc` | 30 | 测试 |

```cpp
class ParkDataCenter {
public:
    DECLARE_SINGLETON(ParkDataCenter);  // 单例宏
    
    // 当前帧 NudgeInfo 访问
    const NudgeInfo& current_nudge_info(size_t ref_key) const;
    NudgeInfo* mutable_current_nudge_info(size_t ref_key);
    
    // 历史帧管理
    void UpdateHistoryNudgeInfo(uint32_t seq_num);
    bool GetLastFrameNudgeTrackInfo(uint32_t seq_num, size_t ref_key, ...);
    
private:
    NudgeInfoMap current_nudge_info_data_;          // 当前帧 (map<key, NudgeInfo>)
    NudgeInfoIndexedQueue history_nudge_info_data_; // 历史帧队列
};
```

### NudgeInfo

| 文件 | 行数 | 职责 |
|------|------|------|
| `nudge_info.h` | 143 | 绕行信息结构 |
| `nudge_info.cc` | 160 | 实现 |

记录障碍物绕行决策的详细信息（障碍物ID、绕行方向、距离等）。

## 数据结构

```
NudgeInfoMap = std::map<std::size_t, NudgeInfo>
    // 按 reference_line_key 索引

NudgeInfoIndexedQueue = IndexedQueue<uint32_t, NudgeInfoMap>
    // 按 sequence_num 索引
    // 容量由 FLAGS_max_frame_history_num 控制
    // FIFO 淘汰策略

⚠️ IndexedQueue 成员为 public，存在封装破坏风险
```

## 使用场景

被以下泊车相关场景使用：
- `park_and_go`
- `valet_parking`
- `valet_parking_park`

用于跨帧跟踪障碍物绕行状态，避免在连续帧中反复切换绕行决策。

## 设计问题

`IndexedQueue` 模板类（`planning_base/common/indexed_queue.h`）的三个核心成员 `capacity_`、`queue_`、`map_` 全部声明为 `public`，外部可任意破坏 queue ↔ map 的一致性。建议改为 private + const 访问器。
