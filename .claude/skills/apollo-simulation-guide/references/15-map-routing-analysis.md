# Map & Routing 源文件解析分析

> 分析 Apollo 场景地图（base_map.bin / routing_map.bin / sim_map.bin）和路由拓扑，
> 提取赛题场景的路网结构、路由信息、车道连接关系。

---

## 一、地图文件结构

### 1.1 文件位置与格式

| 文件 | 位置 | 大小（2026 赛题） | 格式 | 内容 |
|------|------|------------------|------|------|
| `base_map.bin` | `data/map_data/Xh_2026_contest/` | 6.07 MB | Protobuf `apollo.hdmap.Map` | 全量 HD Map（车道/道路/路口/信号灯/...） |
| `routing_map.bin` | 同上 | 0.98 MB | Protobuf TopoGraph | 路由拓扑图（车道节点 + 连接边） |
| `sim_map.bin` | 同上 | 4.82 MB | Protobuf `apollo.hdmap.Map` | 仿真地图（与 base_map.bin 内容相同） |
| `metaInfo.json` | 同上 | 175 B | JSON | 地图元信息（mapid / 坐标范围） |

### 1.2 Map Proto 结构

`base_map.bin` 使用 `apollo.hdmap.Map` 消息（proto 文件位于 bazel cache）:
- **Python 导入路径**:
  ```python
  sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/.../bazel-out/k8-opt/bin/external/apollo_src')
  sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/.../bazel-out/k8-opt/bin')
  from modules.common_msgs.map_msgs import map_pb2
  ```
- **Map 消息字段**: `header`, `crosswalk`, `junction`, `lane`, `stop_sign`, `signal`, `yield`, `overlap`, `clear_area`, `speed_bump`, `road`, `parking_space`, `pnc_junction`, `rsu`, `ad_area`, `barrier_gate`

### 1.3 关键子消息

#### Lane（车道）
```protobuf
message Lane {
  optional Id id = 1;                              // 车道 ID (如 "Lane_277")
  optional Curve central_curve = 2;                // 中心线几何
  optional double length = 5;                      // 长度 (m)
  optional double speed_limit = 6;                 // 限速 (m/s)
  repeated Id overlap_id = 7;                      // 关联的 overlap
  repeated Id predecessor_id = 8;                  // 前驱车道
  repeated Id successor_id = 9;                    // 后继车道
  repeated Id left_neighbor_forward_lane_id = 10;  // 左侧邻居
  repeated Id right_neighbor_forward_lane_id = 11; // 右侧邻居
  optional LaneType type = 12;                     // CITY_DRIVING / BIKING / ...
  optional LaneTurn turn = 13;                     // NO_TURN=1 / LEFT_TURN=2 / RIGHT_TURN=3 / U_TURN=4
  optional LaneDirection direction = 19;           // FORWARD=1 / BACKWARD=2 / BIDIRECTION=3
}
```

#### Road（道路）
```protobuf
message Road {
  optional Id id = 1;
  repeated RoadSection section = 2;     // 道路截面（含 lane_id 列表）
  optional Id junction_id = 3;          // 关联路口
  optional Type type = 4;               // HIGHWAY / CITY_ROAD / PARK
}
```

#### Junction（路口）- 9 种类型
```protobuf
enum Type { UNKNOWN=0; IN_ROAD=1; CROSS_ROAD=2; FORK_ROAD=3; MAIN_SIDE=4; DEAD_END=5; }
```

#### Signal（信号灯）- 6 种类型
```protobuf
enum Type { UNKNOWN=1; MIX_2_HORIZONTAL=2; MIX_2_VERTICAL=3; MIX_3_HORIZONTAL=4; MIX_3_VERTICAL=5; SINGLE=6; }
message Subsignal {
  enum Type { UNKNOWN=1; CIRCLE=2; ARROW_LEFT=3; ARROW_FORWARD=4; ARROW_RIGHT=5;
              ARROW_LEFT_AND_FORWARD=6; ARROW_RIGHT_AND_FORWARD=7; ARROW_U_TURN=8; }
}
```

#### StopSign / YieldSign / Crosswalk / SpeedBump / ParkingSpace
均包含 `Id id` + `repeated Id overlap_id` 字段。

---

## 二、解析 base_map.bin

### 2.1 完整解析脚本

脚本位置：`scripts/parse_map.py`

```python
#!/usr/bin/env python3
import sys, json
sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/.../bazel-out/k8-opt/bin/external/apollo_src')
sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/.../bazel-out/k8-opt/bin')
from modules.common_msgs.map_msgs import map_pb2

hdmap = map_pb2.Map()
with open('data/map_data/Xh_2026_contest/base_map.bin', 'rb') as f:
    hdmap.ParseFromString(f.read())

# 提取所有元素
for lane in hdmap.lane:
    print(f"{lane.id.id}: turn={lane.turn}, speed={lane.speed_limit}, "
          f"pred={len(lane.predecessor_id)}, succ={len(lane.successor_id)}")
```

### 2.2 常见分析需求

| 需求 | 方法 |
|------|------|
| 统计元素数量 | `len(hdmap.lane)`, `len(hdmap.junction)`, `len(hdmap.signal)` 等 |
| 找特定车道 | 遍历 `hdmap.lane`，匹配 `lane.id.id` |
| 找信号-车道关联 | 遍历 `hdmap.overlap`，匹配 object.id 前缀 (Signal_/Lane_) |
| 找 U-Turn 车道 | `[l for l in hdmap.lane if l.turn == 4]` |
| 车速限制分析 | 统计 `lane.speed_limit` 分布 |
| 车道连接关系 | 通过 `lane.predecessor_id` / `lane.successor_id` 构建图 |

### 2.3 输出文件

```bash
python3 scripts/parse_map.py
# → output/map_analysis_full.json    # 全量结构化数据
# → output/map_analysis_summary.json  # 汇总统计
```

---

## 三、解析 routing_map.bin

### 3.1 格式说明

`routing_map.bin` 是 Apollo 路由拓扑图（`TopoGraph` proto），**不是 `RoutingResponse`**。
由于 `TopoGraph` proto 的 Python 绑定未生成，需通过以下方式解析：

### 3.2 步骤 1：protoc --decode_raw 解码

```bash
# 找到 protoc 二进制和 libprotobuf.so
PROTOC=$(find /home/skye/application-pnc/.cache/bazel -name "protoc" -type f | head -1)
LIBDIR=$(dirname $(find /home/skye/application-pnc/.cache/bazel -name "libprotobuf.so" -not -path "*/sandbox_stash/*" | head -1))

# 解码为原始文本
LD_LIBRARY_PATH="$LIBDIR" $PROTOC --decode_raw \
  < data/map_data/Xh_2026_contest/routing_map.bin \
  > output/routing_map_raw.txt
```

### 3.3 TopoGraph 原始结构

```
1: "xh_2026_contest-6"        ← 地图版本
2: ""                          ← 空字段
3 {                            ← 车道节点（558 个）
  1: "Lane_277"               ← 车道 ID
  2: 0x4028523c9163de04       ← 起始 s 坐标 (double)
  3 { ... }                    ← 范围信息
  5: 0x4016849b059fcb40       ← 长度 (double)
  6 { ... }                    ← 中心线几何点序列 (每点 x,y 两个 double)
  7: 0                         ← 方向/标志
  8: "Road_277"                ← 所属道路 ID
}
4 {                            ← 车道连接边（828 个）
  1: "Lane_277"               ← 起始车道
  2: "Lane_1001"              ← 目标车道
  3: 0x0000000000000000       ← 权重 (double, 0=虚拟连接)
  4: 0                        ← 边类型
}
```

### 3.4 边类型含义

| Type | 名称 | 数量（2026） | 含义 | 权重 |
|------|------|------------|------|------|
| **0** | `TET_VIRTUAL` | 613 | 路口内部虚拟连接 / 车道变换 | 0 |
| **1** | `TET_FORWARD` | 108 | 正向行驶（同道路直行） | 实际距离 (m) |
| **2** | `TET_LEFT/RIGHT` | 107 | 转弯连接（左/右转/掉头） | 实际距离 (m) |

### 3.5 步骤 2：Python 解析脚本

脚本位置：`scripts/parse_routing_graph.py`

功能：
- 从 `output/routing_map_raw.txt` 读取 `protoc --decode_raw` 输出
- 正则提取所有节点（Field 3: lane_id + road_id）和边（Field 4: from + to + weight + type）
- 构建有向图（邻接表）
- 分析链路（chains）、起始终止车道、车道连接数
- 输出 `output/routing_graph.json`

```bash
python3 scripts/parse_routing_graph.py
```

### 3.6 路由分析要点

| 分析目标 | 方法 |
|---------|------|
| 正向行驶链路 | 用 Type 1 边追踪 `from→to` 链 |
| 转弯连接 | Type 2 边表示交叉口转向关系 |
| 最长可行驶路径 | 优先 Type 1 > Type 2 > Type 0 追踪 |
| 信号灯路由 | 交叉引用 base_map 的 signal-lane overlap |
| 多车道段（变道） | 同一 Road 内的平行车道 |
| 施工区路由 | 低速（2.78 m/s）车道链 |
| U-Turn 路由 | U_TURN lane 不在路由图中时，找相邻车道替代路由 |
| 环岛路由 | 圆形 Type 0 虚拟链路 |

---

## 四、2026 赛题地图关键数据

### 4.1 地图元素统计

| 元素 | 数量 |
|------|------|
| 车道 (Lane) | 560 |
| 道路 (Road) | 439 |
| 路口 (Junction) | 9 (全部 CROSS_ROAD) |
| 信号灯 (Signal) | 24 |
| 停止标志 (StopSign) | 1 |
| 让行标志 (YieldSign) | 1 |
| 人行横道 (Crosswalk) | 34 |
| 减速带 (SpeedBump) | 6 |
| 停车位 (ParkingSpace) | 43 |
| 重叠关系 (Overlap) | 618 |

### 4.2 路由拓扑统计

| 指标 | 数值 |
|------|------|
| 可路由车道（节点） | 558 |
| 车道连接（边） | 828 |
| 虚拟连接 (Type 0) | 613 (权重=0) |
| 正向行驶 (Type 1) | 108 (权重=距离) |
| 转弯连接 (Type 2) | 107 (权重=距离) |
| 起始车道（无前驱） | 450 |
| 终止车道（无后继） | 450 |
| 正向链路数 | 151 |

### 4.3 关键特殊区域

| 区域 | 车道范围 | 特征 |
|------|---------|------|
| **信号灯核心区** | Lane_530-537, 592-598, 731-737, 1440-1474 | Junction_4/7/11 |
| **变道多车道路段** | Road_604(3车道), Road_656(3车道), Road_1666-1696 | 有平行邻居 |
| **U-Turn 区域** | Lane_1559, 1583 (地图标注) / Lane_1560, 1585 (路由替代) | U_TURN turn type |
| **施工区** | Lane_1860-1987 | 27条车道 10 km/h |
| **停车区** | ParkingSpace_40-58 | 最大集群 19个 |
| **环岛** | Lane_1737-1749 | 8车道 Type 0 圆形链路 |

---

## 五、解析产出文件清单

| 文件 | 路径 | 说明 |
|------|------|------|
| base_map 全量 JSON | `output/map_analysis_full.json` | 560车道+439道路+618重叠 完整数据 |
| base_map 摘要 JSON | `output/map_analysis_summary.json` | 汇总统计 |
| 路由原始解码 | `output/routing_map_raw.txt` | protoc --decode_raw 输出 (198K行) |
| 路由图 JSON | `output/routing_graph.json` | 558节点+828边 结构化数据 |
| 解析脚本 - 地图 | `scripts/parse_map.py` | HD Map protobuf 解析 |
| 解析脚本 - 路由 | `scripts/parse_routing_graph.py` | 路由拓扑解析 |

---

## 六、快速使用指南

### 重新解析全部数据

```bash
# 1. 解析 base_map.bin
python3 scripts/parse_map.py

# 2. 解码 routing_map.bin 为原始文本
LIBDIR=$(dirname $(find .cache/bazel -name "libprotobuf.so" -not -path "*/sandbox_stash/*" | head -1))
PROTOC=$(find .cache/bazel -name "protoc" -type f | head -1)
LD_LIBRARY_PATH="$LIBDIR" $PROTOC --decode_raw \
  < data/map_data/Xh_2026_contest/routing_map.bin \
  > output/routing_map_raw.txt

# 3. 解析路由图
python3 scripts/parse_routing_graph.py
```

### 快速查询

```python
import json

# 读取地图数据
with open('output/map_analysis_full.json') as f:
    data = json.load(f)

# 查询特定车道
lane = next(l for l in data['lane_details'] if l['id'] == 'Lane_1559')

# 读取路由数据
with open('output/routing_graph.json') as f:
    routing = json.load(f)

# 查询车道连接
edges = [e for e in routing['edges'] if e['from'] == 'Lane_1559']
```

### 排查场景路由

| 场景 | 方法 |
|------|------|
| 交通灯 | 交叉分析 signal_details + signal_lane_pairs + 路由前向边 |
| 变道 | 找 road_details 中 num_lanes≥2 的道路及其关联车道 |
| S 弯 | 分析路由链中包含 LEFT_TURN 车道的路径 |
| U-Turn | 地图标注 U_TURN 车道不在路由图中 → 找相邻替代车道 |
| 施工区 | 分析 speed_analysis 中 speed_limit=2.78 的车道链 |
| 站点接驳 | 找 parking_space_ids 附近的 routing chains |
| 环岛 | 分析全部 Type 0 圆形拓扑的链路 |
