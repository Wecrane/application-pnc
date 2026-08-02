# 日志/回放分析实操手册（可复用代码）

> 配套 `../SKILL.md`。以下代码均在 2026-08 实战验证过。

---

## 1. sim_engine.log 评分解析（Python）

```python
import re

SIM = 'sim_engine.log'

# --- 1a. 提取单个指标的 detailed_result pass/fail ---
def parse_metric_passfail(sim_log, metric_name):
    """返回 {timestamp: is_pass}，以及带 debug/delta 的计分帧"""
    passfail = {}
    scored_frames = []   # (timestamp, debug_description, delta_score)
    in_metric = False
    cur = None
    for line in open(sim_log, errors='ignore'):
        if f'name: "{metric_name}"' in line:
            in_metric = True; continue
        if in_metric:
            if 'name: "' in line and metric_name not in line:
                in_metric = False; continue
            mt = re.search(r'timestamp: ([\d.]+)', line)
            if mt: cur = float(mt.group(1))
            mp = re.search(r'is_pass: (true|false)', line)
            if mp and cur is not None:
                passfail[cur] = (mp.group(1) == 'true')
            md = re.search(r'description: "(.+?)"', line)
            if md and cur is not None:
                scored_frames.append(cur)
    return passfail, scored_frames

# 用法
pf, scored = parse_metric_passfail('sim_engine.log', 'DistToObstacleCar')
fails = sorted(t for t, p in pf.items() if not p)
print("失败帧:", fails)      # 如 [0.0, 16.5, 17.0, 80.7]
print("计分帧:", scored)     # 唯一计分帧（带 debug）
```

### 解读要点
- **失败帧分两类**：静默失败（无 debug 无 delta，不计分）+ **唯一计分帧**（带 `debug`+`delta_score`，扣分）
- **`not_pass_timestamp`**（json 汇总里）是失败时刻列表，`metric_score` 是该项分数
- **ReachEnd 是 gate**：`metric_score=0` → 总分 0
- **TimeLimit**：`not_pass` 从 90 开始 → 超时 0 分

---

## 2. planning 日志时间轴校准（T0 反推）

```python
import re

LOG = 'planning.log.INFO.20260802-125751.262559'

def w2s(w):
    h, m, s = w.split(':'); return int(h)*3600 + int(m)*60 + float(s)

# 提取车轨迹（planning_start_point: x,y,theta,k,v,a）
rows = []
for line in open(LOG, errors='ignore'):
    if 'planning_start_point' in line:
        m = re.search(r'(\d\d:\d\d:\d\d\.\d+)', line)
        nums = re.findall(r'[-+]?\d+\.\d+', line.split('x,y,the,k:')[-1])
        if m and len(nums) >= 6:
            rows.append((w2s(m.group(1)), float(nums[0]), float(nums[1]),
                         float(nums[4]), float(nums[5])))  # wall,x,y,v,a

# 车停稳 wall = 最后 v<0.05 的帧
stop_wall = next(r[0] for r in reversed(rows) if r[3] < 0.05)

# 关键：评测计分帧 = 车停稳时刻（评测在车停止时采样）
# 若已知评测计分帧 t_scored（从 sim_engine），则：
#   T0_sim（评测 t=0 对应 wall）= stop_wall - t_scored
# 若已知评测总时长 total（sim_engine total_timestamp），则：
#   T0_sim = stop_wall - (车停稳对应的评测时刻)
```

### 时间轴规则（实测）
- **评测 t = planning t − 13.5s**（KeyPoint 反推）
- **T0 校准最稳方法**：用车停稳 wall + 已知评测计分帧（车停=采样帧）反推
- ⚠️ 别用"planning 第 80.7 帧"当评测 80.7（差 13.5s 全错）

---

## 3. planning 日志关键模式提取

```python
# 3a. fence 位置变化（stop_s 是否抖动 → 车被 fence 牵引蠕动）
fences = []
for line in open(LOG, errors='ignore'):
    if 'reference_line_info.cc:832' in line and 'stop_s' in line:
        m = re.search(r'(\d\d:\d\d:\d\d\.\d+)', line)
        ss = re.search(r'stop_s: ([\d.]+)', line)
        es = re.search(r'end_s: ([\d.]+)', line)
        if m and ss and es:
            fences.append((w2s(m.group(1)), float(ss.group(1)), float(es.group(1))))
# stop_s 持续变化 → fence 不稳定 → 车蠕动

# 3b. 障碍物 ST boundary（哪个障碍在阻塞车）
#  grep "build reference line st boundary. id:X" → X 是障碍 id
#  grep "NO build reference line st boundary. id:X" → 该障碍被 IGNORE/横向远

# 3c. 距终点/距参考线末端
#  grep "distance_destination:X distance_ref_end:Y"
#  distance_ref_end=1.79769e+308(=DBL_MAX) → 参考线末端距离无效(常见异常)
```

---

## 4. 云端回放抓包 + 解密（offlineview）

### 步骤
1. 打开 `offlineview?id=<recordId>`（需登录）
2. 浏览器 DevTools → Sources → 添加 `addInitScript`：
   ```js
   // 捕获 WebSocket 消息并转发到本地 HTTP 服务器
   (function(){
     const orig = WebSocket.prototype.onmessage;
     WebSocket.prototype.addEventListener = function(t, fn){
       if (t === 'message') {
         return orig.call(this, t, (e) => {
           try { fetch('http://127.0.0.1:8899/ws', {method:'POST', body:e.data}); } catch(_){}
           fn(e);
         });
       }
       return orig.call(this, t, fn);
     };
   })();
   ```
3. 点播放 → 帧数据流式 POST → 本地服务器存 `output/cloud_replay.json`
4. 解密：`python3 scripts/decode_cloud_replay.py`（world 字段 = base64 + AES-CBC）
   - 密钥 = `SHA256(UTF8("明月几时有"))`，前 16 字节 = IV，PKCS7
5. 分析：`python3 scripts/analyze_cloud_ped.py`

### 解密后 SimWorld 关键字段
- `autoDrivingCar`: positionX/Y, speed, heading, throttle, brake, currentSignal
- `object[]`: id, positionX/Y, speed, heading, type, STOP 决策位置
- 用途：验证云端真实停车距离/起步时机/决策闪烁

---

## 5. 完整排障模板（复制即用）

```bash
# ① 下载云端 log（油猴脚本，保存到 ~/下载/log/<id>.tgz）
# ② 解压
mkdir -p /tmp/scn_<id> && tar -xzf ~/下载/log/<id>.tgz -C /tmp/scn_<id>
cd /tmp/scn_<id> && tar -xzf individualModuleLogs.tar.gz

# ③ 看评分（找唯一计分帧）
grep -E "metric_score|not_pass_timestamp|score:|name: \"DistToObstacleCar\"" sim_engine.log | head

# ④ 看评测配置
grep -iE "dist_to_obstacle|condition" replay-engine.log.INFO.* | head

# ⑤ planning 车轨迹（见第 2 节代码）
```

---

## 6. 实测验证过的关键数字（2026-08）

| 项 | 值 |
|----|-----|
| 评测 t ↔ planning t | 评测 = planning − **13.5s** |
| DistToObstacleCar 赛题五 | obstacle 7673，距离区间 [2, 5.5]m，扣 20 |
| 车停稳 → 评测结束延迟 | ~1s（0.2 版 80.7→81.7；满分版 69→71.5） |
| 行人 ST boundary 阻塞 | 7673 横向>1.8m 后 ST boundary 空；横向>3.5m 应 IGNORE |
| `distance_ref_end` 无效值 | `1.79769e+308`（DBL_MAX） |
| 车坐标基准 | 绝对坐标 423xxx（参考线起点 ~423278） |
