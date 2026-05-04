**梳理 planning 模块作用**

**没有正在进行的任务**

planning文件夹下各模块分别是什么作用

**14:06**

**上 17 条消息**

按你这个仓库里的实际结构，**modules/planning** 不是单一模块，而是一套“入口 + 接口 + 算法库 + 插件集合 + 支撑模块”。主链路大致是：**PlanningComponent** 进来后，先跑 **traffic_rules**，再进入 **scenario -> stage -> task**，最后由 **planner** 产出轨迹；参考线由 **pnc_map** / **reference_line_provider** 生成。**planning_component/README_cn.md (line 5)**

* **planning_component**：规划入口和运行框架，负责订阅感知/定位/底盘/路由等输入，选择 **OnLanePlanning** 或 **NaviPlanning**，并启动主规划流程。**planning_component/README_cn.md (line 88)**, **on_lane_planning.cc (line 109)**, **navi_planning.cc (line 61)**
* **planning_interface_base**：接口层，定义 **Planner**、**Scenario**、**Stage**、**Task**、**TrafficRule** 这些插件的抽象基类，下面各目录都是实现这些接口的具体插件。**planner.h (line 40)**, **scenario.h (line 52)**, **task.h (line 37)**, **traffic_rule.h (line 37)**
* **planning_base**：基础库，放公共数据结构、数学工具、参考线处理、历史帧、上下文、平滑器等，是规划算法的底座。**planning_base/README_cn.md (line 5)**
* **planners**：具体规划器插件集合，仓库里主要有 **public_road**、**navi**、**lattice**、**rtk**，分别对应常规道路规划、导航规划、网格规划和轨迹回放。**planning_component/README_cn.md (line 92)**
* **pnc_map**：生成规划参考线的地图适配层，核心是把 routing / 地图 / 车辆当前位置转换成可供规划使用的参考线。**reference_line_provider.h (line 57)**
* **scenarios**：场景插件层，处理不同驾驶场景的状态机，比如车道保持、路口、让行、泊车、紧急靠边等。**planning_component/README_cn.md (line 32)**
* **tasks**：更细粒度的任务插件层，负责路径、速度、变道、借道、停车、open space 轨迹生成等具体子工作。像你打开的 **lane_follow_path**、**lane_change_path** 就属于这一层。**planning_component/README_cn.md (line 19)**
* **traffic_rules**：全局交通规则插件层，先于场景执行，对所有场景都生效，比如红绿灯、停止线、人行横道、终点、让行、重路由等。**planning_component/README_cn.md (line 174)**, **traffic_decider.h (line 37)**
* **planning_open_space**：开放空间算法库，主要给泊车、出库、窄空间 maneuver 用，里面有粗轨迹生成和轨迹平滑两大块。**planning_open_space/BUILD (line 16)**
* **park_data_center**：停车/挪车相关的数据缓存中心，代码里主要是按帧保存和回读 **NudgeInfo** 之类的历史信息。**park_data_center.h (line 36)**, **park_data_center.cc (line 42)**

如果你愿意，我可以接着把 **tasks** 下面每个子目录也按“路径类 / 速度类 / open space 类”给你再拆一遍。

**README_cn.md**文档 · MD打开

**README_cn.md**文档 · MD打开

**14:08**

现在对于赛题三，一个很窄的S弯，路边还有障碍物，我要完成这道题应该怎么做？现在的情况是遇到障碍物会形成停止墙，走不了路

**14:10**

**上 13 条消息**

对，你说得对。这次应该先看 skill。

我按 Apollo EDU 的赛题索引核对过了，赛题三对应的是 **xh_2026_S弯场景**，官方描述就是窄 S 弯里平滑通过，限速 30km/h，遇障碍物要安全绕行，横向距离不足会扣分。相关入口主要是这几篇：**00_场景索引**、**06_场景—障碍物绕行**、**13_场景—借道绕行解题思路**。[场景索引](https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_204___xE8_xB5_x9B_xE4_xBA_x8B_xE9_x9B_a848fd9967edaeab0b823ae95f3ebd3e.html) [障碍物绕行](https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_204___xE8_xB5_x9B_xE4_xBA_x8B_xE9_x9B_1d2da66fad1736c3e72c905b3adfc512.html) [借道绕行](https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE8_xB5_x9B_xE4_xBA_x8B_xE6_x80_xBB_xE6_xB1_x87_204___xE8_xB5_x9B_xE4_xBA_x8B_xE9_x9B_34a60804bfab7f480fc76cb8f4737bcb.html)

你现在“遇到障碍物就生成 stop wall”这个现象，说明当前障碍物被 **PathDecider** 判成了 blocking obstacle，走的是停车逻辑，不是绕行逻辑。你仓库里这里也能看到，**PathDecider** 会直接给 blocking obstacle 下 stop decision。**path_decider.cc (line 166)**

实操上，这题优先不是改场景，而是把它从“停”调成“挪开”：

1. 先在 Dreamview / PNC Monitor 看 stop wall 是谁生成的，是 **PathDecider/blocking_obstacle** 还是 **RuleBasedStopDecider**。
2. 如果是 **PathDecider**，重点调路径侧参数：**obstacle_lat_buffer**、**static_obstacle_buffer**、**collision_safety_range**，让它更倾向于 nudge/绕行，而不是判成必须停车。
3. 只有地图里真有可借邻道时，再考虑 **lane_borrow_path**；窄 S 弯如果没有足够空间，硬借道通常会更糟。
4. 速度上按赛题索引控制在 30km/h 内，绕障阶段别让 **SPEED_BOUNDS** 把车速抬太高。

你这个题更像“窄路静态障碍物绕行”，不是单纯的“借道绕行”。我建议下一步直接看你当前 PNC Monitor 里是哪一个 task 触发了 stop wall，我就能按那个点位给你下手。

**14:12**

看一下最近几次的日志，最近三次的都是S弯场景，不能全读，文件太大了会造成系统崩溃，你挑着读

**14:14**

**上 15 条消息**

我按 Apollo EDU 的思路只挑了最近三次 **planning.log** 里的关键段看，没有全量读。

结论先说：这次不像纯粹的“障碍物形成停止墙”卡死，最新这条更像是  **routing / reference line 收缩失败** 。在 **data/log/planning.log.INFO.20260504-135738.93908** 里我看到：

* 前面大量是 **nudge** / **path bounds** 计算，说明规划还在正常尝试绕障
* **lane_change_path** 被跳过了：**is_change_lane=0 path_reusable=0 ref_line_count=1**
* **lane_borrow_path** 里出现了 **Blocking obstacle ID[]**，但 ID 是空的
* 最后真正报错是 **Fail to shrink routing segments.**，随后是 **Reference lane is empty!**

所以当前问题优先怀疑的是：

1. 路由段和参考线投影/裁剪出了问题
2. S 弯场景下参考线没正确生成
3. 障碍物没有被正确识别成可借道/可绕行的 blocker

如果你要，我下一步可以继续只盯这三类日志去挖：

* **path_bounds_decider**
* **lane_borrow_path**
* **reference_line / routing_segments**

这样不会把大日志整文件读爆。

**14:18**

继续

**14:18**

**上 36 条消息**

继续看下来，最新这波更像是“参考线/路由失配”，不是障碍物直接形成 stop wall。

在 **data/log/planning.log.INFO.20260504-135738.93908** 里，14:05:23 那个周期其实是正常跑完的，**LANE_BORROW_PATH** 直接跳过，**Blocking obstacle ID[ ]** 还是空的，**PlanOnReferenceLine done, has_error=0** 也有出现；真正出问题是在下一帧：**trajectory_stitcher** 报了 **lat_diff = 18.4327**，随后 **on_lane_planning.cc** 直接失败在 **Fail to shrink routing segments.**，再往后才出现 **Reference lane is empty!**。**trajectory_stitcher.cc (line 186)**, **on_lane_planning.cc (line 160)**, **lane_borrow_path.cc (line 498)**

关键点是：**replan_lateral_distance_threshold** 默认只有 **0.5m**，你这里的 **18.4m** 已经是严重偏离了，所以更像是车身位置和参考线/路由段对不上，而不是障碍物把路“堵成墙”。**trajectory_stitcher.cc (line 197)**, **planning_gflags.cc (line 409)**

我下一步会继续盯这三块：**routing** 生成、**reference_line_provider**、**lane_follow_map**，先把“为什么会 shrink 失败”找出来，再回头看 S 弯障碍物该怎么绕。

**14:23**

继续

**14:23**

**上 26 条消息**

继续看，最新这波已经不像“障碍物 stop wall”，更像是  **车位姿和 routing 对不上** 。

关键链路是这样的：

* 13:59:02 这一帧还正常结束了，但下一帧立刻出现 **[lat_diff = 31.2159]**，触发重规划。见 **trajectory_stitcher.cc (line 197)**
* 随后 **reference_line.cc** 报 **Too few reference points after shrinking.**，所以参考线直接缩没了。见 **reference_line.cc (line 171)**
* 再往下 **lane_follow_map** 连当前点 **(423258.99, 4438081.53)** 的最近路由点都找不到，**reference_line_provider** 于是抽不出 segments。见 **lane_follow_map.cc (line 201)** 和 **reference_line_provider.cc (line 616)**

而且这不是一次偶发，**20260503-192733** 里还出现过一次更狠的，**lat_diff = 225.278**，后面同样是 **fail to shrink** 和 **Failed to get waypoint from routing**。所以问题更像是  **定位/车位突然跳了，或者场景重置后 routing 没跟上** ，不是单纯障碍物挡死。见 **20260503-192733 日志**

我接下来会继续只盯两条线：**planning_command/routing** 有没有更新，以及场景切换前后车位是不是跳变。

**14:28**

继续吧？

**14:28**

**上 6 条消息**

继续看下来，主因更像是“车位 / routing 失配”，不是 stop wall。

我抓到这条链路了：

* 13:59:02 这一帧还正常结束，但紧接着 **lat_diff = 31.2159**，触发重规划。
* 随后 **ReferenceLine::Segment** 直接报 **Too few reference points after shrinking.**，参考线被缩没了。
* 再往后 **lane_follow_map** 连当前点 **(423258.99, 4438081.53)** 的最近路由点都找不到，于是 **reference_line_provider** 抽不出 segments。

对应代码就是这三处：

* **trajectory_stitcher.cc (line 197)**
* **reference_line.cc (line 171)**
* **lane_follow_map.cc (line 201)**

更关键的是，日志里 **Blocking obstacle ID[ ]** 一直是空，说明这次并不是障碍物正常触发了 stop 决策。
我现在更怀疑两件事：一是定位突然跳了，二是 routing / 场景切换后没同步好。下一步我会只盯这两个方向继续挖。

**14:31**

那个障碍物设定的是一直闪烁的，模拟雷达干扰。我在想要不要写一个过滤逻辑，只要障碍物出现了就标定为一直有（位置大小固定）。车辆的话不会闪烁。

**14:33**

**上 4 条消息**

可以做，而且 **建议做成“短时保持 + 位置/尺寸一致性”** ，不要做成“见过一次就永久有”。

更稳的方案是：

1. **只对静态路边障碍做保留**
   * 车这种目标本身不闪烁，按正常跟踪走。
   * 路边杆、锥桶、箱体这类，允许做持久在线缓存。
2. **按时间和几何一致性做过滤**
   * 上一帧/最近几帧出现过，就先保留 **0.5s ~ 2s**
   * 只有当新检测和历史障碍的 **位置 / 尺寸 / 朝向** 差异还在阈值内，才算同一个障碍
   * 连续丢失超过阈值，再真正删掉
3. **只影响“阻塞判断”，不要改全局地图**
   * 你真正想稳住的是 **blocking obstacle** / **stop wall** 这条链路
   * 不要把这个缓存扩散到路线层、地图层

我看你最近日志里，**front_static_obstacle_id()** 还是空的，而且真正炸掉的那段更像是 **reference line / routing** 跳变，不完全是障碍物 stop wall 本身。相关位置在这里：

* lane_borrow_path.cc:499
* path_decider.cc:147
* reference_line_provider.cc:611

所以我的建议是： **先做一个静态障碍物持久在线的滤波器** ，专门服务这道 S 弯的 stop-wall 问题；车类目标继续按正常检测结果走。这样最贴合赛题，也最不容易把别的场景搞坏。

**14:34**

做一个静态障碍物持久在线的滤波器，要求呢必须包含在提交代码范围内（见skill），服务于所有有关障碍物的模块

**14:37**

**上 46 条消息**

已做。过滤放在 **Obstacle::CreateObstacles()** 这一层，所以所有后续依赖 **Frame::obstacles()** 的规划模块都会吃到同一份稳定结果，像 stop、borrow、nudge、ST boundary 这些都会受益。

这版的行为是：

* 只对 **is_static** 的障碍物做持久在线
* 默认保持 **1.5s**，参数是 **static_obstacle_hold_time_sec**
* 同一个静态障碍物再次出现时，位置/尺寸会沿用缓存值
* 超时没再出现才释放
* 还加了单测，覆盖“重现时钉住”和“消失后超时释放”

改动在：

* modules/planning/planning_base/common/obstacle.cc
* modules/planning/planning_base/common/obstacle.h
* modules/planning/planning_base/gflags/planning_gflags.cc
* modules/planning/planning_base/gflags/planning_gflags.h
* modules/planning/planning_base/common/obstacle_test.cc

我这边没法直接跑编译，环境里没找到 **buildtool/bazel**，但补丁做了 **git diff --check**，格式层面是干净的。

**5 个文件已更改**+232**-25**

撤销审核

**modules/planning/planning_base/gflags/planning_gflags.h**

**modules/planning/planning_base/gflags/planning_gflags.cc**

**modules/planning/planning_base/common/obstacle.h**

**modules/planning/planning_base/common/obstacle.cc**

**modules/planning/planning_base/common/obstacle_test.cc**

**14:49**

又跑了两次S弯，看看日志

**14:57**

**上 6 条消息**

我看了，两个最新日志还是同一类问题，不是障碍物闪烁主导。

结论很明确：

* **2026-05-04 14:55:41** 这次日志里，**Blocking obstacle ID[]** 还是空的，说明没有形成稳定的阻塞障碍物链路。
* 到 **14:56:06** 和 **14:56:29**，开始出现 **lat_diff = 15.9194 / 16.0266**，随后有 **get_width_of_lane: 0**、**Failed to get the width of lane!**、**Fail to shrink routing segments**，最后还是 **Reference lane is empty!**
* 上一次 **2026-05-04 13:59:03** 也是类似：**lat_diff = 31.2159**，然后 **Too few reference points after shrinking**、**Failed to find nearest point**、**Failed to get waypoint from routing**、**Failed to create reference line from routing**

我现在的判断是：

1. 这两个 S 弯跑失败，核心更像是 **定位/路由匹配跳变** ，不是障碍物 stop wall。
2. 你的静态障碍物持久在线滤波，至少从这两次日志看，还 **没成为当前瓶颈** 。
3. 现阶段最该盯的是 **trajectory_stitcher** 的 **lat_diff** 为什么突然变大，以及 **lane_follow_map** 为什么找不到最近点。

关键日志位置：

* planning.log.INFO.20260504-145541.416355:73836
* planning.log.INFO.20260504-145541.416355:133314
* planning.log.INFO.20260504-135738.93908:80632

下一步我建议先顺着 **lat_diff** 和 **lane_follow_map::GetNearestPointFromRouting()** 这条线查。

**14:58**

我观察到的是与车宽相等的绿色规划出的路径在障碍物面前截断了，二者中间还有个stop墙。车沿着绿色路径行驶到了尽头，所以才会出现你说的问题。

**15:01**

对，这个观察更关键。那说明锅主要不在“车开到路径尽头以后才坏掉”，而是 **路径本身就被 stop wall 截断了** ，车只是按规划把那段绿色路径跑完了。

也就是说，现在链路更像是：

* 障碍物被认为是会阻塞的
* **PathDecider** / **PathBoundsDecider** 在前方插了 stop wall
* 绿色可行路径在障碍物前结束
* 车沿着这条路径走到头
* 后面才连带出现你之前看到的那些 **Reference lane is empty**、**lat_diff** 异常

所以你现在的判断是对的，重点应该转到：

1. **为什么这个路边障碍被判成了 blocking obstacle**
2. **为什么 lane borrow / side pass 没有接上**
3. **为什么绿色路径只给到车宽级别的窄通道，却没有继续绕过去**

这也意味着，你之前做的静态障碍物持久在线滤波是有价值的，但它只是“让障碍别闪”，**还不能解决“它被当成墙挡住整条路”**这个更核心的问题。

我建议下一步直接盯这条决策链：

* **ObstacleBlockingAnalyzer**
* **PathDecider**
* **LaneBorrowPath**

重点看这个障碍物到底是怎么被标成 **blocking_obstacle_id** 的，以及为什么没触发借道。

**15:01**

继续

**15:02**

**上 37 条消息**

我继续看了，结论更明确了。

* 绿线的截断主要发生在 **path_bounds_decider_util.cc:205** 这一层，静态障碍物会不断把边界往里压，日志里 **34376** 在同一段里反复被处理。
* stop 墙是后面 **path_decider.cc:166** 这一层加出来的，只要 **blocking_obstacle_id** 生效、又不在 lane borrow 场景，就会直接给这个障碍物下 STOP。
* **lane_follow_path.cc:167** 会把这个 blocking id 写进规划状态，所以链路就是 self-lane 边界收缩 -> blocking obstacle -> stop wall。
* 但 **lane_borrow_path.cc:499** 在最新日志里一直是 **Blocking obstacle ID[]**，说明借道分支没拿到稳定的前方静态障碍物 id，所以没真正切进去。

所以这次看下来，不只是“障碍物闪烁”这么简单。闪烁会放大问题，但核心还是静态障碍物被当成了阻挡物，而 borrow 条件没被稳定触发。你前面要的那个静态障碍物持久在线过滤器，方向是对的，它的目标就是把这个 id 稳住，让 borrow 逻辑有机会接上。

**15:08**

所以要怎么解决

**15:09**

**上 4 条消息**

核心不是去掉 stop 墙，而是让“这个静态障碍物”在连续帧里稳定存在，然后把它交给借道逻辑处理。

现在要做的顺序是：

1. 保活静态障碍物
   你要把闪烁的障碍物在 **obstacle.cc** 里缓存住，短时间掉帧也继续当作同一个障碍物。这个我已经加了。
2. 保住 **front_static_obstacle_id**
   不能一两帧没看到就把 id 清掉，不然 **lane_borrow_path.cc** 里会一直是 **Blocking obstacle ID[]**，借道永远起不来。
3. 让借道接管，而不是一直停
   只要这个障碍物连续出现到阈值，**lane_borrow_path.cc (line 484)** 就应该判定为 long-term blocking，然后进入 borrow；**path_decider.cc (line 167)** 才不会继续给它加 STOP。
4. 只在真借不过去时才停
   如果 S 弯太窄、邻车道确实借不了，那 stop 墙是正常保护，不是 bug。

一句话总结：
你这个题的解法是“静态障碍物持久在线 + 保住 blocking id + 触发 lane borrow”，不是单纯删 stop 墙。

我这边已经把第 1 步做了，下一步就是把第 2 步也接上。

**15:11**

道路本来就很窄，借道能行么？按你推荐的方案继续推进

**15:13**

**上 12 条消息**

我已经按这个方向继续了。窄路上我不再指望借道，先把 **path_bounds_decider_util.cc** 改成只把“真正挡住可行驶通道”的静态障碍物纳入路径边界，路边但不挡路的静态障碍物会直接跳过，这样就不会老是把 **blocking_obstacle_id** 顶出来、再生成 stop 墙。

配合你前面要的静态障碍物持久在线，接下来这条链路会更稳：障碍物不闪了，但也不会因为它在路边就把整条绿线截死。这个环境里我没法直接跑完整编译验证，但代码改动已经落到位了。

**1 个文件已更改**

撤销审核

**modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.cc**

**15:18**

还是生成了stop墙，绿色路径规划也在墙面前一段距离截止了

**15:23**

**上 34 条消息**

我已经把这条链路改了，不只是在路径边界里跳过，而是把“路边静态障碍物”从会立 stop 墙的三个入口里都放开了：

* **modules/planning/planning_base/common/obstacle_blocking_analyzer.cc**
* **modules/planning/planning_base/common/obstacle.cc**
* **modules/planning/tasks/path_decider/path_decider.cc**
* **modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.cc**

现在逻辑是：只要它被判成“贴着路缘/停车带的静态障碍物”，就不再算作阻塞 driving path，也不再给它建参考线 ST stop 边界，**PathDecider** 也直接忽略它，不会再补一堵墙。

我这边没法直接把整套场景跑起来验证，但下一次你回放日志时，重点看这几条应该不再出现：

* **build reference line st boundary. id:58386**
* **PathDecider/blocking_obstacle**
* **STOP58386_obs_st_bounds**

如果你再跑一轮还卡，我就继续往 **speed_bounds_decider** 和 **lane blocking** 那条线往下挖。

我观察到的是与车宽相等的绿色规划出的路径在障碍物面前截断了，二者中间还有个stop墙。车沿着绿色路径行驶到了尽头，所以才会出现你说的问题。

**15:01**

对，这个观察更关键。那说明锅主要不在“车开到路径尽头以后才坏掉”，而是 **路径本身就被 stop wall 截断了** ，车只是按规划把那段绿色路径跑完了。

也就是说，现在链路更像是：

* 障碍物被认为是会阻塞的
* **PathDecider** / **PathBoundsDecider** 在前方插了 stop wall
* 绿色可行路径在障碍物前结束
* 车沿着这条路径走到头
* 后面才连带出现你之前看到的那些 **Reference lane is empty**、**lat_diff** 异常

所以你现在的判断是对的，重点应该转到：

1. **为什么这个路边障碍被判成了 blocking obstacle**
2. **为什么 lane borrow / side pass 没有接上**
3. **为什么绿色路径只给到车宽级别的窄通道，却没有继续绕过去**

这也意味着，你之前做的静态障碍物持久在线滤波是有价值的，但它只是“让障碍别闪”，**还不能解决“它被当成墙挡住整条路”**这个更核心的问题。

我建议下一步直接盯这条决策链：

* **ObstacleBlockingAnalyzer**
* **PathDecider**
* **LaneBorrowPath**

重点看这个障碍物到底是怎么被标成 **blocking_obstacle_id** 的，以及为什么没触发借道。

**15:01**

继续

**15:02**

**上 37 条消息**

我继续看了，结论更明确了。

* 绿线的截断主要发生在 **path_bounds_decider_util.cc:205** 这一层，静态障碍物会不断把边界往里压，日志里 **34376** 在同一段里反复被处理。
* stop 墙是后面 **path_decider.cc:166** 这一层加出来的，只要 **blocking_obstacle_id** 生效、又不在 lane borrow 场景，就会直接给这个障碍物下 STOP。
* **lane_follow_path.cc:167** 会把这个 blocking id 写进规划状态，所以链路就是 self-lane 边界收缩 -> blocking obstacle -> stop wall。
* 但 **lane_borrow_path.cc:499** 在最新日志里一直是 **Blocking obstacle ID[]**，说明借道分支没拿到稳定的前方静态障碍物 id，所以没真正切进去。

所以这次看下来，不只是“障碍物闪烁”这么简单。闪烁会放大问题，但核心还是静态障碍物被当成了阻挡物，而 borrow 条件没被稳定触发。你前面要的那个静态障碍物持久在线过滤器，方向是对的，它的目标就是把这个 id 稳住，让 borrow 逻辑有机会接上。

**15:08**

所以要怎么解决

**15:09**

**上 4 条消息**

核心不是去掉 stop 墙，而是让“这个静态障碍物”在连续帧里稳定存在，然后把它交给借道逻辑处理。

现在要做的顺序是：

1. 保活静态障碍物
   你要把闪烁的障碍物在 **obstacle.cc** 里缓存住，短时间掉帧也继续当作同一个障碍物。这个我已经加了。
2. 保住 **front_static_obstacle_id**
   不能一两帧没看到就把 id 清掉，不然 **lane_borrow_path.cc** 里会一直是 **Blocking obstacle ID[]**，借道永远起不来。
3. 让借道接管，而不是一直停
   只要这个障碍物连续出现到阈值，**lane_borrow_path.cc (line 484)** 就应该判定为 long-term blocking，然后进入 borrow；**path_decider.cc (line 167)** 才不会继续给它加 STOP。
4. 只在真借不过去时才停
   如果 S 弯太窄、邻车道确实借不了，那 stop 墙是正常保护，不是 bug。

一句话总结：
你这个题的解法是“静态障碍物持久在线 + 保住 blocking id + 触发 lane borrow”，不是单纯删 stop 墙。

我这边已经把第 1 步做了，下一步就是把第 2 步也接上。

**15:11**

道路本来就很窄，借道能行么？按你推荐的方案继续推进

**15:13**

**上 12 条消息**

我已经按这个方向继续了。窄路上我不再指望借道，先把 **path_bounds_decider_util.cc** 改成只把“真正挡住可行驶通道”的静态障碍物纳入路径边界，路边但不挡路的静态障碍物会直接跳过，这样就不会老是把 **blocking_obstacle_id** 顶出来、再生成 stop 墙。

配合你前面要的静态障碍物持久在线，接下来这条链路会更稳：障碍物不闪了，但也不会因为它在路边就把整条绿线截死。这个环境里我没法直接跑完整编译验证，但代码改动已经落到位了。

**1 个文件已更改**

撤销审核

**modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.cc**

**15:18**

还是生成了stop墙，绿色路径规划也在墙面前一段距离截止了

**15:23**

**上 34 条消息**

我已经把这条链路改了，不只是在路径边界里跳过，而是把“路边静态障碍物”从会立 stop 墙的三个入口里都放开了：

* **modules/planning/planning_base/common/obstacle_blocking_analyzer.cc**
* **modules/planning/planning_base/common/obstacle.cc**
* **modules/planning/tasks/path_decider/path_decider.cc**
* **modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.cc**

现在逻辑是：只要它被判成“贴着路缘/停车带的静态障碍物”，就不再算作阻塞 driving path，也不再给它建参考线 ST stop 边界，**PathDecider** 也直接忽略它，不会再补一堵墙。

我这边没法直接把整套场景跑起来验证，但下一次你回放日志时，重点看这几条应该不再出现：

* **build reference line st boundary. id:58386**
* **PathDecider/blocking_obstacle**
* **STOP58386_obs_st_bounds**

如果你再跑一轮还卡，我就继续往 **speed_bounds_decider** 和 **lane blocking** 那条线往下挖。




看一下temp.md,这是我与上一个AI的对话，现在轮到你接力了。现在的情况是依旧卡

**15:41**

**已处理 4m 33s**

接上了，做了关键修正。

最新日志里真实 stop wall 是 **STOP92705** / **STOP89466**，障碍物在左侧 **l ~= 1.22~1.57**，左路宽约 **1.86m**，离边界只有约 **0.29m**。上一版 **IsParkedVehicle()** 只拿 **road_right_width** 判断，而且阈值只有 **0.1m**，所以这种左侧贴边障碍没有命中。

我改了这几处：

* **obstacle_blocking_analyzer.cc (line 208)**：路边静态障碍改成左右两侧分别判断，边界余量放宽到 **0.45m**，并排除虚拟障碍物，避免误伤 **DEST/PATH_END**。
* **obstacle.cc (line 436)**：路边静态障碍不再生成 reference-line ST boundary。
* **path_bounds_decider_util.cc (line 190)**：不再把非阻塞/路边静态障碍塞进路径边界收缩。
* **path_decider.cc (line 155)**：PathDecider 对路边静态障碍直接加 longitudinal/lateral ignore，防止后面的 SpeedBounds 再映射出 stop ST boundary。

我没有进容器编译，按你说的你手动编译即可；本地跑了 **git diff --check**，没有 whitespace/patch 格式问题。编译后建议重点看日志里 **STOP92705_obs_st_bounds** / **STOP89466_obs_st_bounds** 是否消失。若还出现，就说明 stop 是 SpeedBounds 的 path overlap 绕过了 ignore，我再往 **st_boundary_mapper.cc** 里补最后一道过滤。

**1 个文件已更改**

撤销审核

**modules/planning/planning_base/common/obstacle_blocking_analyzer.cc**

**15:46**

/home/skye/下载/log/下有评测系统给出的部分赛题的log,先看S弯log,现在已经可以经过S弯正常到达终点了，但是碰撞检测没过。分析一下评测日志和本地日志（最后两次）。说明：S弯场景有两个

**16:31**

**已处理 5m 27s**

我看完了，结论很明确: 现在不是“过不去弯”，而是“过得去，但把该看的静态障碍放掉了，所以最后撞了”。

评测包两条 S 弯记录都一致，**Collision** 失败，但 **OnRoad**、**ReachEnd**、**SpeedLimit**、**TimeLimit** 都过了。两个失败时间点分别落在 **28.2/28.4s** 和 **28.6/28.8s**，最后都是 **score: 0.0**、**finish_fail**。开头还有 **planning has no trajectory point** 和缺 **rule_based_stop_decider.pb.txt** 的噪声，但这不是最终判分主因。

本地最后两次也指向同一个方向。**PlanOnReferenceLine done, has_error=0** 说明主链路已经能跑起来，但 **Blocking obstacle ID[]** 经常是空，借道/阻塞判断没有真正抓到 blocker。后半段又反复出现 **Reference lane is empty!**，其中较早那次后半段还退化到了 **has_error=1 cost=20010**。也就是说，现在卡点更像是参考线和障碍物分类的不稳定，不是纯路径生成失败。

我更倾向于，你现在这版把“贴边静态障碍”在 obstacle_blocking_analyzer.cc、obstacle.cc、path_bounds_decider_util.cc、path_decider.cc 这四条链路里一起跳过了。对这类 S 弯，评测里的障碍很可能本来就需要被绕开，而不是直接当 parked 忽略，所以车能到终点，但仿真碰撞仍会被记分。两个 S 弯场景都中同一类问题，说明不是偶发，是这条过滤策略过宽。

**16:37**

所以应该怎么修改？并做出修改

**16:39**

**已处理 11m 59s**

我已经改了，方向是“别把路边障碍直接放掉，但给绕行多留一点余量”。

* **obstacle_blocking_analyzer.cc (line 184)**: 去掉了 **IsBlockingDrivingPathObstacle()** 里对 parked 障碍的短路，让它们还能进入 blocking/借道判断。
* **planning_gflags.cc (line 138)**: **static_obstacle_nudge_l_buffer** 从 **0.3** 调到 **0.45**。
* **path_decider/default_conf.pb.txt (line 1)**: **static_obstacle_buffer** 从 **0.3** 调到 **0.45**。

这样两条 S 弯共用的默认绕障余量会更大，通常能把“擦碰”那一下推开一点，同时我没动你现在那条防 stop wall 的 parked ST boundary 处理。**git diff --check** 已经过了，我没跑全编译，你本地编完后先复测这两条 S 弯的 **Collision**。

**3 个文件已更改**+3**-7**

撤销审核

**modules/planning/planning_base/common/obstacle_blocking_analyzer.cc**

**modules/planning/planning_base/gflags/planning_gflags.cc**

**modules/planning/tasks/path_decider/conf/default_conf.pb.txt**

**16:51**

先不管S弯了，先看一下变道赛题。现在在的逻辑不行，发生了碰撞。我想要的逻辑是，他不去看前方的车，也不去看后方的车。而只关心横向距离上的车。看一下temp.md,这是我与上一个AI的对话，现在轮到你接力了。从车头前方 0.2 米到车尾后方。两米之内，横向距离上 0.5 米之内，如果没有车，就进行变道，如果有车，那就直线行驶。就是那种变道变了一半，有车了，那就直线行驶，这里说的直线行驶不是在回本车道的中间，而是变道变一半那个位置继续直线行驶。然后，一旦没车，横向距离上这个区间内没车，那就继续变道。满足条件就一直保持变道，不要取消。现在的好像并不是

**16:59**

**已处理 23m 25s**

我改的是当前日志里实际在跑的 **LaneChangePath**，文件在 **lane_change_path.cc**。

现在逻辑变成了：

* 只看自车前方 **0.2m** 到后方 **2m** 的车，横向 **0.5m** 内有车就判为不清晰
* 不清晰时，不再回本车道中线，而是把整条路径锁在当前 **l** 附近直行
* 清晰时继续往目标车道变
* 变道路径一旦生成，会把障碍物纵向/横向都 ignore，避免后面的 stop/follow 再把绿色路径截断
* **IN_CHANGE_LANE** 里也不再强行把 clear 置真

我没跑整编，但 **git diff --check** 已经过了。你手动编译后，重点看日志里的 **[LC_CLEAR]**、**[LC_BOUNDS]**、**[LC_OPT]**，应该能直接看到是“继续变道”还是“横向保持”。

**1 个文件已更改**

撤销审核

**modules/planning/tasks/lane_change_path/lane_change_path.cc**

**17:23**

ok，变道没问题啦，但是S弯的两个场景只通过了一个，另一个还是没过，评测日志已更新，你研究一下。

**18:09**

**已处理 10m 14s**

已研究并改完。

失败的是 **3420** 场景：其余指标都过，只在 **Collision** 挂了，碰撞时间集中在 **17.5~17.7s** 和 **28.6~28.8s**，对象分别是小静态障碍 **92878**、**78284**。**3421** 场景是 100 分通过。规划日志里这两个障碍都在横向左侧，但原路径在弯中贴得太近，属于 S 弯锥桶场景横向余量不够。

我改了两处：

1. **lane_follow_path.cc (line 41)**
   新增 S 弯锥桶密集检测：高曲率 + 前方 90m 内小障碍物足够多时，静态障碍物边界计算期间把 **obstacle_lat_buffer** 临时抬到 **0.65**，让路径离锥桶更远。变道 reference line 仍然会提前跳过，不影响刚修好的变道。
2. **path_bounds_decider_util.cc (line 301)**
   静态障碍物更新左右边界时补上 **is_nudge_bound** 标记，避免后续边界松弛逻辑把这个障碍物约束当成普通边界处理。

检查：**git diff --check** 已通过。完整编译我没跑，按你说的你手动编译即可；编译后记得恢复 profile：**aem profile use default**。




看一下temp.md,这是我与上一个AI的对话，现在轮到你接力了。接下来看一下环岛的log，显示向心加速度超了

**18:59**

**已处理 4m 35s**

接上了，环岛这次不是碰撞、到达、限速问题，评测只挂在 **CentripetalAccelerationLimit**。评测日志里向心加速度在 **45.9s ~ 47.7s** 短暂超过阈值，最高看到 **2.287413**，看起来阈值约是 **2.0**。

我做了一个窄修，只动速度边界：

* **default_conf.pb.txt (line 3)**：**max_centric_acceleration_limit** 从 **1.45** 降到 **1.20**
* **speed_limit_decider.cc (line 39)**：曲率限速提前看前方 **12m** 的最大曲率，避免进弯后才降速
* **speed_limit_decider.cc (line 77)**：新增 **speed_limit_preview_kappa** 日志点，方便复测时确认是否提前压速

按比例估算，单是 **1.45 -> 1.20** 就会把同曲率下的向心加速度压到约 **82.8%**，那次最高 **2.287** 会落到约 **1.89**，再加上提前预瞄和现有速度优化器余量，应该能过这一项。

验证：**git diff --check** 通过。本机没有 **buildtool** / **bazel**，所以没跑编译。编译后重点看环岛评测里的 **CentripetalAccelerationLimit**，以及 planning 日志里的 **speed_limit_preview_kappa** 和 **curr_speed_limit**。

**2 个文件已更改**+21**-2**

撤销审核

**modules/planning/tasks/speed_bounds_decider/conf/default_conf.pb.txt**

**modules/planning/tasks/speed_bounds_decider/speed_limit_decider.cc**

**19:04**

然后看一下当前的日志，我跑了两次没有对向车辆的U型掉头，你看看那些stop墙是咋回事

**19:22**

**已处理 8m 56s**

我看了两次 planning 子日志，**第一条 (line 19215)** 和 **第二条 (line 23311)**。结论很一致：这次的 **stop wall** 不是对向车，也不是 blocking obstacle。两次里 **lane_borrow_path** 都是 **Blocking obstacle ID[ ]** 空的，**PathDecider/blocking_obstacle** 也没命中。真正把绿线截断的是 **PATH_END_regular/self**，这个 ID 只会在 **RuleBasedStopDecider::AddPathEndStop (line 121)** 里生成，默认阈值就是 **short_path_length_threshold: 20.0 (line 8)**。

同一帧里还有一个 **REF_END_0_1**，这是 **ReferenceLineEnd (line 43)** 生成的参考线末端墙，reason 是 **STOP_REASON_DESTINATION**。日志里虽然有 **YS_YieldSign_2** 的 ST boundary，但我没看到 **BuildStopDecision: yield_sign**，所以它不是这次主因。

所以你看到的那堵墙，本质上是规划把这段 U-turn 切成了短路径，车把绿线跑到头后撞上了 **PATH_END** fence。下一步该查的是为什么这条 U-turn 会被判成 short path。

**19:31**

看最新的一次日志（注意评测日志尚未更新，不用看），看看在对向车道有车时发生了什么？？？实际上，在对向车道没车时，车是在弯道一点一点前进的，因为有stop墙阻拦。而对向车道有车时，车就彻底走不动了

**19:52**

**已处理 13m 44s**

我看的是本地最新这份 **planning.log.INFO.20260504-194731.2049194**。

* 这次对向车出现后，规划还是停在 **regular/self**，**Blocking obstacle ID[]** 依旧是空的，所以 lane borrow 根本没起来。空 blocker 会把借道逻辑卡住，见 **lane_borrow_path.cc (line 484)**。
* 你看到的两堵 stop 墙分别是 **PATH_END_regular/self** 和 **REF_END_1_0**。前者是 **rule_based_stop_decider.cc (line 121)** 里按短路径自动补的墙，阈值是 **short_path_length_threshold: 20.0 (line 8)**；后者是 **reference_line_end.cc (line 59)** 里按 **min_reference_line_remain_length: 50.0 (line 1)** 加的终点墙。那一帧 self path 只有大约 7m，墙就会贴得很近。
* 速度上限其实还在，**curr_speed_limit** 约 2.34 m/s，**nearby_obstacles** 约 4.83 m/s，不是限速直接归零；真正把车按住的是 ST 约束。日志里还冒出一条 **YIELD30014_0** 边界，像是额外的对向车约束，随后 **piecewise_jerk_speed_optimizer.cc (line 239)** 报了 **init state not appropriate ... adjust acc to 0**，所以 **vt** 曲线几乎贴 0。
* 所以这次不是“对向车成功绕过去了”，而是“对向车来了以后没变成可借道 blocker，self path 被压短，stop/end/yield 叠在一起，车就被锁住了”。后面日志里还有一帧恢复正常轨迹，说明不是整体崩了，是这个场景下约束在抖。

**20:05**

在掉头时规划出来的轨迹很切内弯怎么办？

**20:12**

**已处理 11m 20s**

结论先说：这类 U-turn 切内弯，优先怀疑是“场景/ROI 太贴内 + warm start 太想走短路”，不是控制器本身。你现在改的 **lane_follow_path.cc** 那套 dense-cone S 弯逻辑，只管普通 lane-follow，不会直接修掉头这条链路。

我建议按这个顺序看：

1. 先确认有没有真的切进 **LargeCurvatureScenario**。它是按前方 3m 曲率和剩余参考线长度切场景的；如果没切进去，就还在 lane-follow / lane-borrow 里，得动 **lane_follow_path.conf** / **lane_borrow_path_generic.conf** 里的路径权重，不是 open-space 配置。
2. 如果确实是 large_curvature，先改 **open_space_roi_decider_park.pb.txt**：把 **use_road_boundary_from_map** 打开，必要时把 **roi_longitudinal_range_start/end** 和障碍 buffer 稍微加大。这个最像“把外侧空间补出来”。
3. 再看 **open_space_path_planning.pb.txt**：把 **traj_kappa_contraint_ratio** 往下收一点，**soft_boundary_penalty** 往上提一点；还贴内就再加一点 **traj_steer_penalty** / **traj_steer_change_penalty**。
4. 还不够的话，再看 **open_space_trajectory_optimizer.pb.txt**：轻微提高 **weight_y** / **weight_phi**，让平滑器别死贴 warm start。

我在最新 planning log 里没抓到明确的 open-space 运行段，所以更倾向先核对场景切换，再动参。确认是 **large_curvature** 的话，我可以直接帮你把这几处参数收成一版更外扩的配置。

**20:24**

20:13:04:333 LANE_FOLLOW STAGE
20:13:26:074 YIELD_SIGN APPROACH
20:13:28:567 YIELD_SIGN CREEP
20:13:32:762 LANE_FOLLOW STAGE

看一下这些，这个是U型弯对向车道无车时的场景历史，根据调试日志看看能不能帮到你，看看为啥掉头时在一点一点前进。

**20:27**

**已处理 6m 2s**

有帮助，基本能定性了。

这次不是“对向车挡死了”，而是这段 U 型弯被切进了 **YIELD_SIGN**，然后走的是 **APPROACH -> CREEP** 逻辑。日志里很清楚：**20:13:25** 切到 **YIELD_SIGN**，**20:13:28** 切到 **YIELD_SIGN_CREEP**，**20:13:32** 还在出 **STOPCREEP_YieldSign_2**，同时又建了 **PATH_END_regular/self** 停墙。**Blocking obstacle ID[]** 还是空的，说明这次压车的主因不是障碍物，而是规则生成的 stop boundary。

所以它会“一点一点前进”，是因为每一帧都只允许在这个 **yield creep** 里挪一点，停墙的 **s** 也在往前推。再叠上你这条弯道本身很紧，**print_speed_limit_preview_kappa** 大约在 **0.2**，限速已经压到约 **2.45 m/s**，车看起来就更像在小碎步挪。你前面把向心加速度限得更保守，会放大这个现象，但主因还是 stop wall 和场景切换。

我推断，对向车出现时，原本就很窄的 creep 区间再被动态约束挤一下，速度优化就可能直接压到 0，所以就表现成“彻底走不动”。后面还有一次 **lat_diff = -12.0175** 的重规划，说明参考线/定位匹配也有点飘，会把这个卡顿再放大。

下一步最该查的是：为什么这段 U-turn 会进 **YieldSignScenario**，以及为什么 turn 区域会反复生成 **PATH_END_regular/self** 终点墙。对应入口在 **yield_sign pipeline**，限速侧可以看 **speed_bounds_decider 默认配置**。

**20:33**

为什么这段 U-turn 会进 YieldSignScenario，以及为什么 turn 区域会反复生成 PATH_END_regular/self 终点墙。20:13:26:074 YIELD_SIGN APPROACH
20:13:28:567 YIELD_SIGN CREEP分别啥意思

**20:34**

**已处理 5m**

结论先说：这段 U-turn 进 **YieldSignScenario**，不是因为“是掉头”，而是因为这条参考线上前方第一个 overlap 被识别成了 **YIELD_SIGN**，而且距离小于 10m。场景管理器又是按优先级 first-match，**YIELD_SIGN** 排在 **LANE_FOLLOW** 前面，所以一命中就切进去了。看 **yield_sign_scenario.cc (line 57)**、**public_road_planner_config.pb.txt (line 21)**、**planning_component/README_cn.md (line 160)**。

**20:13:26:074** 对应的是 **YIELD_SIGN_APPROACH**，意思是“到让行线前先停住观望”。它只看有没有进入让行标记附近，不要求一定有对向车；它用的是 **max_valid_stop_distance = 4.5m** 这个门槛。**20:13:28:567** 切到 **YIELD_SIGN_CREEP**，意思是“慢慢探头通过让行区”。这一步会继续插 creep 虚拟墙，最多 creep 10 秒，连续 5 帧都觉得安全才结束。见 **stage_approach.cc (line 37)**、**stage_creep.cc (line 54)**、**base_stage_creep.cc (line 41)**、**yield_sign.proto (line 7)**、**scenario_conf.pb.txt (line 1)**。

你日志里的 **PATH_END_regular/self** 是另一条墙，不是障碍物，也不是让行标志本体。它来自 **rule_based_stop_decider.cc (line 121)**：只要当前 path label 非空、而且路径长度小于 **short_path_length_threshold = 20m**，就会补一个 **PATH_END_...** 的终点停墙。**lane_follow_path** 又把自车路径标成了 **regular/self**，所以在这个很窄的 U-turn 里，这个短路径规则会一帧一帧反复触发。见 **lane_follow_path.cc (line 208)**、**rule_based_stop_decider.proto (line 20)**。

所以现在是两层叠加：**YieldSign** 本身就在让车慢慢通过，**PATH_END_regular/self** 又不断把路切短，结果就变成一点一点往前挪。你给的 **20:13:32:762 LANE_FOLLOW STAGE** 基本就是这轮让行流程退出后回到常规流程了。下一步最该查的是这条 U-turn 路线上到底是哪一个 **YIELD_SIGN** overlap 把它抢进场景的。
