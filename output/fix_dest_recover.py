#!/usr/bin/env python3
"""修复: 恢复DEST约束 + v_ref用实际巡航(消除终点急刹+冲过DEST)"""
p = 'modules/planning/tasks/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.cc'
t = open(p, encoding='utf-8').read()

# 修改1: 跳过逻辑 b0 > total_length-10 -> b0 > total_length (只跳过参考线外)
old1 = '''                    // 2026-08-04晚: 参考线末端附近的STOP/YIELD(如DEST@129.5 vs
                    // 末端130)不约束——车15s窗口按dx_ref巡航会到达boundary,
                    // 但车到参考线末端就切换(kRefEndOvershoot), 不需在末端停.
                    // 原v_ref=20最晚刹车曲线假设车20巡航过早收紧 → s_bounds
                    // 与车实际(限速4.5+dx_ref16.39)冲突 → QP infeasible
                    // (24.68s实测) → relaxed也失败 → fallback急刹(刹油刹#2).
                    // 跳过末端附近boundary: 车正常巡航切参考线; 真实终点
                    // (DEST深入参考线内)STOP正常生效停车.
                    double b0 = 0.0, bl0 = 0.0;
                    boundary->GetUnblockSRange(0.0, &b0, &bl0);
                    if (b0 > total_length - 10.0) {
                        continue;
                    }'''

new1 = '''                    // 2026-08-04晚2: 只跳过参考线外(b0>total_length)的STOP/YIELD
                    // ——车切参考线不停. 原跳过末端附近(-10)导致DEST被忽略→车
                    // 出限速区后按dx_ref=16.39加速→DEST前31m(34.5s)才急刹
                    // (DP gridded_path_time_graph找不到轨迹→fast stop→停DEST
                    // 后起步冲过9.5m到y4438369, 终点线停车停过了).
                    // v_ref改实际巡航(16.39)后, DEST在参考线内时s_bounds最晚
                    // 刹车曲线不冲突(车26s出区加速→29s撞曲线平滑减速→停DEST),
                    // 不刹油刹不冲过. 24.6s刹油刹#2(v_ref=20曲线过早)已由
                    // v_ref=16.39解决.
                    double b0 = 0.0, bl0 = 0.0;
                    boundary->GetUnblockSRange(0.0, &b0, &bl0);
                    if (b0 > total_length) {
                        continue;
                    }'''

assert t.count(old1) == 1, 'old1 count=%d' % t.count(old1)
t = t.replace(old1, new1)

# 修改2: v_ref 用实际巡航 min(20, default_cruise_speed=16.39)
old2 = '''                {
                    const double v_ref = FLAGS_planning_upper_speed_limit;'''

new2 = '''                {
                    // 2026-08-04晚2: v_ref用实际巡航min(20,default_cruise_speed
                    // =16.39), 不用20. 原20假设车20巡航→曲线在DEST前66.7m就
                    // 开始收紧→与车实际(限速区4.5+出区16.39)不符→24.6s QP
                    // infeasible(刹油刹#2). 16.39: 车DEST前44.7m才开始减速,
                    // 车26s出限速区加速→29s撞曲线平滑减速→停DEST, 不冲突.
                    const double v_ref = std::fmin(FLAGS_planning_upper_speed_limit,
                                                   FLAGS_default_cruise_speed);'''

assert t.count(old2) == 1, 'old2 count=%d' % t.count(old2)
t = t.replace(old2, new2)

open(p, 'w', encoding='utf-8').write(t)
print('OK')
