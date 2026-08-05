#!/usr/bin/env python3
"""修复: QP relaxed也失败时保留speed_data(避免fast stop急刹)"""
p = 'modules/planning/tasks/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.cc'
t = open(p, encoding='utf-8').read()

old = '''        piecewise_jerk_problem.set_dx_bounds(std::move(relaxed_dx_bounds));
        if (!FLAGS_speed_optimize_fail_relax_velocity_constraint || !piecewise_jerk_problem.Optimize()) {
            speed_data->clear();
            print_debug.AddPoint("optimize_st_curve", 0, init_s[0]);'''

new = '''        piecewise_jerk_problem.set_dx_bounds(std::move(relaxed_dx_bounds));
        if (!FLAGS_speed_optimize_fail_relax_velocity_constraint || !piecewise_jerk_problem.Optimize()) {
            // 2026-08-04: 不清空 speed_data(保留上一帧轨迹)——避免 QP 数值不
            // 收敛(maximum iterations)时 TrajectoryFallbackTask 用
            // GenerateFallbackSpeed(FLAGS_speed_fallback_distance=3m)急刹
            // (刹油刹#3, 本地scn4 15:59:58.78实测: 车4.5巡航被fast stop
            // 3m内刹停到0.16). 保留上一帧轨迹→车平滑延续→下帧QP恢复.
            // relaxed对init_v<v_upper帧无效(v_upper=max(4.5,init_v-6t)仍4.5)
            // → 保留旧轨迹是唯一平滑出路. 真实STOP(行人/终点)由s_bounds+
            // P0 v_upper保证, 旧轨迹不会冲过(车已在STOP前减速).
            // speed_data->clear();
            print_debug.AddPoint("optimize_st_curve", 0, init_s[0]);'''

assert t.count(old) == 1, 'count=%d' % t.count(old)
t = t.replace(old, new)
open(p, 'w', encoding='utf-8').write(t)
print('OK')
