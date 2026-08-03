/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file piecewise_jerk_fallback_speed.cc
 **/

#include <algorithm>

#include <string>
#include <utility>
#include <vector>
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/common/st_graph_data.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "modules/planning/tasks/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

bool PiecewiseJerkSpeedOptimizer::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    if (!SpeedOptimizer::Init(config_dir, name, injector)) {
        return false;
    }
    // Load the config_ this task.
    return SpeedOptimizer::LoadConfig<PiecewiseJerkSpeedOptimizerConfig>(&config_);
}

Status PiecewiseJerkSpeedOptimizer::Process(
        const PathData& path_data,
        const TrajectoryPoint& init_point,
        SpeedData* const speed_data) {
    if (reference_line_info_->ReachedDestination()) {
        return Status::OK();
    }

    ACHECK(speed_data != nullptr);
    SpeedData reference_speed_data = *speed_data;

    if (path_data.discretized_path().empty()) {
        const std::string msg = "Empty path data";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
    PrintCurves print_debug;
    const auto& veh_param = common::VehicleConfigHelper::GetConfig().vehicle_param();

    std::array<double, 3> init_s = {0.0, st_graph_data.init_point().v(), st_graph_data.init_point().a()};
    const auto& vehicle_state = frame_->vehicle_state();
    if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
        init_s[1] = std::max(-init_s[1], 0.0);
        init_s[2] = -init_s[2];
        AINFO << "transfer reverse speed" << init_s[0] << "," << init_s[1] << "," << init_s[2];
    }
    double delta_t = 0.1;
    double total_length = st_graph_data.path_length();
    double total_time = st_graph_data.total_time_by_conf();
    int num_of_knots = static_cast<int>(total_time / delta_t) + 1;
    print_debug.AddPoint("optimize_st_curve", 0, init_s[0]);
    print_debug.AddPoint("optimize_vt_curve", 0, init_s[1]);
    print_debug.AddPoint("optimize_at_curve", 0, init_s[2]);
    // Update STBoundary
    const double kEpsilon = 0.01;
    std::vector<std::pair<double, double>> s_bounds;
    // 停止线位置(STOP/YIELD边界, s_bounds上界), 用于dx_ref'一脚刹死'处理
    double stop_s = std::numeric_limits<double>::max();
    for (int i = 0; i < num_of_knots; ++i) {
        double curr_t = i * delta_t;
        double s_lower_bound = 0.0;
        // 参考线末端(2026-08-04调优): 参考线每帧从车位置重建(车恒在 s=0, 距末端
        // total_length≈127m), 末端不是停车点(车会平滑切换参考线继续开)。
        // 原 total_length 硬约束(2026-08-03前) → QP 知道 10s 窗口内 s≤127 提前缓降;
        // 7bda67d 改末端最晚刹车(假设16从s=0巡航) → 对起步中的车(v0<16)仍太紧:
        // 车 v0=12 加速到16后位置超前(122.6@8s)超曲线(122.2) → QP 停止加速
        // (092945实测车12.7封顶 → 刹-油-刹 + infeasible 55次)。
        // 修复: 放宽 s_bounds 上界到 total_length+窗口巡航距离(车16巡航通过末端,
        // 切换参考线继续)。真实停车点(行人/终点)由 STOP 分支(fmin 收紧)+P0
        // (v_upper 按 sqrt(2|dec|·dist) 收紧)控制 → 车在真实 STOP 前最晚刹车, 安全。
        const double kRefEndOvershoot =
                FLAGS_planning_upper_speed_limit * (num_of_knots * delta_t);
        double s_upper_bound = total_length + kRefEndOvershoot;
        for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
            double s_lower = 0.0;
            double s_upper = 0.0;
            if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
                continue;
            }
            switch (boundary->boundary_type()) {
            case STBoundary::BoundaryType::STOP:
            case STBoundary::BoundaryType::YIELD:
                // 最晚刹车(2026-08-03 v3): 平滑分段——消除突变infeasible。
                // v1(恒定stop_s)让QP提前减速(刹-油-刹); v2(突变)在t_brake处
                // s_upper从total_length突降stop_s → QP primal infeasible
                // (025136实测89次, 车停在斑马线前跑一半)。v3平滑:
                //   t<t_brake: 自由巡航(车以v_ref到stop_s-v_ref²/2|dec|)
                //   t_brake≤t≤T: s_upper=stop_s-0.5|dec|(T-t)² 平滑降(车-6急刹)
                //   t>T: s_upper=stop_s(已停) —— 无突变, QP可行。
                {
                    // 2026-08-04晚2: 只跳过参考线外(b0>total_length)的STOP/YIELD
                    // ——车切参考线不停. 原跳过末端附近(-10)导致DEST被忽略→车
                    // 出限速区后按dx_ref=16.39加速→DEST前31m(34.5s)才急刹
                    // (DP找不到轨迹→fast stop→停DEST后起步冲过9.5m, 终点停车
                    // 停过了). v_ref改实际巡航(16.39)后, DEST在参考线内时
                    // s_bounds最晚刹车曲线不冲突(车26s出区加速→29s撞曲线平滑
                    // 减速→停DEST), 不刹油刹不冲过. 24.6s刹油刹#2(v_ref=20
                    // 曲线过早)已由v_ref=16.39解决.
                    double b0 = 0.0, bl0 = 0.0;
                    boundary->GetUnblockSRange(0.0, &b0, &bl0);
                    if (b0 > total_length) {
                        continue;
                    }
                    // 2026-08-04晚2: v_ref用实际巡航min(20,default_cruise_speed
                    // =16.39), 不用20. 原20假设车20巡航→曲线在DEST前66.7m就
                    // 开始收紧→与车实际(限速区4.5+出区16.39)不符→24.6s QP
                    // infeasible(刹油刹#2). 16.39: 车DEST前44.7m才开始减速,
                    // 车26s出限速区加速→29s撞曲线平滑减速→停DEST, 不冲突.
                    const double v_ref = std::fmin(FLAGS_planning_upper_speed_limit,
                                                   FLAGS_default_cruise_speed);
                    // 2026-08-04: 刹车减速度 6.0→3.0(jerk等效)。jerk±2约束下
                    // 车16→0实际需~43m(a从0以jerk-2到-6走39m + -6急刹4m),
                    // 等效 dec=16²/(2·43)≈2.97≈3.0。原用6.0(21.3m)太紧——
                    // QP在21.3m内无法jerk刹停 → 提前减速封顶(095032实测车
                    // 12.4不加速到16, v[30]=15.57→14.32提前降)。用3.0: 车16
                    // 巡航到距STOP 42.7m处-3减速停(最晚刹车, jerk可达)。
                    const double dec_abs = 3.0;
                    const double t_brake = (s_upper - init_s[0] - v_ref * v_ref / (2.0 * dec_abs)) / v_ref;
                    if (t_brake <= 0.0) {
                        // 车已在刹车距离内(stop_s<v_ref²/2|dec|): 恒定stop_s约束
                        s_upper_bound = std::fmin(s_upper_bound, s_upper);
                    } else {
                        const double T_ = t_brake + v_ref / dec_abs;  // 刹停时刻
                        if (curr_t < t_brake) {
                            // 自由巡航(保持 total_length, 车受dx_bounds速度约束)
                        } else if (curr_t <= T_) {
                            // 最晚刹车曲线(平滑): s_upper = stop_s - 0.5|dec|(T-t)²
                            const double s_late = s_upper - 0.5 * dec_abs * (T_ - curr_t) * (T_ - curr_t);
                            s_upper_bound = std::fmin(s_upper_bound, std::max(s_late, 0.0));
                        } else {
                            s_upper_bound = std::fmin(s_upper_bound, s_upper);
                        }
                    }
                    // 排除蠕行STOP(CREEP_前缀): 蠕行需低速通过路口, 若设dx_ref=0会
                    // 急刹停在蠕行目标前+等动态障碍物(本地实测停车8.4s vs 云端1.8s)
                    if (boundary->id().find("CREEP_") == std::string::npos) {
                        stop_s = std::min(stop_s, s_upper);  // 记录停止线位置
                    }
                }
                break;
            case STBoundary::BoundaryType::FOLLOW:
                // TODO(Hongyi): unify follow buffer on decision side
                s_upper_bound = std::fmin(s_upper_bound, s_upper);
                break;
            case STBoundary::BoundaryType::OVERTAKE:
                s_lower_bound = std::fmax(s_lower_bound, s_lower);
                break;
            default:
                break;
            }
        }
        s_upper_bound = std::fmax(s_upper_bound, s_lower_bound + kEpsilon);
        print_debug.AddPoint("st_bounds_lower", curr_t, s_lower_bound);
        print_debug.AddPoint("st_bounds_upper", curr_t, s_upper_bound);
        if (s_lower_bound > s_upper_bound) {
            const std::string msg = "s_lower_bound larger than s_upper_bound on STGraph";
            AERROR << msg;
            speed_data->clear();
            print_debug.PrintToLog();
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }

        s_bounds.emplace_back(s_lower_bound, s_upper_bound);
    }

    // Update SpeedBoundary and ref_s
    std::vector<double> x_ref(num_of_knots, total_length);
    std::vector<double> dx_ref(num_of_knots, reference_line_info_->GetCruiseSpeed());
    std::vector<double> dx_ref_weight(num_of_knots, config_.ref_v_weight());
    std::vector<double> penalty_dx;
    std::vector<std::pair<double, double>> s_dot_bounds;
    const SpeedLimit& speed_limit = st_graph_data.speed_limit();
    // P1-E v3(2026-08-03): 物理可达位置累积初始化——替代DP超前参考x_ref。
    // s_est/v_est: 车从本帧实际速度v0按+3加速的可达位置/速度(物理上限,不超前)。
    const double v0 = std::max(0.0, init_s[1]);
    double s_est = init_s[0];
    double v_est = v0;
    for (int i = 0; i < num_of_knots; ++i) {
        double curr_t = i * delta_t;
        // get path_s
        SpeedPoint sp;
        reference_speed_data.EvaluateByTime(curr_t, &sp);
        const double path_s = sp.s();
        x_ref[i] = path_s;
        // get curvature
        PathPoint path_point = path_data.GetPathPointWithPathS(path_s);
        penalty_dx.push_back(std::fabs(path_point.kappa()) * config_.kappa_penalty_weight());
        // get v_upper_bound
        const double v_lower_bound = 0.0;
        double v_upper_bound = FLAGS_planning_upper_speed_limit;
        v_upper_bound = std::fmin(speed_limit.GetSpeedLimitByS(path_s), v_upper_bound);
        // 限速预减速(2026-08-02修复): 限速段边界 v_upper 硬突变(16.667→4.5) →
        // QP 需1个节点(0.1s)内降速(a≈-104超acc限制) → primal infeasible(实测75次)
        // → fallback 用 max(16.667,init_v) 抹掉限速 → 车14.9m/s全速通过Signal_5
        // 路口(评测限速5, 超速扣分)。这里对前方每个限速点(lv@ls)用减速度 dec 回推
        // 本点允许速度: v_allow = sqrt(lv² + 2*dec*(ls-path_s)), v_upper 取最小值。
        // 车在限速区前提前减速, 进入限速区时已达标 → QP 全程可行(不fallback)。
        // 回推减速度必须<fallback减速度(5<6): v_allow<init_v 在限速区前23.6m触发
        // (16m/s@5m/s²), fallback用6m/s²更快减速 → 到限速区≤4.5。
        // P0-B(2026-08-03): kLimitDecel 4.0→5.0——回推起点从32m(4m/s²)前移到23.6m,
        // 减轻"提前减速"(机制专项Q1根因: 限速区前30m+就被硬性要求4m/s²减速)。
        static constexpr double kLimitLookAhead = 100.0;
        // 2026-08-04晚: kLimitDecel 4.0→2.5(scn4本地刹油刹#1根因). 4.0回推
        // 仍太紧: v_upper包络以4.0/s降, 车jerk约束实际只能~3.5/s减速, 且
        // 控制执行滞后0.4s(车晚6.4m开始, 初始超包络~1.6m/s) → 车持续超
        // v_upper(init_v>v_up[0], 实测14.6s超0.28→9.2s超2.0) → QP
        // infeasible(91次) → fallback急刹 → 刹过头到0.64(刹油刹#1).
        // 2.5回推47m: 车实际3.5减速>包络2.5, 追得上 → 进限速区≤4.5稳态,
        // 不超速不infeasible. 减速更早但平滑(-3.5), 不刹油刹.
        static constexpr double kLimitDecel = 2.5;
        for (const auto& lp : speed_limit.speed_limit_points()) {
            if (lp.first <= path_s) {
                continue;
            }
            if (lp.first - path_s > kLimitLookAhead) {
                break;  // 限速点按 s 有序
            }
            const double v_allow = std::sqrt(lp.second * lp.second + 2.0 * kLimitDecel * (lp.first - path_s));
            v_upper_bound = std::fmin(v_upper_bound, v_allow);
        }
        // P0-A(2026-08-03): 限速回推只压【硬边界 s_dot_bounds/v_upper】，不再压软
        // dx_ref。dx_ref 保持巡航速度(GetCruiseSpeed)，限速区减速完全由硬约束驱动
        // ——消除机制专项Q1确认的"提前减速"软目标牵引(dx_ref被限速回推拉低→QP无
        // 动力回冲→提前减速)。安全性不变: s_dot_bounds 硬约束(v_upper)仍在。
        // P1-E v3(2026-08-03): 连续刹车包络, 基于'物理可达位置's_est累积。
        // dx_ref=min(巡航, √(2·max_dec·max(0, stop_s-s_est))) —— s_est从本帧车
        // 实际速度v0按+3加速累积(物理可达,不超前), 包络在车真正接近stop_s-21.3m
        // 才压 → QP 巡航到最晚刹车点再-6急刹。
        // 根因(023701 pjs2): v1用DP参考x_ref——DP先于speed_decider运行不知道STOP,
        // 参考x_ref超前接近stop_s(x_ref[40]=25.1 vs stop_s=33.4) → 包络提前压
        // dx_ref[40]=10 → QP跟随提前急刹(距停止线~50m就刹到1) → 刹-油-刹。
        // ⚠️v2(v0*t直线)末端陡降→infeasible 282次已回退; v3用v_est累积平滑, 无陡降。
        if (stop_s < total_length) {
            const double dist_to_stop = std::max(0.0, stop_s - s_est);
            // 2026-08-04: 同STOP分支改jerk等效减速度3.0(车16→0需~43m)。
            // 原6.0(21.3m)包络太紧 → QP提前减速封顶(095032实测12.4)。
            // 2026-08-04晚2: 改jerk精确——低v时sqrt(2·3·dist)太松(车v=1.11
            // 只距STOP 0.21m<0.78m jerk最小距离) → QP无法jerk刹停 → 终点段
            // infeasible滑行(101332)。jerk精确: 低v纯jerk段(1.5·d)^(2/3),
            // 高v jerk过渡(3s到-6)后-6急刹。与P0 v_upper一致。
            const double brake_envelope = (dist_to_stop < 18.0)
                    ? std::pow(1.5 * dist_to_stop, 2.0 / 3.0)
                    : -9.0 + std::sqrt(108.0 + 12.0 * dist_to_stop);
            dx_ref[i] = std::fmin(dx_ref[i], brake_envelope);
        }
        // P0(2026-08-04修复): 接近STOP时同步收紧v_upper——消除infeasible根因。
        // 032120实测: 终点段参考线缩到15-21m, stop_s=5-11m, 车高速(9-10.5m/s)接近
        // 但 v_up[0] 仍=16(不收) → QP无法在stop_s前刹停 → primal infeasible(22次)
        // → fallback放宽 → 车冲过STOP/终点 → 终点倒溜(v=-0.118, 冲过DEST 0.19m)。
        // 修复: v_upper按最晚刹车距离收紧 v_allow_stop=√(2|dec|·(stop_s-s_est)),
        // 与s_bounds最晚刹车曲线自洽(车到stop_s时v=0) → QP全程可行不fallback,
        // 车在STOP/终点前刹住不倒溜。车距STOP>42.7m时 v_allow>16 不压(不影响
        // 正常巡航/限速回推, 两者fmin取更小者)。
        if (stop_s < total_length) {
            const double dist_to_stop = std::max(0.0, stop_s - s_est);
            // 2026-08-04: 同STOP分支改jerk等效减速度3.0(与s_bounds曲线自洽,
            // 车到stop_s时v=0)。原6.0(21.3m)与曲线都太紧 → 车12.4封顶。
            // 用3.0: 车16巡航到距STOP 42.7m处v_upper收紧(>42.7m不压)。
            // 2026-08-04晚: 车已停(速度<1.0)且已接近STOP(<5m, 已到停车位
            // 置) → v_upper=0停稳, 消除刹-油-刹蠕动。100131实测: 车停在斑
            // 马线前(位置合格)但P0允许v_upper=4.36 + 行人ST boundary
            // s_up[40]=1.22 + 参考线每帧重建车s=0恒定 → 车0.02-0.9m/s蠕动
            // 死循环。行驶中(速度>=1)或未到(<5m外) → 正常jerk等效刹车包络。
            if (init_s[1] < 1.0 && dist_to_stop < 5.0) {
                v_upper_bound = std::fmin(v_upper_bound, 0.0);
            } else {
                // 2026-08-04晚2: jerk精确最大速度(车在dist内jerk刹停)。
                // 低v纯jerk段: 从v(a=0)以jerk-2刹停 s=2/3·v^1.5 → v=(1.5d)^(2/3)。
                // 高v: jerk过渡3s(a到-6,走3v-9)后-6急刹 → 反函数 v=-9+√(108+12d)。
                // 原sqrt(2·3·dist)(-3恒定抛物线)低v太松: 车v=1.11只距STOP
                // 0.21m<0.78m(jerk最小) → QP无法jerk刹停 → 终点段连续
                // infeasible(101332 t=60.8-64.9) → fallback滑行5s才停。
                const double v_allow_stop = (dist_to_stop < 18.0)
                        ? std::pow(1.5 * dist_to_stop, 2.0 / 3.0)
                        : -9.0 + std::sqrt(108.0 + 12.0 * dist_to_stop);
                v_upper_bound = std::fmin(v_upper_bound, v_allow_stop);
            }
        }
        // 车可达速度/位置累积(物理上限: 从v0以+3加速逼近dx_ref, 平滑无陡降)
        v_est = std::min(dx_ref[i], v_est + 3.0 * delta_t);
        s_est += v_est * delta_t;
        s_dot_bounds.emplace_back(v_lower_bound, std::fmax(v_upper_bound, 0.0));
        print_debug.AddPoint("st_reference_line", curr_t, x_ref[i]);
        print_debug.AddPoint("st_penalty_dx", curr_t, penalty_dx.back());
        print_debug.AddPoint("vt_reference_line", curr_t, dx_ref[i]);
        print_debug.AddPoint("vt_weighting", curr_t, dx_ref_weight[i]);
        print_debug.AddPoint("vt_boundary_lower", curr_t, v_lower_bound);
        print_debug.AddPoint("sv_boundary_lower", path_s, v_lower_bound);
        print_debug.AddPoint("sk_curve", path_s, path_point.kappa());
        print_debug.AddPoint("vt_boundary_upper", curr_t, v_upper_bound);
        print_debug.AddPoint("sv_boundary_upper", path_s, v_upper_bound);
    }
    AdjustInitStatus(s_dot_bounds, delta_t, init_s);
    PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t, init_s);
    piecewise_jerk_problem.set_weight_ddx(config_.acc_weight());
    piecewise_jerk_problem.set_weight_dddx(config_.jerk_weight());
    piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});
    piecewise_jerk_problem.set_x_bounds(0.0, total_length);
    // 提速(2026-08-02): 加速上限用 gflag(默认2.7=0.9*评测3.0), 减速保持 vehicle_param(-6)
    piecewise_jerk_problem.set_ddx_bounds(veh_param.max_deceleration(), FLAGS_planning_max_acceleration);
    piecewise_jerk_problem.set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound, FLAGS_longitudinal_jerk_upper_bound);
    // 诊断日志(2026-08-03): 定位人行道刹-油-刹/提前刹车——s_bounds在t轴是否提前收窄。
    // 带边界保护(避免上次s_up越界): 索引 clamp 到 [0, size-1]。
    {
        const int n_sb = static_cast<int>(s_bounds.size());
        const int i40 = std::min(40, n_sb - 1);
        const int i80 = std::min(80, n_sb - 1);
        const int n_dx = static_cast<int>(dx_ref.size());
        const int j40 = std::min(40, n_dx - 1);
        // 限速诊断(2026-08-03): 030359实测车在斑马线限速区(4.5)内 v_up=16(限速
        // 失效, 车8m/s过斑马线)。打印 QP 收到的 speed_limit 在车位置(s≈0)/前
        // 方60m 的限速值和限速点数量, 确认是'限速没进QP'还是'查询错位'。
        // 带保护(限速点<2时 GetSpeedLimitByS 会 CHECK 崩, 手动 lower_bound)。
        const auto& sl_pts = speed_limit.speed_limit_points();
        double sl0 = FLAGS_planning_upper_speed_limit;
        double sl60 = FLAGS_planning_upper_speed_limit;
        if (sl_pts.size() >= 2U) {
            auto get_sl = [&sl_pts](double s) {
                auto it = std::lower_bound(
                        sl_pts.begin(), sl_pts.end(), s,
                        [](const std::pair<double, double>& p, double v) { return p.first < v; });
                if (it == sl_pts.end()) {
                    return (it - 1)->second;
                }
                return it->second;
            };
            sl0 = get_sl(0.0);
            sl60 = get_sl(60.0);
        }
        // STOP/YIELD 诊断(2026-08-04): 092945实测 stop_s 从116(DEST)跳0(车位置)
        // → s_bounds 全压0 + P0 v_upper=0 → infeasible 55次 + 刹-油-刹。
        // 打印所有 STOP/YIELD boundary 的 id 和 s_upper(t=0), 定位哪个在0。
        std::string stops_str;
        for (const auto* bd : st_graph_data.st_boundaries()) {
            if (bd->boundary_type() == STBoundary::BoundaryType::STOP
                || bd->boundary_type() == STBoundary::BoundaryType::YIELD) {
                double su = 0.0, sl = 0.0;
                bd->GetUnblockSRange(0.0, &su, &sl);
                stops_str += bd->id() + "@" + std::to_string(su) + ";";
            }
        }
        AINFO << "[pjs2] init_v=" << init_s[1] << " stop_s=" << stop_s << " s_up[0/40/80]=" << s_bounds[0].second << "/"
              << s_bounds[i40].second << "/" << s_bounds[i80].second << " dx_ref[0/40]=" << dx_ref[0] << "/"
              << dx_ref[j40] << " v_up[0]=" << s_dot_bounds[0].second << " ref_len=" << total_length
              << " sl[0/60]=" << sl0 << "/" << sl60 << " npts=" << sl_pts.size() << " stops=" << stops_str;
    }
    piecewise_jerk_problem.set_x_bounds(std::move(s_bounds));
    piecewise_jerk_problem.set_dx_ref(dx_ref_weight, dx_ref);
    piecewise_jerk_problem.set_x_ref(config_.ref_s_weight(), std::move(x_ref));
    piecewise_jerk_problem.set_penalty_dx(penalty_dx);
    // 保存限速约束副本(fallback 保留限速用, set_dx_bounds 会 move 走原数据)
    const std::vector<std::pair<double, double>> s_dot_bounds_copy = s_dot_bounds;
    piecewise_jerk_problem.set_dx_bounds(std::move(s_dot_bounds));

    // Solve the problem
    if (!piecewise_jerk_problem.Optimize()) {
        const std::string msg = "Piecewise jerk speed optimizer failed!";
        AERROR << msg << ".try to fallback.";
        // 修复(2026-08-02): 原 fallback 用 max(16.667, init_v) 抹掉全部限速 →
        // 车全速通过限速区。改为可达性放宽: 保留原限速, 但每点 v_upper 不低于
        // "从 init_v 以 6.0m/s² 可达的速度" → QP 可行同时限速仍生效。
        // 用 6.0(>回推的4.0): 车在限速区前28.6m触发fallback后以6m/s²更快减速,
        // 到限速区时已≤4.5(16m/s@6m/s²需19.5m, 28.6m足够)。
        std::vector<std::pair<double, double>> relaxed_dx_bounds;
        relaxed_dx_bounds.reserve(num_of_knots);
        for (int i = 0; i < num_of_knots; ++i) {
            const double v_feasible = std::max(0.0, init_s[1] - 6.0 * (i * delta_t));
            relaxed_dx_bounds.emplace_back(0.0, std::fmax(s_dot_bounds_copy[i].second, v_feasible));
        }
        piecewise_jerk_problem.set_dx_bounds(std::move(relaxed_dx_bounds));
        if (!FLAGS_speed_optimize_fail_relax_velocity_constraint || !piecewise_jerk_problem.Optimize()) {
            // 2026-08-04: 不清空 speed_data(保留上一帧轨迹)——避免 QP 数值不
            // 收敛(maximum iterations)时 TrajectoryFallbackTask 用
            // GenerateFallbackSpeed(FLAGS_speed_fallback_distance=3m)急刹
            // (刹油刹#3, 本地scn4 15:59:58.78实测: 车4.5巡航被fast stop
            // 3m内刹停到0.16). 保留上一帧轨迹→车平滑延续→下帧QP恢复.
            // relaxed对init_v<v_upper帧无效(v_upper=max(4.5,init_v-6t)仍4.5)
            // → 保留旧轨迹是唯一平滑出路. 真实STOP(行人/终点)由s_bounds+
            // P0 v_upper保证, 旧轨迹不会冲过(车已在STOP前减速).
            // 2026-08-05(007轮a1/a3修复): 保留轨迹裁剪到限速回推包络(v_upper)。
            // 007轮实测: a1车13s QP失败(init_v波动11.1>v_up 6.56) → 保留6.92
            // 旧轨迹 → 车在限速5区内7.78m/s超速(评测扣40分). 裁剪后车≤v_upper
            // (进入限速区时≤4.8), 不超速; 保留机制仍避免scn4刹油刹(巡航无
            // 限速压力时v_upper=20不裁剪, 车4.5平滑延续).
            if (!speed_data->empty()) {
                double pre_s = (*speed_data)[0].s();
                for (int i = 0; i < static_cast<int>(speed_data->size()); ++i) {
                    auto& sp = (*speed_data)[i];
                    const double t = sp.t();
                    const int idx = std::min(
                            static_cast<int>(t / delta_t), num_of_knots - 1);
                    const double v_up =
                            std::fmax(0.0, s_dot_bounds_copy[idx].second);
                    const double v = std::fmin(sp.v(), v_up);
                    sp.set_v(v);
                    if (i > 0) {
                        const double dt = sp.t() - (*speed_data)[i - 1].t();
                        sp.set_s(pre_s +
                                 0.5 * ((*speed_data)[i - 1].v() + v) * dt);
                    }
                    pre_s = sp.s();
                }
            }
            // speed_data->clear();
            print_debug.AddPoint("optimize_st_curve", 0, init_s[0]);
            print_debug.AddPoint("optimize_vt_curve", 0, init_s[1]);
            print_debug.AddPoint("optimize_at_curve", 0, init_s[2]);
            AINFO << "jerk_bound: " << FLAGS_longitudinal_jerk_lower_bound << ","
                  << FLAGS_longitudinal_jerk_upper_bound;
            AINFO << "acc bound: " << veh_param.max_deceleration() << "," << veh_param.max_acceleration();
            print_debug.PrintToLog();
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
    }

    // Extract output
    const std::vector<double>& s = piecewise_jerk_problem.opt_x();
    const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
    const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();
    // 诊断日志(2026-08-03): QP求解后速度——025653实测输入全对(dx_ref=16/v_up=16/
    // stop_s无穷)但车缓降 → 确认是QP输出缓降 还是 control层未跟上。
    // 带边界保护(防越界, 之前pjs-out无保护导致planning只跑1帧)。
    {
        const int n_ds = static_cast<int>(ds.size());
        const int k1 = std::min(1, n_ds - 1);
        const int k10 = std::min(10, n_ds - 1);
        const int k30 = std::min(30, n_ds - 1);
        AINFO << "[pjs-out] v[0/1/10/30]=" << ds[0] << "/" << ds[k1] << "/" << ds[k10] << "/" << ds[k30]
              << " a[0]=" << dds[0] << " s[0]=" << s[0];
    }
    for (int i = 0; i < num_of_knots; ++i) {
        ADEBUG << "For t[" << i * delta_t << "], s = " << s[i] << ", v = " << ds[i] << ", a = " << dds[i];
        print_debug.AddPoint("optimize_st_curve", i * delta_t, s[i]);
        print_debug.AddPoint("optimize_vt_curve", i * delta_t, ds[i]);
        print_debug.AddPoint("optimize_at_curve", i * delta_t, dds[i]);
    }
    speed_data->clear();
    speed_data->AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
    for (int i = 1; i < num_of_knots; ++i) {
        // Avoid the very last points when already stopped
        if (ds[i] <= 0.0) {
            break;
        }
        speed_data->AppendSpeedPoint(s[i], delta_t * i, ds[i], dds[i], (dds[i] - dds[i - 1]) / delta_t);
    }
    SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
    RecordDebugInfo(*speed_data, st_graph_data.mutable_st_graph_debug());
    print_debug.PrintToLog();
    return Status::OK();
}
void PiecewiseJerkSpeedOptimizer::AdjustInitStatus(
        const std::vector<std::pair<double, double>> s_dot_bound,
        double delta_t,
        std::array<double, 3>& init_s) {
    double v_min = init_s[1];
    double v_max = init_s[1];
    double a_min = init_s[2];
    double a_max = init_s[2];
    double last_a_min = 0;
    double last_a_max = 0;
    for (size_t i = 1; i < s_dot_bound.size(); i++) {
        last_a_min = a_min;
        last_a_max = a_max;
        a_min = a_min + delta_t * FLAGS_longitudinal_jerk_upper_bound;
        a_max = a_max + delta_t * FLAGS_longitudinal_jerk_lower_bound;
        v_min = v_min + 0.5 * delta_t * (a_min + last_a_min);
        v_max = v_max + 0.5 * delta_t * (a_max + last_a_max);
        if (v_min < s_dot_bound[i].first || v_max > s_dot_bound[i].second) {
            AWARN << "init state not appropriate in" << i << "," << v_min << "," << v_max
                  << "adjust acc to 0 in init state " << init_s[0] << "," << init_s[1] << "," << init_s[2];
            init_s[2] = 0;
            return;
        }
    }
}
}  // namespace planning
}  // namespace apollo
