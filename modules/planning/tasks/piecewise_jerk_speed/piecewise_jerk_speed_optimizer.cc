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
        double s_upper_bound = total_length;
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
                    const double v_ref = FLAGS_planning_upper_speed_limit;
                    const double dec_abs = std::abs(veh_param.max_deceleration());
                    const double t_brake =
                            (s_upper - init_s[0] - v_ref * v_ref / (2.0 * dec_abs)) / v_ref;
                    if (t_brake <= 0.0) {
                        // 车已在刹车距离内(stop_s<v_ref²/2|dec|): 恒定stop_s约束
                        s_upper_bound = std::fmin(s_upper_bound, s_upper);
                    } else {
                        const double T_ = t_brake + v_ref / dec_abs;  // 刹停时刻
                        if (curr_t < t_brake) {
                            // 自由巡航(保持 total_length, 车受dx_bounds速度约束)
                        } else if (curr_t <= T_) {
                            // 最晚刹车曲线(平滑): s_upper = stop_s - 0.5|dec|(T-t)²
                            const double s_late =
                                    s_upper - 0.5 * dec_abs * (T_ - curr_t) * (T_ - curr_t);
                            s_upper_bound =
                                    std::fmin(s_upper_bound, std::max(s_late, 0.0));
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
        static constexpr double kLimitDecel = 5.0;
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
            const double brake_envelope = std::sqrt(2.0 * std::abs(veh_param.max_deceleration()) * dist_to_stop);
            dx_ref[i] = std::fmin(dx_ref[i], brake_envelope);
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
        AINFO << "[pjs2] init_v=" << init_s[1] << " stop_s=" << stop_s << " s_up[0/40/80]=" << s_bounds[0].second << "/"
              << s_bounds[i40].second << "/" << s_bounds[i80].second << " dx_ref[0/40]=" << dx_ref[0] << "/"
              << dx_ref[j40] << " v_up[0]=" << s_dot_bounds[0].second << " ref_len=" << total_length;
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
            speed_data->clear();
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
        AINFO << "[pjs-out] v[0/1/10/30]=" << ds[0] << "/" << ds[k1] << "/"
              << ds[k10] << "/" << ds[k30] << " a[0]=" << dds[0]
              << " s[0]=" << s[0];
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
