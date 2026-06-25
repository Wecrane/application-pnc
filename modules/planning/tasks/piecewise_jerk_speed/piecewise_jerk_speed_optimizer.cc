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
#include <cmath>

#include <string>
#include <utility>
#include <vector>
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/common/st_graph_data.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "modules/planning/tasks/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.h"

namespace apollo {
namespace planning {

namespace {

constexpr char kContestRoundaboutScenarioName[] = "CONTEST_ROUNDABOUT";
constexpr double kRoundaboutCommitLookForward = 6.0;
constexpr double kRoundaboutCommitLookBack = 3.0;

bool IsContestRoundaboutScenario(const std::shared_ptr<DependencyInjector>& injector) {
    return injector != nullptr && injector->planning_context() != nullptr
            && injector->planning_context()->planning_status().scenario().scenario_type()
            == kContestRoundaboutScenarioName;
}

bool IsRoundaboutCommitArea(const ReferenceLineInfo* reference_line_info) {
    if (reference_line_info == nullptr) {
        return false;
    }
    const double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
    for (const auto& overlap : reference_line_info->reference_line().map_path().pnc_junction_overlaps()) {
        if (overlap.start_s <= adc_end_s + kRoundaboutCommitLookForward
            && overlap.end_s >= adc_end_s - kRoundaboutCommitLookBack) {
            return true;
        }
    }
    return false;
}

}  // namespace

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
    const bool roundabout_launch_area
            = IsContestRoundaboutScenario(injector_) && IsRoundaboutCommitArea(reference_line_info_);
    const bool uturn_release_launch = path_data.path_label().find("uturn_release") != std::string::npos;
    const bool window_release_launch = init_s[1] < (uturn_release_launch ? 1.8 : 1.0)
            && (uturn_release_launch || roundabout_launch_area);
    if (window_release_launch) {
        init_s[2] = std::max(init_s[2], uturn_release_launch ? 3.0 : 2.0);
        AINFO << "[WINDOW][Speed] release launch boost, label=" << path_data.path_label() << ", init_v=" << init_s[1]
              << ", init_a=" << init_s[2] << ", roundabout=" << roundabout_launch_area
              << ", adc_x=" << vehicle_state.x() << ", adc_y=" << vehicle_state.y();
    }
    const bool reverse_speed_profile = path_data.is_reverse_path();
    const bool chassis_reverse = vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE;
    if (reverse_speed_profile || chassis_reverse) {
        const double raw_v = init_s[1];
        const double raw_a = init_s[2];
        init_s[1] = std::fabs(raw_v);
        if (raw_v < -1e-3 || chassis_reverse) {
            init_s[2] = -raw_a;
        }
        AINFO << "transfer reverse speed, reverse_path=" << reverse_speed_profile
              << ", chassis_reverse=" << chassis_reverse << ", raw=(" << raw_v << "," << raw_a << "), init=("
              << init_s[0] << "," << init_s[1] << "," << init_s[2] << ")";
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
                s_upper_bound = std::fmin(s_upper_bound, s_upper);
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
        // 预留余量使有效限速 = 28.5 km/h，防止优化器在 jerk 最小化时略微超调
        constexpr double kSpeedLimitMargin = 1.5 / 3.6;  // 1.5 km/h → m/s
        const bool reverse_launch = reverse_speed_profile && curr_t <= 2.0;
        v_upper_bound = std::fmax(
                0.0,
                v_upper_bound - ((window_release_launch || reverse_launch) ? 0.0 : kSpeedLimitMargin));
        if (reverse_launch) {
            dx_ref_weight[i] = std::max(dx_ref_weight[i], 120.0);
            dx_ref[i] = v_upper_bound;
            x_ref[i] = std::min(total_length, init_s[1] * curr_t + 0.5 * 2.5 * curr_t * curr_t);
        } else if (window_release_launch && curr_t <= 3.0) {
            const double launch_accel_ref = uturn_release_launch ? 4.5 : 3.0;
            dx_ref_weight[i] = std::max(dx_ref_weight[i], uturn_release_launch ? 120.0 : 80.0);
            dx_ref[i] = v_upper_bound;
            x_ref[i] = std::min(total_length, init_s[1] * curr_t + 0.5 * launch_accel_ref * curr_t * curr_t);
        } else {
            dx_ref[i] = std::fmin(v_upper_bound, dx_ref[i]);
        }
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

    // If the vehicle's initial speed exceeds the speed limit at the first
    // point (common when entering a high-curvature section like a U-turn),
    // relax the first few speed bound points to allow a feasible
    // deceleration ramp.  Without this, the speed optimizer sees an
    // impossible initial condition and fails with "primal infeasible".
    if (init_s[1] > s_dot_bounds[0].second + 1e-3) {
        constexpr int kSpeedBoundRelaxCount = 8;
        const double init_v = init_s[1];
        for (int i = 0; i < kSpeedBoundRelaxCount && i < num_of_knots; ++i) {
            // Linearly ramp from init_v down to the original bound over the
            // relaxation window, then keep the original bound.
            double relaxed_upper
                    = init_v + (s_dot_bounds[i].second - init_v) * static_cast<double>(i) / kSpeedBoundRelaxCount;
            s_dot_bounds[i].second = std::fmax(relaxed_upper, s_dot_bounds[i].second);
        }
    }
    PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t, init_s);
    piecewise_jerk_problem.set_weight_ddx(
            reverse_speed_profile ? config_.acc_weight() * 0.08
                                  : (window_release_launch ? config_.acc_weight() * 0.08 : config_.acc_weight()));
    piecewise_jerk_problem.set_weight_dddx(
            reverse_speed_profile ? config_.jerk_weight() * 0.02
                                  : (window_release_launch ? config_.jerk_weight() * 0.02 : config_.jerk_weight()));
    piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});
    piecewise_jerk_problem.set_x_bounds(0.0, total_length);
    const double launch_max_acc = reverse_speed_profile ? std::max(veh_param.max_acceleration(), 3.0)
                                  : uturn_release_launch ? std::max(veh_param.max_acceleration(), 3.5)
                                                         : veh_param.max_acceleration();
    const double launch_max_jerk = (reverse_speed_profile || uturn_release_launch)
                                           ? std::max(FLAGS_longitudinal_jerk_upper_bound, 6.0)
                                           : FLAGS_longitudinal_jerk_upper_bound;
    piecewise_jerk_problem.set_ddx_bounds(veh_param.max_deceleration(), launch_max_acc);
    piecewise_jerk_problem.set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound, launch_max_jerk);
    piecewise_jerk_problem.set_x_bounds(std::move(s_bounds));
    piecewise_jerk_problem.set_dx_ref(dx_ref_weight, dx_ref);
    piecewise_jerk_problem.set_x_ref(config_.ref_s_weight(), std::move(x_ref));
    piecewise_jerk_problem.set_penalty_dx(penalty_dx);
    piecewise_jerk_problem.set_dx_bounds(std::move(s_dot_bounds));

    // Solve the problem
    if (!piecewise_jerk_problem.Optimize()) {
        const std::string msg = "Piecewise jerk speed optimizer failed!";
        AERROR << msg << ".try to fallback.";
        piecewise_jerk_problem.set_dx_bounds(
                0.0, std::fmax(FLAGS_planning_upper_speed_limit, st_graph_data.init_point().v()));
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
