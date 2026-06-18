/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file path_time_heuristic_optimizer.cc
 **/

#include "modules/planning/tasks/path_time_heuristic/path_time_heuristic_optimizer.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/st_graph_data.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/tasks/path_time_heuristic/gridded_path_time_graph.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

bool PathTimeHeuristicOptimizer::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    if (!SpeedOptimizer::Init(config_dir, name, injector)) {
        return false;
    }
    // Load the config_ this task.
    return SpeedOptimizer::LoadConfig<SpeedHeuristicOptimizerConfig>(&config_);
}

bool PathTimeHeuristicOptimizer::SearchPathTimeGraph(SpeedData* speed_data) const {
    const auto& dp_st_speed_optimizer_config = reference_line_info_->IsChangeLanePath()
            ? config_.lane_change_speed_config()
            : config_.default_speed_config();

    GriddedPathTimeGraph st_graph(
            reference_line_info_->st_graph_data(),
            dp_st_speed_optimizer_config,
            reference_line_info_->path_decision()->obstacles().Items(),
            init_point_);

    if (!st_graph.Search(speed_data).ok()) {
        AERROR << "failed to search graph with dynamic programming.";
        return false;
    }
    return true;
}

Status PathTimeHeuristicOptimizer::Process(
        const PathData& path_data,
        const common::TrajectoryPoint& init_point,
        SpeedData* const speed_data) {
    init_point_ = init_point;

    if (path_data.discretized_path().empty()) {
        const std::string msg = "Empty path data";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // ── 模拟站点接驳倒车策略：在 DP 阶段生成三段式速度曲线 ──
    // 站点接驳 OpenSpace 将路径与速度统一优化，倒车匀速平稳。
    // 此处必须在 DP（SpeedDecider 之前）生成有效速度数据，否则空 SpeedData
    // 会导致 SpeedDecider 失败并触发 trajectory_fallback。
    // 同时清空 ST 障碍物边界：倒车路径使用倒车参考线，而 ST 边界映射用前向
    // 参考线投影障碍物，坐标系不匹配会产生错误的"停止墙"导致油门刹车振荡。
    if (path_data.is_reverse_path()) {
        // 清空 ST 障碍物边界：倒车路径使用倒车参考线，而 SpeedBoundsDecider
        // 用前向参考线投影障碍物到 ST 图，坐标系不匹配会产生错误"停止墙"。
        // 重新 LoadData 以空边界覆盖，避免 SpeedDecider 误触发刹车。
        StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
        std::vector<const STBoundary*> empty_boundaries;
        st_graph_data.LoadData(
                empty_boundaries,
                0.0,
                st_graph_data.init_point(),
                st_graph_data.speed_limit(),
                st_graph_data.cruise_speed(),
                path_data.discretized_path().Length(),
                st_graph_data.total_time_by_conf(),
                st_graph_data.mutable_st_graph_debug());
        reference_line_info_->path_decision()->EraseStBoundaries();

        speed_data->clear();
        const double path_length = path_data.discretized_path().Length();
        if (path_length < 1e-3) {
            AWARN << "[REVERSE][DP] reverse path too short, skip speed generation";
            return Status::OK();
        }

        constexpr double kReverseTargetSpeed = 5.0;
        constexpr double kReverseAccel = 2.0;
        constexpr double kReverseDecel = -2.0;
        constexpr double kDt = 0.1;

        const double accel_end_s = path_length * 0.25;
        const double cruise_end_s = path_length * 0.75;

        double t = 0.0;
        double s = 0.0;
        double v = 0.0;
        double last_s = 0.0;
        double last_v = 0.0;

        const double init_v = std::fabs(init_point.v());
        if (init_v > 0.01) {
            v = std::min(init_v, kReverseTargetSpeed);
            speed_data->AppendSpeedPoint(0.0, 0.0, v, 0.0, 0.0);
            last_v = v;
        }

        while (s < path_length - 1e-4) {
            if (s < accel_end_s) {
                v = std::min(
                        kReverseTargetSpeed,
                        std::sqrt(std::max(0.0, last_v * last_v + 2.0 * kReverseAccel * (s - last_s + 0.01))));
            } else if (s < cruise_end_s) {
                v = kReverseTargetSpeed;
            } else {
                const double remain = path_length - s;
                v = std::min(kReverseTargetSpeed, std::sqrt(std::max(0.0, -2.0 * kReverseDecel * remain)));
            }
            v = std::max(0.0, std::min(v, kReverseTargetSpeed));

            const double ds = std::max(1e-4, v * kDt);
            s += ds;
            t += kDt;
            if (s > path_length) {
                s = path_length;
                v = 0.0;
            }

            const double a = (t > 0.0 && ds > 1e-6) ? (v - last_v) / kDt : 0.0;
            speed_data->AppendSpeedPoint(s, t, v, a, 0.0);
            last_s = s;
            last_v = v;
        }

        if (v > 0.01) {
            speed_data->AppendSpeedPoint(path_length, t + 0.1, 0.0, kReverseDecel, 0.0);
        }

        SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
        AINFO << "[REVERSE][DP] 3-segment profile: target=" << kReverseTargetSpeed << " m/s, accel=" << kReverseAccel
              << " m/s², path_length=" << path_length << " m, total_time=" << t << " s, init_v=" << init_v
              << " m/s, points=" << speed_data->size() << ", st_boundaries_cleared=1";
        return Status::OK();
    }

    if (!SearchPathTimeGraph(speed_data)) {
    const std::string msg = absl::StrCat(Name(), ": Failed to search graph with dynamic programming.");
    AERROR << msg;
    RecordDebugInfo(*speed_data, reference_line_info_->mutable_st_graph_data()->mutable_st_graph_debug());
    return Status(ErrorCode::PLANNING_ERROR, msg);
}
RecordDebugInfo(*speed_data, reference_line_info_->mutable_st_graph_data()->mutable_st_graph_debug());
return Status::OK();
}

}  // namespace planning
}  // namespace apollo
