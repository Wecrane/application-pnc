/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>
#include <vector>

#include "modules/planning/planning_base/common/dependency_injector.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/path_boundary.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"

namespace apollo {
namespace planning {

struct ContestLaneFollowPathContext {
    bool lane_change = false;
    bool dense_s_curve = false;
    bool u_turn = false;
};

ContestLaneFollowPathContext BuildContestLaneFollowPathContext(
        const std::shared_ptr<DependencyInjector>& injector,
        const ReferenceLineInfo& reference_line_info);

void ApplyContestLaneChangeBoundaryOverride(
        const ReferenceLineInfo& reference_line_info,
        const SLState& init_sl_state,
        PathBoundary* path_bound);

void ApplyContestUTurnBoundaryExpansion(
        const ContestLaneFollowPathContext& context,
        PathBoundary* path_bound);

void ApplyDenseConeSCurveBoundaryTuning(
        const ContestLaneFollowPathContext& context,
        double* obstacle_lat_buffer,
        bool* enable_adc_vertex_constraint);

double AdjustJerkBoundaryForContestLaneFollow(
        const ReferenceLine& reference_line,
        const PathBoundary& path_boundary,
        double jerk_bound);

void ApplyContestLaneFollowPathReference(
        const ContestLaneFollowPathContext& context,
        const PathBoundary& path_boundary,
        double path_reference_l_weight,
        double jerk_bound,
        double final_jerk_bound,
        std::vector<double>* ref_l,
        std::vector<double>* weight_ref_l);

}  // namespace planning
}  // namespace apollo
