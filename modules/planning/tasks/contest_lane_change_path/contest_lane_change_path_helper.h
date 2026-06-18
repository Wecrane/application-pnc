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

#include "modules/planning/planning_base/common/dependency_injector.h"
#include "modules/planning/planning_base/common/path_boundary.h"
#include "modules/planning/planning_base/common/reference_line_info.h"

namespace apollo {
namespace planning {

bool IsContestLaneChangeScenario(const std::shared_ptr<DependencyInjector>& injector);

double ContestLaneChangeMinStartSpeed();

bool IsContestLaneChangeWindowClear(ReferenceLineInfo* reference_line_info);

bool ShouldHoldLateralInContestLaneChange(bool is_contest_lane_change, bool is_clear_to_change_lane);

void ApplyContestLaneChangeHoldBoundary(double current_l, PathBoundary* path_bound);

void ApplyContestLaneChangeSpeedLimit(bool is_contest_lane_change, ReferenceLineInfo* reference_line_info);

void IgnoreObstaclesForContestLaneChange(bool is_contest_lane_change, ReferenceLineInfo* reference_line_info);

}  // namespace planning
}  // namespace apollo
