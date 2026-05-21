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

#include "modules/planning/scenarios/contest/stage_contest_lane_follow.h"

#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/scenarios/contest/context.h"
#include "modules/planning/scenarios/contest/contest_scenario_util.h"

namespace apollo {
namespace planning {

StageResult ContestLaneFollowStage::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  StageResult result = LaneFollowStage::Process(planning_init_point, frame);
  if (result.HasError()) {
    return result;
  }
  if (!StillInScenario(*frame)) {
    return FinishScenario();
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}

bool ContestLaneFollowStage::StillInScenario(const Frame& frame) const {
  if (frame.reference_line_info().empty()) {
    return false;
  }
  const auto* context = GetContextAs<ContestScenarioContext>();
  switch (context->kind) {
    case ContestScenarioKind::LANE_CHANGE:
      return contest::IsContestLaneChange(frame);
    case ContestScenarioKind::S_CURVE:
      return contest::IsContestSCurve(frame.reference_line_info().front(),
                                      context->scenario_config);
    case ContestScenarioKind::U_TURN:
      return contest::IsContestUTurn(frame.reference_line_info().front(),
                                     context->scenario_config);
    case ContestScenarioKind::CONSTRUCTION_ZONE:
      return contest::IsContestConstructionZone(
          frame.reference_line_info().front(), context->scenario_config);
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
