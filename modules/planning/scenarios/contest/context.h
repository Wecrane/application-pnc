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

#include "modules/planning/planning_interface_base/scenario_base/scenario.h"
#include "modules/planning/scenarios/contest/proto/contest.pb.h"

namespace apollo {
namespace planning {

enum class ContestScenarioKind {
  LANE_CHANGE = 0,
  S_CURVE = 1,
  U_TURN = 2,
  CONSTRUCTION_ZONE = 3,
  STATION_SHUTTLE = 4,
};

struct ContestScenarioContext : public ScenarioContext {
  ScenarioContestConfig scenario_config;
  ContestScenarioKind kind = ContestScenarioKind::LANE_CHANGE;
  // 站点接驳状态
  bool shuttle_arrived_at_station = false;
  double shuttle_dwell_start_time = 0.0;
  bool shuttle_departed = false;
};

}  // namespace planning
}  // namespace apollo
