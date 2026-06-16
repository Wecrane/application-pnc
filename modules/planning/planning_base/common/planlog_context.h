/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <cstdint>
#include <string>

namespace apollo {
namespace planning {

// Thread-local context for planning log macros.
// Each planning thread maintains its own independent state.
// Set by scenario_manager, stage, and planning component at key points.
//
// Usage:
//   PlanningLogContext::set_frame_seq(1234);
//   PlanningLogContext::set_scenario_name("lane_follow");
//   PlanningLogContext::set_stage_name("LANE_FOLLOW_STAGE");
//
// These values are automatically attached to log messages by the
// PLAN_LOG / PSCENARIO_* / PSTAGE_* macros and by PlanningLogSink.
struct PlanningLogContext {
  // ---- Frame sequence number ----
  static void set_frame_seq(uint32_t seq);
  static uint32_t frame_seq();

  // ---- Scenario name (e.g. "lane_follow", "stop_sign_unprotected") ----
  static void set_scenario_name(const std::string& name);
  static const std::string& scenario_name();

  // ---- Stage name (e.g. "LANE_FOLLOW_STAGE", "STAGE_PRE_STOP") ----
  static void set_stage_name(const std::string& name);
  static const std::string& stage_name();

  // ---- Planning module name (default "planning") ----
  static void set_planning_name(const std::string& name);
  static const std::string& planning_name();
};

}  // namespace planning
}  // namespace apollo
