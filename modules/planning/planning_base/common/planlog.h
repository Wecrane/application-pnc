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

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/planlog_context.h"

namespace apollo {
namespace planning {

// Initialize the planning structured logging system.
// Call once at PlanningComponent::Init().
// Creates log directory, registers PlanningLogSink with glog.
void InitPlanningLogger();

// Shutdown the planning structured logging system.
// Call once at PlanningComponent destruction.
// Removes PlanningLogSink from glog and flushes/closes all file writers.
void ShutdownPlanningLogger();

}  // namespace planning
}  // namespace apollo

// ============================================================
// NEW LOGGING MACROS — only active when USE_NEW_LOG is defined
// These coexist with existing AINFO/AERROR/AWARN/ADEBUG macros
// and add structured context prefixes automatically.
// ============================================================
#ifdef USE_NEW_LOG

// ---- Base frame-level macro: auto-prepends [frm:<seq>] ----
#define PLAN_LOG(level) LOG(level) << "[frm:" << ::apollo::planning::PlanningLogContext::frame_seq() << "] "

// ---- Scenario-aware macros: auto-prepend [scn:<name>] ----
#define PSCENARIO_INFO LOG(INFO) << "[scn:" << ::apollo::planning::PlanningLogContext::scenario_name() << "] "
#define PSCENARIO_WARN LOG(WARNING) << "[scn:" << ::apollo::planning::PlanningLogContext::scenario_name() << "] "

// ---- Stage-aware debug macro: auto-prepend [stg:<name>] ----
#define PSTAGE_DEBUG VLOG(1) << "[stg:" << ::apollo::planning::PlanningLogContext::stage_name() << "] "

// ---- Frame summary macro: routed to summary.log ----
#define PFRAME_SUMMARY LOG(INFO) << "[SUMMARY] "

// ---- Decision log macro: routed to decision.log ----
#define PDECISION_LOG LOG(INFO) << "[DECISION] "

// ---- State transition log macro: routed to decision.log ----
#define PSTATE_LOG LOG(INFO) << "[STATE] "

#endif  // USE_NEW_LOG
