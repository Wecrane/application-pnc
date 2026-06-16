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

#include "modules/planning/planning_base/common/planlog_context.h"

namespace apollo {
namespace planning {

namespace {

// Thread-local storage for planning log context.
// Each thread maintains its own independent copy, which is critical
// because glog messages from different threads are handled independently
// and PlanningLogSink::send() is called from whichever thread issued the log.

thread_local uint32_t g_planlog_frame_seq = 0;
thread_local std::string g_planlog_scenario_name = "unknown";
thread_local std::string g_planlog_stage_name = "unknown";
thread_local std::string g_planlog_planning_name = "planning";

}  // namespace

// ---- Frame sequence ----

void PlanningLogContext::set_frame_seq(uint32_t seq) {
    g_planlog_frame_seq = seq;
}

uint32_t PlanningLogContext::frame_seq() {
    return g_planlog_frame_seq;
}

// ---- Scenario name ----

void PlanningLogContext::set_scenario_name(const std::string& name) {
    g_planlog_scenario_name = name;
}

const std::string& PlanningLogContext::scenario_name() {
    return g_planlog_scenario_name;
}

// ---- Stage name ----

void PlanningLogContext::set_stage_name(const std::string& name) {
    g_planlog_stage_name = name;
}

const std::string& PlanningLogContext::stage_name() {
    return g_planlog_stage_name;
}

// ---- Planning name ----

void PlanningLogContext::set_planning_name(const std::string& name) {
    g_planlog_planning_name = name;
}

const std::string& PlanningLogContext::planning_name() {
    return g_planlog_planning_name;
}

}  // namespace planning
}  // namespace apollo
