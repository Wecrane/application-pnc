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

#include "modules/planning/planning_base/common/planlog_sink.h"

#include <boost/filesystem.hpp>

#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {

PlanningLogSink* g_planning_log_sink = nullptr;

}  // namespace

void InitPlanningLogger() {
    if (g_planning_log_sink != nullptr) {
        // Already initialized (idempotent).
        return;
    }

    // Ensure log directory exists.
    std::string log_dir = FLAGS_planning_log_dir;
    if (log_dir.empty()) {
        log_dir = "data/log/planning";
    }
    boost::system::error_code ec;
    boost::filesystem::create_directories(log_dir, ec);
    boost::filesystem::create_directories(log_dir + "/trace", ec);
    boost::filesystem::create_directories(log_dir + "/per_scenario", ec);

    // Create and register the custom LogSink.
    g_planning_log_sink = new PlanningLogSink();
    google::AddLogSink(g_planning_log_sink);

    AINFO << "PlanningLogSink initialized, log_dir=" << log_dir;
}

void ShutdownPlanningLogger() {
    if (g_planning_log_sink != nullptr) {
        google::RemoveLogSink(g_planning_log_sink);
        delete g_planning_log_sink;
        g_planning_log_sink = nullptr;
    }
}

}  // namespace planning
}  // namespace apollo
