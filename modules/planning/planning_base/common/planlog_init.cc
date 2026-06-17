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

#include <algorithm>
#include <cstdio>
#include <ctime>

#include <boost/filesystem.hpp>

#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {

PlanningLogSink* g_planning_log_sink = nullptr;

// Max size of a log file before startup rotation (100 MB).
constexpr uintmax_t kMaxLogFileSize = 100 * 1024 * 1024;

// Rotate a log file on startup: rename old file to .N if it exceeds the size limit.
void RotateLogFileOnStartup(const std::string& file_path) {
    boost::system::error_code ec;
    if (!boost::filesystem::exists(file_path, ec) || ec) {
        return;
    }
    uintmax_t size = boost::filesystem::file_size(file_path, ec);
    if (ec || size < kMaxLogFileSize) {
        return;  // File is small enough, no rotation needed.
    }
    // Rotate: rename old file to .1, shift existing .N to .N+1.
    // Keep at most 5 rotated files.
    constexpr int kMaxRotatedFiles = 5;
    for (int i = kMaxRotatedFiles; i >= 1; --i) {
        std::string old_path = file_path + "." + std::to_string(i);
        std::string new_path = file_path + "." + std::to_string(i + 1);
        if (i == kMaxRotatedFiles && boost::filesystem::exists(new_path, ec)) {
            boost::filesystem::remove(new_path, ec);
        }
        if (boost::filesystem::exists(old_path, ec)) {
            boost::filesystem::rename(old_path, new_path, ec);
        }
    }
    boost::filesystem::rename(file_path, file_path + ".1", ec);
}

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

    // Rotate main log files on startup to prevent unbounded growth.
    RotateLogFileOnStartup(log_dir + "/error.log");
    RotateLogFileOnStartup(log_dir + "/summary.log");
    RotateLogFileOnStartup(log_dir + "/decision.log");

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
