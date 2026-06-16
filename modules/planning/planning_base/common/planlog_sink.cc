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

#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>

#include <boost/filesystem.hpp>

#include "modules/planning/planning_base/common/planlog_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {

constexpr int kDefaultTraceRotateMinutes = 10;

// Format a struct tm (from glog) into ISO 8601 string.
// glog provides: tm_mon 0-11, tm_mday 1-31, tm_hour 0-23, tm_min 0-59.
// usecs is derived from glog's internal microsecond counter.
void FormatISO8601(char* buf, size_t buf_size, const struct ::tm* tm_time, int usecs) {
    snprintf(
            buf,
            buf_size,
            "%04d-%02d-%02dT%02d:%02d:%02d.%06dZ",
            tm_time->tm_year + 1900,
            tm_time->tm_mon + 1,
            tm_time->tm_mday,
            tm_time->tm_hour,
            tm_time->tm_min,
            tm_time->tm_sec,
            usecs);
}

// Escape special JSON characters in a message.
std::string EscapeJsonString(const char* message, size_t len) {
    std::string result;
    result.reserve(len + 16);
    for (size_t i = 0; i < len; ++i) {
        char c = message[i];
        switch (c) {
        case '"':
            result += "\\\"";
            break;
        case '\\':
            result += "\\\\";
            break;
        case '\n':
            result += "\\n";
            break;
        case '\r':
            result += "\\r";
            break;
        case '\t':
            result += "\\t";
            break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                // Non-printable control characters are dropped.
                break;
            }
            result += c;
            break;
        }
    }
    return result;
}

// Convert glog severity enum to human-readable string.
std::string SeverityToString(int severity) {
    switch (severity) {
    case google::GLOG_INFO:
        return "INFO";
    case google::GLOG_WARNING:
        return "WARN";
    case google::GLOG_ERROR:
        return "ERROR";
    case google::GLOG_FATAL:
        return "FATAL";
    default:
        return "UNKNOWN";
    }
}

// Map glog severity to our configured log level.
int SeverityToLevel(int severity) {
    switch (severity) {
    case google::GLOG_FATAL:
        return 0;
    case google::GLOG_ERROR:
        return 1;
    case google::GLOG_INFO:
        return 2;  // SUMMARY, DECISION, STATE all use INFO
    case google::GLOG_WARNING:
        return 1;
    default:
        return 2;
    }
}

}  // namespace

// ============================================================================
// Constructor
// ============================================================================

PlanningLogSink::PlanningLogSink() : trace_start_time_(0), trace_rotate_minutes_(kDefaultTraceRotateMinutes) {
    // Determine log directory from gflag.
    log_dir_ = FLAGS_planning_log_dir;
    if (log_dir_.empty()) {
        log_dir_ = "data/log/planning";
    }
    trace_dir_ = log_dir_ + "/trace";

    // Build file paths.
    error_path_ = log_dir_ + "/error.log";
    summary_path_ = log_dir_ + "/summary.log";
    decision_path_ = log_dir_ + "/decision.log";

    // Configure trace rotation interval from gflag.
    if (FLAGS_planning_log_trace_rotate_minutes > 0) {
        trace_rotate_minutes_ = FLAGS_planning_log_trace_rotate_minutes;
    }

    // Create output directories recursively.
    boost::system::error_code ec;
    boost::filesystem::create_directories(log_dir_, ec);
    boost::filesystem::create_directories(trace_dir_, ec);

    // Open error writer (append mode).
    error_writer_.open(error_path_, std::ios::out | std::ios::app);
    // Open summary writer (append mode).
    summary_writer_.open(summary_path_, std::ios::out | std::ios::app);
    // Open decision writer (append mode).
    decision_writer_.open(decision_path_, std::ios::out | std::ios::app);

    // Prepare trace writer — will be opened on first write.
    trace_start_time_ = time(nullptr);
}

// ============================================================================
// Destructor
// ============================================================================

PlanningLogSink::~PlanningLogSink() {
    FlushAll();
    if (error_writer_.is_open())
        error_writer_.close();
    if (summary_writer_.is_open())
        summary_writer_.close();
    if (decision_writer_.is_open())
        decision_writer_.close();
    if (trace_writer_.is_open())
        trace_writer_.close();
    {
        std::lock_guard<std::mutex> lock(scenario_mutex_);
        for (auto& pair : scenario_writers_) {
            if (pair.second.is_open())
                pair.second.close();
        }
    }
}

// ============================================================================
// FlushAll
// ============================================================================

void PlanningLogSink::FlushAll() {
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        if (error_writer_.is_open())
            error_writer_.flush();
    }
    {
        std::lock_guard<std::mutex> lock(summary_mutex_);
        if (summary_writer_.is_open())
            summary_writer_.flush();
    }
    {
        std::lock_guard<std::mutex> lock(decision_mutex_);
        if (decision_writer_.is_open())
            decision_writer_.flush();
    }
    {
        std::lock_guard<std::mutex> lock(trace_mutex_);
        if (trace_writer_.is_open())
            trace_writer_.flush();
    }
    {
        std::lock_guard<std::mutex> lock(scenario_mutex_);
        for (auto& pair : scenario_writers_) {
            if (pair.second.is_open())
                pair.second.flush();
        }
    }
}

// ============================================================================
// IsLevelEnabled
// ============================================================================

bool PlanningLogSink::IsLevelEnabled(int level) const {
    return level <= FLAGS_planning_log_level;
}

// ============================================================================
// BuildJson — serialize log entry to a JSON line
// ============================================================================

std::string PlanningLogSink::BuildJson(
        google::LogSeverity severity,
        const char* base_filename,
        int line,
        const struct ::tm* tm_time,
        const char* message,
        size_t message_len) {
    char time_buf[32];
    // glog's LogMessage::Flush() provides usecs via the timeinfo struct,
    // but the exact usec field isn't in struct tm. We approximate with 0
    // because the microsecond is already baked into tm_time's formatting
    // when glog calls LogSink::send.
    FormatISO8601(time_buf, sizeof(time_buf), tm_time, 0);

    std::string escaped_msg = EscapeJsonString(message, message_len);

    pid_t tid = static_cast<pid_t>(syscall(SYS_gettid));

    const std::string& scenario = PlanningLogContext::scenario_name();
    const std::string& stage = PlanningLogContext::stage_name();
    uint32_t frame = PlanningLogContext::frame_seq();
    const std::string& mod = PlanningLogContext::planning_name();

    std::ostringstream json;
    json << "{"
         << "\"ts\":\"" << time_buf << "\","
         << "\"lvl\":\"" << SeverityToString(severity) << "\","
         << "\"mod\":\"" << mod << "\","
         << "\"scn\":\"" << scenario << "\","
         << "\"stg\":\"" << stage << "\","
         << "\"frm\":" << frame << ","
         << "\"src\":{\"file\":\"" << (base_filename ? base_filename : "unknown") << "\",\"line\":" << line << "},"
         << "\"tid\":" << tid << ","
         << "\"msg\":\"" << escaped_msg << "\""
         << "}";
    return json.str();
}

// ============================================================================
// WriteLine — thread-safe write to a file stream
// ============================================================================

void PlanningLogSink::WriteLine(std::ofstream& writer, std::mutex& mtx, const std::string& line) {
    std::lock_guard<std::mutex> lock(mtx);
    if (writer.is_open()) {
        writer << line << "\n";
    }
}

// ============================================================================
// RotateTraceIfNeeded — rotate trace file based on time interval
// ============================================================================

void PlanningLogSink::RotateTraceIfNeeded() {
    time_t now = time(nullptr);
    double elapsed_minutes = difftime(now, trace_start_time_) / 60.0;

    if (elapsed_minutes >= trace_rotate_minutes_) {
        // Close current trace writer
        if (trace_writer_.is_open()) {
            trace_writer_.close();
        }

        // Generate new trace filename with timestamp
        char time_buf[20];
        struct tm* tm_now = localtime(&now);
        snprintf(
                time_buf,
                sizeof(time_buf),
                "%04d%02d%02d_%02d%02d",
                tm_now->tm_year + 1900,
                tm_now->tm_mon + 1,
                tm_now->tm_mday,
                tm_now->tm_hour,
                tm_now->tm_min);

        std::string new_trace_path = trace_dir_ + "/" + time_buf + "_trace.jsonl";
        trace_writer_.open(new_trace_path, std::ios::out | std::ios::app);
        trace_start_time_ = now;
    }
}

// ============================================================================
// RouteMessage — route JSON line to appropriate file writer(s)
// ============================================================================

void PlanningLogSink::RouteMessage(
        const std::string& json_line,
        int level,
        const std::string& tag,
        const std::string& scenario) {
    // Route 1: ERROR/FATAL → error.log (always, regardless of level setting)
    if (level <= 1) {
        WriteLine(error_writer_, error_mutex_, json_line);
    }

    // Route 2: Tag-based routing
    if (tag == "summary") {
        WriteLine(summary_writer_, summary_mutex_, json_line);
    } else if (tag == "decision" || tag == "state") {
        WriteLine(decision_writer_, decision_mutex_, json_line);
    } else if (tag == "scenario") {
        if (FLAGS_planning_log_per_scenario) {
            std::lock_guard<std::mutex> lock(scenario_mutex_);
            auto it = scenario_writers_.find(scenario);
            if (it == scenario_writers_.end()) {
                std::string scenario_path = log_dir_ + "/per_scenario/" + scenario + ".jsonl";
                boost::system::error_code ec;
                boost::filesystem::create_directories(log_dir_ + "/per_scenario", ec);
                scenario_writers_[scenario].open(scenario_path, std::ios::out | std::ios::app);
                it = scenario_writers_.find(scenario);
            }
            if (it != scenario_writers_.end() && it->second.is_open()) {
                it->second << json_line << "\n";
            }
        } else {
            // Fallback: write scenario-tagged messages to decision.log
            WriteLine(decision_writer_, decision_mutex_, json_line);
        }
    } else {
        // Fallback: unrecognized tags → decision.log (captures all planning logs)
        WriteLine(decision_writer_, decision_mutex_, json_line);
        RotateTraceIfNeeded();
        if (trace_writer_.is_open()) {
            trace_writer_ << json_line << "\n";
        }
    }
}

// ============================================================================
// send() — main glog callback
// ============================================================================

void PlanningLogSink::send(
        google::LogSeverity severity,
        const char* full_filename,
        const char* base_filename,
        int line,
        const struct ::tm* tm_time,
        const char* message,
        size_t message_len) {
    (void)full_filename;  // Not used in JSON output

    int level = SeverityToLevel(severity);

    // Filter by configured log level (except errors which always pass through).
    if (level > 1 && !IsLevelEnabled(level)) {
        // Skip INFO-level messages that are below the configured threshold.
        // But ERROR/FATAL (level 0-1) always pass through.
        return;
    }

    // Build JSON line.
    std::string json_line = BuildJson(severity, base_filename, line, tm_time, message, message_len);

    // Parse message tag for routing decisions.
    // Tags are prefixes like "[SUMMARY]", "[DECISION]", "[STATE]", "[scn:".
    std::string tag;
    if (message_len >= 9 && std::strncmp(message, "[SUMMARY]", 9) == 0) {
        tag = "summary";
    } else if (message_len >= 10 && std::strncmp(message, "[DECISION]", 10) == 0) {
        tag = "decision";
    } else if (message_len >= 7 && std::strncmp(message, "[STATE]", 7) == 0) {
        tag = "state";
    } else if (message_len >= 5 && std::strncmp(message, "[scn:", 5) == 0) {
        tag = "scenario";
    }

    // Get current scenario name for per-scenario routing.
    const std::string& scenario = PlanningLogContext::scenario_name();

    // Route to appropriate file(s).
    RouteMessage(json_line, level, tag, scenario);
}

}  // namespace planning
}  // namespace apollo
