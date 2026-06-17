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
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
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
                // Escape non-printable control characters as \uXXXX (RFC 8259).
                char buf[8];
                snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
                result += buf;
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
// NOTE: GLOG_WARNING is mapped to level -1 to distinguish it from ERROR.
// WARNING messages are always written but do NOT go to error.log.
int SeverityToLevel(int severity) {
    switch (severity) {
    case google::GLOG_FATAL:
        return 0;
    case google::GLOG_ERROR:
        return 1;
    case google::GLOG_INFO:
        return 2;  // SUMMARY, DECISION, STATE all use INFO
    case google::GLOG_WARNING:
        return -1;  // Always enabled, separate from ERROR
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
    if (!error_writer_.is_open()) {
        std::cerr << "[PLANLOG] ERROR: Failed to open error log: " << error_path_ << std::endl;
    }
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
    { std::lock_guard<std::mutex> lock(error_mutex_);
      if (error_writer_.is_open()) error_writer_.close(); }
    { std::lock_guard<std::mutex> lock(summary_mutex_);
      if (summary_writer_.is_open()) summary_writer_.close(); }
    { std::lock_guard<std::mutex> lock(decision_mutex_);
      if (decision_writer_.is_open()) decision_writer_.close(); }
    { std::lock_guard<std::mutex> lock(trace_mutex_);
      if (trace_writer_.is_open()) trace_writer_.close(); }
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

// Extract key=value pairs from message body.
// Parses patterns like: key1=val1 key2=2.5 key3=text
// Values containing spaces are not supported (stop at next space).
// Returns a JSON object string like {"key1":"val1","key2":2.5,"key3":"text"},
// or empty string if no key=value pairs found.
std::string PlanningLogSink::ExtractDatFields(const char* message, size_t message_len) {
    if (message == nullptr || message_len == 0)
        return "";

    std::string msg(message, message_len);

    // Skip leading tag prefix like "[SUMMARY] " or "[DECISION] " or "[scn:XXX] "
    size_t body_start = 0;
    if (msg.size() >= 2 && msg[0] == '[') {
        size_t close_bracket = msg.find("] ");
        if (close_bracket != std::string::npos) {
            body_start = close_bracket + 2;  // skip past "] "
        }
    }

    std::string body = msg.substr(body_start);
    if (body.empty())
        return "";

    std::ostringstream dat;
    dat << "{";
    bool first = true;
    size_t pos = 0;

    while (pos < body.size()) {
        // Find '=' sign
        size_t eq = body.find('=', pos);
        if (eq == std::string::npos)
            break;

        // Extract key (from pos to eq, backtrack to last space)
        size_t key_start = pos;
        size_t key_end = eq;
        // Find the start of this key (previous space or beginning)
        size_t space_before = body.rfind(' ', eq);
        if (space_before != std::string::npos && space_before >= pos) {
            key_start = space_before + 1;
        }

        std::string key = body.substr(key_start, key_end - key_start);
        if (key.empty()) {
            pos = eq + 1;
            continue;
        }

        // Extract value (from eq+1 to next space or end)
        size_t val_start = eq + 1;
        size_t val_end = body.find(' ', val_start);
        if (val_end == std::string::npos)
            val_end = body.size();

        std::string value = body.substr(val_start, val_end - val_start);
        if (value.empty()) {
            pos = val_end;
            continue;
        }

        // Determine if value is numeric or string.
        // Supports: integers, decimals, negatives, scientific notation (e.g. 1.5e10).
        bool is_numeric = true;
        bool has_dot = false;
        bool has_digit = false;
        bool has_exp = false;
        for (size_t i = 0; i < value.size(); ++i) {
            char c = value[i];
            if (c == '-' || c == '+') {
                // Sign only valid at start or immediately after 'e'/'E'.
                if (i == 0 || (i > 0 && (value[i-1] == 'e' || value[i-1] == 'E'))) {
                    continue;
                }
                is_numeric = false;
                break;
            }
            if (c == '.' && !has_dot && !has_exp) {
                has_dot = true;
                continue;
            }
            if ((c == 'e' || c == 'E') && !has_exp && has_digit) {
                has_exp = true;
                has_dot = false;  // no more dots after exponent
                continue;
            }
            if (c >= '0' && c <= '9') {
                has_digit = true;
                continue;
            }
            is_numeric = false;
            break;
        }
        // Must contain at least one digit; single '-' or '+' is not numeric.
        if (!has_digit) {
            is_numeric = false;
        }

        if (!first)
            dat << ",";
        first = false;

        // Escape key and value for JSON
        dat << "\"" << EscapeJsonString(key.c_str(), key.size()) << "\":";
        if (is_numeric) {
            dat << value;
        } else {
            dat << "\"" << EscapeJsonString(value.c_str(), value.size()) << "\"";
        }

        pos = val_end;
    }

    dat << "}";

    // Only return if we actually found some fields
    if (first)
        return "";  // no fields found
    return dat.str();
}

std::string PlanningLogSink::BuildJson(
        google::LogSeverity severity,
        const char* base_filename,
        int line,
        const struct ::tm* tm_time,
        const char* message,
        size_t message_len) {
    // Get real microsecond timestamp (glog only provides second-level tm_time).
    auto now = std::chrono::system_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count() % 1000000;
    char time_buf[32];
    FormatISO8601(time_buf, sizeof(time_buf), tm_time, static_cast<int>(us));

    std::string escaped_msg = EscapeJsonString(message, message_len);

    pid_t tid = static_cast<pid_t>(syscall(SYS_gettid));

    // Escape context fields that may contain special JSON characters.
    const std::string& scenario = PlanningLogContext::scenario_name();
    const std::string& stage = PlanningLogContext::stage_name();
    uint32_t frame = PlanningLogContext::frame_seq();
    const std::string& mod = PlanningLogContext::planning_name();
    std::string escaped_mod = EscapeJsonString(mod.c_str(), mod.size());
    std::string escaped_scenario = EscapeJsonString(scenario.c_str(), scenario.size());
    std::string escaped_stage = EscapeJsonString(stage.c_str(), stage.size());
    std::string escaped_filename = EscapeJsonString(
        (base_filename ? base_filename : "unknown"),
        (base_filename ? std::strlen(base_filename) : 7));

    std::ostringstream json;
    json << "{"
         << "\"ts\":\"" << time_buf << "\","
         << "\"lvl\":\"" << SeverityToString(severity) << "\","
         << "\"mod\":\"" << escaped_mod << "\","
         << "\"scn\":\"" << escaped_scenario << "\","
         << "\"stg\":\"" << escaped_stage << "\","
         << "\"frm\":" << frame << ","
         << "\"src\":{\"file\":\"" << escaped_filename << "\",\"line\":" << line << "},"
         << "\"tid\":" << tid << ","
         << "\"msg\":\"" << escaped_msg << "\"";

    // Extract structured key=value pairs from message body for "dat" field.
    std::string dat_json = ExtractDatFields(message, message_len);
    if (!dat_json.empty()) {
        json << ",\"dat\":" << dat_json;
    }

    json << "}";
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

    // Determine if we need to open a new trace file.
    bool need_open = false;
    if (!trace_writer_.is_open()) {
        // First-time open: ensure trace file is available immediately.
        need_open = true;
    } else {
        double elapsed_minutes = difftime(now, trace_start_time_) / 60.0;
        if (elapsed_minutes >= trace_rotate_minutes_) {
            // Close current trace writer before rotating.
            trace_writer_.close();
            need_open = true;
        }
    }

    if (!need_open) return;

    // Generate new trace filename with timestamp.
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
    }

    // Route 3: Trace mode — write ALL messages when trace enabled
    if (FLAGS_planning_log_level >= 5) {
        std::lock_guard<std::mutex> lock(trace_mutex_);
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

    // Parse message tag for routing decisions and tag-based level filtering.
    // Tags are prefixes like "[SUMMARY]", "[DECISION]", "[STATE]", "[scn:".
    std::string tag;
    int tag_level = 2;  // default: INFO level
    if (message_len >= 9 && std::strncmp(message, "[SUMMARY]", 9) == 0) {
        tag = "summary";
        tag_level = 2;
    } else if (message_len >= 10 && std::strncmp(message, "[DECISION]", 10) == 0) {
        tag = "decision";
        tag_level = 3;
    } else if (message_len >= 7 && std::strncmp(message, "[STATE]", 7) == 0) {
        tag = "state";
        tag_level = 4;
    } else if (message_len >= 5 && std::strncmp(message, "[scn:", 5) == 0) {
        tag = "scenario";
        tag_level = 2;
    } else if (message_len >= 5 && std::strncmp(message, "[stg:", 5) == 0) {
        tag = "scenario";  // Stage messages route similarly to scenario
        tag_level = 4;
    } else if (message_len >= 5 && std::strncmp(message, "[frm:", 5) == 0) {
        tag = "scenario";  // Frame-level messages route similarly
        tag_level = 2;
    }

    // Filter by configured log level.
    // FATAL/ERROR (severity-level 0-1) always pass through.
    // For INFO-level messages, use tag_level for fine-grained filtering
    // (SUMMARY=2, DECISION=3, STATE=4) so that --planning_log_level=N
    // correctly distinguishes these semantic levels.
    if (level <= 1) {
        // FATAL/ERROR always pass through.
    } else if (level == -1) {
        // WARNING always enabled, no filtering.
    } else if (tag_level > FLAGS_planning_log_level) {
        return;  // Below configured threshold.
    }

    // Build JSON line.
    std::string json_line = BuildJson(severity, base_filename, line, tm_time, message, message_len);

    // Get current scenario name for per-scenario routing.
    const std::string& scenario = PlanningLogContext::scenario_name();

    // Route to appropriate file(s).
    RouteMessage(json_line, level, tag, scenario);
}

}  // namespace planning
}  // namespace apollo
