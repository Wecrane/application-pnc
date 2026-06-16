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

#include <ctime>
#include <fstream>
#include <map>
#include <mutex>
#include <string>

#include <glog/logging.h>

namespace apollo {
namespace planning {

// PlanningLogSink — custom glog sink that intercepts all glog messages
// and routes them to structured JSON log files based on severity, tag,
// scenario, and log level configuration.
//
// File routing:
//   - FATAL/ERROR messages with no tag  → error.log
//   - Messages containing "[SUMMARY]"   → summary.log
//   - Messages containing "[DECISION]"  → decision.log
//     or "[STATE]"
//   - Messages containing "[scn:"       → per_scenario/<scenario>.jsonl
//     (when per-scenario mode enabled)
//   - All messages (when trace level)   → trace/<timestamp>_trace.jsonl
//
// Thread safety: all file writers are protected by std::mutex.
// Directory creation uses boost::filesystem.
class PlanningLogSink : public google::LogSink {
 public:
  PlanningLogSink();
  virtual ~PlanningLogSink();

  // Called by glog for every log message.
  // We serialize to JSON and route to appropriate file(s).
  void send(google::LogSeverity severity, const char* full_filename,
            const char* base_filename, int line, const struct ::tm* tm_time,
            const char* message, size_t message_len) override;

  // Disable glog's wait mechanism — we flush on our own schedule.
  void WaitTillSent() override {}

  // Force flush all open file writers.
  void FlushAll();

 private:
  // ---- JSON serialization ----
  std::string BuildJson(google::LogSeverity severity, const char* base_filename,
                        int line, const struct ::tm* tm_time,
                        const char* message, size_t message_len);

  // ---- File routing ----
  void RouteMessage(const std::string& json_line, int level,
                    const std::string& tag, const std::string& scenario);

  void WriteLine(std::ofstream& writer, std::mutex& mtx,
                 const std::string& line);
  void RotateTraceIfNeeded();

  // Check if a log message level meets the configured threshold.
  bool IsLevelEnabled(int level) const;

  // ---- Directory / file paths ----
  std::string log_dir_;
  std::string trace_dir_;
  std::string error_path_;
  std::string summary_path_;
  std::string decision_path_;

  // ---- File writers ----
  std::ofstream error_writer_;
  std::ofstream summary_writer_;
  std::ofstream decision_writer_;
  std::ofstream trace_writer_;
  std::map<std::string, std::ofstream> scenario_writers_;

  // ---- Mutexes for thread safety ----
  std::mutex error_mutex_;
  std::mutex summary_mutex_;
  std::mutex decision_mutex_;
  std::mutex trace_mutex_;
  std::mutex scenario_mutex_;

  // ---- Trace rotation state ----
  time_t trace_start_time_;
  int trace_rotate_minutes_;
};

}  // namespace planning
}  // namespace apollo
