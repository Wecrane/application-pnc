/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/**
 * @file stage_intersection_cruise.cc
 **/

#include "modules/planning/scenarios/stop_sign_unprotected/stage_intersection_cruise.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/stop_sign_unprotected/context.h"

namespace apollo {
namespace planning {

StageResult StopSignUnprotectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "StopSignUnprotectedStageIntersectionCruise plan error";
  }

  // 跳过蠕行修复(2026-08-02): 车必须已过停止标志才判定IntersectionCruise完成。
  // 否则车还在停止标志前就FinishScenario, ScenarioManager重新触发停止标志场景
  // → 场景反复切换(STOP→CRUISE→STOP...)。跳过蠕行后车未进路口, 需先过停止标志。
  bool passed_stop_sign = true;
  const auto& stop_sign_status =
      injector_->planning_context()->planning_status().stop_sign();
  const std::string stop_sign_id =
      stop_sign_status.current_stop_sign_overlap_id();
  hdmap::PathOverlap* ss_overlap =
      frame->reference_line_info().front().GetOverlapOnReferenceLine(
          stop_sign_id, ReferenceLineInfo::STOP_SIGN);
  if (ss_overlap != nullptr) {
    const double adc_back_s =
        frame->reference_line_info().front().AdcSlBoundary().start_s();
    passed_stop_sign = adc_back_s >= ss_overlap->end_s;
  }
  bool stage_done = passed_stop_sign &&
                    CheckDone(*frame, injector_->planning_context(), false);
  if (stage_done) {
    return FinishStage();
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StopSignUnprotectedStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

hdmap::PathOverlap*
StopSignUnprotectedStageIntersectionCruise::GetTrafficSignOverlap(
    const ReferenceLineInfo& reference_line_info,
    const PlanningContext* context) const {
  // stop_sign scenarios
  const auto& stop_sign_status = context->planning_status().stop_sign();
  const std::string traffic_sign_overlap_id =
      stop_sign_status.current_stop_sign_overlap_id();
  hdmap::PathOverlap* traffic_sign_overlap =
      reference_line_info.GetOverlapOnReferenceLine(
          traffic_sign_overlap_id, ReferenceLineInfo::STOP_SIGN);
  return traffic_sign_overlap;
}

}  // namespace planning
}  // namespace apollo
