/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/
#include "modules/planning/scenarios/valet_parking/stage_dwelling_at_bus_bay.h"

#include <cmath>
#include <limits>

#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

namespace {

constexpr double kFallbackDistThreshold = 1.6;
constexpr double kFallbackHeadingThreshold = 0.45;
constexpr double kDwellDurationSeconds = 5.0;

bool EvaluateFallbackCompletion(
        const Frame& frame,
        bool open_space_done,
        double* dist_to_end,
        double* heading_err,
        double* opposite_heading_err) {
    const auto& os = frame.open_space_info();
    const auto& end_pose = os.open_space_end_pose();
    if (end_pose.size() < 4) {
        AINFO << "Bus-bay dwell: fallback skipped, end_pose size=" << end_pose.size();
        return false;
    }

    common::math::Vec2d end_xy(end_pose[0], end_pose[1]);
    end_xy.SelfRotate(os.origin_heading());
    end_xy += os.origin_point();
    double end_theta = common::math::NormalizeAngle(end_pose[2] + os.origin_heading());

    *dist_to_end = std::hypot(frame.vehicle_state().x() - end_xy.x(), frame.vehicle_state().y() - end_xy.y());
    *heading_err = std::fabs(common::math::AngleDiff(frame.vehicle_state().heading(), end_theta));
    *opposite_heading_err = std::fabs(common::math::AngleDiff(frame.vehicle_state().heading(), end_theta + M_PI));

    double speed = std::fabs(frame.vehicle_state().linear_velocity());
    double max_stop = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().max_abs_speed_when_stopped();

    bool fallback_ok = !open_space_done && speed <= max_stop && *dist_to_end <= kFallbackDistThreshold
            && (*heading_err <= kFallbackHeadingThreshold || *opposite_heading_err <= kFallbackHeadingThreshold);

    AINFO << "Bus-bay dwell: finish check, os_done=" << open_space_done << ", fallback=" << fallback_ok
          << ", dist_to_end=" << *dist_to_end << ", heading_err=" << *heading_err
          << ", opp_heading_err=" << *opposite_heading_err << ", speed=" << speed << ", max_stop=" << max_stop
          << ", adc=(" << frame.vehicle_state().x() << "," << frame.vehicle_state().y() << ","
          << frame.vehicle_state().heading() << "), end=(" << end_xy.x() << "," << end_xy.y() << "," << end_theta
          << ")";

    if (fallback_ok) {
        AINFO << "Bus-bay dwell: fallback accepted, dist=" << *dist_to_end << ", heading_err=" << *heading_err;
    }
    return fallback_ok;
}

}  // namespace

StageResult StageDwellingAtBusBay::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    auto* sc = GetContextAs<BusBayTransferContext>();
    sc->LatchStaticObstacles(*frame, "dwelling");
    sc->InjectLatchedStaticObstacles(frame);
    frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
    *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) = sc->target_parking_spot_id;

    StageResult result = ExecuteTaskOnOpenSpace(frame);
    if (result.HasError()) {
        AERROR << "Bus-bay dwell: planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    bool os_done = frame->open_space_info().openspace_planning_finish();
    double dist = std::numeric_limits<double>::infinity();
    double h_err = std::numeric_limits<double>::infinity();
    double opp_h_err = std::numeric_limits<double>::infinity();
    bool fallback = EvaluateFallbackCompletion(*frame, os_done, &dist, &h_err, &opp_h_err);

    if (!os_done && !fallback) {
        return result.SetStageStatus(StageStatusType::RUNNING);
    }

    // parking/docking reached — reset open space flag
    frame->mutable_open_space_info()->set_openspace_planning_finish(false);

    double adc_speed = std::fabs(frame->vehicle_state().linear_velocity());
    double max_stop = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().max_abs_speed_when_stopped();

    if (adc_speed > max_stop) {
        dwell_timer_active_ = false;
        AINFO << "Bus-bay dwell: reached but ADC moving, speed=" << adc_speed << ", max_stop=" << max_stop;
        return result.SetStageStatus(StageStatusType::RUNNING);
    }

    // ---- dwell timer ----
    double now = cyber::Clock::NowInSeconds();
    if (!dwell_timer_active_) {
        dwell_timer_active_ = true;
        dwell_start_timestamp_ = now;
        AINFO << "Bus-bay dwell: timer started, wait=" << kDwellDurationSeconds << "s";
    }
    double elapsed = now - dwell_start_timestamp_;
    AINFO << "Bus-bay dwell: waiting, elapsed=" << elapsed << ", target=" << kDwellDurationSeconds << "s";
    if (elapsed >= kDwellDurationSeconds) {
        AINFO << "Bus-bay dwell: complete, skip departing, return to lane follow directly";

        // Clear open-space flag so LaneFollow uses ReferenceLine tasks
        frame->mutable_open_space_info()->set_is_on_open_space_trajectory(false);
        frame->mutable_open_space_info()->set_openspace_planning_finish(false);

        // Mark destination as passed to suppress the destination stop wall,
        // so LaneFollow can drive past the parking spot without stopping.
        // This also serves as the "recently exited bus bay" signal that
        // HasForcedLaneBorrowContext() checks to enable all bus-bay bypasses
        // (CheckLaneBorrow, IsEnableNudge, boundary type ignore, etc.)
        // without forcing lane borrow pre-activation.
        injector_->planning_context()->mutable_planning_status()->mutable_destination()->set_has_passed_destination(
                true);
        AINFO << "Bus-bay dwell: marked destination as passed to remove stop wall";

        sc->shuttle_mission_completed = true;
        return FinishScenario();
    }

    return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StageDwellingAtBusBay::FinishStage() {
    return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
