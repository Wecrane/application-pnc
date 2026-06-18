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

#include "modules/planning/scenarios/valet_parking/stage_departing_from_bus_bay.h"

#include <cmath>
#include <limits>

#include "modules/common/math/math_utils.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

namespace {

constexpr double kMaxReturnLateral = 1.5;
constexpr double kMaxReturnHeadingDiff = 0.5;
constexpr double kMinEarlyReturnSpeed = 0.4;
constexpr double kMaxEarlyReturnDist = 15.0;

}  // namespace

StageResult StageDepartingFromBusBay::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    (void)planning_init_point;
    CHECK_NOTNULL(frame);
    StageResult result;

    if (!departing_status_initialized_) {
        InitDepartingStatus(frame);
        departing_status_initialized_ = true;
    }

    auto* sc = GetContextAs<BusBayTransferContext>();
    sc->LatchStaticObstacles(*frame, "departing");
    sc->InjectLatchedStaticObstacles(frame);
    frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);

    result = ExecuteTaskOnOpenSpace(frame);
    LogDepartingRoiDiagnostics(*frame);
    if (result.HasError()) {
        AERROR << "Bus-bay departing: planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    // ---- finish evaluation ----
    const bool os_done = frame->open_space_info().openspace_planning_finish();
    const bool return_ready = CheckReadyToReturnLaneFollow(*frame);
    double adc_speed = frame->vehicle_state().linear_velocity();

    const auto& os_info = frame->open_space_info();
    const auto& end_pose = os_info.open_space_end_pose();
    const auto& origin = os_info.origin_point();
    double origin_h = os_info.origin_heading();

    common::math::Vec2d local_xy(frame->vehicle_state().x(), frame->vehicle_state().y());
    local_xy -= origin;
    local_xy.SelfRotate(-origin_h);

    double dist_to_end = std::numeric_limits<double>::infinity();
    double x_remaining = std::numeric_limits<double>::infinity();
    if (end_pose.size() >= 2) {
        common::math::Vec2d target(end_pose[0], end_pose[1]);
        dist_to_end = local_xy.DistanceTo(target);
        x_remaining = end_pose[0] - local_xy.x();
    }

    bool near_end = dist_to_end < kMaxEarlyReturnDist;
    bool early = !os_done && return_ready && adc_speed > kMinEarlyReturnSpeed && near_end;

    AINFO << "Bus-bay departing: finish eval, os_done=" << os_done << ", return_ready=" << return_ready
          << ", early=" << early << ", speed=" << adc_speed << ", min_early_spd=" << kMinEarlyReturnSpeed
          << ", near_end=" << near_end << ", dist=" << dist_to_end << ", max_early_dist=" << kMaxEarlyReturnDist
          << ", x_to_go=" << x_remaining << ", local_xy=(" << local_xy.x() << "," << local_xy.y()
          << "), end_pose_sz=" << end_pose.size();

    if (os_done || early) {
        frame->mutable_open_space_info()->set_openspace_planning_finish(false);
        if (return_ready) {
            sc->shuttle_mission_completed = true;
            AINFO << "Bus-bay departing: done, return to lane follow, early=" << early << ", speed=" << adc_speed;
            return FinishScenario();
        }
    }

    return result.SetStageStatus(StageStatusType::RUNNING);
}

void StageDepartingFromBusBay::InitDepartingStatus(Frame* frame) {
    auto* status = injector_->planning_context()->mutable_planning_status()->mutable_park_and_go();
    status->Clear();
    const auto& vs = frame->vehicle_state();
    status->mutable_adc_init_position()->set_x(vs.x());
    status->mutable_adc_init_position()->set_y(vs.y());
    status->mutable_adc_init_position()->set_z(0.0);
    status->set_adc_init_heading(vs.heading());
    status->set_in_check_stage(false);
    AINFO << "Bus-bay departing: init status, x=" << vs.x() << ", y=" << vs.y() << ", h=" << vs.heading();
}

void StageDepartingFromBusBay::LogDepartingRoiDiagnostics(const Frame& frame) const {
    const auto& os = frame.open_space_info();
    const auto& roi = os.ROI_xy_boundary();
    const auto& end_pose = os.open_space_end_pose();
    const auto& origin = os.origin_point();
    double origin_h = os.origin_heading();

    common::math::Vec2d local(frame.vehicle_state().x(), frame.vehicle_state().y());
    local -= origin;
    local.SelfRotate(-origin_h);

    bool adc_in = false;
    bool end_in = false;
    if (roi.size() >= 4) {
        adc_in = local.x() >= roi[0] && local.x() <= roi[1] && local.y() >= roi[2] && local.y() <= roi[3];
        if (end_pose.size() >= 2) {
            end_in = end_pose[0] >= roi[0] && end_pose[0] <= roi[1] && end_pose[1] >= roi[2] && end_pose[1] <= roi[3];
        }
    }

    AINFO << "Bus-bay departing: ROI log, origin=(" << origin.x() << "," << origin.y() << "), origin_h=" << origin_h
          << ", local_xy=(" << local.x() << "," << local.y() << "), roi_sz=" << roi.size() << ", adc_in_roi=" << adc_in
          << ", end_sz=" << end_pose.size() << ", end_in_roi=" << end_in;
    if (roi.size() >= 4) {
        AINFO << "Bus-bay departing: ROI bounds, x=[" << roi[0] << "," << roi[1] << "], y=[" << roi[2] << "," << roi[3]
              << "]";
    }
    if (end_pose.size() >= 4) {
        AINFO << "Bus-bay departing: end_pose, x=" << end_pose[0] << ", y=" << end_pose[1] << ", theta=" << end_pose[2]
              << ", v=" << end_pose[3];
    }
}

bool StageDepartingFromBusBay::CheckReadyToReturnLaneFollow(const Frame& frame) const {
    if (frame.reference_line_info().empty()) {
        AINFO << "Bus-bay departing: not ready (no ref line)";
        return false;
    }

    const auto& vs = frame.vehicle_state();
    common::math::Vec2d adc_pos(vs.x(), vs.y());
    const auto& rl = frame.reference_line_info().front().reference_line();

    common::SLPoint sl;
    rl.XYToSL(adc_pos, &sl);
    auto ref_pt = rl.GetReferencePoint(sl.s());
    double h_diff = std::fabs(common::math::NormalizeAngle(vs.heading() - ref_pt.heading()));

    bool ready = std::fabs(sl.l()) < kMaxReturnLateral && h_diff < kMaxReturnHeadingDiff;
    AINFO << "Bus-bay departing: return check, s=" << sl.s() << ", l=" << sl.l() << ", adc_h=" << vs.heading()
          << ", ref_h=" << ref_pt.heading() << ", h_diff=" << h_diff << ", ready=" << ready;
    return ready;
}

}  // namespace planning
}  // namespace apollo
