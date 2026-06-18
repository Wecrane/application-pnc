/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/contest_lane_borrow_path/reverse_recovery_helper.h"

#include <algorithm>
#include <cmath>

#include "cyber/common/log.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace planning {

namespace {

constexpr double kReverseSampleStep = -0.1;
constexpr double kReferenceLineStartPadding = 0.5;
constexpr double kLateralMargin = 0.5;
constexpr double kLateralFallbackMargin = 0.2;
constexpr double kStraightReferenceLineStep = 0.5;
constexpr double kMinStraightReferenceLineLength = 12.0;
constexpr double kStraightReferenceLineMargin = 4.0;

}  // namespace

bool BuildReverseStraightReferenceLine(
        ReverseRecoveryState* state,
        double adc_x,
        double adc_y,
        double adc_heading,
        double reverse_distance) {
    if (state == nullptr) {
        return false;
    }
    const double straight_ref_length
            = std::max(kMinStraightReferenceLineLength, reverse_distance + kStraightReferenceLineMargin);
    if (straight_ref_length < reverse_distance + 1.0) {
        AERROR << "[CZ][REVERSE] straight reference line length is too short.";
        return false;
    }

    // 查找 ADC 最近车道
    hdmap::LaneInfoConstPtr main_lane;
    double lane_s = 0.0;
    double lane_l = 0.0;
    common::PointENU adc_point;
    adc_point.set_x(adc_x);
    adc_point.set_y(adc_y);
    adc_point.set_z(0.0);
    hdmap::HDMapUtil::BaseMap().GetNearestLaneWithDistance(adc_point, 5.0, &main_lane, &lane_s, &lane_l);
    if (main_lane == nullptr) {
        AERROR << "[CZ][REVERSE] failed to build straight reference line: no nearby lane.";
        return false;
    }

    const double heading = adc_heading;
    const double cos_heading = std::cos(heading);
    const double sin_heading = std::sin(heading);
    const double ref_start_x = adc_x - straight_ref_length * cos_heading;
    const double ref_start_y = adc_y - straight_ref_length * sin_heading;
    const double adc_ref_s = straight_ref_length;

    std::vector<ReferencePoint> ref_points;
    ref_points.reserve(static_cast<size_t>((straight_ref_length * 2.0) / kStraightReferenceLineStep + 1.0));
    for (double ref_s = 0.0; ref_s <= straight_ref_length * 2.0 + 1e-6; ref_s += kStraightReferenceLineStep) {
        const double px = ref_start_x + ref_s * cos_heading;
        const double py = ref_start_y + ref_s * sin_heading;
        hdmap::MapPathPoint map_point({px, py}, heading);
        hdmap::LaneWaypoint lane_waypoint;
        lane_waypoint.lane = main_lane;
        lane_waypoint.s = std::max(0.0, lane_s + ref_s - adc_ref_s);
        map_point.add_lane_waypoint(lane_waypoint);
        ref_points.emplace_back(map_point, 0.0, 0.0);
    }

    if (ref_points.size() < 2) {
        AERROR << "[CZ][REVERSE] failed to build straight reference line: insufficient points.";
        return false;
    }
    state->reference_line_cache = std::make_unique<ReferenceLine>(ref_points);
    state->fixed_start_s = adc_ref_s;
    state->fixed_end_s = adc_ref_s - reverse_distance;
    state->fixed_l = 0.0;
    state->fixed_start_x = adc_x;
    state->fixed_start_y = adc_y;
    state->fixed_heading = heading;

    // ── 模拟站点接驳倒车策略：直接在倒车参考线上注入限速 ──
    // 站点接驳 OpenSpace 使用 max_speed_reverse: 1.0 m/s 用于精细泊车，
    // 施工区倒车是直线远离锥桶，可用更高速度。
    // 三段式速度曲线（加速→匀速→减速）由 PiecewiseJerkSpeedOptimizer 生成，
    // 此处的 AddSpeedLimit 提供速度上界约束，确保不超速。
    constexpr double kReverseSpeedLimit = 5.0;  // m/s (~18 km/h)，快速倒车
    state->reference_line_cache->AddSpeedLimit(state->fixed_end_s, state->fixed_start_s, kReverseSpeedLimit);

    AINFO << "[CZ][REVERSE] built straight reference line, points=" << ref_points.size() << ", ref_start_xy=("
          << ref_start_x << "," << ref_start_y << "), adc_ref_s=" << adc_ref_s << ", end_s=" << state->fixed_end_s
          << ", reverse_distance=" << reverse_distance << ", speed_limit=" << kReverseSpeedLimit
          << ", heading=" << heading << ", lane_s=" << lane_s << ", lane_l=" << lane_l;
    return true;
}

bool GenerateCachedReversePathBoundary(const ReverseRecoveryState& state, PathBoundary* boundary) {
    if (boundary == nullptr || state.reference_line_cache == nullptr) {
        return false;
    }
    boundary->clear();
    const double adc_s = state.fixed_start_s;
    const double adc_l = state.fixed_l;
    const double actual_target = state.fixed_end_s;

    if (adc_s <= actual_target) {
        AERROR << "[CZ][REVERSE] failed to generate cached reverse boundary: invalid s range, start=" << adc_s
               << ", end=" << actual_target;
        return false;
    }

    boundary->set_delta_s(kReverseSampleStep);
    boundary->set_label("regular/reverse_path");

    const ReferenceLine& ref_line = *state.reference_line_cache;
    for (double s = adc_s; s > actual_target; s += kReverseSampleStep) {
        double lane_left = 0.0, lane_right = 0.0;
        if (!ref_line.GetLaneWidth(s, &lane_left, &lane_right)) {
            break;
        }
        double offset = 0.0;
        ref_line.GetOffsetToMap(s, &offset);
        const double left_bound = lane_left - offset;
        const double right_bound = -lane_right - offset;

        double l_lower = std::max(right_bound, adc_l - kLateralMargin);
        double l_upper = std::min(left_bound, adc_l + kLateralMargin);

        if (adc_l > l_upper) {
            l_upper = std::min(left_bound, adc_l + kLateralFallbackMargin);
        }
        if (adc_l < l_lower) {
            l_lower = std::max(right_bound, adc_l - kLateralFallbackMargin);
        }

        if (l_lower >= l_upper) {
            l_lower = right_bound;
            l_upper = left_bound;
        }
        boundary->emplace_back(s, l_lower, l_upper);
    }

    if (boundary->empty()) {
        AERROR << "[CZ][REVERSE] failed to generate cached reverse boundary";
        return false;
    }

    AINFO << "[CZ][REVERSE] cached boundary: s=[" << boundary->back().s << "," << boundary->front().s
          << "], fixed_l=" << adc_l << ", points=" << boundary->size();
    return true;
}

bool GenerateReverseRecoveryPathBoundary(
        const ReferenceLine& reference_line,
        double adc_s,
        double adc_l,
        double reverse_distance,
        PathBoundary* boundary) {
    if (boundary == nullptr) {
        return false;
    }
    boundary->clear();

    const double target_s = adc_s - reverse_distance;
    const double ref_min_s = reference_line.GetMapPath().accumulated_s().front();
    const double actual_target = std::max(target_s, ref_min_s + kReferenceLineStartPadding);

    boundary->set_delta_s(kReverseSampleStep);
    boundary->set_label("regular/reverse_path");

    for (double s = adc_s; s > actual_target; s += kReverseSampleStep) {
        double lane_left = 0.0;
        double lane_right = 0.0;
        if (!reference_line.GetLaneWidth(s, &lane_left, &lane_right)) {
            break;
        }

        double offset = 0.0;
        reference_line.GetOffsetToMap(s, &offset);
        const double left_bound = lane_left - offset;
        const double right_bound = -lane_right - offset;

        double l_lower = std::max(right_bound, adc_l - kLateralMargin);
        double l_upper = std::min(left_bound, adc_l + kLateralMargin);

        if (adc_l > l_upper) {
            l_upper = std::min(left_bound, adc_l + kLateralFallbackMargin);
        }
        if (adc_l < l_lower) {
            l_lower = std::max(right_bound, adc_l - kLateralFallbackMargin);
        }

        if (l_lower >= l_upper) {
            l_lower = right_bound;
            l_upper = left_bound;
        }
        boundary->emplace_back(s, l_lower, l_upper);
    }

    if (boundary->empty()) {
        AERROR << "[REVERSE] Failed to generate reverse boundary";
        return false;
    }

    AINFO << "[REVERSE] Boundary: s=[" << boundary->back().s << "," << boundary->front().s
          << "], points=" << boundary->size();
    return true;
}

}  // namespace planning
}  // namespace apollo
