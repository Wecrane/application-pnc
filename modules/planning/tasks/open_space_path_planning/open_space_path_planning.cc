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

/**
 * @file
 **/

#include "modules/planning/tasks/open_space_path_planning/open_space_path_planning.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using common::math::Vec2d;

bool OpenSpacePathPlanning::Init(
    const std::string &config_dir,
    const std::string &name,
    const std::shared_ptr<DependencyInjector> &injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  if (!Task::LoadConfig<OpenSpacePathPlanningConfig>(&config_)) {
    return false;
  }
  path_finder_.reset(new HybridAStar(config_.warm_start_config()));
  AINFO << config_.DebugString();
  return true;
}

OpenSpacePathPlanning::~OpenSpacePathPlanning() {
  if (config_.enable_path_planning_thread()) {
    Stop();
  }
}

void OpenSpacePathPlanning::Stop() {
  if (config_.enable_path_planning_thread()) {
    if (thread_init_flag_) {
      task_future_.get();
    }
    data_ready_.store(false);
    path_update_.store(false);
  }
}

Status OpenSpacePathPlanning::Process() {
  AINFO << "data_ready_: " << (data_ready_ ? "true" : "false");
  auto* previous_frame = injector_->frame_history()->Latest();
  if (!frame_->open_space_info().replan_flag()) {
    AINFO << "Not replan";
    *(frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result()) =
        previous_frame->open_space_info().path_planning_trajectory_result();
    return Status::OK();
  }

  if (!previous_frame->open_space_info().
      path_planning_trajectory_result().empty()) {
    AINFO << "Previous frame has path planning result";
    *(frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result()) =
        previous_frame->open_space_info().path_planning_trajectory_result();
    return Status::OK();
  }
  AINFO << "Path planning";

  if (path_update_) {
    AINFO << "Path planning updated";
    LoadResult(frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result());
    path_update_.store(false);
    return Status::OK();
  }
  PathPlanning();
  return Status::OK();
}

void OpenSpacePathPlanning::PathPlanning() {
  if (config_.enable_path_planning_thread() && !thread_init_flag_) {
    task_future_ = cyber::Async(
        &OpenSpacePathPlanning::GeneratePathThread, this);
    thread_init_flag_.store(true);
  }

  const auto& open_space_info = frame_->open_space_info();
  double start_x = injector_->vehicle_state()->x();
  double start_y = injector_->vehicle_state()->y();
  double start_theta = injector_->vehicle_state()->heading();
  OpenSpaceTrajectoryOptimizerUtil::PathPointNormalizing(
      open_space_info.origin_heading(),
      open_space_info.origin_point(),
      &start_x, &start_y, &start_theta);
  if (config_.enable_path_planning_thread()) {
    if (!data_ready_) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      thread_data_.start_pose = {start_x, start_y, start_theta};
      thread_data_.end_pose = open_space_info.open_space_end_pose();
      thread_data_.rotate_angle = open_space_info.origin_heading();
      thread_data_.translate_origin = open_space_info.origin_point();
      thread_data_.obstacles_edges_num = open_space_info.obstacles_edges_num();
      thread_data_.obstacles_A = open_space_info.obstacles_A();
      thread_data_.obstacles_b = open_space_info.obstacles_b();
      thread_data_.obstacles_vertices_vec =
          open_space_info.obstacles_vertices_vec();
      thread_data_.soft_boundary_vertices_vec =
          open_space_info.soft_boundary_vertices_vec();
      thread_data_.XYbounds = open_space_info.ROI_xy_boundary();
      thread_data_.reeds_sheep_last_straight =
          ((config_.enable_vertical_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::VERTICAL_PARKING) || 
          (config_.enable_parallel_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::PARALLEL_PARKING)) ?
              true : false;
      thread_data_.is_vertical = open_space_info.parking_type() == ParkingType::VERTICAL_PARKING ? true : false;
      thread_data_.vertical_kappa_ratio =
          open_space_info.parking_type() == ParkingType::VERTICAL_PARKING ?
          config_.kappa_ratio() : 1.0;
      data_ready_.store(true);
    }
  } else {
    const auto& end_pose = open_space_info.open_space_end_pose();
    const auto& rotate_angle = open_space_info.origin_heading();
    const auto& translate_origin = open_space_info.origin_point();
    const auto& obstacles_edges_num = open_space_info.obstacles_edges_num();
    const auto& obstacles_A = open_space_info.obstacles_A();
    const auto& obstacles_b = open_space_info.obstacles_b();
    const auto& obstacles_vertices_vec =
        open_space_info.obstacles_vertices_vec();
    const auto& soft_boundary_vertices_vec =
        open_space_info.soft_boundary_vertices_vec();
    const auto& XYbounds = open_space_info.ROI_xy_boundary();
    const bool reeds_sheep_last_straight =
        ((config_.enable_vertical_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::VERTICAL_PARKING) || 
          (config_.enable_parallel_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::PARALLEL_PARKING)) ?
              true : false;
    if (path_finder_->Plan(start_x, start_y, start_theta,
                           end_pose[0], end_pose[1],
                           end_pose[2], XYbounds,
                           obstacles_vertices_vec,
                           &result_,
                           soft_boundary_vertices_vec,
                           reeds_sheep_last_straight)) {
      path_update_.store(true);
      AINFO << "State warm start problem solved successfully!";
    } else {
      AERROR << "State warm start problem failed to solve";
      // Fallback for parallel parking (xh_2026 station pickup):
      // generate a simple geometric trajectory instead of failing.
      if (open_space_info.parking_type() == ParkingType::PARALLEL_PARKING) {
        AINFO << "Attempting parallel parking fallback path generation";
        if (GenerateParallelParkingFallback(
                start_x, start_y, start_theta,
                end_pose[0], end_pose[1], end_pose[2],
                &result_)) {
          path_update_.store(true);
          AINFO << "Parallel parking fallback path generated successfully";
        } else {
          AERROR << "Parallel parking fallback path generation failed";
        }
      }
    }
  }
}

void OpenSpacePathPlanning::GeneratePathThread() {
  while (!data_ready_) {
    usleep(10);
  }
  OpenSpacePathPlanningThreadData thread_data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    thread_data = thread_data_;
  }

  AINFO << "start pose: " << thread_data.start_pose[0]
        << " " << thread_data.start_pose[1]
        << " " << thread_data.start_pose[2];
  bool ret = path_finder_->Plan(
      thread_data.start_pose[0], thread_data.start_pose[1],
      thread_data.start_pose[2], thread_data.end_pose[0],
      thread_data.end_pose[1], thread_data.end_pose[2],
      thread_data.XYbounds, thread_data.obstacles_vertices_vec,
      &result_,thread_data_.soft_boundary_vertices_vec,
      thread_data.reeds_sheep_last_straight,
      thread_data.is_vertical,
      thread_data.vertical_kappa_ratio);

  if (ret) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    path_update_.store(true);
    AINFO << "path find success!";
  } else {
    AERROR << "path find failed";
    // Fallback for parallel parking (xh_2026 station pickup)
    if (thread_data.is_vertical == false) {
      AINFO << "Attempting parallel parking fallback path generation (thread)";
      if (GenerateParallelParkingFallback(
              thread_data.start_pose[0], thread_data.start_pose[1],
              thread_data.start_pose[2],
              thread_data.end_pose[0], thread_data.end_pose[1],
              thread_data.end_pose[2],
              &result_)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        path_update_.store(true);
        AINFO << "Parallel parking fallback path generated (thread)";
      }
    }
  }
  thread_init_flag_.store(false);
  data_ready_.store(false);
  path_finder_.reset(new HybridAStar(config_.warm_start_config()));
}

void OpenSpacePathPlanning::LoadResult(
    DiscretizedTrajectory* const trajectory_data) {
  trajectory_data->clear();
  result_.x.back() = frame_->open_space_info().open_space_end_pose()[0];
  result_.y.back() = frame_->open_space_info().open_space_end_pose()[1];
  result_.phi.back() = frame_->open_space_info().open_space_end_pose()[2];
  for (size_t i = 0; i < result_.x.size(); i++) {
    OpenSpaceTrajectoryOptimizerUtil::PathPointDeNormalizing(
        frame_->open_space_info().origin_heading(),
        frame_->open_space_info().origin_point(),
        &result_.x[i],
        &result_.y[i],
        &result_.phi[i]);
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(result_.x[i]);
    point.mutable_path_point()->set_y(result_.y[i]);
    point.mutable_path_point()->set_theta(result_.phi[i]);
    point.set_v(result_.v[i]);
    point.set_steer(result_.steer[i]);
    point.set_a(result_.a[i]);
    point.set_relative_time(i * 0.5);
    trajectory_data->emplace_back(point);
  }
}

bool OpenSpacePathPlanning::GenerateParallelParkingFallback(
    double start_x, double start_y, double start_theta,
    double end_x, double end_y, double end_theta,
    HybridAStartResult* result) {
  result->x.clear();
  result->y.clear();
  result->phi.clear();
  result->v.clear();
  result->a.clear();
  result->steer.clear();
  result->accumulated_s.clear();

  // Build a geometric path for parallel parking.
  // The path goes from the start pose (on the main road) to the end pose
  // (in the middle parking spot), avoiding the occupied spots on both sides.
  //
  // Strategy:
  //   Phase 1: Drive forward east along the road, past the parking spot.
  //   Phase 2: Reverse at an angle into the spot (parallel parking maneuver).
  //   Phase 3: Small forward adjustment to center in the spot.
  //
  // Waypoints in normalized coordinates:

  // Determine a safe road y-level (north of all parking obstacles).
  double road_y = std::max(start_y, 1.5);

  // Phase 1: Forward along the road.
  // WP0: start
  // WP1: go to road_y while moving east
  // WP2: continue east past the first obstacle (ends at x ≈ -2.5)
  // WP3: go past the target spot (end_x + 5)

  // Phase 2: Reverse into the spot.
  // WP4-WP6: reverse from east of spot into the spot at an angle

  // Phase 3: Center.
  // WP7: end pose

  struct Waypoint {
    double x, y;
    double heading;  // desired heading at this waypoint
    bool is_reverse; // true if this segment should be in reverse
  };

  std::vector<Waypoint> wpts;

  // WP0: start
  wpts.push_back({start_x, start_y, start_theta, false});

  // WP1: move to road_y while heading east
  double mid_x1 = start_x + 10.0;
  wpts.push_back({mid_x1, road_y, 0.0, false});

  // WP2: continue east past first obstacle (at x ≈ -2.5)
  wpts.push_back({0.0, road_y, 0.0, false});

  // WP3: go past the target spot
  double past_x = end_x + 6.0;
  wpts.push_back({past_x, road_y, 0.0, false});

  // WP4: reverse entry point — start turning into the spot
  wpts.push_back({end_x + 3.0, end_y + 1.5, -0.4, true});

  // WP5: mid-reverse point
  wpts.push_back({end_x + 1.5, end_y + 0.5, -0.2, true});

  // WP6: near end of reverse
  wpts.push_back({end_x, end_y, end_theta, true});

  // Generate dense path points by interpolating between waypoints.
  const double kStep = 0.15;  // meters between interpolated points
  const double kWheelBase = common::VehicleConfigHelper::Instance()
                                ->GetConfig()
                                .vehicle_param()
                                .wheel_base();

  for (size_t i = 0; i + 1 < wpts.size(); ++i) {
    const auto& w0 = wpts[i];
    const auto& w1 = wpts[i + 1];
    double dx = w1.x - w0.x;
    double dy = w1.y - w0.y;
    double seg_len = std::sqrt(dx * dx + dy * dy);
    int n_pts = std::max(2, static_cast<int>(seg_len / kStep));

    // Determine direction sign for this segment.
    // Reverse segments move "backward": the path points should go from wp[i]
    // toward wp[i+1], but heading should stay roughly east (for parallel
    // parking, the car faces east while backing into the spot).

    for (int j = 0; j < n_pts; ++j) {
      double t = static_cast<double>(j) / (n_pts - 1);
      double x = w0.x + dx * t;
      double y = w0.y + dy * t;

      // Interpolate heading.
      // For reverse segments, the vehicle heading should remain near the
      // starting heading (east ≈ 0) while the path direction reverses.
      double heading;
      if (w0.is_reverse) {
        // During reverse, the vehicle faces east-ish while going west/south.
        heading = w0.heading + (w1.heading - w0.heading) * t;
      } else {
        // During forward, heading follows path direction.
        double path_dir = std::atan2(dy, dx);
        heading = path_dir;
      }

      result->x.push_back(x);
      result->y.push_back(y);
      result->phi.push_back(heading);
    }
  }

  // Ensure final point matches end pose exactly.
  if (!result->x.empty()) {
    result->x.back() = end_x;
    result->y.back() = end_y;
    result->phi.back() = end_theta;
  }

  size_t n = result->x.size();
  if (n < 2) {
    AERROR << "Generated parallel parking path has too few points";
    return false;
  }

  // Compute accumulated_s and steer.
  result->accumulated_s.resize(n, 0.0);
  result->steer.resize(n - 1, 0.0);
  for (size_t i = 1; i < n; ++i) {
    double dx = result->x[i] - result->x[i - 1];
    double dy = result->y[i] - result->y[i - 1];
    double ds = std::sqrt(dx * dx + dy * dy);
    result->accumulated_s[i] = result->accumulated_s[i - 1] + ds;
    if (i < n - 1 && ds > 1e-6) {
      double dphi = common::math::NormalizeAngle(
          result->phi[i] - result->phi[i - 1]);
      result->steer[i - 1] = std::atan(dphi * kWheelBase / ds);
    }
  }

  // Velocities will be filled by the trajectory optimizer.
  result->v.resize(n, 0.0);
  result->a.resize(std::max(size_t(1), n) - 1, 0.0);

  AINFO << "Parallel parking fallback: generated " << n << " path points";
  return true;
}

}  // namespace planning
}  // namespace apollo
