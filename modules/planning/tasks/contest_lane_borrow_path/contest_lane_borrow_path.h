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

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/tasks/contest_lane_borrow_path/construction_zone_helper.h"
#include "modules/planning/tasks/contest_lane_borrow_path/reverse_recovery_helper.h"
#include "modules/planning/tasks/contest_lane_borrow_path/proto/contest_lane_borrow_path.pb.h"

namespace apollo {
namespace planning {

enum SidePassDirection { LEFT_BORROW = 1, RIGHT_BORROW = 2 };

class ContestLaneBorrowPath : public PathGeneration {
public:
    bool Init(
            const std::string& config_dir,
            const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

private:
    apollo::common::Status Process(Frame* frame, ReferenceLineInfo* reference_line_info) override;
    /**
     * @brief Calculate all path boundaries
     * @param boundary is calculated path boundaries
     */
    bool DecidePathBounds(std::vector<PathBoundary>* boundary);
    /**
     * @brief Optimize paths for each path boundary
     * @param path_boundaries is input path boundaries
     * @param candidate_path_data is output paths
     */
    bool OptimizePath(const std::vector<PathBoundary>& path_boundaries, std::vector<PathData>* candidate_path_data);
    /**
     * @brief Assess the feasibility of each path and select the best one
     * @param candidate_path_data is input paths
     * @param final_path is output the best path
     */
    bool AssessPath(std::vector<PathData>* candidate_path_data, PathData* final_path);
    bool HasDynamicVehicleConflictOnBorrowPath(const std::vector<PathData>& candidate_path_data) const;
    bool GenerateBorrowHoldPath(PathData* final_path);
    /**
     * @brief Generate path boundary by left or right neightbor lane and self lane
     * @param pass_direction is side pass direction (left or right)
     * @param final_path is output the best path
     */
    bool GetBoundaryFromNeighborLane(
            const SidePassDirection pass_direction,
            PathBoundary* const path_bound,
            std::string* borrow_lane_type);
    /**
     * @brief Determine whether to borrow neighbor lane
     * @return if need to borrow lane return true
     */
    bool IsNecessaryToBorrowLane();

    bool HasSingleReferenceLine(const Frame& frame);

    bool IsWithinSidePassingSpeedADC(const Frame& frame);

    bool IsLongTermBlockingObstacle();

    bool IsBlockingObstacleWithinDestination(const ReferenceLineInfo& reference_line_info);

    bool IsBlockingObstacleFarFromIntersection(const ReferenceLineInfo& reference_line_info);

    bool IsSidePassableObstacle(const ReferenceLineInfo& reference_line_info);

    void UpdateSelfPathInfo();
    bool IsContestConstructionScenario() const;
    bool IsContestUTurnScenario() const;
    bool ShouldUseUTurnConeNudge() const;
    void ApplyUTurnConeNudge(std::vector<SLPolygon>* obs_sl_polygons) const;
    void ForceConstructionLaneBorrow(int cone_count);
    void UpdateConstructionZoneState();
    bool MaybeGenerateReverseRecoveryBoundary(std::vector<PathBoundary>* boundary);
    void ApplyConstructionZoneSpeedLimitAndLabel(ReferenceLineInfo* reference_line_info) const;
    void IgnoreAllObstacles(ReferenceLineInfo* reference_line_info) const;
    void ResetConstructZoneState(const std::string& reason);
    /**
     * @brief Check whether neighbor lane is borrowable
     * @param reference_line_info is input reference line info
     * @param left_neighbor_lane_borrowable is output that left neighbor lane can
     * be borrowable will be true
     * @param right_neighbor_lane_borrowable is output  that right neighbor lane
     * can be borrowable will be true
     */
    void CheckLaneBorrow(
            const ReferenceLineInfo& reference_line_info,
            bool* left_neighbor_lane_borrowable,
            bool* right_neighbor_lane_borrowable);
    /**
     * @brief Check whether neighbor lane is borrowable according to boudary type,
     * if solid line can not be borrowed
     * @param reference_line_info is input reference line info
     * @param check_s is check s position boundary type
     * @param lane_borrow_info is borrow side.
     */
    bool CheckLaneBoundaryType(
            const ReferenceLineInfo& reference_line_info,
            const double check_s,
            const SidePassDirection& lane_borrow_info);
    /**
     * @brief Set path point decision guide info which describe
     * @param reference_line_info is input reference line info
     * @param check_s is check s position boundary type
     * @param lane_borrow_info is borrow side.
     */
    void SetPathInfo(PathData* const path_data);

    /**
     * @brief Generate a path boundary spanning all available lanes
     *        (both left and right neighbors) for construction zone passage.
     */
    void GetConstructZoneBoundary(PathBoundary* const path_bound);
    /**
     * @brief Decide path bounds for construction zone mode,
     *        generating a single bidirectional boundary.
     */
    bool DecideConstructZoneBoundary(std::vector<PathBoundary>* boundary);
    void ComputeConstructZoneBoundary(
            std::vector<SLPolygon>& cones,
            const ConstructionConeXYMap& cone_xy,
            PathBoundary* const path_bound);
    ContestLaneBorrowPathConfig config_;
    std::vector<SidePassDirection> decided_side_pass_direction_;
    int use_self_lane_;
    std::string blocking_obstacle_id_;
    ConstructionZoneState construction_zone_;
    bool u_turn_construct_ = false;
    std::unique_ptr<PathData> last_frame_;
    static constexpr int kLowConeExitThreshold = 30;  // 3秒@10Hz，锥桶持续消失才退出

    ReverseRecoveryState reverse_recovery_;
    static constexpr double kReverseDistance = 8.0;            // 倒车目标距离 (m)
    static constexpr double kMinReverseDistance = 0.5;         // 倒车目标距离配置下限
    // 倒车完成：沿固定参考线剩余距离小于此阈值即视为到位
    static constexpr double kReverseTargetRemainThreshold = 0.7;
    static constexpr double kMinReverseTargetRemainThreshold = 0.05;
    // 防止定位/启动瞬间抖动导致倒车刚开始就结束。
    static constexpr int kReverseMinFrames = 10;
    static constexpr int kReverseMaxFrames = 250;        // 倒车最大帧数 (25s @ 10Hz)
    static constexpr double kStuckSpeedThreshold = 0.3;  // 判定"卡死"的速度阈值 (m/s)
    static constexpr int kStuckFrameThreshold = 15;      // 连续低速帧数阈值

    // 倒车路径生成：生成一条向后直线倒车路径（已废弃，改用缓存机制）
    bool GenerateReversePathBoundary(PathBoundary* boundary);
};

/////////////////////////////////////////////////////////////////////////////
// Below are helper functions.

int ContainsOutOnReverseLane(
        const std::vector<std::tuple<double, PathData::PathPointType, double>>& path_point_decision);

int GetBackToInLaneIndex(const std::vector<std::tuple<double, PathData::PathPointType, double>>& path_point_decision);

bool ComparePathData(const PathData& lhs, const PathData& rhs, const Obstacle* blocking_obstacle);

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ContestLaneBorrowPath, Task)
}  // namespace planning
}  // namespace apollo
