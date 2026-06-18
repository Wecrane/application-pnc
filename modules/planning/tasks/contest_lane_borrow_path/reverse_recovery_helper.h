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

#pragma once

#include <memory>

#include "modules/planning/planning_base/common/path_boundary.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

namespace apollo {
namespace planning {

struct ReverseRecoveryState {
    bool active = false;
    int frame_count = 0;
    double start_s = 0.0;
    double start_x = 0.0;
    double start_y = 0.0;
    double last_adc_s = 0.0;
    double last_adc_x = 0.0;
    double last_adc_y = 0.0;

    // ── 固定倒车路径缓存（参考 .Apollo-wzr 的缓存机制）──
    // 倒车时构建一条固定直线参考线，沿该参考线生成倒车边界并跨帧复用，
    // 避免每帧重新生成导致的不稳定和参考线不匹配问题。
    bool path_initialized = false;                         // 倒车路径是否已固定
    PathBoundary boundary_cache;                           // 固定倒车边界缓存
    std::unique_ptr<ReferenceLine> reference_line_cache;   // 固定倒车参考线缓存
    double fixed_start_s = 0.0;                            // 固定倒车参考线上 ADC 起点 s
    double fixed_end_s = 0.0;                              // 固定倒车路径终点 s
    double fixed_l = 0.0;                                  // 固定倒车横向位置
    double fixed_start_x = 0.0;                            // 固定倒车起点真实 x（用于定位投影）
    double fixed_start_y = 0.0;                            // 固定倒车起点真实 y
    double fixed_heading = 0.0;                            // 固定倒车参考线 heading

    void Reset() {
        active = false;
        frame_count = 0;
        start_s = 0.0;
        start_x = 0.0;
        start_y = 0.0;
        last_adc_s = 0.0;
        last_adc_x = 0.0;
        last_adc_y = 0.0;
        path_initialized = false;
        boundary_cache.clear();
        reference_line_cache.reset();
        fixed_start_s = 0.0;
        fixed_end_s = 0.0;
        fixed_l = 0.0;
        fixed_start_x = 0.0;
        fixed_start_y = 0.0;
        fixed_heading = 0.0;
    }
};

/**
 * @brief 构建倒车用固定直线参考线（参考 .Apollo-wzr LaneBorrowPath::BuildReverseStraightReferenceLine）
 * @param state 倒车状态，用于写入固定参考线及起终点 s 坐标
 * @param adc_x ADC 当前 x
 * @param adc_y ADC 当前 y
 * @param adc_heading ADC 当前 heading
 * @param reverse_distance 倒车目标距离
 * @return 成功返回 true
 */
bool BuildReverseStraightReferenceLine(
        ReverseRecoveryState* state,
        double adc_x,
        double adc_y,
        double adc_heading,
        double reverse_distance);

/**
 * @brief 沿固定倒车参考线生成倒车路径边界（仅首帧调用，后续复用缓存）
 * @param state 倒车状态（读取 fixed_start_s / fixed_end_s / fixed_l / reference_line_cache）
 * @param boundary 输出边界
 * @return 成功返回 true
 */
bool GenerateCachedReversePathBoundary(
        const ReverseRecoveryState& state,
        PathBoundary* boundary);

/**
 * @brief 基于当前前向 reference_line 动态生成倒车边界（每次调用重新生成）
 * @param reference_line 当前参考线
 * @param adc_s ADC 当前 s
 * @param adc_l ADC 当前 l
 * @param reverse_distance 倒车距离
 * @param boundary 输出边界
 * @return 成功返回 true
 */
bool GenerateReverseRecoveryPathBoundary(
        const ReferenceLine& reference_line,
        double adc_s,
        double adc_l,
        double reverse_distance,
        PathBoundary* boundary);

}  // namespace planning
}  // namespace apollo
