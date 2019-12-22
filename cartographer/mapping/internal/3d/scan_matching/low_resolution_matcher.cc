/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/3d/scan_matching/low_resolution_matcher.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
// 体素进行低精度scan match
std::function<float(const transform::Rigid3f&)> CreateLowResolutionMatcher(
    const HybridGrid* low_resolution_grid, const sensor::PointCloud* points) {
  return [=](const transform::Rigid3f& pose) {
    // 初始化分数为0 
    float score = 0.f;
    for (const Eigen::Vector3f& point :
         sensor::TransformPointCloud(*points, pose)) {
      // TODO(zhengj, whess): Interpolate the Grid to get better score.
      // 计算得分
      score += low_resolution_grid->GetProbability(
          low_resolution_grid->GetCellIndex(point));
    }
    // 返回得分
    return score / points->size();
  };
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
