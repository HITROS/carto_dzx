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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_PRECOMPUTATION_GRID_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_PRECOMPUTATION_GRID_3D_H_

#include "cartographer/mapping/3d/hybrid_grid.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

class PrecomputationGrid3D : public HybridGridBase<uint8> {
 public:
  explicit PrecomputationGrid3D(const float resolution)
      : HybridGridBase<uint8>(resolution) {}

  // Maps values from [0, 255] to [kMinProbability, kMaxProbability].
  // map值的概率，最大和最小的范围
  static float ToProbability(float value) {
    return kMinProbability +
           value * ((kMaxProbability - kMinProbability) / 255.f);
  }
};

// Converts a HybridGrid to a PrecomputationGrid3D representing the same data,
// but only using 8 bit instead of 2 x 16 bit.
// 将混合网格转换为表示相同数据的预计算网格3D，但只使用8位而不是2×16位。
PrecomputationGrid3D ConvertToPrecomputationGrid(const HybridGrid& hybrid_grid);

// Returns a grid of the same resolution containing the maximum value of
// original voxels in 'grid'. This maximum is over the 8 voxels that have
// any combination of index components optionally increased by 'shift'.
// If 'shift' is 2 ** (depth - 1), where depth 0 is the original grid, and this
// is using the precomputed grid of one depth before, this results in
// precomputation grids analogous to the 2D case.
// 返回分辨率相同的网格，其中包含“网格”中原始体素的最大值。
// 这个最大值超过了8个体素，这些体素具有可选地通过“移位”增加的索引分量的任意组合。
// 如果“shift”是2 **(深度- 1)，其中深度0是原始网格，
// 并且这是使用之前一个深度的预计算网格，这将导致类似于2D情况的预计算网格
PrecomputationGrid3D PrecomputeGrid(const PrecomputationGrid3D& grid,
                                    bool half_resolution,
                                    const Eigen::Array3i& shift);

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_PRECOMPUTATION_GRID_3D_H_
