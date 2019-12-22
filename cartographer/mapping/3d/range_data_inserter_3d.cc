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

#include "cartographer/mapping/3d/range_data_inserter_3d.h"

#include "Eigen/Core"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {
// 定义插入miss函数
void InsertMissesIntoGrid(const std::vector<uint16>& miss_table,// miss格子
                          const Eigen::Vector3f& origin,// 初始矩阵
                          const sensor::PointCloud& returns,// 点云的返回值
                          HybridGrid* hybrid_grid,
                          const int num_free_space_voxels) {// free 的数量
  // 初始化索引值
  const Eigen::Array3i origin_cell = hybrid_grid->GetCellIndex(origin);
  // 产生miss
  for (const Eigen::Vector3f& hit : returns) {
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);

    const Eigen::Array3i delta = hit_cell - origin_cell;
    const int num_samples = delta.cwiseAbs().maxCoeff();
    CHECK_LT(num_samples, 1 << 15);
    // 'num_samples' is the number of samples we equi-distantly place on the
    // line between 'origin' and 'hit'. (including a fractional part for sub-
    // voxels) It is chosen so that between two samples we change from one voxel
    // to the next on the fastest changing dimension.
    // num_samples”是我们等距离放置在“原点”和“命中”之间的线上的样本数。
    // (包括子体素的分数部分)选择它，以便在两个样本之间，我们在变化最快的维度上从一个体素变化到下一个体素
    // Only the last 'num_free_space_voxels' are updated for performance.
    // 只有最新的数量才会被用来更新hit和miss
    for (int position = std::max(0, num_samples - num_free_space_voxels);
         position < num_samples; ++position) {
      const Eigen::Array3i miss_cell =
          origin_cell + delta * position / num_samples;
      hybrid_grid->ApplyLookupTable(miss_cell, miss_table);
    }
  }
}

}  // namespace
// 配置参数选择
proto::RangeDataInserterOptions3D CreateRangeDataInserterOptions3D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::RangeDataInserterOptions3D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_num_free_space_voxels(
      parameter_dictionary->GetInt("num_free_space_voxels"));
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

RangeDataInserter3D::RangeDataInserter3D(
    const proto::RangeDataInserterOptions3D& options)
    : options_(options),
      hit_table_(
          ComputeLookupTableToApplyOdds(Odds(options_.hit_probability()))),
      miss_table_(
          ComputeLookupTableToApplyOdds(Odds(options_.miss_probability()))) {}

void RangeDataInserter3D::Insert(const sensor::RangeData& range_data,
                                 HybridGrid* hybrid_grid) const {
  CHECK_NOTNULL(hybrid_grid);

  for (const Eigen::Vector3f& hit : range_data.returns) {
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);
    hybrid_grid->ApplyLookupTable(hit_cell, hit_table_);
  }

  // By not starting a new update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  InsertMissesIntoGrid(miss_table_, range_data.origin, range_data.returns,
                       hybrid_grid, options_.num_free_space_voxels());
  hybrid_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
