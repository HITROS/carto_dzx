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

#include "cartographer/mapping/3d/submap_3d.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/sensor/range_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {
// 定义像素数据结构体
struct PixelData {
  int min_z = INT_MAX;// 最大z值
  int max_z = INT_MIN;// 最小z值
  int count = 0;// 计数
  float probability_sum = 0.f;// 概率和
  float max_probability = 0.5f;// 最大的概率
};

// Filters 'range_data', retaining only the returns that have no more than
// 'max_range' distance from the origin. Removes misses and reflectivity
// information.
// 对范围数据进行滤波，去除不在范围内的数据
sensor::RangeData FilterRangeDataByMaxRange(const sensor::RangeData& range_data,
                                            const float max_range) {
  // 激光数据结果
  sensor::RangeData result{range_data.origin, {}, {}};
  // 检查hit激光数据是否在范围内，返回插入结果
  for (const Eigen::Vector3f& hit : range_data.returns) {
    if ((hit - range_data.origin).norm() <= max_range) {
      result.returns.push_back(hit);
    }
  }
  return result;
}
// 定义像素数据向量
std::vector<PixelData> AccumulatePixelData(
    // 定义宽度，高度和长度范围
    const int width, const int height, const Eigen::Array2i& min_index,
    const Eigen::Array2i& max_index,
    const std::vector<Eigen::Array4i>& voxel_indices_and_probabilities) {
  // 向量参数为宽度*高度
  std::vector<PixelData> accumulated_pixel_data(width * height);
  for (const Eigen::Array4i& voxel_index_and_probability :
       voxel_indices_and_probabilities) {
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    // 判断是否超出范围
    if ((pixel_index < min_index).any() || (pixel_index > max_index).any()) {
      // Out of bounds. This could happen because of floating point inaccuracy.
      continue;
    }
    // 定义xy值
    const int x = max_index.x() - pixel_index[0];
    const int y = max_index.y() - pixel_index[1];
    PixelData& pixel = accumulated_pixel_data[x * width + y];
    ++pixel.count;
    // z值
    pixel.min_z = std::min(pixel.min_z, voxel_index_and_probability[2]);
    pixel.max_z = std::max(pixel.max_z, voxel_index_and_probability[2]);
    // 概率值计算
    const float probability =
        ValueToProbability(voxel_index_and_probability[3]);
    pixel.probability_sum += probability;
    pixel.max_probability = std::max(pixel.max_probability, probability);
  }
  // 返回像素数据
  return accumulated_pixel_data;
}

// The first three entries of each returned value are a cell_index and the
// last is the corresponding probability value. We batch them together like
// this to only have one vector and have better cache locality.
// 每个返回值的前三个条目是cell_index，最后一个是相应的概率值。
// 我们像这样将它们捆绑在一起，以便只有一个向量，并且具有更好的缓存局部性。
std::vector<Eigen::Array4i> ExtractVoxelData(
    const HybridGrid& hybrid_grid, const transform::Rigid3f& transform,
    Eigen::Array2i* min_index, Eigen::Array2i* max_index) {
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities;
  // 分辨率的倒数
  const float resolution_inverse = 1.f / hybrid_grid.resolution();

  constexpr float kXrayObstructedCellProbabilityLimit = 0.501f;
  // 概率值
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const uint16 probability_value = it.GetValue();
    const float probability = ValueToProbability(probability_value);
    if (probability < kXrayObstructedCellProbabilityLimit) {
      // We ignore non-obstructed cells.
      continue;
    }
   // submap cell中心的向量
    const Eigen::Vector3f cell_center_submap =
        hybrid_grid.GetCenterOfCell(it.GetCellIndex());
    // 在global里面的向量值
    const Eigen::Vector3f cell_center_global = transform * cell_center_submap;
    // 带有概率的四维向量值
    const Eigen::Array4i voxel_index_and_probability(
        common::RoundToInt(cell_center_global.x() * resolution_inverse),
        common::RoundToInt(cell_center_global.y() * resolution_inverse),
        common::RoundToInt(cell_center_global.z() * resolution_inverse),
        probability_value);
    // 回调向量值
    voxel_indices_and_probabilities.push_back(voxel_index_and_probability);
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    *min_index = min_index->cwiseMin(pixel_index);
    *max_index = max_index->cwiseMax(pixel_index);
  }
  // 返回带有概率值的向量
  return voxel_indices_and_probabilities;
}

// Builds texture data containing interleaved value and alpha for the
// visualization from 'accumulated_pixel_data'
// 可视化
// 计算像素值
std::string ComputePixelValues(
    const std::vector<PixelData>& accumulated_pixel_data) {
  std::string cell_data;
  // 把cell数据每一维范围扩大两倍
  cell_data.reserve(2 * accumulated_pixel_data.size());
  constexpr float kMinZDifference = 3.f;
  constexpr float kFreeSpaceWeight = 0.15f;
  for (const PixelData& pixel : accumulated_pixel_data) {
    // TODO(whess): Take into account submap rotation.
    // TODO(whess): Document the approach and make it more independent from the
    // chosen resolution.
    // 判断z范围
    const float z_difference = pixel.count > 0 ? pixel.max_z - pixel.min_z : 0;
    if (z_difference < kMinZDifference) {
      cell_data.push_back(0);  // value
      cell_data.push_back(0);  // alpha
      continue;
    }
    const float free_space = std::max(z_difference - pixel.count, 0.f);
    const float free_space_weight = kFreeSpaceWeight * free_space;
    const float total_weight = pixel.count + free_space_weight;
    const float free_space_probability = 1.f - pixel.max_probability;
    const float average_probability = ClampProbability(
        (pixel.probability_sum + free_space_probability * free_space_weight) /
        total_weight);
    const int delta = 128 - ProbabilityToLogOddsInteger(average_probability);
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cell_data.push_back(value);                         // value
    cell_data.push_back((value || alpha) ? alpha : 1);  // alpha
  }
  return cell_data;
}
// 序列化
void AddToTextureProto(
    const HybridGrid& hybrid_grid, const transform::Rigid3d& global_submap_pose,
    proto::SubmapQuery::Response::SubmapTexture* const texture) {
  // Generate an X-ray view through the 'hybrid_grid', aligned to the
  // xy-plane in the global map frame.
  // 通过“混合网格”生成与全局地图框中xy平面对齐的x光视图
  const float resolution = hybrid_grid.resolution();
  texture->set_resolution(resolution);

  // Compute a bounding box for the texture.
  // 计算纹理的边界框
  Eigen::Array2i min_index(INT_MAX, INT_MAX);
  Eigen::Array2i max_index(INT_MIN, INT_MIN);
  const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
      ExtractVoxelData(hybrid_grid, global_submap_pose.cast<float>(),
                       &min_index, &max_index);
  // 计算宽度
  const int width = max_index.y() - min_index.y() + 1;
  // 计算高度
  const int height = max_index.x() - min_index.x() + 1;
  texture->set_width(width);
  texture->set_height(height);

  const std::vector<PixelData> accumulated_pixel_data = AccumulatePixelData(
      width, height, min_index, max_index, voxel_indices_and_probabilities);
  const std::string cell_data = ComputePixelValues(accumulated_pixel_data);

  common::FastGzipString(cell_data, texture->mutable_cells());
  *texture->mutable_slice_pose() = transform::ToProto(
      global_submap_pose.inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(
          max_index.x() * resolution, max_index.y() * resolution,
          global_submap_pose.translation().z())));
}

}  // namespace
// 序列化参数配置
proto::SubmapsOptions3D CreateSubmapsOptions3D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::SubmapsOptions3D options;
  options.set_high_resolution(
      parameter_dictionary->GetDouble("high_resolution"));
  options.set_high_resolution_max_range(
      parameter_dictionary->GetDouble("high_resolution_max_range"));
  options.set_low_resolution(parameter_dictionary->GetDouble("low_resolution"));
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions3D(
          parameter_dictionary->GetDictionary("range_data_inserter").get());
  CHECK_GT(options.num_range_data(), 0);
  return options;
}
// 高分辨率和低分辨率
Submap3D::Submap3D(const float high_resolution, const float low_resolution,
                   const transform::Rigid3d& local_submap_pose)
    : Submap(local_submap_pose),
      high_resolution_hybrid_grid_(
          common::make_unique<HybridGrid>(high_resolution)),
      low_resolution_hybrid_grid_(
          common::make_unique<HybridGrid>(low_resolution)) {}

Submap3D::Submap3D(const proto::Submap3D& proto)
    // 进行坐标转换
    : Submap(transform::ToRigid3(proto.local_pose())),
      high_resolution_hybrid_grid_(
          common::make_unique<HybridGrid>(proto.high_resolution_hybrid_grid())),
      low_resolution_hybrid_grid_(
          common::make_unique<HybridGrid>(proto.low_resolution_hybrid_grid())) {
  set_num_range_data(proto.num_range_data());
  set_finished(proto.finished());
}

void Submap3D::ToProto(proto::Submap* const proto,
                       bool include_probability_grid_data) const {
  auto* const submap_3d = proto->mutable_submap_3d();
  *submap_3d->mutable_local_pose() = transform::ToProto(local_pose());
  submap_3d->set_num_range_data(num_range_data());
  submap_3d->set_finished(finished());
  if (include_probability_grid_data) {
    *submap_3d->mutable_high_resolution_hybrid_grid() =
        high_resolution_hybrid_grid().ToProto();
    *submap_3d->mutable_low_resolution_hybrid_grid() =
        low_resolution_hybrid_grid().ToProto();
  }
}
// 反序列化得到值进行更新
void Submap3D::UpdateFromProto(const proto::Submap& proto) {
  // 检查序列内是否有子地图
  CHECK(proto.has_submap_3d());
  const auto& submap_3d = proto.submap_3d();
  // 激光数据数量
  set_num_range_data(submap_3d.num_range_data());
  set_finished(submap_3d.finished());
  // 高分辨率
  if (submap_3d.has_high_resolution_hybrid_grid()) {
    high_resolution_hybrid_grid_ =
        submap_3d.has_high_resolution_hybrid_grid()
            ? common::make_unique<HybridGrid>(
                  submap_3d.high_resolution_hybrid_grid())
            : nullptr;
  }
  // 低分辨率
  if (submap_3d.has_low_resolution_hybrid_grid()) {
    low_resolution_hybrid_grid_ =
        submap_3d.has_low_resolution_hybrid_grid()
            ? common::make_unique<HybridGrid>(
                  submap_3d.low_resolution_hybrid_grid())
            : nullptr;
  }
}

void Submap3D::ToResponseProto(
    const transform::Rigid3d& global_submap_pose,
    proto::SubmapQuery::Response* const response) const {
  response->set_submap_version(num_range_data());

  AddToTextureProto(*high_resolution_hybrid_grid_, global_submap_pose,
                    response->add_textures());
  AddToTextureProto(*low_resolution_hybrid_grid_, global_submap_pose,
                    response->add_textures());
}
// 向地图内添加激光数据
void Submap3D::InsertRangeData(const sensor::RangeData& range_data,
                               const RangeDataInserter3D& range_data_inserter,
                               const int high_resolution_max_range) {
  CHECK(!finished());
  const sensor::RangeData transformed_range_data = sensor::TransformRangeData(
      range_data, local_pose().inverse().cast<float>());
  range_data_inserter.Insert(
      FilterRangeDataByMaxRange(transformed_range_data,
                                high_resolution_max_range),
      high_resolution_hybrid_grid_.get());
  range_data_inserter.Insert(transformed_range_data,
                             low_resolution_hybrid_grid_.get());
  set_num_range_data(num_range_data() + 1);
}

void Submap3D::Finish() {
  CHECK(!finished());
  set_finished(true);
}
// 和2d是一样的
ActiveSubmaps3D::ActiveSubmaps3D(const proto::SubmapsOptions3D& options)
    : options_(options),
      range_data_inserter_(options.range_data_inserter_options()) {
  // We always want to have at least one submap which we can return and will
  // create it at the origin in absence of a better choice.
  //
  // TODO(whess): Start with no submaps, so that all of them can be
  // approximately gravity aligned.
  AddSubmap(transform::Rigid3d::Identity());
}

std::vector<std::shared_ptr<Submap3D>> ActiveSubmaps3D::submaps() const {
  return submaps_;
}
// match索引
int ActiveSubmaps3D::matching_index() const { return matching_submap_index_; }
// 向activesubmap插入激光数据
void ActiveSubmaps3D::InsertRangeData(
    const sensor::RangeData& range_data,
    const Eigen::Quaterniond& gravity_alignment) {
  for (auto& submap : submaps_) {
    submap->InsertRangeData(range_data, range_data_inserter_,
                            options_.high_resolution_max_range());
  }
  // 如果到达激光限定值，就再生成一个子地图
  if (submaps_.back()->num_range_data() == options_.num_range_data()) {
    AddSubmap(transform::Rigid3d(range_data.origin.cast<double>(),
                                 gravity_alignment));
  }
}
// 添加submap函数体
void ActiveSubmaps3D::AddSubmap(const transform::Rigid3d& local_submap_pose) {
  if (submaps_.size() > 1) {
    submaps_.front()->Finish();
    ++matching_submap_index_;
    submaps_.erase(submaps_.begin());
  }
  submaps_.emplace_back(new Submap3D(options_.high_resolution(),
                                     options_.low_resolution(),
                                     local_submap_pose));
  LOG(INFO) << "Added submap " << matching_submap_index_ + submaps_.size();
}

}  // namespace mapping
}  // namespace cartographer
