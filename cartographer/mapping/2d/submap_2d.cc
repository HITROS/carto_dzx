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

#include "cartographer/mapping/2d/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
// 产生submap参数选择
// 参数配置在/src/cartographer/configuration_files/trajectory_builder_2d.lua中
proto::SubmapsOptions2D CreateSubmapsOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SubmapsOptions2D options;
  // 设置激光数据的数量
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  // 设置可变的概率栅格
  *options.mutable_grid_options_2d() = CreateGridOptions2D(
      parameter_dictionary->GetDictionary("grid_options_2d").get());
  // 设置可变的激光数据插入
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());
  // 判断激光数据是否插入
  bool valid_range_data_inserter_grid_combination = false;
  const proto::GridOptions2D_GridType& grid_type =
      options.grid_options_2d().grid_type();
  const proto::RangeDataInserterOptions_RangeDataInserterType&
      range_data_inserter_type =
          options.range_data_inserter_options().range_data_inserter_type();
  if (grid_type == proto::GridOptions2D::PROBABILITY_GRID &&
      range_data_inserter_type ==
          proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D) {
    valid_range_data_inserter_grid_combination = true;
  }
  CHECK(valid_range_data_inserter_grid_combination)
      << "Invalid combination grid_type " << grid_type
      << " with range_data_inserter_type " << range_data_inserter_type;
  CHECK_GT(options.num_range_data(), 0);
  return options;
}
// 转换坐标
// 可以看到，在构造2D子图时，Submap的坐标系旋转角度设置为0.而原点由参数origin给出
Submap2D::Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid)
    : Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))) {
  grid_ = std::move(grid);
}

Submap2D::Submap2D(const proto::Submap2D& proto)
    : Submap(transform::ToRigid3(proto.local_pose())) {
  if (proto.has_grid()) {
    CHECK(proto.grid().has_probability_grid_2d());
    grid_ = common::make_unique<ProbabilityGrid>(proto.grid());
  }
  set_num_range_data(proto.num_range_data());
  set_finished(proto.finished());
}
// 反序列化得到子地图
void Submap2D::ToProto(proto::Submap* const proto,
                       bool include_probability_grid_data) const {
  auto* const submap_2d = proto->mutable_submap_2d();
  *submap_2d->mutable_local_pose() = transform::ToProto(local_pose());
  submap_2d->set_num_range_data(num_range_data());
  submap_2d->set_finished(finished());
  if (include_probability_grid_data) {
    CHECK(grid_);
    *submap_2d->mutable_grid() = grid_->ToProto();
  }
}
// 更新子地图
void Submap2D::UpdateFromProto(const proto::Submap& proto) {
  CHECK(proto.has_submap_2d());
  const auto& submap_2d = proto.submap_2d();
  set_num_range_data(submap_2d.num_range_data());
  set_finished(submap_2d.finished());
  if (proto.submap_2d().has_grid()) {
    CHECK(proto.submap_2d().grid().has_probability_grid_2d());
    grid_ = common::make_unique<ProbabilityGrid>(submap_2d.grid());
  }
}

void Submap2D::ToResponseProto(
    const transform::Rigid3d&,
    proto::SubmapQuery::Response* const response) const {
  if (!grid_) return;
  response->set_submap_version(num_range_data());
  proto::SubmapQuery::Response::SubmapTexture* const texture =
      response->add_textures();
  grid()->DrawToSubmapTexture(texture, local_pose());
}
// 添加激光数据
void Submap2D::InsertRangeData(
    const sensor::RangeData& range_data,
    const RangeDataInserterInterface* range_data_inserter) {
  CHECK(grid_);//检查是否栅格化
  CHECK(!finished());//检查图是否已被finished
  //调用RangeDataInserterInterface来更新概率图
  range_data_inserter->Insert(range_data, grid_.get());
  set_num_range_data(num_range_data() + 1);
}
// 检查submap是否已经完成
void Submap2D::Finish() {
  CHECK(grid_);
  CHECK(!finished());
  //计算裁剪栅格图
  grid_ = grid_->ComputeCroppedGrid();
  set_finished(true);
}

ActiveSubmaps2D::ActiveSubmaps2D(const proto::SubmapsOptions2D& options)
    : options_(options),
      range_data_inserter_(std::move(CreateRangeDataInserter())) {
  // We always want to have at least one likelihood field which we can return,
  // and will create it at the origin in absence of a better choice.
  AddSubmap(Eigen::Vector2f::Zero());
}
// 保证总有一个submap存在
std::vector<std::shared_ptr<Submap2D>> ActiveSubmaps2D::submaps() const {
  return submaps_;
}

int ActiveSubmaps2D::matching_index() const { return matching_submap_index_; }
// 向返回后的submap插入激光数据
void ActiveSubmaps2D::InsertRangeData(const sensor::RangeData& range_data) {
  for (auto& submap : submaps_) {
    submap->InsertRangeData(range_data, range_data_inserter_.get());
  }
  if (submaps_.back()->num_range_data() == options_.num_range_data()) {
    AddSubmap(range_data.origin.head<2>());
  }
}

std::unique_ptr<RangeDataInserterInterface>
ActiveSubmaps2D::CreateRangeDataInserter() {
  return common::make_unique<ProbabilityGridRangeDataInserter2D>(
      options_.range_data_inserter_options()
          .probability_grid_range_data_inserter_options_2d());
}
// 产生概率栅格
std::unique_ptr<GridInterface> ActiveSubmaps2D::CreateGrid(
    const Eigen::Vector2f& origin) {
  constexpr int kInitialSubmapSize = 100;
  float resolution = options_.grid_options_2d().resolution();
  return common::make_unique<ProbabilityGrid>(
      // 以地图中心为起始点
      MapLimits(resolution,
                origin.cast<double>() + 0.5 * kInitialSubmapSize * resolution *
                                            Eigen::Vector2d::Ones(),
                CellLimits(kInitialSubmapSize, kInitialSubmapSize)));
}
// 创建完成submap
void ActiveSubmaps2D::FinishSubmap() {
  Submap2D* submap = submaps_.front().get();
  submap->Finish();
  ++matching_submap_index_;
  submaps_.erase(submaps_.begin());
}

void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
  if (submaps_.size() > 1) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    FinishSubmap();
  }

  submaps_.push_back(common::make_unique<Submap2D>(
      origin, std::unique_ptr<Grid2D>(
                  static_cast<Grid2D*>(CreateGrid(origin).release()))));
  LOG(INFO) << "Added submap " << matching_submap_index_ + submaps_.size();
}

}  // namespace mapping
}  // namespace cartographer
