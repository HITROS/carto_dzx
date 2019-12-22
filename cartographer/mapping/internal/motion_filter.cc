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

#include "cartographer/mapping/internal/motion_filter.h"

#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
// 设定运动滤波的参数
proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::MotionFilterOptions options;
  // 设定最大的时间限制
  options.set_max_time_seconds(
      parameter_dictionary->GetDouble("max_time_seconds"));
  // 设定最大的距离
  options.set_max_distance_meters(
      parameter_dictionary->GetDouble("max_distance_meters"))；
  // 设定最大的角度
  options.set_max_angle_radians(
      parameter_dictionary->GetDouble("max_angle_radians"));
  return options;
}
// 滤波函数
MotionFilter::MotionFilter(const proto::MotionFilterOptions& options)
    : options_(options) {}

bool MotionFilter::IsSimilar(const common::Time time,
                             const transform::Rigid3d& pose) 
{
  // 设定减少节点的数量
  LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
      << "Motion filter reduced the number of nodes to "
      << 100. * num_different_ / num_total_ << "%.";
  ++num_total_;
  // 如果满足要求，返回true
  if (num_total_ > 1 &&//总的pose数大于1
      time - last_time_ <= common::FromSeconds(options_.max_time_seconds()) &&
      (pose.translation() - last_pose_.translation()).norm() <=
          options_.max_distance_meters() &&//移动距离小于给定阈值
      transform::GetAngle(pose.inverse() * last_pose_) <=
          options_.max_angle_radians()) //偏转角度小于给定阈值
          {
    return true;
    // 这三个阈值定义在配置文件/src/cartographer/configuration_files/trajectory_builder_2d.lua中
  }
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  // 不满足要求，返回false
  return false;
}

}  // namespace mapping
}  // namespace cartographer
