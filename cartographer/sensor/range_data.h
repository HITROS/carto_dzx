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

#ifndef CARTOGRAPHER_SENSOR_RANGE_DATA_H_
#define CARTOGRAPHER_SENSOR_RANGE_DATA_H_

#include "cartographer/common/port.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace sensor {

// Rays begin at 'origin'. 'returns' are the points where obstructions were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
// 激光数据开始在初始地点，返回值为点云数据，miss点是激光经过的点，没有障碍物的点，以确定的距离插入到了
// 激光数据中，假定初始点和miss点之间为空旷的区域
struct RangeData {
  // 初始点
  Eigen::Vector3f origin;
  // 反射点
  PointCloud returns;
  // freespace
  PointCloud misses;
};

// Like 'RangeData', but with 'TimedPointClouds'.
// 激光数据带有时间点云限制
struct TimedRangeData {
  Eigen::Vector3f origin;
  TimedPointCloud returns;
  TimedPointCloud misses;
};
// 进行点云数据转换，转换到机器人坐标下
RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);

TimedRangeData TransformTimedRangeData(const TimedRangeData& range_data,
                                       const transform::Rigid3f& transform);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
// 插入z范围内的激光
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
TimedRangeData CropTimedRangeData(const TimedRangeData& range_data, float min_z,
                                  float max_z);

// Converts 'range_data' to a proto::RangeData.
// 将激光数据序列化
proto::RangeData ToProto(const RangeData& range_data);

// Converts 'proto' to RangeData.
// 将激光数据反序列化
RangeData FromProto(const proto::RangeData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_RANGE_DATA_H_
