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

#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {
// 点云数据是三维坐标内的向量集合，用x，y，z来表示
// 对于2D来说，z=0
// Stores 3D positions of points.
// For 2D points, the third entry is 0.f.
typedef std::vector<Eigen::Vector3f> PointCloud;

// Stores 3D positions of points with their relative measurement time in the
// fourth entry. Time is in seconds, increasing and relative to the moment when
// the last point was acquired. So, the fourth entry for the last point is 0.f.
// If timing is not available, all fourth entries are 0.f. For 2D points, the
// third entry is 0.f (and the fourth entry is time).
typedef std::vector<Eigen::Vector4f> TimedPointCloud;

struct PointCloudWithIntensities {
  // 点云数据
  TimedPointCloud points;
  // 光线强度
  std::vector<float> intensities;
};
// 转换点云数据
// Transforms 'point_cloud' according to 'transform'.
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

// Transforms 'point_cloud' according to 'transform'.
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform);
// 返回去除落在z轴外的新的点云数据
// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
                          float max_z);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
TimedPointCloud CropTimedPointCloud(const TimedPointCloud& point_cloud,
                                    float min_z, float max_z);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
