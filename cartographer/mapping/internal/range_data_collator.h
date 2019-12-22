/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {

// Synchronizes TimedPointCloudData from different sensors. Input needs only be
// monotonous in 'TimedPointCloudData::time', output is monotonous in per-point
// timing. Up to one message per sensor is buffered, so a delay of the period of
// the slowest sensor may be introduced, which can be alleviated by passing
// subdivisions.
// 从不同的传感器同步TimedPointCloudData。 输入只需要在'TimedPointCloudData :: time'中单调，输出在每点定时单调。 
//  每个传感器最多可缓冲一条消息，因此可能会引入最慢传感器的周期延迟，这可以通过细分来减轻。
// 这里就是对传感器数据的整理，使每一个传感器数据都可以同步进行
class RangeDataCollator {
 public:
  explicit RangeDataCollator(
      const std::vector<std::string>& expected_range_sensor_ids)
      : expected_sensor_ids_(expected_range_sensor_ids.begin(),
                             expected_range_sensor_ids.end()) {}

  sensor::TimedPointCloudOriginData AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data);

 private:
  sensor::TimedPointCloudOriginData CropAndMerge();

  const std::set<std::string> expected_sensor_ids_;
  // Store at most one message for each sensor.
  std::map<std::string, sensor::TimedPointCloudData> id_to_pending_data_;
  common::Time current_start_ = common::Time::min();
  common::Time current_end_ = common::Time::min();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_