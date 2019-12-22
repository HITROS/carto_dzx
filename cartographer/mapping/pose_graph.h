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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_H_

#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/pose_graph_options.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/map_by_time.h"
#include "cartographer/sensor/odometry_data.h"

namespace cartographer {
namespace mapping {

proto::PoseGraphOptions CreatePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class PoseGraph : public PoseGraphInterface {
 public:
  struct InitialTrajectoryPose {
    int to_trajectory_id;
    transform::Rigid3d relative_pose;
    common::Time time;
  };

  PoseGraph() {}
  ~PoseGraph() override {}

  PoseGraph(const PoseGraph&) = delete;
  PoseGraph& operator=(const PoseGraph&) = delete;

  // Inserts an IMU measurement.
  // 加入imu数据测量
  virtual void AddImuData(int trajectory_id,
                          const sensor::ImuData& imu_data) = 0;

  // Inserts an odometry measurement.
  // 加入里程计数据测量
  virtual void AddOdometryData(int trajectory_id,
                               const sensor::OdometryData& odometry_data) = 0;

  // Inserts a fixed frame pose measurement.
  // 加入固定坐标系测量
  virtual void AddFixedFramePoseData(
      int trajectory_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data) = 0;

  // Inserts landmarks observations.
  // 加入地标观察值
  virtual void AddLandmarkData(int trajectory_id,
                               const sensor::LandmarkData& landmark_data) = 0;

  // Finishes the given trajectory.
  // 完成已经给定的轨迹
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Freezes a trajectory. Poses in this trajectory will not be optimized.
  // 冻结一个轨迹，位姿在这个轨迹里将不会被优化
  virtual void FreezeTrajectory(int trajectory_id) = 0;

  // Adds a 'submap' from a proto with the given 'global_pose' to the
  // appropriate trajectory.
  // 加入来自全局位姿的子地图到合适的轨迹里面
  virtual void AddSubmapFromProto(const transform::Rigid3d& global_pose,
                                  const proto::Submap& submap) = 0;

  // Adds a 'node' from a proto with the given 'global_pose' to the
  // appropriate trajectory.
  // 加入来自全局位姿的节点到合适的轨迹里面
  virtual void AddNodeFromProto(const transform::Rigid3d& global_pose,
                                const proto::Node& node) = 0;

  // Sets the trajectory data from a proto.
  // 设定来自proto的轨迹数据
  virtual void SetTrajectoryDataFromProto(
      const mapping::proto::TrajectoryData& data) = 0;

  // Adds information that 'node_id' was inserted into 'submap_id'. The submap
  // has to be deserialized first.
  // 添加已经加入到子地图里面的节点id
  virtual void AddNodeToSubmap(const NodeId& node_id,
                               const SubmapId& submap_id) = 0;

  // Adds serialized constraints. The corresponding trajectory nodes and submaps
  // have to be deserialized before calling this function.
  // 加入序列化的约束
  virtual void AddSerializedConstraints(
      const std::vector<Constraint>& constraints) = 0;

  // Adds a 'trimmer'. It will be used after all data added before it has been
  // included in the pose graph.
  // 添加修剪，它将会被用到在所有数据加入之后和没有加入到位姿图之前
  virtual void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) = 0;

  // Gets the current trajectory clusters.
  // 得到目前的轨迹集群
  virtual std::vector<std::vector<int>> GetConnectedTrajectories() const = 0;

  // Returns the current optimized transform and submap itself for the given
  // 'submap_id'. Returns 'nullptr' for the 'submap' member if the submap does
  // not exist (anymore).
  // 返回优化完的坐标和子地图。返回空指针如果子地图不存在
  virtual SubmapData GetSubmapData(const SubmapId& submap_id) const = 0;

  proto::PoseGraph ToProto() const override;

  // Returns the IMU data.
  virtual sensor::MapByTime<sensor::ImuData> GetImuData() const = 0;

  // Returns the odometry data.
  virtual sensor::MapByTime<sensor::OdometryData> GetOdometryData() const = 0;

  // Returns the fixed frame pose data.
  virtual sensor::MapByTime<sensor::FixedFramePoseData> GetFixedFramePoseData()
      const = 0;

  // Returns the landmark data.
  virtual std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
  GetLandmarkNodes() const = 0;

  // Sets a relative initial pose 'relative_pose' for 'from_trajectory_id' with
  // respect to 'to_trajectory_id' at time 'time'.
  virtual void SetInitialTrajectoryPose(int from_trajectory_id,
                                        int to_trajectory_id,
                                        const transform::Rigid3d& pose,
                                        const common::Time time) = 0;
};

std::vector<PoseGraph::Constraint> FromProto(
    const ::google::protobuf::RepeatedPtrField<
        ::cartographer::mapping::proto::PoseGraph::Constraint>&
        constraint_protos);
proto::PoseGraph::Constraint ToProto(const PoseGraph::Constraint& constraint);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_H_
