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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_

#include <vector>

#include "cartographer/common/optional.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

class PoseGraphInterface {
 public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  // 定义约束结构体
  struct Constraint {
    struct Pose {
      transform::Rigid3d zbar_ij;//相对位姿
      double translation_weight;//translation的权重
      double rotation_weight;//rotation的权重
    };

    SubmapId submap_id;  // 'i' in the paper.
    NodeId node_id;      // 'j' in the paper.

    // Pose of the node 'j' relative to submap 'i'.
    // 节点j的位姿相关于子地图i
    Pose pose;

    // Differentiates between intra-submap (where node 'j' was inserted into
    // submap 'i') and inter-submap constraints (where node 'j' was not inserted
    // into submap 'i').
    // intra-submap：节点j被插入到了子地图i中
    // inter-submap：节点j没有被插入到子地图i中
    enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  };

  struct LandmarkNode {
    // Landmark相当于路标，每个Node与Landmark之间的相对位姿也应该加入到约束之中
    struct LandmarkObservation {
      int trajectory_id;
      common::Time time;
      transform::Rigid3d landmark_to_tracking_transform;//相对位姿
      double translation_weight;//translation的权重
      double rotation_weight;//rotation的权重
    };
    std::vector<LandmarkObservation> landmark_observations;
    common::optional<transform::Rigid3d> global_landmark_pose;
  };

  struct SubmapPose {
    int version;
    transform::Rigid3d pose;
  };

  struct SubmapData {
    std::shared_ptr<const Submap> submap;
    transform::Rigid3d pose;
  };

  struct TrajectoryData {
    double gravity_constant = 9.8;
    std::array<double, 4> imu_calibration{{1., 0., 0., 0.}};
    common::optional<transform::Rigid3d> fixed_frame_origin_in_map;
  };

  using GlobalSlamOptimizationCallback =
      std::function<void(const std::map<int /* trajectory_id */, SubmapId>&,
                         const std::map<int /* trajectory_id */, NodeId>&)>;

  PoseGraphInterface() {}
  virtual ~PoseGraphInterface() {}

  PoseGraphInterface(const PoseGraphInterface&) = delete;
  PoseGraphInterface& operator=(const PoseGraphInterface&) = delete;

  // Waits for all computations to finish and computes optimized poses.
  // 等待完成计算优化位姿
  virtual void RunFinalOptimization() = 0;

  // Returns data for all submaps.
  // 返回所有子地图的数据
  virtual MapById<SubmapId, SubmapData> GetAllSubmapData() const = 0;

  // Returns the global poses for all submaps.
  // 为所有位姿返回全局地图
  virtual MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const = 0;

  // Returns the transform converting data in the local map frame (i.e. the
  // continuous, non-loop-closed frame) into the global map frame (i.e. the
  // discontinuous, loop-closed frame).
  // 转换没有加入闭环的局部地图的坐标到全局地图坐标下
  virtual transform::Rigid3d GetLocalToGlobalTransform(
      int trajectory_id) const = 0;

  // Returns the current optimized trajectories.
  // 返回目前的优化轨迹
  virtual MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const = 0;

  // Returns the current optimized trajectory poses.
  // 返回目前的优化轨迹节点
  virtual MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses()
      const = 0;

  // Returns the current optimized landmark poses.
  // 返回最近的优化地标节点
  virtual std::map<std::string, transform::Rigid3d> GetLandmarkPoses()
      const = 0;

  // Sets global pose of landmark 'landmark_id' to given 'global_pose'.
  // 设置全局地图下的地标节点位姿
  virtual void SetLandmarkPose(const std::string& landmark_id,
                               const transform::Rigid3d& global_pose) = 0;

  // Checks if the given trajectory is finished.
  // 检查轨迹是否已经完成
  virtual bool IsTrajectoryFinished(int trajectory_id) const = 0;

  // Checks if the given trajectory is frozen.
  // 检查轨迹是否被冻住
  virtual bool IsTrajectoryFrozen(int trajectory_id) const = 0;

  // Returns the trajectory data.
  // 返回轨迹数据
  virtual std::map<int, TrajectoryData> GetTrajectoryData() const = 0;

  // Returns the collection of constraints.
  // 返回对约束的收集
  virtual std::vector<Constraint> constraints() const = 0;

  // Serializes the constraints and trajectories.
  // 一系列的约束和轨迹
  virtual proto::PoseGraph ToProto() const = 0;

  // Sets the callback function that is invoked whenever the global optimization
  // problem is solved.
  // 设置返回函数一旦全局优化问题被打断
  virtual void SetGlobalSlamOptimizationCallback(
      GlobalSlamOptimizationCallback callback) = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
