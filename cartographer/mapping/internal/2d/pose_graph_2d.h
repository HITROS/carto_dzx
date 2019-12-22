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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"
#include "cartographer/mapping/internal/trajectory_connectivity_state.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping:
// Each node has been matched against one or more submaps (adding a constraint
// for each match), both poses of nodes and of submaps are to be optimized.
// All constraints are between a submap i and a node j.
class PoseGraph2D : public PoseGraph {
 public:
  PoseGraph2D(
      const proto::PoseGraphOptions& options,
      std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
      common::ThreadPool* thread_pool);
  ~PoseGraph2D() override;

  PoseGraph2D(const PoseGraph2D&) = delete;
  PoseGraph2D& operator=(const PoseGraph2D&) = delete;
  // 构建位姿图
  // Adds a new node with 'constant_data'. Its 'constant_data->local_pose' was
  // determined by scan matching against 'insertion_submaps.front()' and the
  // node data was inserted into the 'insertion_submaps'. If
  // 'insertion_submaps.front().finished()' is 'true', data was inserted into
  // this submap for the last time.
  // 用“ constant_data”添加一个新节点。 
  // 通过与“ insertion_submaps.front（）”进行扫描匹配来确定其“ constant_data-> local_pose”，
  // 并将节点数据插入“ insertion_submaps”中。 如果“ insertion_submaps.front（）
  // finished（）”为“ true”，则数据是最后一次插入此子图中。
  NodeId AddNode(
      std::shared_ptr<const TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps)
      EXCLUDES(mutex_);

  void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) override
      EXCLUDES(mutex_);
  void AddOdometryData(int trajectory_id,
                       const sensor::OdometryData& odometry_data) override
      EXCLUDES(mutex_);
  void AddFixedFramePoseData(
      int trajectory_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data) override
      EXCLUDES(mutex_);
  void AddLandmarkData(int trajectory_id,
                       const sensor::LandmarkData& landmark_data) override
      EXCLUDES(mutex_);

  void FinishTrajectory(int trajectory_id) override;
  bool IsTrajectoryFinished(int trajectory_id) const override REQUIRES(mutex_);
  void FreezeTrajectory(int trajectory_id) override;
  bool IsTrajectoryFrozen(int trajectory_id) const override REQUIRES(mutex_);
  void AddSubmapFromProto(const transform::Rigid3d& global_submap_pose,
                          const proto::Submap& submap) override;
  void AddNodeFromProto(const transform::Rigid3d& global_pose,
                        const proto::Node& node) override;
  void SetTrajectoryDataFromProto(const proto::TrajectoryData& data) override;
  void AddNodeToSubmap(const NodeId& node_id,
                       const SubmapId& submap_id) override;
  void AddSerializedConstraints(
      const std::vector<Constraint>& constraints) override;
  void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) override;
  void RunFinalOptimization() override;
  std::vector<std::vector<int>> GetConnectedTrajectories() const override;
  PoseGraphInterface::SubmapData GetSubmapData(const SubmapId& submap_id) const
      EXCLUDES(mutex_) override;
  MapById<SubmapId, PoseGraphInterface::SubmapData> GetAllSubmapData() const
      EXCLUDES(mutex_) override;
  MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const
      EXCLUDES(mutex_) override;
  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) const
      EXCLUDES(mutex_) override;
  MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const override
      EXCLUDES(mutex_);
  MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const override
      EXCLUDES(mutex_);
  std::map<std::string, transform::Rigid3d> GetLandmarkPoses() const override
      EXCLUDES(mutex_);
  void SetLandmarkPose(const std::string& landmark_id,
                       const transform::Rigid3d& global_pose) override
      EXCLUDES(mutex_);
  sensor::MapByTime<sensor::ImuData> GetImuData() const override
      EXCLUDES(mutex_);
  sensor::MapByTime<sensor::OdometryData> GetOdometryData() const override
      EXCLUDES(mutex_);
  sensor::MapByTime<sensor::FixedFramePoseData> GetFixedFramePoseData()
      const override EXCLUDES(mutex_);
  std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
  GetLandmarkNodes() const override EXCLUDES(mutex_);
  std::map<int, TrajectoryData> GetTrajectoryData() const override
      EXCLUDES(mutex_);
  std::vector<Constraint> constraints() const override EXCLUDES(mutex_);
  void SetInitialTrajectoryPose(int from_trajectory_id, int to_trajectory_id,
                                const transform::Rigid3d& pose,
                                const common::Time time) override
      EXCLUDES(mutex_);
  void SetGlobalSlamOptimizationCallback(
      PoseGraphInterface::GlobalSlamOptimizationCallback callback) override;
  transform::Rigid3d GetInterpolatedGlobalTrajectoryPose(
      int trajectory_id, const common::Time time) const REQUIRES(mutex_);

 private:
  // The current state of the submap in the background threads. When this
  // transitions to kFinished, all nodes are tried to match against this submap.
  // Likewise, all new nodes are matched against submaps which are finished.
  // 子地图在后台线程中的当前状态。
  // 当过渡到kFinished时，将尝试所有节点都与此子图进行匹配。同样，所有新节点都与完成的子图进行匹配。
  enum class SubmapState { kActive, kFinished };
  struct InternalSubmapData {
    std::shared_ptr<const Submap2D> submap;

    // IDs of the nodes that were inserted into this map together with
    // constraints for them. They are not to be matched again when this submap
    // becomes 'finished'.
    // 节点的id被当做约束插入到地图当中，将不会再次被匹配，当这个子地图完成之后
    std::set<NodeId> node_ids;

    SubmapState state = SubmapState::kActive;
  };

  MapById<SubmapId, PoseGraphInterface::SubmapData> GetSubmapDataUnderLock()
      const REQUIRES(mutex_);

  // Handles a new work item.
  void AddWorkItem(const std::function<void()>& work_item) REQUIRES(mutex_);

  // Adds connectivity and sampler for a trajectory if it does not exist.
  void AddTrajectoryIfNeeded(int trajectory_id) REQUIRES(mutex_);

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  std::vector<SubmapId> InitializeGlobalSubmapPoses(
      int trajectory_id, const common::Time time,
      const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps)
      REQUIRES(mutex_);

  // Adds constraints for a node, and starts scan matching in the background.
  // 为一个节点添加约束，在后台进行扫描匹配
  void ComputeConstraintsForNode(
      const NodeId& node_id,
      std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,
      bool newly_finished_submap) REQUIRES(mutex_);

  // Computes constraints for a node and submap pair.
  // 建立约束为一会节点和子地图
  void ComputeConstraint(const NodeId& node_id, const SubmapId& submap_id)
      REQUIRES(mutex_);

  // Adds constraints for older nodes whenever a new submap is finished.
  // 当一个新的子地图被创建完成时，为原来的节点添加新的约束
  void ComputeConstraintsForOldNodes(const SubmapId& submap_id)
      REQUIRES(mutex_);

  // Runs the optimization, executes the trimmers and processes the work queue.
  // 新的子地图加入进来时进行优化
  void HandleWorkQueue(const constraints::ConstraintBuilder2D::Result& result)
      REQUIRES(mutex_);

  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  // 等待优化完成
  void WaitForAllComputations() EXCLUDES(mutex_);

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  // 一次只能有一个优化在进行
  void RunOptimization() EXCLUDES(mutex_);

  // Computes the local to global map frame transform based on the given
  // 'global_submap_poses'.
  // 计算局部地图和全局地图之间的坐标转换
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
      int trajectory_id) const REQUIRES(mutex_);
  // 得到子地图数据
  SubmapData GetSubmapDataUnderLock(const SubmapId& submap_id) const
      REQUIRES(mutex_);
  // 得到最新节点的数据
  common::Time GetLatestNodeTime(const NodeId& node_id,
                                 const SubmapId& submap_id) const
      REQUIRES(mutex_);

  // Updates the trajectory connectivity structure with a new constraint.
  // 更新位姿约束
  void UpdateTrajectoryConnectivity(const Constraint& constraint)
      REQUIRES(mutex_);

  const proto::PoseGraphOptions options_;
  GlobalSlamOptimizationCallback global_slam_optimization_callback_;
  mutable common::Mutex mutex_;

  // If it exists, further work items must be added to this queue, and will be
  // considered later.
  std::unique_ptr<std::deque<std::function<void()>>> work_queue_
      GUARDED_BY(mutex_);

  // How our various trajectories are related.
  // 多条轨迹是怎样进行相关的
  TrajectoryConnectivityState trajectory_connectivity_state_;

  // We globally localize a fraction of the nodes from each trajectory.
  // 我们从全局上定位每一条轨迹的节点
  std::unordered_map<int, std::unique_ptr<common::FixedRatioSampler>>
      global_localization_samplers_ GUARDED_BY(mutex_);

  // Number of nodes added since last loop closure.
  // 自从上一条回环之后大量的节点被添加
  int num_nodes_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

  // Whether the optimization has to be run before more data is added.
  // 判断更多的节点加入进去之后优化问题是否必须要进行
  bool run_loop_closure_ GUARDED_BY(mutex_) = false;

  // Schedules optimization (i.e. loop closure) to run.
  // 准备进行优化
  void DispatchOptimization() REQUIRES(mutex_);

  // Current optimization problem.
  // 目前的优化问题
  std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem_;
  constraints::ConstraintBuilder2D constraint_builder_ GUARDED_BY(mutex_);
  std::vector<Constraint> constraints_ GUARDED_BY(mutex_);

  // Submaps get assigned an ID and state as soon as they are seen, even
  // before they take part in the background computations.
  // 优化尽可能多的子地图
  MapById<SubmapId, InternalSubmapData> submap_data_ GUARDED_BY(mutex_);

  // Data that are currently being shown.
  // 显示数据
  MapById<NodeId, TrajectoryNode> trajectory_nodes_ GUARDED_BY(mutex_);
  int num_trajectory_nodes_ GUARDED_BY(mutex_) = 0;

  // Global submap poses currently used for displaying data.
  // 全局子地图被用来显示数据
  MapById<SubmapId, optimization::SubmapSpec2D> global_submap_poses_
      GUARDED_BY(mutex_);

  // Global landmark poses with all observations.
  // 全局landmark位姿
  std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
      landmark_nodes_ GUARDED_BY(mutex_);

  // List of all trimmers to consult when optimizations finish.
  // 当优化完成时列举所有的剪枝
  std::vector<std::unique_ptr<PoseGraphTrimmer>> trimmers_ GUARDED_BY(mutex_);

  // Set of all frozen trajectories not being optimized.
  // 设定所有的没有被优化的轨迹
  std::set<int> frozen_trajectories_ GUARDED_BY(mutex_);

  // Set of all finished trajectories.
  // 设定所有已经完成的轨迹
  std::set<int> finished_trajectories_ GUARDED_BY(mutex_);

  // Set of all initial trajectory poses.
  // 设定所有初始化的轨迹节点
  std::map<int, InitialTrajectoryPose> initial_trajectory_poses_
      GUARDED_BY(mutex_);
      // 位姿图创建完成

  // Allows querying and manipulating the pose graph by the 'trimmers_'. The
  // 'mutex_' of the pose graph is held while this class is used.
  // 允许被trimmers访问和计算位姿图
  class TrimmingHandle : public Trimmable {
   public:
    TrimmingHandle(PoseGraph2D* parent);
    ~TrimmingHandle() override {}

    int num_submaps(int trajectory_id) const override;
    std::vector<SubmapId> GetSubmapIds(int trajectory_id) const override;
    MapById<SubmapId, SubmapData> GetOptimizedSubmapData() const override
        REQUIRES(parent_->mutex_);
    const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes() const override
        REQUIRES(parent_->mutex_);
    const std::vector<Constraint>& GetConstraints() const override
        REQUIRES(parent_->mutex_);
    void MarkSubmapAsTrimmed(const SubmapId& submap_id)
        REQUIRES(parent_->mutex_) override;
    bool IsFinished(int trajectory_id) const override REQUIRES(parent_->mutex_);

   private:
    PoseGraph2D* const parent_;
  };
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
