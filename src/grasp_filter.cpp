/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman <dave@picknik.ai>
   Desc:   Filters grasps based on kinematic feasibility and collision
*/

// moveit_grasps
#include <moveit_grasps/grasp_filter.h>
#include <moveit_grasps/state_validity_callback.h>

// moveit
#include <moveit/transforms/transforms.h>
#include <moveit/collision_detection/collision_tools.h>

// Conversions
#include <eigen_conversions/eigen_msg.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_grasps
{
// Constructor
GraspFilter::GraspFilter(const robot_state::RobotStatePtr& robot_state,
                         const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools)
  : name_("grasp_filter"), visual_tools_(visual_tools), nh_("~/moveit_grasps/filter")
{
  // Make a copy of the robot state so that we are sure outside influence does not break our grasp filter
  robot_state_ = std::make_shared<moveit::core::RobotState>(*robot_state);
  robot_state_->update();  // make sure transforms are computed

  // Load visulization settings
  const std::string parent_name = "grasp_filter";  // for namespacing logging messages
  std::size_t error = 0;
  // clang-format off
  error += !rosparam_shortcuts::get(parent_name, nh_, "collision_verbose", collision_verbose_);
  error += !rosparam_shortcuts::get(parent_name, nh_, "statistics_verbose", statistics_verbose_);
  error += !rosparam_shortcuts::get(parent_name, nh_, "collision_verbose_speed", collision_verbose_speed_);
  error += !rosparam_shortcuts::get(parent_name, nh_, "show_filtered_grasps", show_filtered_grasps_);
  error += !rosparam_shortcuts::get(parent_name, nh_, "show_filtered_arm_solutions", show_filtered_arm_solutions_);
  error += !rosparam_shortcuts::get(parent_name, nh_, "show_cutting_planes", show_cutting_planes_);
  error += !rosparam_shortcuts::get(parent_name, nh_, "show_filtered_arm_solutions_speed", show_filtered_arm_solutions_speed_);
  error += !rosparam_shortcuts::get(parent_name, nh_, "show_filtered_arm_solutions_pregrasp_speed", show_filtered_arm_solutions_pregrasp_speed_);
  error += !rosparam_shortcuts::get(parent_name, nh_, "show_grasp_filter_collision_if_failed", show_grasp_filter_collision_if_failed_);
  // clang-format on

  rosparam_shortcuts::shutdownIfError(parent_name, error);

  // Add planning scene publisher for debugging
  planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("/published_planning_scene", 100, false);
}

bool GraspFilter::filterGrasps(std::vector<GraspCandidatePtr>& grasp_candidates,
                               const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                               const robot_model::JointModelGroup* arm_jmg,
                               const moveit::core::RobotStatePtr& seed_state, bool filter_pregrasp,
                               const std::string& target_object_id)
{
  planning_scene::PlanningScenePtr planning_scene;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);
    planning_scene = planning_scene::PlanningScene::clone(scene);
  }
  return filterGrasps(grasp_candidates, planning_scene, arm_jmg, seed_state, filter_pregrasp, target_object_id);
}

bool GraspFilter::filterGrasps(std::vector<GraspCandidatePtr>& grasp_candidates,
                               const planning_scene::PlanningScenePtr& planning_scene,
                               const robot_model::JointModelGroup* arm_jmg,
                               const moveit::core::RobotStatePtr& seed_state, bool filter_pregrasp,
                               const std::string& target_object_id)
{
  bool verbose = false;

  // Error check
  if (grasp_candidates.empty())
  {
    ROS_ERROR_NAMED(name_, "Unable to filter grasps because vector is empty");
    return false;
  }
  if (!filter_pregrasp)
    ROS_WARN_STREAM_NAMED(name_, "Not filtering pre-grasp - GraspCandidate may have bad data");

  // Visualize the cutting planes if desired
  visualizeCuttingPlanes();

  // Get the solver timeout from kinematics.yaml
  solver_timeout_ = arm_jmg->getDefaultIKTimeout();
  ROS_DEBUG_STREAM_NAMED(name_ + ".superdebug", "Grasp filter IK timeout " << solver_timeout_);

  // Choose how many degrees of freedom
  num_variables_ = arm_jmg->getVariableCount();
  ROS_DEBUG_STREAM_NAMED(name_ + ".superdebug", "Solver for " << num_variables_ << " degrees of freedom");

  // Get the end effector joint model group
  if (arm_jmg->getAttachedEndEffectorNames().size() == 0)
  {
    ROS_ERROR_STREAM_NAMED(name_, "No end effectors attached to this arm");
    return false;
  }
  else if (arm_jmg->getAttachedEndEffectorNames().size() > 1)
  {
    ROS_ERROR_STREAM_NAMED(name_, "More than one end effectors attached to this arm");
    return false;
  }

  // Try to filter grasps not in verbose mode
  std::size_t remaining_grasps = filterGraspsHelper(grasp_candidates, planning_scene, arm_jmg, seed_state,
                                                    filter_pregrasp, verbose, target_object_id);

  // Print stats
  printFilterStatistics(grasp_candidates);

  if (remaining_grasps == 0)
  {
    ROS_INFO_STREAM_NAMED(name_, "Grasp filters removed all grasps");
    if (show_grasp_filter_collision_if_failed_)
    {
      ROS_INFO_STREAM_NAMED(name_, "Re-running in verbose mode since it failed");
      verbose = true;
      remaining_grasps = filterGraspsHelper(grasp_candidates, planning_scene, arm_jmg, seed_state, filter_pregrasp,
                                            verbose, target_object_id);
    }
    else
      ROS_INFO_STREAM_NAMED(name_, "NOT re-running in verbose mode");
  }

  // Visualize valid grasps as arrows with cartesian path as well
  if (show_filtered_grasps_)
  {
    ROS_INFO_STREAM_NAMED(name_, "Showing filtered grasps");
    visualizeGrasps(grasp_candidates, arm_jmg);
  }

  // Visualize valid grasp as arm positions
  if (show_filtered_arm_solutions_)
  {
    ROS_INFO_STREAM_NAMED(name_, "Showing filtered arm solutions");
    visualizeCandidateGrasps(grasp_candidates);
  }

  if (grasp_candidates.empty())
  {
    ROS_WARN_STREAM_NAMED(name_, "No grasps remaining after filtering");
    return false;
  }

  return true;
}

bool GraspFilter::filterGraspByPlane(GraspCandidatePtr& grasp_candidate, const Eigen::Isometry3d& filter_pose,
                                     GraspParallelPlane plane, int direction) const
{
  Eigen::Isometry3d grasp_pose;
  Eigen::Vector3d grasp_position;

  // get grasp translation in filter pose CS
  grasp_pose = visual_tools_->convertPose(grasp_candidate->grasp_.grasp_pose.pose);
  grasp_position = filter_pose.inverse() * grasp_pose.translation();

  // filter grasps by cutting plane
  double epsilon = 0.00000001;
  switch (plane)
  {
    case XY:
      if ((direction == -1 && grasp_position(2) < 0 + epsilon) || (direction == 1 && grasp_position(2) > 0 - epsilon))
        grasp_candidate->grasp_filtered_code_ = GraspFilterCode::GRASP_FILTERED_BY_CUTTING_PLANE;
      break;
    case XZ:
      if ((direction == -1 && grasp_position(1) < 0 + epsilon) || (direction == 1 && grasp_position(1) > 0 - epsilon))
        grasp_candidate->grasp_filtered_code_ = GraspFilterCode::GRASP_FILTERED_BY_CUTTING_PLANE;
      break;
    case YZ:
      if ((direction == -1 && grasp_position(0) < 0 + epsilon) || (direction == 1 && grasp_position(0) > 0 - epsilon))
        grasp_candidate->grasp_filtered_code_ = GraspFilterCode::GRASP_FILTERED_BY_CUTTING_PLANE;
      break;
    default:
      ROS_WARN_STREAM_NAMED("filter_by_plane", "plane not specified correctly");
      break;
  }

  return grasp_candidate->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_CUTTING_PLANE;
}

bool GraspFilter::filterGraspByOrientation(GraspCandidatePtr& grasp_candidate, const Eigen::Isometry3d& desired_pose,
                                           double max_angular_offset) const
{
  Eigen::Isometry3d tcp_grasp_pose;
  Eigen::Isometry3d eef_mount_grasp_pose;
  Eigen::Vector3d desired_z_axis;
  Eigen::Vector3d grasp_z_axis;
  double angle;

  // convert grasp pose back to standard grasping orientation
  eef_mount_grasp_pose = visual_tools_->convertPose(grasp_candidate->grasp_.grasp_pose.pose);
  tcp_grasp_pose = eef_mount_grasp_pose * grasp_candidate->grasp_data_->tcp_to_eef_mount_.inverse();

  // compute the angle between the z-axes of the desired and grasp poses
  grasp_z_axis = tcp_grasp_pose.rotation() * Eigen::Vector3d(0, 0, 1);
  desired_z_axis = desired_pose.rotation() * Eigen::Vector3d(0, 0, 1);
  double cos_angle = grasp_z_axis.normalized().dot(desired_z_axis.normalized());
  angle = acos(std::max(-1.0, std::min(1.0, cos_angle)));

  if (angle > max_angular_offset)
  {
    ROS_DEBUG_STREAM_NAMED(name_ + ".superdebug", "grasp filtered by orientation");
    grasp_candidate->grasp_filtered_code_ = GraspFilterCode::GRASP_FILTERED_BY_ORIENTATION;
    return true;
  }
  else
    return false;
}

bool GraspFilter::filterGraspByGraspIK(const GraspCandidatePtr& grasp_candidate, std::vector<double>& grasp_ik_solution,
                                       const IkThreadStructPtr& ik_thread_struct) const
{
  // Get pose
  ik_thread_struct->ik_pose_ = grasp_candidate->grasp_.grasp_pose;

  // Create constraint_fn
  moveit::core::GroupStateValidityCallbackFn constraint_fn =
      boost::bind(&isGraspStateValid, ik_thread_struct->planning_scene_.get(),
                  collision_verbose_ || ik_thread_struct->visual_debug_, collision_verbose_speed_, visual_tools_, _1,
                  _2, _3);

  // Set gripper position (eg. how open the eef is) to the custom open position
  grasp_candidate->getGraspStateOpenEEOnly(ik_thread_struct->robot_state_);

  // Solve IK Problem for grasp posture
  grasp_ik_solution.resize(0);
  if (!findIKSolution(grasp_ik_solution, ik_thread_struct, grasp_candidate, constraint_fn))
  {
    ROS_DEBUG_STREAM_NAMED(name_ + ".superdebug", "Unable to find the-grasp IK solution");
    grasp_candidate->grasp_filtered_code_ = GraspFilterCode::GRASP_FILTERED_BY_IK;
  }
  return grasp_candidate->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_IK;
}

bool GraspFilter::filterGraspByPreGraspIK(const GraspCandidatePtr& grasp_candidate,
                                          std::vector<double>& pregrasp_ik_solution,
                                          const IkThreadStructPtr& ik_thread_struct) const
{
  if (!ik_thread_struct->filter_pregrasp_)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Not filtering pregrasp");
    return true;
  }

  // Set IK target pose to the pre-grasp pose
  const std::string& ee_parent_link_name = grasp_candidate->grasp_data_->ee_jmg_->getEndEffectorParentGroup().second;
  ik_thread_struct->ik_pose_ = GraspGenerator::getPreGraspPose(grasp_candidate, ee_parent_link_name);

  moveit::core::GroupStateValidityCallbackFn constraint_fn =
      boost::bind(&isGraspStateValid, ik_thread_struct->planning_scene_.get(),
                  collision_verbose_ || ik_thread_struct->visual_debug_, collision_verbose_speed_, visual_tools_, _1,
                  _2, _3);

  // Solve IK Problem for pregrasp
  pregrasp_ik_solution.resize(0);
  if (!findIKSolution(pregrasp_ik_solution, ik_thread_struct, grasp_candidate, constraint_fn))
  {
    ROS_DEBUG_STREAM_NAMED(name_ + ".superdebug", "Unable to find PRE-grasp IK solution");
    grasp_candidate->grasp_filtered_code_ = GraspFilterCode::PREGRASP_FILTERED_BY_IK;
  }
  return grasp_candidate->grasp_filtered_code_ == GraspFilterCode::PREGRASP_FILTERED_BY_IK;
}

std::size_t
GraspFilter::filterGraspsHelper(std::vector<GraspCandidatePtr>& grasp_candidates,
                                const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                const robot_model::JointModelGroup* arm_jmg,
                                const moveit::core::RobotStatePtr& seed_state, bool filter_pregrasp, bool visualize,
                                const std::string& target_object_id)
{
  planning_scene::PlanningScenePtr planning_scene;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);
    planning_scene = planning_scene::PlanningScene::clone(scene);
  }
  return filterGraspsHelper(grasp_candidates, planning_scene, arm_jmg, seed_state, filter_pregrasp, visualize,
                            target_object_id);
}

std::size_t GraspFilter::filterGraspsHelper(std::vector<GraspCandidatePtr>& grasp_candidates,
                                            const planning_scene::PlanningScenePtr& planning_scene,
                                            const robot_model::JointModelGroup* arm_jmg,
                                            const moveit::core::RobotStatePtr& seed_state, bool filter_pregrasp,
                                            bool visualize, const std::string& target_object_id)
{
  // Clone the planning scene for const correctness
  planning_scene::PlanningScenePtr planning_scene_clone = planning_scene::PlanningScene::clone(planning_scene);

  // Setup collision checking
  *robot_state_ = planning_scene_clone->getCurrentState();

  // Check that we have grasp candidates
  if (!grasp_candidates.size())
  {
    ROS_WARN_NAMED(name_, "filterGraspsHelper passed empty grasp candidates");
    return 0;
  }

  // Choose Number of cores
  std::size_t num_threads = omp_get_max_threads();
  if (num_threads > grasp_candidates.size())
  {
    num_threads = grasp_candidates.size();
  }

  // Debug
  if (visualize || collision_verbose_)
  {
    num_threads = 1;
    ROS_WARN_STREAM_NAMED(name_, "Using only " << num_threads << " threads because verbose is true");
  }
  ROS_INFO_STREAM_NAMED(name_, "Filtering " << grasp_candidates.size() << " candidate grasps with " << num_threads
                                            << " threads");

  // Load kinematic solvers if not already loaded
  if (kin_solvers_[arm_jmg->getName()].size() != num_threads)
  {
    kin_solvers_[arm_jmg->getName()].clear();

    // Create an ik solver for every thread
    for (std::size_t i = 0; i < num_threads; ++i)
    {
      // ROS_DEBUG_STREAM_NAMED(name_,"Creating ik solver " << i);
      kin_solvers_[arm_jmg->getName()].push_back(arm_jmg->getSolverInstance());

      // Test to make sure we have a valid kinematics solver
      if (!kin_solvers_[arm_jmg->getName()][i])
      {
        ROS_ERROR_STREAM_NAMED(name_, "No kinematic solver found");
        return 0;
      }
    }
  }

  // Robot states
  // Create a robot state for every thread
  if (robot_states_.size() != num_threads)
  {
    robot_states_.clear();
    for (std::size_t i = 0; i < num_threads; ++i)
    {
      // Copy the previous robot state
      robot_states_.push_back(std::make_shared<robot_state::RobotState>(*robot_state_));
    }
  }
  else  // update the states
  {
    for (std::size_t i = 0; i < num_threads; ++i)
    {
      // Copy the previous robot state
      *(robot_states_[i]) = *robot_state_;
    }
  }

  // Transform poses
  // bring the pose to the frame of the IK solver
  const std::string& ik_frame = kin_solvers_[arm_jmg->getName()][0]->getBaseFrame();
  Eigen::Isometry3d link_transform;
  ROS_DEBUG_STREAM_NAMED(name_ + ".superdebug",
                         "Frame transform from ik_frame: " << ik_frame << " and robot model frame: "
                                                           << robot_state_->getRobotModel()->getModelFrame());
  if (!moveit::core::Transforms::sameFrame(ik_frame, robot_state_->getRobotModel()->getModelFrame()))
  {
    const robot_model::LinkModel* lm =
        robot_state_->getLinkModel((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);

    if (!lm)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to find frame for link transform");
      return 0;
    }

    link_transform = robot_state_->getGlobalLinkTransform(lm).inverse();
  }

  // Ensure the ACM entries are set to ignore collisions between the eef and the target object
  if (!target_object_id.empty())
  {
    if (!planning_scene_clone->knowsFrameTransform(target_object_id))
    {
      ROS_ERROR_STREAM_NAMED(name_, "target_object_id: " << target_object_id << " unknown to the planning scene");
      return 0;
    }
    std::vector<std::string> ee_links = grasp_candidates.front()->grasp_data_->ee_jmg_->getLinkModelNames();
    if (!ee_links.empty())
    {
      setACMFingerEntry(target_object_id, true, ee_links, planning_scene_clone);
    }
  }

  // Create the seed state vector
  std::vector<double> ik_seed_state;
  seed_state->copyJointGroupPositions(arm_jmg, ik_seed_state);

  // Thread data
  // Allocate only once to increase performance
  std::vector<IkThreadStructPtr> ik_thread_structs;
  ik_thread_structs.resize(num_threads);
  for (std::size_t thread_id = 0; thread_id < num_threads; ++thread_id)
  {
    ik_thread_structs[thread_id] =
        std::make_shared<IkThreadStruct>(grasp_candidates, planning_scene_clone, link_transform,
                                         0,  // this is filled in by OpenMP
                                         kin_solvers_[arm_jmg->getName()][thread_id], robot_states_[thread_id],
                                         solver_timeout_, filter_pregrasp, visualize, thread_id, target_object_id);
    ik_thread_structs[thread_id]->ik_seed_state_ = ik_seed_state;
  }

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Loop through poses and find those that are kinematically feasible

  omp_set_num_threads(num_threads);
#pragma omp parallel for schedule(dynamic)
  for (std::size_t grasp_id = 0; grasp_id < grasp_candidates.size(); ++grasp_id)
  {
    std::size_t thread_id = omp_get_thread_num();
    ROS_DEBUG_STREAM_NAMED(name_ + ".superdebug", "Thread " << thread_id << " processing grasp " << grasp_id);

    // If in verbose mode allow for quick exit
    if (ik_thread_structs[thread_id]->visual_debug_ && !ros::ok())
      continue;  // breaking a for loop is not allows with OpenMP

    // Assign grasp to process
    ik_thread_structs[thread_id]->grasp_id = grasp_id;

    // Process the grasp if it hasn't already been filtered out
    if (grasp_candidates[grasp_id]->isValid())
      processCandidateGrasp(ik_thread_structs[thread_id]);
  }

  if (statistics_verbose_)
  {
    // End Benchmark time
    double duration = (ros::Time::now() - start_time).toSec();
    ROS_INFO_STREAM_NAMED(name_ + ".print_filter_statistics", "===================================================");
    ROS_INFO_STREAM_NAMED(name_ + ".print_filter_statistics", "FILTER DURATION");
    ROS_INFO_STREAM_NAMED(name_ + ".print_filter_statistics", "Grasp Filter Duration :\t" << duration);
    ROS_INFO_STREAM_NAMED(name_ + ".print_filter_statistics", "---------------------------------------------------");
  }

  std::size_t not_filtered = 0;
  for (std::size_t i = 0; i < grasp_candidates.size(); ++i)
    if (grasp_candidates[i]->isValid())
      ++not_filtered;

  return not_filtered;
}

void GraspFilter::printFilterStatistics(const std::vector<GraspCandidatePtr>& grasp_candidates) const
{
  if (!statistics_verbose_)
    return;
  static const std::string logger_name = name_ + ".filter_statistics";
  // Count number of grasps remaining
  std::size_t not_filtered = 0;
  std::size_t grasp_filtered_by_ik = 0;
  std::size_t grasp_filtered_by_cutting_plane = 0;
  std::size_t grasp_filtered_by_orientation = 0;
  std::size_t pregrasp_filtered_by_ik = 0;

  for (std::size_t i = 0; i < grasp_candidates.size(); ++i)
  {
    if (grasp_candidates[i]->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_IK)
      ++grasp_filtered_by_ik;
    else if (grasp_candidates[i]->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_CUTTING_PLANE)
      ++grasp_filtered_by_cutting_plane;
    else if (grasp_candidates[i]->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_ORIENTATION)
      ++grasp_filtered_by_orientation;
    else if (grasp_candidates[i]->grasp_filtered_code_ == GraspFilterCode::PREGRASP_FILTERED_BY_IK)
      ++pregrasp_filtered_by_ik;
    else if (grasp_candidates[i]->isValid())
      ++not_filtered;
  }

  ROS_INFO_STREAM_NAMED(logger_name, "=======================================================");
  ROS_INFO_STREAM_NAMED(logger_name, "GRASP FILTER RESULTS ");
  ROS_INFO_STREAM_NAMED(logger_name, "Total candidate grasps          " << grasp_candidates.size());
  ROS_INFO_STREAM_NAMED(logger_name, "Total valid grasps              " << not_filtered);
  ROS_INFO_STREAM_NAMED(logger_name, "-------------------------------------------------------");
  ROS_INFO_STREAM_NAMED(logger_name, "grasp_filtered_by_cutting_plane " << grasp_filtered_by_cutting_plane);
  ROS_INFO_STREAM_NAMED(logger_name, "grasp_filtered_by_orientation   " << grasp_filtered_by_orientation);
  ROS_INFO_STREAM_NAMED(logger_name, "grasp_filtered_by_ik            " << grasp_filtered_by_ik);
  ROS_INFO_STREAM_NAMED(logger_name, "pregrasp_filtered_by_ik         " << pregrasp_filtered_by_ik);
}

bool GraspFilter::processCandidateGrasp(const IkThreadStructPtr& ik_thread_struct)
{
  ROS_DEBUG_STREAM_NAMED(name_ + ".superdebug", "Checking grasp #" << ik_thread_struct->grasp_id);

  // Helper pointer
  GraspCandidatePtr& grasp_candidate = ik_thread_struct->grasp_candidates_[ik_thread_struct->grasp_id];

  // Get pose
  ik_thread_struct->ik_pose_ = grasp_candidate->grasp_.grasp_pose;

  // Filter by cutting planes
  for (auto& cutting_plane : cutting_planes_)
  {
    if (filterGraspByPlane(grasp_candidate, cutting_plane->pose_, cutting_plane->plane_, cutting_plane->direction_))
    {
      return false;
    }
  }

  // Filter by desired orientation
  for (auto& desired_grasp_orientation : desired_grasp_orientations_)
  {
    if (filterGraspByOrientation(grasp_candidate, desired_grasp_orientation->pose_,
                                 desired_grasp_orientation->max_angle_offset_))
    {
      return false;
    }
  }

  std::vector<double> grasp_ik_solution;
  if (filterGraspByGraspIK(grasp_candidate, grasp_ik_solution, ik_thread_struct))
  {
    return false;
  }

  // Assign the grasp_ik solution.
  // TODO: This should be moved out of filtering
  grasp_candidate->grasp_ik_solution_ = grasp_ik_solution;

  // Copy solution to seed state so that next solution is faster
  ik_thread_struct->ik_seed_state_ = grasp_ik_solution;

  std::vector<double> pregrasp_ik_solution;
  if (filterGraspByPreGraspIK(grasp_candidate, pregrasp_ik_solution, ik_thread_struct))
  {
    return false;
  }
  // Assign the pregrasp_ik solution.
  // TODO: This should be moved out of filtering
  grasp_candidate->pregrasp_ik_solution_ = pregrasp_ik_solution;

  return true;
}

bool GraspFilter::findIKSolution(std::vector<double>& ik_solution, const IkThreadStructPtr& ik_thread_struct,
                                 const GraspCandidatePtr& grasp_candidate,
                                 const moveit::core::GroupStateValidityCallbackFn& constraint_fn) const
{
  robot_state::RobotState state = ik_thread_struct->planning_scene_->getCurrentState();
  if (ik_thread_struct->ik_seed_state_.size() == grasp_candidate->grasp_data_->arm_jmg_->getActiveJointModels().size())
  {
    state.setJointGroupPositions(grasp_candidate->grasp_data_->arm_jmg_, ik_thread_struct->ik_seed_state_);
    state.update();
  }

  // Copy in attached bodies which get removed by setJointGroupPositions for some reason
  std::vector<const robot_state::AttachedBody*> attached_bodies;
  ik_thread_struct->planning_scene_->getCurrentState().getAttachedBodies(attached_bodies);

  for (const robot_state::AttachedBody* ab : attached_bodies)
    state.attachBody(ab->getName(), ab->getPose(), ab->getShapes(), ab->getShapePoses(), ab->getTouchLinks(),
                     ab->getAttachedLinkName(), ab->getDetachPosture(), ab->getSubframes());

  bool ik_success = state.setFromIK(grasp_candidate->grasp_data_->arm_jmg_, ik_thread_struct->ik_pose_.pose,
                                    ik_thread_struct->timeout_, constraint_fn);

  // Results
  if (ik_success)
  {
    state.copyJointGroupPositions(grasp_candidate->grasp_data_->arm_jmg_, ik_solution);
    return true;
  }
  else
  {
    // The grasp was valid but the pre-grasp was not
    ROS_DEBUG_STREAM_NAMED(name_ + ".superdebug", "IK solution not found");
    return false;
  }
}

void GraspFilter::addCuttingPlane(const Eigen::Isometry3d& pose, GraspParallelPlane plane, int direction)
{
  cutting_planes_.push_back(std::make_shared<CuttingPlane>(pose, plane, direction));
}

void GraspFilter::addDesiredGraspOrientation(const Eigen::Isometry3d& pose, double max_angle_offset)
{
  desired_grasp_orientations_.push_back(std::make_shared<DesiredGraspOrientation>(pose, max_angle_offset));
}

bool GraspFilter::removeInvalidAndFilter(std::vector<GraspCandidatePtr>& grasp_candidates) const
{
  std::size_t original_num_grasps = grasp_candidates.size();

  // Remove all invalid grasps
  for (std::vector<GraspCandidatePtr>::iterator it = grasp_candidates.begin(); it != grasp_candidates.end();)
  {
    // Check if valid grasp
    if (!(*it)->isValid())
    {
      it = grasp_candidates.erase(it);  // not valid
    }
    else
    {
      ++it;  // was valid, keep
    }
  }
  ROS_INFO_STREAM_NAMED(name_, "Removed " << original_num_grasps - grasp_candidates.size()
                                          << " invalid grasp candidates, " << grasp_candidates.size() << " remaining");

  // Error Check
  if (grasp_candidates.empty())
  {
    ROS_WARN_STREAM_NAMED(name_, "No remaining grasp candidates");
    return false;
  }

  // Order remaining valid grasps by best score
  std::sort(grasp_candidates.begin(), grasp_candidates.end(), compareGraspScores);

  ROS_INFO_STREAM_NAMED(name_, "Sorted valid grasps, highest quality is "
                                   << grasp_candidates.front()->grasp_.grasp_quality << " and lowest quality is "
                                   << grasp_candidates.back()->grasp_.grasp_quality);

  return true;
}

bool GraspFilter::visualizeGrasps(const std::vector<GraspCandidatePtr>& grasp_candidates,
                                  const moveit::core::JointModelGroup* arm_jmg)
{
  // Publish in batch
  visual_tools_->enableBatchPublishing(true);

  /*
    NOTE: duplicated in README.md
    MAGENTA - grasp filtered by cutting plane
    YELLOW - grasp filtered by orientation
    RED - grasp filtered by ik
    PINK - grasp filtered by collision
    BLUE - pregrasp filtered by ik
    CYAN - pregrasp filtered by collision
    GREY - misc filtered
    GREEN - valid
  */
  ROS_INFO_STREAM_NAMED(name_, "Showing " << grasp_candidates.size() << " solutions at a speed of "
                                          << show_filtered_arm_solutions_speed_ << "sec per solution");
  ROS_INFO_STREAM_NAMED(name_, "---------------------------------------------");
  ROS_INFO_STREAM_NAMED(name_, "   MAGENTA - grasp filtered by cutting plane");
  ROS_INFO_STREAM_NAMED(name_, "   YELLOW  - grasp filtered by orientation");
  ROS_INFO_STREAM_NAMED(name_, "   RED     - grasp filtered by ik");
  ROS_INFO_STREAM_NAMED(name_, "   PINK    - grasp filtered by collision");
  ROS_INFO_STREAM_NAMED(name_, "   BLUE    - pregrasp filtered by ik");
  ROS_INFO_STREAM_NAMED(name_, "   CYAN    - pregrasp filtered by collision");
  ROS_INFO_STREAM_NAMED(name_, "   GREY    - misc filtered");
  ROS_INFO_STREAM_NAMED(name_, "   GREEN   - valid");
  ROS_INFO_STREAM_NAMED(name_, "---------------------------------------------");
  for (std::size_t i = 0; i < grasp_candidates.size(); ++i)
  {
    double size = 0.03;  // 0.01 * grasp_candidates[i]->grasp_.grasp_quality;

    if (grasp_candidates[i]->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_IK)
    {
      visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose.pose, rviz_visual_tools::RED,
                                   rviz_visual_tools::SMALL, size);
    }
    else if (grasp_candidates[i]->grasp_filtered_code_ == GraspFilterCode::PREGRASP_FILTERED_BY_IK)
    {
      visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose.pose, rviz_visual_tools::BLUE,
                                   rviz_visual_tools::SMALL, size);
    }
    else if (grasp_candidates[i]->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_CUTTING_PLANE)
    {
      visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose.pose, rviz_visual_tools::MAGENTA,
                                   rviz_visual_tools::SMALL, size);
    }
    else if (grasp_candidates[i]->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_ORIENTATION)
    {
      visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose.pose, rviz_visual_tools::YELLOW,
                                   rviz_visual_tools::SMALL, size);
    }
    else if (grasp_candidates[i]->grasp_filtered_code_ == GraspFilterCode::NOT_FILTERED)
    {
      visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose.pose, rviz_visual_tools::GREEN,
                                   rviz_visual_tools::SMALL, size);
    }
    else
      visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose.pose, rviz_visual_tools::GREY,
                                   rviz_visual_tools::SMALL, size);

    // Publish in batch
    visual_tools_->trigger();
    ros::Duration(.01).sleep();
  }

  return true;
}

bool GraspFilter::visualizeIKSolutions(const std::vector<GraspCandidatePtr>& grasp_candidates,
                                       const moveit::core::JointModelGroup* arm_jmg, double animation_speed)
{
  // Convert the grasp_candidates into a format moveit_visual_tools can use
  std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
  for (std::size_t i = 0; i < grasp_candidates.size(); ++i)
  {
    // Check if valid grasp
    if (!grasp_candidates[i]->isValid())
      continue;

    trajectory_msgs::JointTrajectoryPoint new_point;
    new_point.positions = grasp_candidates[i]->grasp_ik_solution_;
    ik_solutions.push_back(new_point);
  }

  if (ik_solutions.empty())
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to visualize IK solutions because there are no valid ones");
    return false;
  }

  return visual_tools_->publishIKSolutions(ik_solutions, arm_jmg, animation_speed);
}

bool GraspFilter::visualizeCandidateGrasps(const std::vector<GraspCandidatePtr>& grasp_candidates)
{
  for (std::size_t i = 0; i < grasp_candidates.size(); ++i)
  {
    // Check if valid grasp
    if (!grasp_candidates[i]->isValid())
      continue;

    // Apply the pregrasp state
    if (grasp_candidates[i]->getPreGraspState(robot_state_))
    {
      // Show in Rviz
      visual_tools_->publishRobotState(robot_state_, rviz_visual_tools::ORANGE);
      visual_tools_->trigger();
      ros::Duration(show_filtered_arm_solutions_pregrasp_speed_).sleep();
    }

    // Apply the grasp state
    if (grasp_candidates[i]->getGraspStateClosed(robot_state_))
    {
      // Show in Rviz
      visual_tools_->publishRobotState(robot_state_, rviz_visual_tools::WHITE);
      visual_tools_->trigger();
      ros::Duration(show_filtered_arm_solutions_speed_).sleep();
    }
  }

  return true;
}

bool GraspFilter::visualizeCuttingPlanes()
{
  if (show_cutting_planes_)
  {
    for (auto& cutting_plane : cutting_planes_)
    {
      switch (cutting_plane->plane_)
      {
        case XY:
          visual_tools_->publishXYPlane(cutting_plane->pose_);
          break;
        case XZ:
          visual_tools_->publishXZPlane(cutting_plane->pose_);
          break;
        case YZ:
          visual_tools_->publishYZPlane(cutting_plane->pose_);
          break;
        default:
          ROS_ERROR_STREAM_NAMED(name_, "Unknown cutting plane type");
      }
    }
    visual_tools_->trigger();
  }
  return true;
}

void GraspFilter::clearCuttingPlanes()
{
  cutting_planes_.clear();
}

void GraspFilter::clearDesiredGraspOrientations()
{
  desired_grasp_orientations_.clear();
}

bool GraspFilter::addCuttingPlanesForBin(const Eigen::Isometry3d& world_to_bin, const Eigen::Isometry3d& bin_to_product,
                                         const double& bin_width, const double& bin_height)
{
  // Add grasp filters
  clearCuttingPlanes();
  clearDesiredGraspOrientations();

  // Bottom of bin
  addCuttingPlane(world_to_bin, XY, -1);

  // Right wall of bin
  addCuttingPlane(world_to_bin, XZ, -1);

  // Top of bin
  Eigen::Isometry3d world_to_bin_top = world_to_bin;
  world_to_bin_top.translation() += Eigen::Vector3d(0, bin_width, bin_height);
  addCuttingPlane(world_to_bin_top, XY, 1);

  // Left wall of bin
  addCuttingPlane(world_to_bin_top, XZ, 1);

  // // Back half of product
  Eigen::Isometry3d world_to_bin_back = world_to_bin;
  world_to_bin_back.translation() +=
      Eigen::Vector3d(bin_to_product.translation().x(), bin_width / 2.0, bin_height / 2.0);
  addCuttingPlane(world_to_bin_back, YZ, 1);

  return true;
}

void GraspFilter::setACMFingerEntry(const std::string& object_name, bool allowed,
                                    const std::vector<std::string>& ee_link_names,
                                    const planning_scene::PlanningScenePtr& scene)
{
  static const std::string logger_name = name_ + ".set_acm_finger_entry";
  ROS_DEBUG_STREAM_NAMED(logger_name, "" << object_name.c_str() << ", " << (allowed ? "true" : "false"));

  // Lock planning scene
  for (std::size_t i = 0; i < ee_link_names.size(); ++i)
  {
    ROS_DEBUG_NAMED(logger_name, "collisions between %s and %s : %s", object_name.c_str(), ee_link_names[i].c_str(),
                    allowed ? "allowed" : "not allowed");
    scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, ee_link_names[i], allowed);
  }

  // Debug current matrix
  if (false)
  {
    moveit_msgs::AllowedCollisionMatrix msg;
    scene->getAllowedCollisionMatrix().getMessage(msg);
    ROS_DEBUG_STREAM_NAMED(logger_name, "Current collision matrix: " << msg);
  }
}

void GraspFilter::publishPlanningScene(const planning_scene::PlanningScenePtr& ps) const
{
  moveit_msgs::PlanningScene msg;
  ps->getPlanningSceneMsg(msg);
  planning_scene_publisher_.publish(msg);
}

}  // namespace moveit_grasps
