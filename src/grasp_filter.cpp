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

/* Author: Dave Coleman <dave@dav.ee>
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

namespace
{
bool ikCallbackFnAdapter(moveit::core::RobotState* state, const moveit::core::JointModelGroup* group,
                         const moveit::core::GroupStateValidityCallbackFn& constraint, const geometry_msgs::Pose&,
                         const std::vector<double>& ik_sol, moveit_msgs::MoveItErrorCodes& error_code)
{
  const std::vector<unsigned int>& bij = group->getKinematicsSolverJointBijection();
  std::vector<double> solution(bij.size());
  for (std::size_t i = 0; i < bij.size(); ++i)
    solution[bij[i]] = ik_sol[i];
  if (constraint(state, group, &solution[0]))
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  return true;
}
}

namespace moveit_grasps
{
// Constructor
GraspFilter::GraspFilter(robot_state::RobotStatePtr robot_state,
                         moveit_visual_tools::MoveItVisualToolsPtr& visual_tools)
  : visual_tools_(visual_tools), nh_("~/moveit_grasps/filter")
{
  // Make a copy of the robot state so that we are sure outside influence does not break our grasp filter
  robot_state_.reset(new moveit::core::RobotState(*robot_state));
  robot_state_->update();  // make sure transforms are computed

  // Load visulization settings
  const std::string parent_name = "grasp_filter";  // for namespacing logging messages
  rosparam_shortcuts::get(parent_name, nh_, "collision_verbose", collision_verbose_);
  rosparam_shortcuts::get(parent_name, nh_, "statistics_verbose", statistics_verbose_);
  rosparam_shortcuts::get(parent_name, nh_, "collision_verbose_speed", collision_verbose_speed_);
  rosparam_shortcuts::get(parent_name, nh_, "show_filtered_grasps", show_filtered_grasps_);
  rosparam_shortcuts::get(parent_name, nh_, "show_filtered_arm_solutions", show_filtered_arm_solutions_);
  rosparam_shortcuts::get(parent_name, nh_, "show_cutting_planes", show_cutting_planes_);
  rosparam_shortcuts::get(parent_name, nh_, "show_filtered_arm_solutions_speed", show_filtered_arm_solutions_speed_);
  rosparam_shortcuts::get(parent_name, nh_, "show_filtered_arm_solutions_pregrasp_speed",
                          show_filtered_arm_solutions_pregrasp_speed_);
  rosparam_shortcuts::get(parent_name, nh_, "show_grasp_filter_collision_if_failed",
                          show_grasp_filter_collision_if_failed_);

  ROS_INFO_STREAM_NAMED("grasp_filter", "GraspFilter Ready.");
}

bool GraspFilter::filterGrasps(std::vector<GraspCandidatePtr>& grasp_candidates,
                               planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                               const robot_model::JointModelGroup* arm_jmg,
                               const moveit::core::RobotStatePtr seed_state, bool filter_pregrasp)
{
  bool verbose = false;

  // -----------------------------------------------------------------------------------------------
  // Error check
  if (grasp_candidates.empty())
  {
    ROS_ERROR_NAMED("grasp_filter", "Unable to filter grasps because vector is empty");
    return false;
  }
  if (!filter_pregrasp)
    ROS_WARN_STREAM_NAMED("grasp_filter", "Not filtering pre-grasp - GraspCandidate may have bad data");

  // -----------------------------------------------------------------------------------------------
  // Visualize the cutting planes if desired
  visualizeCuttingPlanes();

  // -----------------------------------------------------------------------------------------------
  // Get the solver timeout from kinematics.yaml
  solver_timeout_ = arm_jmg->getDefaultIKTimeout();
  ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Grasp filter IK timeout " << solver_timeout_);

  // -----------------------------------------------------------------------------------------------
  // Choose how many degrees of freedom
  num_variables_ = arm_jmg->getVariableCount();
  ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Solver for " << num_variables_ << " degrees of freedom");

  // -----------------------------------------------------------------------------------------------
  // Get the end effector joint model group
  if (arm_jmg->getAttachedEndEffectorNames().size() == 0)
  {
    ROS_ERROR_STREAM_NAMED("grasp_filter", "No end effectors attached to this arm");
    return false;
  }
  else if (arm_jmg->getAttachedEndEffectorNames().size() > 1)
  {
    ROS_ERROR_STREAM_NAMED("grasp_filter", "More than one end effectors attached to this arm");
    return false;
  }

  // Try to filter grasps not in verbose mode
  std::size_t remaining_grasps =
      filterGraspsHelper(grasp_candidates, planning_scene_monitor, arm_jmg, seed_state, filter_pregrasp, verbose);

  if (remaining_grasps == 0)
  {
    ROS_WARN_STREAM_NAMED("grasp_filter", "Grasp filters removed all grasps!");
    if (show_grasp_filter_collision_if_failed_)
    {
      ROS_INFO_STREAM_NAMED("grasp_filter", "Re-running in verbose mode since it failed");
      verbose = true;
      remaining_grasps =
          filterGraspsHelper(grasp_candidates, planning_scene_monitor, arm_jmg, seed_state, filter_pregrasp, verbose);
    }
    else
      ROS_INFO_STREAM_NAMED("grasp_filter", "NOT re-running in verbose mode");
  }

  // Visualize valid grasps as arrows with cartesian path as well
  if (show_filtered_grasps_)
  {
    ROS_INFO_STREAM_NAMED("grasp_filter", "Showing filtered grasps");
    visualizeGrasps(grasp_candidates, arm_jmg);
  }

  // Visualize valid grasp as arm positions
  if (show_filtered_arm_solutions_)
  {
    ROS_INFO_STREAM_NAMED("grasp_filter", "Showing filtered arm solutions");
    visualizeCandidateGrasps(grasp_candidates);
  }

  if (grasp_candidates.empty())
  {
    ROS_WARN_STREAM_NAMED("grasp_filter", "No grasps remaining after filtering");
    return false;
  }

  return true;
}

bool GraspFilter::filterGraspByPlane(GraspCandidatePtr grasp_candidate, Eigen::Affine3d filter_pose,
                                     grasp_parallel_plane plane, int direction)
{
  Eigen::Affine3d grasp_pose;
  Eigen::Vector3d grasp_position;

  // get grasp translation in filter pose CS
  grasp_pose = visual_tools_->convertPose(grasp_candidate->grasp_.grasp_pose.pose);
  grasp_position = filter_pose.inverse() * grasp_pose.translation();

  // filter grasps by cutting plane
  double epsilon = 0.00000001;  //
  switch (plane)
  {
    case XY:
      if ((direction == -1 && grasp_position(2) < 0 + epsilon) || (direction == 1 && grasp_position(2) > 0 - epsilon))
        grasp_candidate->grasp_filtered_by_cutting_plane_ = true;
      break;
    case XZ:
      if ((direction == -1 && grasp_position(1) < 0 + epsilon) || (direction == 1 && grasp_position(1) > 0 - epsilon))
        grasp_candidate->grasp_filtered_by_cutting_plane_ = true;
      break;
    case YZ:
      if ((direction == -1 && grasp_position(0) < 0 + epsilon) || (direction == 1 && grasp_position(0) > 0 - epsilon))
        grasp_candidate->grasp_filtered_by_cutting_plane_ = true;
      break;
    default:
      ROS_WARN_STREAM_NAMED("filter_by_plane", "plane not specified correctly");
      break;
  }

  return grasp_candidate->grasp_filtered_by_cutting_plane_;
}

bool GraspFilter::filterGraspByOrientation(GraspCandidatePtr grasp_candidate, Eigen::Affine3d desired_pose,
                                           double max_angular_offset)
{
  Eigen::Affine3d std_grasp_pose;
  Eigen::Affine3d grasp_pose;
  Eigen::Vector3d desired_z_axis;
  Eigen::Vector3d grasp_z_axis;
  double angle;

  // convert grasp pose back to standard grasping orientation
  grasp_pose = visual_tools_->convertPose(grasp_candidate->grasp_.grasp_pose.pose);
  std_grasp_pose = grasp_pose * grasp_candidate->grasp_data_->grasp_pose_to_eef_pose_.inverse();

  // compute the angle between the z-axes of the desired and grasp poses
  grasp_z_axis = std_grasp_pose.rotation() * Eigen::Vector3d(0, 0, 1);
  desired_z_axis = desired_pose.rotation() * Eigen::Vector3d(0, 0, 1);
  angle = acos(grasp_z_axis.normalized().dot(desired_z_axis.normalized()));

  if (angle > max_angular_offset)
  {
    grasp_candidate->grasp_filtered_by_orientation_ = true;
    return true;
  }
  else
    return false;
}

std::size_t GraspFilter::filterGraspsHelper(std::vector<GraspCandidatePtr>& grasp_candidates,
                                            planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                            const robot_model::JointModelGroup* arm_jmg,
                                            const moveit::core::RobotStatePtr seed_state, bool filter_pregrasp,
                                            bool verbose)
{
  // -----------------------------------------------------------------------------------------------
  // Setup collision checking

  // Copy planning scene that is locked
  planning_scene::PlanningScenePtr cloned_scene;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);
    cloned_scene = planning_scene::PlanningScene::clone(scene);
  }
  *robot_state_ = cloned_scene->getCurrentState();

  // -----------------------------------------------------------------------------------------------
  // Choose Number of cores
  std::size_t num_threads = omp_get_max_threads();
  if (num_threads > grasp_candidates.size())
  {
    num_threads = grasp_candidates.size();
  }

  // Debug
  if (verbose || collision_verbose_)
  {
    num_threads = 1;
    ROS_WARN_STREAM_NAMED("grasp_filter", "Using only " << num_threads << " threads because verbose is true");
  }
  ROS_INFO_STREAM_NAMED("grasp_filter", "Filtering " << grasp_candidates.size() << " candidate grasps with "
                                                     << num_threads << " threads");

  // -----------------------------------------------------------------------------------------------
  // Load kinematic solvers if not already loaded
  if (kin_solvers_[arm_jmg->getName()].size() != num_threads)
  {
    kin_solvers_[arm_jmg->getName()].clear();

    // Create an ik solver for every thread
    for (std::size_t i = 0; i < num_threads; ++i)
    {
      // ROS_DEBUG_STREAM_NAMED("grasp_filter","Creating ik solver " << i);
      kin_solvers_[arm_jmg->getName()].push_back(arm_jmg->getSolverInstance());

      // Test to make sure we have a valid kinematics solver
      if (!kin_solvers_[arm_jmg->getName()][i])
      {
        ROS_ERROR_STREAM_NAMED("grasp_filter", "No kinematic solver found");
        return 0;
      }
    }
  }

  // Robot states -------------------------------------------------------------------------------
  // Create a robot state for every thread
  if (robot_states_.size() != num_threads)
  {
    robot_states_.clear();
    for (std::size_t i = 0; i < num_threads; ++i)
    {
      // Copy the previous robot state
      robot_states_.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(*robot_state_)));
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

  // Transform poses -------------------------------------------------------------------------------
  // bring the pose to the frame of the IK solver
  const std::string& ik_frame = kin_solvers_[arm_jmg->getName()][0]->getBaseFrame();
  Eigen::Affine3d link_transform;
  ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug",
                         "Frame transform from ik_frame: " << ik_frame << " and robot model frame: "
                                                           << robot_state_->getRobotModel()->getModelFrame());
  if (!moveit::core::Transforms::sameFrame(ik_frame, robot_state_->getRobotModel()->getModelFrame()))
  {
    const robot_model::LinkModel* lm =
        robot_state_->getLinkModel((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);

    if (!lm)
    {
      ROS_ERROR_STREAM_NAMED("grasp_filter", "Unable to find frame for link transform");
      return 0;
    }

    link_transform = robot_state_->getGlobalLinkTransform(lm).inverse();
  }

  // Create the seed state vector
  std::vector<double> ik_seed_state;
  seed_state->copyJointGroupPositions(arm_jmg, ik_seed_state);

  // Thread data -------------------------------------------------------------------------------
  // Allocate only once to increase performance
  std::vector<IkThreadStructPtr> ik_thread_structs;
  ik_thread_structs.resize(num_threads);
  for (std::size_t thread_id = 0; thread_id < num_threads; ++thread_id)
  {
    ik_thread_structs[thread_id].reset(new moveit_grasps::IkThreadStruct(grasp_candidates, cloned_scene, link_transform,
                                                                         0,  // this is filled in by OpenMP
                                                                         kin_solvers_[arm_jmg->getName()][thread_id],
                                                                         robot_states_[thread_id], solver_timeout_,
                                                                         filter_pregrasp, verbose, thread_id));
    ik_thread_structs[thread_id]->ik_seed_state_ = ik_seed_state;
  }

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // -----------------------------------------------------------------------------------------------
  // Loop through poses and find those that are kinematically feasible

  omp_set_num_threads(num_threads);
#pragma omp parallel for schedule(dynamic)
  for (std::size_t grasp_id = 0; grasp_id < grasp_candidates.size(); ++grasp_id)
  {
    std::size_t thread_id = omp_get_thread_num();
    ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Thread " << thread_id << " processing grasp " << grasp_id);

    // If in verbose mode allow for quick exit
    if (ik_thread_structs[thread_id]->verbose_ && !ros::ok())
      continue;  // breaking a for loop is not allows with OpenMP

    // Assign grasp to process
    ik_thread_structs[thread_id]->grasp_id = grasp_id;

    // Process the grasp
    processCandidateGrasp(ik_thread_structs[thread_id]);
  }

  // Count number of grasps remaining
  std::size_t remaining_grasps = 0;
  std::size_t grasp_filtered_by_ik = 0;
  std::size_t grasp_filtered_by_cutting_plane = 0;
  std::size_t grasp_filtered_by_orientation = 0;
  std::size_t pregrasp_filtered_by_ik = 0;

  for (std::size_t i = 0; i < grasp_candidates.size(); ++i)
  {
    if (grasp_candidates[i]->grasp_filtered_by_ik_)
      grasp_filtered_by_ik++;
    else if (grasp_candidates[i]->grasp_filtered_by_cutting_plane_)
      grasp_filtered_by_cutting_plane++;
    else if (grasp_candidates[i]->grasp_filtered_by_orientation_)
      grasp_filtered_by_orientation++;
    else if (grasp_candidates[i]->pregrasp_filtered_by_ik_)
      pregrasp_filtered_by_ik++;
    else
      remaining_grasps++;
  }

  if (remaining_grasps + grasp_filtered_by_ik + grasp_filtered_by_cutting_plane + grasp_filtered_by_orientation +
          pregrasp_filtered_by_ik !=
      grasp_candidates.size())
    ROS_ERROR_STREAM_NAMED("grasp_filter", "Logged filter reasons do not add up to total number of grasps. Internal "
                                           "error.");

  // End Benchmark time
  double duration = (ros::Time::now() - start_time).toSec();

  // Keep a running average of calculation time
  static double total_duration = 0;
  static std::size_t total_filter_calls = 0;
  total_duration += duration;
  total_filter_calls += 1;
  double average_duration = total_duration / total_filter_calls;

  if (statistics_verbose_)
  {
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "GRASP FILTER RESULTS " << std::endl;
    std::cout << "total candidate grasps          " << grasp_candidates.size() << std::endl;
    std::cout << "grasp_filtered_by_cutting_plane " << grasp_filtered_by_cutting_plane << std::endl;
    std::cout << "grasp_filtered_by_orientation   " << grasp_filtered_by_orientation << std::endl;
    std::cout << "grasp_filtered_by_ik            " << grasp_filtered_by_ik << std::endl;
    std::cout << "pregrasp_filtered_by_ik         " << pregrasp_filtered_by_ik << std::endl;
    std::cout << "remaining grasps                " << remaining_grasps << std::endl;
    std::cout << "time duration:                  " << duration << std::endl;
    std::cout << "average time duration:          " << average_duration << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
  }

  return remaining_grasps;
}

bool GraspFilter::processCandidateGrasp(IkThreadStructPtr& ik_thread_struct)
{
  ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Checking grasp #" << ik_thread_struct->grasp_id);

  // Helper pointer
  GraspCandidatePtr& grasp_candidate = ik_thread_struct->grasp_candidates_[ik_thread_struct->grasp_id];

  // Get pose
  ik_thread_struct->ik_pose_ = grasp_candidate->grasp_.grasp_pose;

  // Debug
  if (ik_thread_struct->verbose_ && false)
  {
    ik_thread_struct->ik_pose_.header.frame_id = ik_thread_struct->kin_solver_->getBaseFrame();
    visual_tools_->publishZArrow(ik_thread_struct->ik_pose_.pose, rviz_visual_tools::RED, rviz_visual_tools::REGULAR,
                                 0.1);
  }

  // Filter by cutting planes
  for (std::size_t i = 0; i < cutting_planes_.size(); i++)
  {
    if (filterGraspByPlane(grasp_candidate, cutting_planes_[i]->pose_, cutting_planes_[i]->plane_,
                           cutting_planes_[i]->direction_) == true)
    {
      grasp_candidate->grasp_filtered_by_cutting_plane_ = true;
      return false;
    }
  }

  // Filter by desired orientation
  for (std::size_t i = 0; i < desired_grasp_orientations_.size(); i++)
  {
    if (filterGraspByOrientation(grasp_candidate, desired_grasp_orientations_[i]->pose_,
                                 desired_grasp_orientations_[i]->max_angle_offset_) == true)
    {
      grasp_candidate->grasp_filtered_by_orientation_ = true;
      return false;
    }
  }

  moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
      &isGraspStateValid, ik_thread_struct->planning_scene_.get(), collision_verbose_ || ik_thread_struct->verbose_,
      collision_verbose_speed_, visual_tools_, _1, _2, _3);

  // Set gripper position (how open the fingers are) to the custom open position
  grasp_candidate->getGraspStateOpenEEOnly(ik_thread_struct->robot_state_);

  // Solve IK Problem for grasp posture
  if (!findIKSolution(grasp_candidate->grasp_ik_solution_, ik_thread_struct, grasp_candidate, constraint_fn))
  {
    ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Unable to find the-grasp IK solution");
    grasp_candidate->grasp_filtered_by_ik_ = true;
    return false;
  }

  // Copy solution to seed state so that next solution is faster
  ik_thread_struct->ik_seed_state_ = grasp_candidate->grasp_ik_solution_;

  // Check if IK solution for grasp pose is valid for fingers closed as well
  if (!checkFingersClosedIK(grasp_candidate->grasp_ik_solution_, ik_thread_struct, grasp_candidate, constraint_fn))
  {
    ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Unable to find the-grasp IK solution with CLOSED fingers");
    grasp_candidate->grasp_filtered_by_ik_closed_ = true;
    return false;
  }

  // Start pre-grasp section ----------------------------------------------------------
  if (ik_thread_struct->filter_pregrasp_)  // optionally check the pregrasp
  {
    // Convert to a pre-grasp
    const std::string& ee_parent_link_name = grasp_candidate->grasp_data_->ee_jmg_->getEndEffectorParentGroup().second;
    ik_thread_struct->ik_pose_ = GraspGenerator::getPreGraspPose(grasp_candidate->grasp_, ee_parent_link_name);

    // Set gripper position (how open the fingers are) to CLOSED
    // grasp_candidate->getGraspStateClosedEEOnly(ik_thread_struct->robot_state_);

    // Solve IK Problem for pregrasp
    if (!findIKSolution(grasp_candidate->pregrasp_ik_solution_, ik_thread_struct, grasp_candidate, constraint_fn))
    {
      ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Unable to find PRE-grasp IK solution");
      grasp_candidate->pregrasp_filtered_by_ik_ = true;
      return false;
    }
    else if (grasp_candidate->pregrasp_ik_solution_.empty())
    {
      ROS_ERROR_STREAM_NAMED("grasp_filter", "IK solution found but vector is empty??");
    }
  }
  else
  {
    ROS_WARN_STREAM_NAMED("grasp_filter", "Not filtering pregrasp!!");
  }

  return true;
}

bool GraspFilter::findIKSolution(std::vector<double>& ik_solution, IkThreadStructPtr& ik_thread_struct,
                                 GraspCandidatePtr& grasp_candidate,
                                 const moveit::core::GroupStateValidityCallbackFn& constraint_fn)
{
  // Transform current pose to frame of planning group
  Eigen::Affine3d eigen_pose;
  tf::poseMsgToEigen(ik_thread_struct->ik_pose_.pose, eigen_pose);
  eigen_pose = ik_thread_struct->link_transform_ * eigen_pose;
  tf::poseEigenToMsg(eigen_pose, ik_thread_struct->ik_pose_.pose);

  // Set callback function
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn;
  if (constraint_fn)
    ik_callback_fn = boost::bind(&ikCallbackFnAdapter, ik_thread_struct->robot_state_.get(),
                                 grasp_candidate->grasp_data_->arm_jmg_, constraint_fn, _1, _2, _3);

  // Test it with IK
  ik_thread_struct->kin_solver_->searchPositionIK(ik_thread_struct->ik_pose_.pose, ik_thread_struct->ik_seed_state_,
                                                  ik_thread_struct->timeout_, ik_solution, ik_callback_fn,
                                                  ik_thread_struct->error_code_);

  // Results
  if (ik_thread_struct->error_code_.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION)
  {
    // The grasp was valid but the pre-grasp was not
    ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "No IK solution");
    return false;
  }
  else if (ik_thread_struct->error_code_.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Timed Out.");
    return false;
  }
  else if (ik_thread_struct->error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR_STREAM_NAMED("grasp_filter",
                           "Unknown MoveItErrorCode from IK solver: " << ik_thread_struct->error_code_.val);
    return false;
  }

  return true;
}

bool GraspFilter::checkFingersClosedIK(std::vector<double>& ik_solution, IkThreadStructPtr& ik_thread_struct,
                                       GraspCandidatePtr& grasp_candidate,
                                       const moveit::core::GroupStateValidityCallbackFn& constraint_fn)
{
  // Set gripper position (how open the fingers are) to CLOSED
  grasp_candidate->getGraspStateClosedEEOnly(ik_thread_struct->robot_state_);

  // Set callback function
  if (!constraint_fn(ik_thread_struct->robot_state_.get(), grasp_candidate->grasp_data_->arm_jmg_, &ik_solution[0]))
  {
    ROS_WARN_STREAM_NAMED("grasp_filter", "Grasp filtered because in collision with fingers CLOSED");
    return false;
  }

  return true;
}

void GraspFilter::addCuttingPlane(Eigen::Affine3d pose, grasp_parallel_plane plane, int direction)
{
  cutting_planes_.push_back(CuttingPlanePtr(new CuttingPlane(pose, plane, direction)));
}

void GraspFilter::addDesiredGraspOrientation(Eigen::Affine3d pose, double max_angle_offset)
{
  desired_grasp_orientations_.push_back(
      DesiredGraspOrientationPtr(new DesiredGraspOrientation(pose, max_angle_offset)));
}

bool GraspFilter::removeInvalidAndFilter(std::vector<GraspCandidatePtr>& grasp_candidates)
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
  ROS_INFO_STREAM_NAMED("grasp_filter", "Removed " << original_num_grasps - grasp_candidates.size()
                                                   << " invalid grasp candidates, " << grasp_candidates.size()
                                                   << " remaining");

  // Error Check
  if (grasp_candidates.empty())
  {
    ROS_ERROR_STREAM_NAMED("grasp_filter", "No remaining candidates grasps");
    return false;
  }

  // Order remaining valid grasps by best score
  std::sort(grasp_candidates.begin(), grasp_candidates.end(), compareGraspScores);

  ROS_INFO_STREAM_NAMED("grasp_filter", "Sorted valid grasps, highest quality is "
                                            << grasp_candidates.front()->grasp_.grasp_quality
                                            << " and lowest quality is "
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
    GREEN - valid
  */
  std::cout << "HERE " << std::endl;
  for (std::size_t i = 0; i < grasp_candidates.size(); ++i)
  {
    double size = 0.1;  // 0.01 * grasp_candidates[i]->grasp_.grasp_quality;

    if (grasp_candidates[i]->grasp_filtered_by_ik_)
    {
      visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose.pose, rviz_visual_tools::RED,
                                   rviz_visual_tools::REGULAR, size);
    }
    else if (grasp_candidates[i]->pregrasp_filtered_by_ik_)
    {
      visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose.pose, rviz_visual_tools::BLUE,
                                   rviz_visual_tools::REGULAR, size);
    }
    else if (grasp_candidates[i]->grasp_filtered_by_cutting_plane_)
    {
      visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose.pose, rviz_visual_tools::MAGENTA,
                                   rviz_visual_tools::REGULAR, size);
    }
    else if (grasp_candidates[i]->grasp_filtered_by_orientation_)
    {
      visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose.pose, rviz_visual_tools::YELLOW,
                                   rviz_visual_tools::REGULAR, size);
    }
    else
      visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose.pose, rviz_visual_tools::GREEN,
                                   rviz_visual_tools::REGULAR, size);
  }

  // Publish in batch
  visual_tools_->trigger();
  ros::Duration(4).sleep();

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
    ROS_ERROR_STREAM_NAMED("grasp_filter", "Unable to visualize IK solutions because there are no valid ones");
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
    grasp_candidates[i]->getPreGraspState(robot_state_);

    // Show in Rviz
    visual_tools_->publishRobotState(robot_state_);
    ros::Duration(show_filtered_arm_solutions_pregrasp_speed_).sleep();

    // Apply the grasp state
    grasp_candidates[i]->getGraspStateClosed(robot_state_);

    // Show in Rviz
    visual_tools_->publishRobotState(robot_state_);
    ros::Duration(show_filtered_arm_solutions_speed_).sleep();
  }

  return true;
}

bool GraspFilter::visualizeCuttingPlanes()
{
  if (show_cutting_planes_)
  {
    for (std::size_t i = 0; i < cutting_planes_.size(); i++)
    {
      switch (cutting_planes_[i]->plane_)
      {
        case XY:
          visual_tools_->publishXYPlane(cutting_planes_[i]->pose_);
          break;
        case XZ:
          visual_tools_->publishXZPlane(cutting_planes_[i]->pose_);
          break;
        case YZ:
          visual_tools_->publishYZPlane(cutting_planes_[i]->pose_);
          break;
        default:
          ROS_ERROR_STREAM_NAMED("grasp_filter", "Unknown cutting plane type");
      }
    }
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

bool GraspFilter::addCuttingPlanesForBin(const Eigen::Affine3d& world_to_bin, const Eigen::Affine3d& bin_to_product,
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
  Eigen::Affine3d world_to_bin_top = world_to_bin;
  world_to_bin_top.translation() += Eigen::Vector3d(0, bin_width, bin_height);
  addCuttingPlane(world_to_bin_top, XY, 1);

  // Left wall of bin
  addCuttingPlane(world_to_bin_top, XZ, 1);

  // // Back half of product
  Eigen::Affine3d world_to_bin_back = world_to_bin;
  world_to_bin_back.translation() +=
      Eigen::Vector3d(bin_to_product.translation().x(), bin_width / 2.0, bin_height / 2.0);
  addCuttingPlane(world_to_bin_back, YZ, 1);

  return true;
}

}  // namespace
