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
   Desc:   Find the approach, lift, and retreat path for a candidate grasp (if a valid one exists)
*/

// moveit_grasps
#include <moveit_grasps/grasp_planner.h>
#include <moveit_grasps/state_validity_callback.h>
#include <moveit/robot_state/cartesian_interpolator.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_grasps
{
constexpr char ENABLED_PARENT_NAME[] = "grasp_planner";  // for namespacing logging messages
constexpr char ENABLED_SETTINGS_NAMESPACE[] = "moveit_grasps/planner";

GraspPlanner::GraspPlanner(const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools)
  : nh_("~"), visual_tools_(visual_tools)
{
  loadEnabledSettings();
}

bool GraspPlanner::planAllApproachLiftRetreat(
    std::vector<GraspCandidatePtr>& grasp_candidates, const robot_state::RobotStatePtr& robot_state,
    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& grasp_object_id)
{
  boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
  ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor));
  return planAllApproachLiftRetreat(grasp_candidates, robot_state,
                                    static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls), grasp_object_id);
}

bool GraspPlanner::planAllApproachLiftRetreat(std::vector<GraspCandidatePtr>& grasp_candidates,
                                              const robot_state::RobotStatePtr& robot_state,
                                              const planning_scene::PlanningSceneConstPtr& planning_scene,
                                              const std::string& grasp_object_id)
{
  ROS_INFO_STREAM_NAMED("grasp_planner", "Planning all remaining grasps with approach lift retreat cartesian path");

  // For each remaining grasp, calculate entire approach, lift, and retreat path.
  // Remove those that have no valid path
  bool verbose_cartesian_filtering = isEnabled("verbose_cartesian_filtering");
  std::size_t grasp_candidates_before_cartesian_path = grasp_candidates.size();

  std::size_t count = 0;
  for (std::vector<GraspCandidatePtr>::iterator grasp_it = grasp_candidates.begin(); grasp_it != grasp_candidates.end();)
  {
    if (!ros::ok())
      return false;

    if (isEnabled("verbose_cartesian_filtering"))
    {
      ROS_INFO_STREAM_NAMED("grasp_planner", "");
      ROS_INFO_STREAM_NAMED("grasp_planner", "Attempting to plan cartesian grasp path #"
                                                 << count++ << ". " << grasp_candidates.size() << " remaining.");
    }

    if (!planApproachLiftRetreat(*grasp_it, robot_state, planning_scene, verbose_cartesian_filtering, grasp_object_id))
    {
      ROS_INFO_STREAM_NAMED("grasp_planner", "Grasp candidate was unable to find valid cartesian waypoint path");

      grasp_it = grasp_candidates.erase(grasp_it);  // not valid
    }
    else
    {
      ++grasp_it;  // move to next grasp
    }

    if (isEnabled("show_cartesian_waypoints"))
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->trigger();
    }
  }

  // Results
  if (isEnabled("statistics_verbose"))
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "Total grasp candidates: " << grasp_candidates_before_cartesian_path << std::endl;
    std::cout << "Failed due to invalid cartesian path: "
              << grasp_candidates_before_cartesian_path - grasp_candidates.size() << std::endl;
    std::cout << "Remaining grasp candidates: " << grasp_candidates.size() << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;
  }

  // If no grasp candidates had valid paths, then we return false
  if (grasp_candidates.size() == 0)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_planner", "No valid grasp plan possible");
    return false;
  }
  return true;
}

bool GraspPlanner::planApproachLiftRetreat(GraspCandidatePtr& grasp_candidate,
                                           const robot_state::RobotStatePtr& robot_state,
                                           const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                           bool verbose_cartesian_filtering, const std::string& grasp_object_id)
{
  boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
  ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor));
  return planApproachLiftRetreat(grasp_candidate, robot_state,
                                 static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls),
                                 verbose_cartesian_filtering, grasp_object_id);
}

bool GraspPlanner::planApproachLiftRetreat(GraspCandidatePtr& grasp_candidate,
                                           const robot_state::RobotStatePtr& robot_state,
                                           const planning_scene::PlanningSceneConstPtr& planning_scene,
                                           bool verbose_cartesian_filtering, const std::string& grasp_object_id)
{
  EigenSTL::vector_Isometry3d waypoints;
  GraspGenerator::getGraspWaypoints(grasp_candidate, waypoints);

  // Visualize waypoints
  bool show_cartesian_waypoints = isEnabled("show_cartesian_waypoints");
  if (show_cartesian_waypoints)
  {
    visual_tools_->publishAxisLabeled(waypoints[0], "pregrasp");
    visual_tools_->publishAxisLabeled(waypoints[1], "grasp");
    visual_tools_->publishAxisLabeled(waypoints[2], "lifted");
    visual_tools_->publishAxisLabeled(waypoints[3], "retreat");
    visual_tools_->trigger();

    if (grasp_candidate->getPreGraspState(visual_tools_->getSharedRobotState()))
    {
      waitForNextStep("see pre grasp state");
      visual_tools_->publishRobotState(visual_tools_->getSharedRobotState(), rviz_visual_tools::RED);
      visual_tools_->trigger();
    }
    if (grasp_candidate->getGraspStateClosed(visual_tools_->getSharedRobotState()))
    {
      waitForNextStep("see grasp state");
      visual_tools_->publishRobotState(visual_tools_->getSharedRobotState(), rviz_visual_tools::BLUE);
      visual_tools_->trigger();
    }
    waitForNextStep("continue cartesian planning");
  }

  // Remove the pre-grasp point since we plan from that start state
  waypoints.erase(waypoints.begin());

  // Starting state
  moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(*robot_state));
  if (!grasp_candidate->getPreGraspState(start_state))
  {
    ROS_WARN_STREAM_NAMED("grasp_planner.waypoints", "Unable to set pregrasp");
    return false;
  }

  if (!computeCartesianWaypointPath(grasp_candidate, planning_scene, start_state, waypoints, grasp_object_id))
  {
    ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints", "Unable to plan approach lift retreat path");

    return false;
  }

  // Feedback
  ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints", "Found valid and complete waypoint manipulation path for grasp "
                                                    "candidate");

  // Show visuals
  if (show_cartesian_waypoints)
  {
    ROS_INFO_STREAM_NAMED("grasp_planner.waypoints", "Visualize end effector position of cartesian path for "
                                                         << grasp_candidate->segmented_cartesian_traj_.size()
                                                         << " segments");
    visual_tools_->publishTrajectoryPoints(grasp_candidate->segmented_cartesian_traj_[APPROACH],
                                           grasp_candidate->grasp_data_->parent_link_, rviz_visual_tools::YELLOW);
    visual_tools_->publishTrajectoryPoints(grasp_candidate->segmented_cartesian_traj_[LIFT],
                                           grasp_candidate->grasp_data_->parent_link_, rviz_visual_tools::ORANGE);
    visual_tools_->publishTrajectoryPoints(grasp_candidate->segmented_cartesian_traj_[RETREAT],
                                           grasp_candidate->grasp_data_->parent_link_, rviz_visual_tools::RED);
    visual_tools_->trigger();

    bool wait_for_animation = true;
    visual_tools_->publishTrajectoryPath(grasp_candidate->segmented_cartesian_traj_[APPROACH],
                                         grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
    visual_tools_->publishTrajectoryPath(grasp_candidate->segmented_cartesian_traj_[LIFT],
                                         grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
    visual_tools_->publishTrajectoryPath(grasp_candidate->segmented_cartesian_traj_[RETREAT],
                                         grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
  }

  if (verbose_cartesian_filtering)
    waitForNextStep("try next candidate grasp");

  return true;
}

bool GraspPlanner::computeCartesianWaypointPath(
    GraspCandidatePtr& grasp_candidate, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
    const moveit::core::RobotStatePtr& start_state, const EigenSTL::vector_Isometry3d& waypoints,
    const std::string& grasp_object_id)
{
  boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
  ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor));

  return computeCartesianWaypointPath(grasp_candidate, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls),
                                      start_state, waypoints, grasp_object_id);
}

bool GraspPlanner::computeCartesianWaypointPath(GraspCandidatePtr& grasp_candidate,
                                                const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                const moveit::core::RobotStatePtr& start_state,
                                                const EigenSTL::vector_Isometry3d& waypoints,
                                                const std::string& grasp_object_id)
{
  // End effector parent link (arm tip for ik solving)
  const moveit::core::LinkModel* ik_tip_link = grasp_candidate->grasp_data_->parent_link_;

  // Resolution of trajectory
  // The maximum distance in Cartesian space between consecutive points on the resulting path
  const double max_step = 0.01;

  // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in joint space
  const double jump_threshold = 4;  // config_->jump_threshold_; // aka jump factor

  // Collision setting
  const bool collision_checking_verbose = isEnabled("collision_checking_verbose");
  const bool only_check_self_collision = false;

  // Reference frame setting
  const bool global_reference_frame = true;

  // Check for kinematic solver
  if (!grasp_candidate->grasp_data_->arm_jmg_->canSetStateFromIK(ik_tip_link->getName()))
  {
    ROS_ERROR_STREAM_NAMED("grasp_planner.waypoints", "No IK Solver loaded - make sure moveit_config/kinamatics.yaml "
                                                      "is loaded in this namespace");
    return false;
  }

  std::size_t attempts = 0;
  static const std::size_t MAX_IK_ATTEMPTS = 5;
  bool valid_path_found = false;
  while (attempts < MAX_IK_ATTEMPTS)
  {
    if (attempts > 0)
    {
      ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints", "Attempting IK solution, attempt # " << attempts + 1);
    }
    attempts++;

    moveit::core::RobotStatePtr start_state_copy(new moveit::core::RobotState(*start_state));
    if (!grasp_candidate->getPreGraspState(start_state_copy))
    {
      ROS_ERROR_STREAM_NAMED("grasp_planner.waypoints", "Unable to set pregrasp");
      return false;
    }

    // Create a clone where we modify the ACM to disable collision checking between the end effector and object
    planning_scene::PlanningScenePtr scene = planning_scene::PlanningScene::clone(planning_scene);
    // If the grasp_object_id is set then we disable collision checking in the planning scene clone
    if (!grasp_object_id.empty() && scene->knowsFrameTransform(grasp_object_id))
    {
      std::vector<std::string> ee_link_names = grasp_candidate->grasp_data_->ee_jmg_->getLinkModelNames();
      for (const auto& ee_link : ee_link_names)
        scene->getAllowedCollisionMatrixNonConst().setEntry(grasp_object_id, ee_link, true);
    }
    // Collision check
    moveit::core::GroupStateValidityCallbackFn constraint_fn =
        boost::bind(&isGraspStateValid, scene.get(), collision_checking_verbose, only_check_self_collision,
                    visual_tools_, _1, _2, _3);

    // Compute Cartesian Path
    grasp_candidate->segmented_cartesian_traj_.clear();
    grasp_candidate->segmented_cartesian_traj_.resize(3);
    double valid_approach_percentage = robot_state::CartesianInterpolator::computeCartesianPath(
        start_state_copy.get(), grasp_candidate->grasp_data_->arm_jmg_,
        grasp_candidate->segmented_cartesian_traj_[APPROACH], ik_tip_link, waypoints[APPROACH], global_reference_frame,
        robot_state::MaxEEFStep(max_step), robot_state::JumpThreshold(jump_threshold), constraint_fn,
        kinematics::KinematicsQueryOptions());

    if (!grasp_candidate->getGraspStateClosedEEOnly(start_state_copy))
    {
      ROS_ERROR_STREAM_NAMED("grasp_planner", "Unable to set pregrasp");
      return false;
    }

    // Attach CO to end effector
    if (!grasp_object_id.empty() && scene->knowsFrameTransform(grasp_object_id))
    {
      // Create an AttachedCollisionObject
      moveit_msgs::AttachedCollisionObject aco;

      // Create a reference to the collision object for convenience
      moveit_msgs::CollisionObject& suction_voxel_co = aco.object;

      suction_voxel_co.id = grasp_object_id;
      suction_voxel_co.header.frame_id = ik_tip_link->getName();

      // Set the aco attached link name
      aco.link_name = ik_tip_link->getName();

      // move the robot in the ps to the end of the approach path.
      scene->setCurrentState(*start_state_copy);

      // Mark object to be added
      suction_voxel_co.operation = moveit_msgs::CollisionObject::ADD;
      if (!scene->processAttachedCollisionObjectMsg(aco))
      {
        ROS_WARN_STREAM_NAMED("grasp_planner", "Failed to attach: " << aco.object.id);
      }
      else
      {
        auto start_state_with_body = std::make_shared<robot_state::RobotState>(scene->getCurrentState());
        std::vector<const robot_state::AttachedBody*> attached_bodies;
        start_state_with_body->getAttachedBodies(attached_bodies);
        for (const auto& ab : attached_bodies)
          start_state_copy->attachBody(ab->getName(), ab->getPose(), ab->getShapes(), ab->getShapePoses(),
                                       ab->getTouchLinks(), ab->getAttachedLinkName(), ab->getDetachPosture(),
                                       ab->getSubframes());
        constraint_fn = boost::bind(&isGraspStateValid, scene.get(), collision_checking_verbose,
                                    only_check_self_collision, visual_tools_, _1, _2, _3);
      }
    }

    double valid_lift_retreat_percentage = robot_state::CartesianInterpolator::computeCartesianPath(
        start_state_copy.get(), grasp_candidate->grasp_data_->arm_jmg_,
        grasp_candidate->segmented_cartesian_traj_[LIFT], ik_tip_link, waypoints[LIFT], global_reference_frame,
        robot_state::MaxEEFStep(max_step), robot_state::JumpThreshold(jump_threshold), constraint_fn,
        kinematics::KinematicsQueryOptions());

    valid_lift_retreat_percentage *= robot_state::CartesianInterpolator::computeCartesianPath(
        start_state_copy.get(), grasp_candidate->grasp_data_->arm_jmg_,
        grasp_candidate->segmented_cartesian_traj_[RETREAT], ik_tip_link, waypoints[RETREAT], global_reference_frame,
        robot_state::MaxEEFStep(max_step), robot_state::JumpThreshold(jump_threshold), constraint_fn,
        kinematics::KinematicsQueryOptions());

    ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints", "valid_approach_percentage: " << valid_approach_percentage
                                                                                    << " \tvalid_lift_retreat_"
                                                                                       "percentage: "
                                                                                    << valid_lift_retreat_percentage);

    // The retreat has to work for the most part but doesn't need to be perfect
    double min_allowed_valid_lift_retreat_percentage = 0.90;
    if (valid_approach_percentage == 1 && valid_lift_retreat_percentage >= min_allowed_valid_lift_retreat_percentage)
    {
      ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints", "Found valid cartesian path");
      valid_path_found = true;
      break;
    }
  }  // end while AND scoped pointer of locked planning scenep

  if (!valid_path_found)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints",
                           "UNABLE to find valid waypoint cartesian path after " << MAX_IK_ATTEMPTS << " attempts");
    return false;
  }

  return true;
}

void GraspPlanner::waitForNextStep(const std::string& message)
{
  if (wait_for_next_step_callback_)
    wait_for_next_step_callback_(message);
}

void GraspPlanner::setWaitForNextStepCallback(WaitForNextStepCallback callback)
{
  wait_for_next_step_callback_ = callback;
}

bool GraspPlanner::loadEnabledSettings()
{
  // Check if the map has been loaded yet
  if (!enabled_settings_loaded_)
  {
    enabled_settings_loaded_ = true;
    return rosparam_shortcuts::get(ENABLED_PARENT_NAME, nh_, ENABLED_SETTINGS_NAMESPACE, enabled_setting_);
  }
  return true;
}

bool GraspPlanner::isEnabled(const std::string& setting_name)
{
  // Check if the map has been loaded yet. it is preferred if this is called manually
  if (!enabled_settings_loaded_)
    ROS_ERROR_STREAM_NAMED("rosparam_shortcuts", "Enabled settings are not yet loaded e.g. call loadEnabledSettings()");

  std::map<std::string, bool>::iterator it = enabled_setting_.find(setting_name);
  if (it != enabled_setting_.end())
  {
    // Element found;
    return it->second;
  }
  ROS_ERROR_STREAM_NAMED("rosparam_shortcuts", "isEnabled() key '" << nh_.getNamespace() << "/"
                                                                   << ENABLED_SETTINGS_NAMESPACE << "/" << setting_name
                                                                   << "' does not exist on the parameter server");

  return false;
}

}  // namespace moveit_grasps
