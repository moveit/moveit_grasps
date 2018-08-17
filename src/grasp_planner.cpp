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
   Desc:   Find the approach, lift, and retreat path for a candidate grasp (if a valid one exists)
*/

// moveit_grasps
#include <moveit_grasps/grasp_planner.h>
#include <moveit_grasps/state_validity_callback.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_grasps
{
GraspPlanner::GraspPlanner(moveit_visual_tools::MoveItVisualToolsPtr& visual_tools)
  : visual_tools_(visual_tools), nh_("~")
{
  // Load verbose/visualization settings
  const std::string parent_name = "grasp_planner";  // for namespacing logging messages
  const std::string settings_namespace = "moveit_grasps/planner";
  loadEnabledSettings(parent_name, settings_namespace);

  ROS_INFO_STREAM_NAMED("grasp_planner", "GraspPlanner Ready.");
}

bool GraspPlanner::planAllApproachLiftRetreat(std::vector<GraspCandidatePtr>& grasp_candidates,
                                              robot_state::RobotStatePtr current_state,
                                              planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                              const GraspDataPtr grasp_data, const double& bin_height,
                                              Eigen::Affine3d bin_to_object)
{
  std::cout << std::endl;
  ROS_INFO_STREAM_NAMED("grasp_planner", "Planning all remaining grasps with approach lift retreat cartesian path");

  // For each remaining grasp, calculate entire approach, lift, and retreat path.
  // Remove those that have no valid path
  bool verbose_cartesian_filtering = isEnabled("verbose_cartesian_filtering");
  std::size_t grasp_candidates_before_cartesian_path = grasp_candidates.size();

  std::size_t count = 0;
  for (std::vector<GraspCandidatePtr>::iterator grasp_it = grasp_candidates.begin();
       grasp_it != grasp_candidates.end();)
  {
    if (!ros::ok())
      return false;

    ROS_INFO_STREAM_NAMED("grasp_planner", "");
    ROS_INFO_STREAM_NAMED("grasp_planner", "Attempting to plan cartesian grasp path #" << count++);

    if (!planApproachLiftRetreat(*grasp_it, current_state, planning_scene_monitor, grasp_data,
                                 verbose_cartesian_filtering, bin_height, bin_to_object))
    {
      ROS_INFO_STREAM_NAMED("grasp_planner", "Grasp candidate was unable to find valid cartesian waypoint path");

      grasp_it = grasp_candidates.erase(grasp_it);  // not valid
    }
    else
    {
      //++grasp_it; // move to next grasp

      // Once we have one valid path, just quit so we can use that one
      ROS_INFO_STREAM_NAMED("grasp_planner", "Valid grasp plan generated");
      break;
    }

    if (verbose_cartesian_filtering)
    {
      visual_tools_->deleteAllMarkers();
    }
  }

  // Results
  if (verbose_cartesian_filtering)
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

  return true;
}

bool GraspPlanner::planApproachLiftRetreat(GraspCandidatePtr grasp_candidate, robot_state::RobotStatePtr current_state,
                                           planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                           const GraspDataPtr grasp_data, bool verbose_cartesian_filtering,
                                           const double& bin_height, Eigen::Affine3d bin_to_object)
{
  // Get settings from grasp generator
  const geometry_msgs::PoseStamped& grasp_pose_msg = grasp_candidate->grasp_.grasp_pose;
  const geometry_msgs::PoseStamped pregrasp_pose_msg =
      GraspGenerator::getPreGraspPose(grasp_candidate->grasp_, grasp_candidate->grasp_data_->parent_link_->getName());

  // Calculate the lift distance to center the object's centroid vertically in the bin
  double lift_distance = bin_height / 2.0 - bin_to_object.translation().z();

  // Check if lift distance is too little
  if (lift_distance < grasp_candidate->grasp_data_->lift_distance_desired_)
  {
    ROS_WARN_STREAM_NAMED("grasp_planner", "Lift distance " << lift_distance << " less than minimum allowed of "
                                                            << grasp_candidate->grasp_data_->lift_distance_desired_);

    // std::cout << "bin_height / 2.0 " << bin_height / 2.0
    //          << " bin_to_object.translation().z() " <<  bin_to_object.translation().z() << std::endl;
    // visuals_->visual_tools_->publishAxisLabeled(bin->getCentroid(), "bin");
    // visuals_->visual_tools_->publishAxisLabeled(bin_to_object, "object");

    lift_distance = grasp_candidate->grasp_data_->lift_distance_desired_;
  }
  ROS_DEBUG_STREAM_NAMED("grasp_planner.lift_distance", "Lift distance calculated to be " << lift_distance);

  // Create waypoints
  Eigen::Affine3d pregrasp_pose = visual_tools_->convertPose(pregrasp_pose_msg.pose);
  Eigen::Affine3d grasp_pose = visual_tools_->convertPose(grasp_pose_msg.pose);
  Eigen::Affine3d lifted_grasp_pose = grasp_pose;
  lifted_grasp_pose.translation().z() += lift_distance;

  // Error checking for lift distance
  // if ( lifted_grasp_pose.translation().z() > bin->getTopLeft().translation().z())
  // {
  //   ROS_ERROR_STREAM_NAMED("grasp_planner","Max lift distance reached, requested " << lift_distance <<
  //                          " which has a z height of " << lifted_grasp_pose.translation().z()
  //                          << " of " << bin->getTopLeft().translation().z());
  //   return false;
  // }

  // METHOD 1 - retreat in same direction as approach
  // Eigen::Affine3d lifted_pregrasp_pose = pregrasp_pose;
  // lifted_pregrasp_pose.translation().z() += grasp_candidate->grasp_data_->lift_distance_desired_;

  // METHOD 2
  Eigen::Affine3d retreat_pose = lifted_grasp_pose;
  retreat_pose.translation().x() -= grasp_candidate->grasp_data_->retreat_distance_desired_;

  EigenSTL::vector_Affine3d waypoints;
  // waypoints.push_back(pregrasp_pose); // this is included in the robot_state being used to calculate cartesian path
  waypoints.push_back(grasp_pose);
  waypoints.push_back(lifted_grasp_pose);
  // waypoints.push_back(lifted_pregrasp_pose);
  waypoints.push_back(retreat_pose);

  // Visualize waypoints
  bool show_cartesian_waypoints = isEnabled("show_cartesian_waypoints");
  if (show_cartesian_waypoints)
  {
    // bool static_id = false;
    // visual_tools_->publishZArrow(pregrasp_pose, rvt::GREEN, rvt::SMALL);
    visual_tools_->publishAxisLabeled(pregrasp_pose, "pregrasp");

    // visual_tools_->publishZArrow(grasp_pose, rvt::YELLOW, rvt::SMALL);
    visual_tools_->publishAxisLabeled(grasp_pose, "grasp");

    // visual_tools_->publishZArrow(lifted_grasp_pose, rviz_visual_tools::ORANGE, rviz_visual_tools::SMALL);
    visual_tools_->publishAxisLabeled(lifted_grasp_pose, "lifted");

    // visual_tools_->publishZArrow(retreat_pose, rviz_visual_tools::RED, rviz_visual_tools::SMALL);
    visual_tools_->publishAxisLabeled(retreat_pose, "retreat");

    // Show the grasp state
    grasp_candidate->getGraspStateOpen(visual_tools_->getSharedRobotState());

    visual_tools_->publishRobotState(visual_tools_->getSharedRobotState(), rviz_visual_tools::TRANSLUCENT);

    waitForNextStep("see closed grasp state");
    grasp_candidate->getGraspStateClosed(visual_tools_->getSharedRobotState());
    visual_tools_->publishRobotState(visual_tools_->getSharedRobotState(), rviz_visual_tools::TRANSLUCENT);

    waitForNextStep("see pre grasp state");
    grasp_candidate->getPreGraspState(visual_tools_->getSharedRobotState());
    visual_tools_->publishRobotState(visual_tools_->getSharedRobotState(), rviz_visual_tools::TRANSLUCENT);

    waitForNextStep("continue cartesian planning");
  }

  // Starting state
  moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(*current_state));
  if (!grasp_candidate->getPreGraspState(start_state))
  {
    ROS_ERROR_STREAM_NAMED("grasp_planner.waypoints", "Unable to set pregrasp");
    return false;
  }

  GraspTrajectories segmented_cartesian_traj;
  if (!computeCartesianWaypointPath(grasp_candidate->grasp_data_->arm_jmg_, grasp_data, planning_scene_monitor,
                                    start_state, waypoints, segmented_cartesian_traj))
  {
    ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints", "Unable to plan approach lift retreat path");

    waitForNextStep("try next candidate grasp");

    return false;
  }

  // Feedback
  ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints", "Found valid and complete waypoint manipulation path for grasp "
                                                    "candidate");

  // Get arm planning group
  const moveit::core::JointModelGroup* arm_jmg = grasp_candidate->grasp_data_->arm_jmg_;

  // Show visuals
  if (verbose_cartesian_filtering)
  {
    ROS_INFO_STREAM_NAMED("grasp_planner.waypoints", "Visualize end effector position of cartesian path for "
                                                         << segmented_cartesian_traj.size() << " segments");
    visual_tools_->publishTrajectoryPoints(segmented_cartesian_traj[APPROACH], grasp_data->parent_link_,
                                           rviz_visual_tools::YELLOW);
    visual_tools_->publishTrajectoryPoints(segmented_cartesian_traj[LIFT], grasp_data->parent_link_,
                                           rviz_visual_tools::ORANGE);
    visual_tools_->publishTrajectoryPoints(segmented_cartesian_traj[RETREAT], grasp_data->parent_link_,
                                           rviz_visual_tools::RED);
  }
  // Turn off auto mode
  // remote_control_->setStop();

  // Save this result
  grasp_candidate->segmented_cartesian_traj_ = segmented_cartesian_traj;

  if (verbose_cartesian_filtering)
    waitForNextStep("try next candidate grasp");

  return true;
}

bool GraspPlanner::computeCartesianWaypointPath(const moveit::core::JointModelGroup* arm_jmg,
                                                const GraspDataPtr grasp_data,
                                                planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                                const moveit::core::RobotStatePtr start_state,
                                                const EigenSTL::vector_Affine3d& waypoints,
                                                GraspTrajectories& segmented_cartesian_traj)
{
  // End effector parent link (arm tip for ik solving)
  const moveit::core::LinkModel* ik_tip_link = grasp_data->parent_link_;

  // Resolution of trajectory
  const double max_step =
      0.01;  // The maximum distance in Cartesian space between consecutive points on the resulting path

  // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in joint space
  const double jump_threshold = 4;  // config_->jump_threshold_; // aka jump factor

  // Collision setting
  const bool collision_checking_verbose = isEnabled("collision_checking_verbose");
  const bool only_check_self_collision = false;

  // Reference frame setting
  const bool global_reference_frame = true;

  // Check for kinematic solver
  if (!arm_jmg->canSetStateFromIK(ik_tip_link->getName()))
  {
    ROS_ERROR_STREAM_NAMED("grasp_planner.waypoints", "No IK Solver loaded - make sure moveit_config/kinamatics.yaml "
                                                      "is loaded in this namespace");
    return false;
  }

  // Results
  double last_valid_percentage;

  std::size_t attempts = 0;
  static const std::size_t MAX_IK_ATTEMPTS = 5;
  bool valid_path_found = false;
  while (attempts < MAX_IK_ATTEMPTS)
  {
    if (attempts > 0)
    {
      // std::cout << std::endl;
      // std::cout << "-------------------------------------------------------" << std::endl;
      ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints", "Attempting IK solution, attempt # " << attempts + 1);
    }
    attempts++;

    // Collision check
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor));
    moveit::core::GroupStateValidityCallbackFn constraint_fn =
        boost::bind(&isGraspStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
                    collision_checking_verbose, only_check_self_collision, visual_tools_, _1, _2, _3);

    // Test
    moveit::core::RobotState start_state_copy(*start_state);

    // Compute Cartesian Path
    segmented_cartesian_traj.clear();
    ROS_ERROR_STREAM_NAMED("grasp_planner.waypoints", "computeCartesianPathSegmented was never merged into main "
                                                      "moveit");
    // last_valid_percentage = start_state_copy.computeCartesianPathSegmented(arm_jmg, segmented_cartesian_traj,
    //                                                                        ik_tip_link,
    //                                                                        waypoints, global_reference_frame,
    //                                                                        max_step, jump_threshold, constraint_fn,
    //                                                                        kinematics::KinematicsQueryOptions());

    ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints",
                           "Cartesian last_valid_percentage: " << last_valid_percentage
                                                               << " number of segments in trajectory: "
                                                               << segmented_cartesian_traj.size());

    double min_allowed_valid_percentage = 0.9;
    if (last_valid_percentage == 0)
    {
      ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints", "Failed to computer cartesian path: last_valid_percentage is "
                                                        "0");
    }
    else if (last_valid_percentage < min_allowed_valid_percentage)
    {
      ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints",
                             "Resulting cartesian path is less than "
                                 << min_allowed_valid_percentage
                                 << " % of the desired distance, % valid: " << last_valid_percentage);
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints", "Found valid cartesian path");
      valid_path_found = true;
      break;
    }
  }  // end while AND scoped pointer of locked planning scenep

  if (!valid_path_found)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_planner.waypoints", "UNABLE to find valid waypoint cartesian path after "
                                                          << MAX_IK_ATTEMPTS << " attempts");
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

bool GraspPlanner::loadEnabledSettings(const std::string& parent_name, const std::string& setting_namespace)
{
  // Check if the map has been loaded yet
  if (!enabled_setttings_loaded_)
  {
    enabled_setttings_loaded_ = true;
    return rosparam_shortcuts::get(parent_name, nh_, setting_namespace, enabled_);
  }
  return true;
}

bool GraspPlanner::isEnabled(const std::string& setting_name)
{
  // Check if the map has been loaded yet. it is preferred if this is manually
  if (!enabled_setttings_loaded_)
    ROS_ERROR_STREAM_NAMED("rosparam_shortcuts", "Enabled settings are not yet loaded e.g. call loadEnabledSettings()");

  std::map<std::string, bool>::iterator it = enabled_.find(setting_name);
  if (it != enabled_.end())
  {
    // Element found;
    return it->second;
  }
  ROS_ERROR_STREAM_NAMED("rosparam_shortcuts", "isEnabled() key '" << setting_name << "' does not exist on the "
                                                                                      "parameter server");
  return false;
}

}  // end namespace
