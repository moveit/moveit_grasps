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

#ifndef MOVEIT_GRASPS__GRASP_PLANNER_
#define MOVEIT_GRASPS__GRASP_PLANNER_

// ROS
#include <ros/ros.h>

// moveit_grasps
#include <moveit_grasps/grasp_candidate.h>
#include <moveit_grasps/grasp_generator.h>

// moveit
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit_grasps
{
// Allow an interrupt to be called that waits for user input, useful for debugging
typedef boost::function<void(std::string message)> WaitForNextStepCallback;

class GraspPlanner
{
public:
  /**
   * \brief Constructor
   */
  GraspPlanner(const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools);

  /**
   * \brief Plan entire cartesian manipulation sequence
   * \param grasp_candidates - GraspCandidates for which we will compute apprach, lift and retreat paths
   * \param robot_state - robot_state to be used for computeCartesianPath
   * \param planning_scene_monitor - Current state of the world
   * \param grasp_data - robot gripper configuration
   * \param grasp_object_id - The name of the target grasp object to be used in the planning scene.
   * \return true on success
   */
  bool planAllApproachLiftRetreat(std::vector<GraspCandidatePtr>& grasp_candidates,
                                  const robot_state::RobotStatePtr& robot_state,
                                  const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                  const std::string& grasp_object_id = "");

  bool planAllApproachLiftRetreat(std::vector<GraspCandidatePtr>& grasp_candidates,
                                  const robot_state::RobotStatePtr& robot_state,
                                  const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const std::string& grasp_object_id = "");

  /**
   * \brief Plan entire cartesian manipulation sequence
   * \param input - description
   * \return true on success
   */
  bool planApproachLiftRetreat(GraspCandidatePtr& grasp_candidate, const robot_state::RobotStatePtr& robot_state,
                               const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                               bool verbose_cartesian_filtering, const std::string& grasp_object_id = "");

  bool planApproachLiftRetreat(GraspCandidatePtr& grasp_candidate, const robot_state::RobotStatePtr& robot_state,
                               const planning_scene::PlanningSceneConstPtr& planning_scene,
                               bool verbose_cartesian_filtering, const std::string& grasp_object_id = "");

  /**
   * \brief Compute a cartesian path along waypoints
   * \return true on success
   */
  bool computeCartesianWaypointPath(GraspCandidatePtr& grasp_candidate,
                                    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                    const moveit::core::RobotStatePtr& start_state,
                                    const EigenSTL::vector_Isometry3d& waypoints,
                                    const std::string& grasp_object_id = "");

  bool computeCartesianWaypointPath(GraspCandidatePtr& grasp_candidate,
                                    const planning_scene::PlanningSceneConstPtr& planning_scene,
                                    const moveit::core::RobotStatePtr& start_state,
                                    const EigenSTL::vector_Isometry3d& waypoints,
                                    const std::string& grasp_object_id = "");

  /**
   * \brief Wait for user input to proceeed
   * \param message - text to display to user when waiting
   */
  void waitForNextStep(const std::string& message);

  /**
   * \brief Allow an interrupt to be called that waits for user input, useful for debugging
   * \param message - text to display to user when waiting
   */
  // TODO(davetcoleman): Ideally all the classes in moveit_grasps would leverage this approach
  void setWaitForNextStepCallback(WaitForNextStepCallback callback);

  /**
   * \brief Load parameter settings of the server under ~/debug_level
   * \param parent_name - only used for debugging, allows one to see what paremeters are loaded in what namespace
   * \param setting_namespace - where on this node's namespace to load settings
   *        e.g. /this_name/setting_namespace/some_parameter
   * \return true on success
   */
  bool loadEnabledSettings();

  /**
   * \brief Check if a setting is enabled
   * \param setting_name - name of key on the parameter server as loaded in the 'setting_namespace'
   * \return true if setting is enabled
   */
  bool isEnabled(const std::string& setting_name);

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  WaitForNextStepCallback wait_for_next_step_callback_;

  // Visualization settings
  bool enabled_settings_loaded_ = false;
  std::map<std::string, bool> enabled_setting_;

};  // end class

// Create smart pointers for this class
typedef std::shared_ptr<GraspPlanner> GraspPlannerPtr;
typedef std::shared_ptr<const GraspPlanner> GraspPlannerConstPtr;

}  // namespace moveit_grasps

#endif
