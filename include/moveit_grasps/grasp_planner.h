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

#ifndef MOVEIT_GRASPS__GRASP_PLANNER_
#define MOVEIT_GRASPS__GRASP_PLANNER_

// ROS
#include <ros/ros.h>

// moveit_grasps
#include <moveit_grasps/grasp_filter.h>

namespace moveit_grasps
{

class GraspPlanner
{
public:

  /**
   * \brief Constructor
   */
  GraspPlanner(moveit_visual_tools::MoveItVisualToolsPtr& visual_tools );

  /**
   * \brief Plan entire cartesian manipulation sequence
   * \param input - description
   * \return true on success
   */
  bool planAllApproachLiftRetreat(std::vector<GraspCandidatePtr> &grasp_candidates,
                               robot_state::RobotStatePtr current_state,
                               planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                               const GraspDataPtr grasp_data,
                                  const double& bin_height, Eigen::Affine3d bin_to_object);

  /**
   * \brief Plan entire cartesian manipulation sequence
   * \param input - description
   * \return true on success
   */
  bool planApproachLiftRetreat(GraspCandidatePtr grasp_candidate,
                               robot_state::RobotStatePtr current_state,
                               planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                               const GraspDataPtr grasp_data,
                               bool verbose_cartesian_filtering,
                               const double& bin_height, Eigen::Affine3d bin_to_object);

  /**
   * \brief Compute a cartesian path along waypoints
   * \return true on success
   */
  bool computeCartesianWaypointPath(const moveit::core::JointModelGroup* arm_jmg,
                                    const GraspDataPtr grasp_data,
                                    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                    const moveit::core::RobotStatePtr start_state,
                                    const EigenSTL::vector_Affine3d &waypoints,
                                    GraspTrajectories &segmented_cartesian_traj);

private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Class for publishing stuff to rviz

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<GraspPlanner> GraspPlannerPtr;
typedef boost::shared_ptr<const GraspPlanner> GraspPlannerConstPtr;

} // end namespace

#endif
