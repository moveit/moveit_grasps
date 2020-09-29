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

#include <moveit_grasps/grasp_candidate.h>

namespace moveit_grasps
{
GraspCandidate::GraspCandidate(const moveit_msgs::Grasp& grasp, const GraspDataPtr& grasp_data,
                               const Eigen::Isometry3d& cuboid_pose)
  : grasp_(grasp)
  , grasp_data_(grasp_data)
  , cuboid_pose_(cuboid_pose)
  , grasp_filtered_code_(GraspFilterCode::NOT_FILTERED)
{
}

bool GraspCandidate::getPreGraspState(moveit::core::RobotStatePtr& robot_state)
{
  // Error check
  if (pregrasp_ik_solution_.empty())
  {
    ROS_ERROR_STREAM_NAMED("grasp_candidate", "No pregrasp ik solution available to set");
    return false;
  }

  // Apply IK solved arm joints to state
  robot_state->setJointGroupPositions(grasp_data_->arm_jmg_, pregrasp_ik_solution_);

  // Set end effector to correct configuration
  grasp_data_->setRobotState(robot_state, grasp_.pre_grasp_posture);

  return true;
}

bool GraspCandidate::getGraspStateOpen(moveit::core::RobotStatePtr& robot_state)
{
  // Error check
  if (grasp_ik_solution_.empty())
  {
    ROS_ERROR_STREAM_NAMED("grasp_candidate", "No grasp ik solution available to set");
    return false;
  }

  // Apply IK solved arm joints to state
  robot_state->setJointGroupPositions(grasp_data_->arm_jmg_, grasp_ik_solution_);

  // Set end effector to correct configuration
  return getGraspStateOpenEEOnly(robot_state);
}

bool GraspCandidate::getGraspStateOpenEEOnly(moveit::core::RobotStatePtr& robot_state)
{
  return grasp_data_->setRobotState(robot_state, grasp_.pre_grasp_posture);
}

bool GraspCandidate::getGraspStateClosed(moveit::core::RobotStatePtr& robot_state)
{
  if (grasp_ik_solution_.empty())
  {
    ROS_ERROR_STREAM_NAMED("grasp_candidate", "No grasp ik solution available to set");
    return false;
  }

  // Apply IK solved arm joints to state
  robot_state->setJointGroupPositions(grasp_data_->arm_jmg_, grasp_ik_solution_);

  // Set end effector to correct configuration
  return getGraspStateClosedEEOnly(robot_state);
}

bool GraspCandidate::getGraspStateClosedEEOnly(moveit::core::RobotStatePtr& robot_state)
{
  return grasp_data_->setRobotState(robot_state, grasp_.grasp_posture);
}

bool GraspCandidate::isValid()
{
  return grasp_filtered_code_ == GraspFilterCode::NOT_FILTERED;
}

}  // namespace moveit_grasps
