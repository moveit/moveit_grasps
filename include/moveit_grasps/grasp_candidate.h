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

#ifndef MOVEIT_GRASPS__GRASP_CANDIDATE_
#define MOVEIT_GRASPS__GRASP_CANDIDATE_

// ROS
#include <ros/ros.h>
#include <moveit_msgs/Grasp.h>

// Grasping
#include <moveit_grasps/grasp_data.h>
//#include <moveit_grasps/grasp_generator.h>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Grasp.h>

namespace moveit_grasps
{

/**
 * \brief Contains collected data for each potential grasp after it has been verified / filtered
 *        This includes the pregrasp and grasp IK solution
 */
class GraspCandidate
{
public:

  GraspCandidate(moveit_msgs::Grasp grasp, const GraspDataPtr grasp_data)
    : grasp_(grasp)
    , grasp_data_(grasp_data)
    , grasp_filtered_by_ik_(false)
    , grasp_filtered_by_cutting_plane_(false)
    , grasp_filtered_by_orientation_(false)
    , pregrasp_filtered_by_ik_(false)
  {}

  bool getPreGraspState(moveit::core::RobotStatePtr &robot_state)
  {
    // Error check
    if (pregrasp_ik_solution_.empty())
    {
      ROS_ERROR_STREAM_NAMED("grasp_candidate","No pregrasp ik solution available to set");
      return false;
    }

    // Apply IK solved arm joints to state
    robot_state->setJointGroupPositions(grasp_data_->arm_jmg_, pregrasp_ik_solution_);

    // Set end effector to correct configuration
    grasp_data_->setRobotStatePreGrasp(robot_state);

    return true;
  }

  bool getGraspStateOpen(moveit::core::RobotStatePtr robot_state)
  {
    // Error check
    if (grasp_ik_solution_.empty())
    {
      ROS_ERROR_STREAM_NAMED("grasp_candidate","No grasp ik solution available to set");
      return false;
    }

    // Apply IK solved arm joints to state
    robot_state->setJointGroupPositions(grasp_data_->arm_jmg_, grasp_ik_solution_);

    // Set end effector to correct configuration
    grasp_data_->setRobotStatePreGrasp(robot_state);

    return true;
  }

  bool getGraspStateClosed(moveit::core::RobotStatePtr robot_state)
  {
    // Apply IK solved arm joints to state
    robot_state->setJointGroupPositions(grasp_data_->arm_jmg_, grasp_ik_solution_);

    // Set end effector to correct configuration
    grasp_data_->setRobotStateGrasp(robot_state);

    return true;
  }

  bool isValid()
  {
    if (grasp_filtered_by_ik_ ||
        grasp_filtered_by_cutting_plane_ ||
        grasp_filtered_by_orientation_ ||
        pregrasp_filtered_by_ik_)
      return false;
    else
      return true;
  }

  moveit_msgs::Grasp grasp_;
  /*
    CONTENTS OF moveit_msgs::GRASP

    # A name for this grasp
    string id

    # The internal posture of the hand for the pre-grasp
    # only positions are used
    trajectory_msgs/JointTrajectory pre_grasp_posture

    # The internal posture of the hand for the grasp
    # positions and efforts are used
    trajectory_msgs/JointTrajectory grasp_posture

    # The position of the end-effector for the grasp.  This is the pose of
    # the "parent_link" of the end-effector, not actually the pose of any
    # link *in* the end-effector.  Typically this would be the pose of the
    # most distal wrist link before the hand (end-effector) links began.
    geometry_msgs/PoseStamped grasp_pose

    # The estimated probability of success for this grasp, or some other
    # measure of how "good" it is.
    float64 grasp_quality

    # The approach direction to take before picking an object
    GripperTranslation pre_grasp_approach
    {
       # defines a translation for the gripper, used in pickup or place tasks
       # for example for lifting an object off a table or approaching the table for placing

       # the direction of the translation
       geometry_msgs/Vector3Stamped direction

       # the desired translation distance
       float32 desired_distance

       # the min distance that must be considered feasible before the
       # grasp is even attempted
       float32 min_distance
    }

    # The retreat direction to take after a grasp has been completed (object is attached)
    GripperTranslation post_grasp_retreat

    # The retreat motion to perform when releasing the object; this information
    # is not necessary for the grasp itself, but when releasing the object,
    # the information will be necessary. The grasp used to perform a pickup
    # is returned as part of the result, so this information is available for
    # later use.
    GripperTranslation post_place_retreat

    # the maximum contact force to use while grasping (<=0 to disable)
    float32 max_contact_force

    # an optional list of obstacles that we have semantic information about
    # and that can be touched/pushed/moved in the course of grasping
    string[] allowed_touch_objects
  */

  const GraspDataPtr grasp_data_;
  std::vector<double> grasp_ik_solution_;
  std::vector<double> pregrasp_ik_solution_;
  bool grasp_filtered_by_ik_;
  bool grasp_filtered_by_cutting_plane_; // grasp pose is in an unreachable part of the environment (ex: inside or behind a wall)
  bool grasp_filtered_by_orientation_; // grasp pose is not desireable
  bool pregrasp_filtered_by_ik_;
}; // class

typedef boost::shared_ptr<GraspCandidate> GraspCandidatePtr;

/**
 * \brief Used for sorting an array of CandidateGrasps
 * \return true if A is less than B
 */
bool compareGraspScores (GraspCandidatePtr grasp_a, GraspCandidatePtr grasp_b)
{
  // Determine if A or B has higher quality
  return (grasp_a->grasp_.grasp_quality > grasp_b->grasp_.grasp_quality);
}

} // namespace

#endif
