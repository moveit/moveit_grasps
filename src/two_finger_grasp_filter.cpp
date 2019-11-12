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

// Parent Class
#include <moveit_grasps/state_validity_callback.h>

// moveit_grasps
#include <moveit_grasps/two_finger_grasp_filter.h>

// moveit
#include <moveit/transforms/transforms.h>
#include <moveit/collision_detection/collision_tools.h>

namespace moveit_grasps
{
// Constructor
TwoFingerGraspFilter::TwoFingerGraspFilter(robot_state::RobotStatePtr robot_state,
                                           moveit_visual_tools::MoveItVisualToolsPtr& visual_tools)
  : GraspFilter(robot_state, visual_tools)
{
}

bool TwoFingerGraspFilter::processCandidateGrasp(IkThreadStructPtr& ik_thread_struct)
{
  ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Checking grasp #" << ik_thread_struct->grasp_id);

  // Helper pointer
  GraspCandidatePtr& grasp_candidate = ik_thread_struct->grasp_candidates_[ik_thread_struct->grasp_id];

  // Get pose
  ik_thread_struct->ik_pose_ = grasp_candidate->grasp_.grasp_pose;

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

  // Start pre-grasp section
  if (ik_thread_struct->filter_pregrasp_)  // optionally check the pregrasp
  {
    // Convert to a pre-grasp
    const std::string& ee_parent_link_name = grasp_candidate->grasp_data_->ee_jmg_->getEndEffectorParentGroup().second;
    ik_thread_struct->ik_pose_ = GraspGenerator::getPreGraspPose(grasp_candidate, ee_parent_link_name);

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
    ROS_DEBUG_STREAM_NAMED("grasp_filter", "Not filtering pregrasp!!");
  }

  return true;
}

bool TwoFingerGraspFilter::checkFingersClosedIK(std::vector<double>& ik_solution, IkThreadStructPtr& ik_thread_struct,
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

}  // namespace
