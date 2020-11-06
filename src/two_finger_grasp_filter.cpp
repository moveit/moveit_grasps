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
TwoFingerGraspFilter::TwoFingerGraspFilter(const robot_state::RobotStatePtr& robot_state,
                                           const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools)
  : GraspFilter(robot_state, visual_tools), name_("two_finger_grasp_filter")
{
}

bool TwoFingerGraspFilter::processCandidateGrasp(const IkThreadStructPtr& ik_thread_struct)
{
  bool filer_results = GraspFilter::processCandidateGrasp(ik_thread_struct);
  if (!filer_results)
    return false;

  // Helper pointer
  GraspCandidatePtr& grasp_candidate = ik_thread_struct->grasp_candidates_[ik_thread_struct->grasp_id];

  moveit::core::GroupStateValidityCallbackFn constraint_fn =
      boost::bind(&isGraspStateValid, ik_thread_struct->planning_scene_.get(),
                  collision_verbose_ || ik_thread_struct->visual_debug_, collision_verbose_speed_, visual_tools_, _1,
                  _2, _3);

  // Check if IK solution for grasp pose is valid for fingers closed as well
  return checkFingersClosedIK(grasp_candidate->grasp_ik_solution_, ik_thread_struct, grasp_candidate, constraint_fn);
}

bool TwoFingerGraspFilter::checkFingersClosedIK(std::vector<double>& ik_solution,
                                                const IkThreadStructPtr& ik_thread_struct,
                                                GraspCandidatePtr& grasp_candidate,
                                                const moveit::core::GroupStateValidityCallbackFn& constraint_fn) const
{
  // Set gripper position (how open the fingers are) to CLOSED
  grasp_candidate->getGraspStateClosedEEOnly(ik_thread_struct->robot_state_);

  // Check constraint function
  if (!constraint_fn(ik_thread_struct->robot_state_.get(), grasp_candidate->grasp_data_->arm_jmg_, &ik_solution[0]))
  {
    ROS_WARN_STREAM_NAMED(name_ + ".superdebug", "Grasp filtered because in collision with fingers CLOSED");
    grasp_candidate->grasp_filtered_code_ = GraspFilterCode::GRASP_FILTERED_BY_IK_CLOSED;
    return false;
  }

  return true;
}

}  // namespace moveit_grasps
