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
#include <moveit_grasps/suction_grasp_filter.h>
#include <moveit_grasps/suction_grasp_candidate.h>
#include <moveit_grasps/state_validity_callback.h>

// moveit
#include <moveit/transforms/transforms.h>
#include <moveit/collision_detection/collision_tools.h>

namespace moveit_grasps
{
// Constructor
SuctionGraspFilter::SuctionGraspFilter(robot_state::RobotStatePtr robot_state,
                                       moveit_visual_tools::MoveItVisualToolsPtr& visual_tools)
  : GraspFilter::GraspFilter(robot_state, visual_tools), suction_voxel_overlap_cutoff_(0)
{
}

bool SuctionGraspFilter::filterGrasps(std::vector<GraspCandidatePtr>& grasp_candidates,
                                      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                      const robot_model::JointModelGroup* arm_jmg,
                                      const moveit::core::RobotStatePtr seed_state, bool filter_pregrasp)
{
  // Pre-filter for suction voxel overlap
  if (suction_voxel_overlap_cutoff_ > 0)
  {
    std::size_t count = 0;
    std::size_t grasp_candidates_before = grasp_candidates.size();
    for (std::size_t ix = 0; ix < grasp_candidates.size();)
    {
      ROS_DEBUG_STREAM_NAMED("grasp_filter.pre_filter", "---------------\nGrasp candidate: " << count++);
      bool valid = false;
      auto suction_candidate = static_cast<SuctionGraspCandidate*>(grasp_candidates[ix].get());
      if (suction_candidate)
      {
        for (double& voxel_overlap : suction_candidate->suction_voxel_overlap_)
        {
          if (voxel_overlap > suction_voxel_overlap_cutoff_)
          {
            valid = true;
            break;
          }
        }
      }
      if (!valid)
      {
        ROS_DEBUG_STREAM_NAMED("grasp_filter.pre_filter", "Invalid");
        grasp_candidates.erase(grasp_candidates.begin() + ix);
      }
      else
      {
        ++ix;
      }
    }
    if (statistics_verbose_)
    {
      ROS_DEBUG_STREAM_NAMED("grasp_filter.pre_filter",
                             "-------------------------------------------------------"
                                 << "\nGRASP PRE-FILTER RESULTS"
                                 << "\ntotal_grasp_candidates:     " << grasp_candidates_before
                                 << "\nremaining grasp candidates: " << grasp_candidates.size()
                                 << "\n-------------------------------------------------------");
    }
  }
  if (!grasp_candidates.size())
    return false;

  return GraspFilter::filterGrasps(grasp_candidates, planning_scene_monitor, arm_jmg, seed_state, filter_pregrasp);
}

void SuctionGraspFilter::setSuctionVoxelOverlap(double cutoff)
{
  suction_voxel_overlap_cutoff_ = cutoff;
}

}  // namespace
