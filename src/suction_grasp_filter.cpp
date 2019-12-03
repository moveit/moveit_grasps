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
#include <memory>

namespace moveit_grasps
{
// Constructor
SuctionGraspFilter::SuctionGraspFilter(const robot_state::RobotStatePtr& robot_state,
                                       const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools)
  : GraspFilter::GraspFilter(robot_state, visual_tools), suction_voxel_overlap_cutoff_(0)
{
}

bool SuctionGraspFilter::filterBySuctionVoxelOverlapCutoff(std::vector<GraspCandidatePtr>& grasp_candidates)
{
  if (suction_voxel_overlap_cutoff_ <= 0)
    return grasp_candidates.size();

  // Pre-filter for suction voxel overlap
  std::size_t count = 0;
  std::size_t valid_grasps = 0;
  for (std::size_t ix = 0; ix < grasp_candidates.size(); ++ix)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_filter.pre_filter", "---------------\nGrasp candidate: " << count++);
    bool valid = false;
    auto suction_grasp_candidate = std::dynamic_pointer_cast<SuctionGraspCandidate>(grasp_candidates[ix]);
    if (!suction_grasp_candidate)
    {
      ROS_ERROR_NAMED("grasp_filter.pre_filter", "grasp_candidate is not castable as SuctionGraspCandidatePtr");
      return 0;
    }
    suction_grasp_candidate->setSuctionVoxelCutoff(suction_voxel_overlap_cutoff_);
    std::vector<bool> suction_voxel_enabled = suction_grasp_candidate->getSuctionVoxelEnabled();
    for (const bool& voxel_enabled : suction_voxel_enabled)
    {
      if (voxel_enabled)
      {
        valid = true;
        ++valid_grasps;
        break;
      }
    }
    if (!valid)
    {
      grasp_candidates[ix]->grasp_filtered_code_ = SuctionGraspFilterCode::GRASP_FILTERED_BY_SUCTION_VOXEL_OVERLAP;
    }
  }
  if (statistics_verbose_)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_filter.pre_filter",
                           "-------------------------------------------------------"
                               << "\nGRASP PRE-FILTER RESULTS"
                               << "\ntotal_grasp_candidates:     " << grasp_candidates.size()
                               << "\nremaining grasp candidates: " << valid_grasps
                               << "\n-------------------------------------------------------");
  }
  return valid_grasps;
}

std::size_t SuctionGraspFilter::filterGraspsHelper(std::vector<GraspCandidatePtr>& grasp_candidates,
                                                   const planning_scene::PlanningScenePtr& planning_scene,
                                                   const robot_model::JointModelGroup* arm_jmg,
                                                   const moveit::core::RobotStatePtr& seed_state, bool filter_pregrasp, bool verbose)
{
  filterBySuctionVoxelOverlapCutoff(grasp_candidates);
  return GraspFilter::filterGraspsHelper(grasp_candidates, planning_scene, arm_jmg, seed_state, filter_pregrasp, verbose);
}

bool SuctionGraspFilter::filterCandidateGrasp(const IkThreadStructPtr& ik_thread_struct) const
{
  bool filer_results = GraspFilter::filterCandidateGrasp(ik_thread_struct);
  if (!filer_results)
    return false;

  // Helper pointer
  GraspCandidatePtr& grasp_candidate = ik_thread_struct->grasp_candidates_[ik_thread_struct->grasp_id];

  // attachActiveSuctionCupCO(grasp_candidate, ik_thread_struct->planning_scene_);

  moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
      &isGraspStateValid, ik_thread_struct->planning_scene_.get(), collision_verbose_ || ik_thread_struct->visual_debug_,
      collision_verbose_speed_, visual_tools_, _1, _2, _3);

  // Check if IK solution for grasp pose is valid for fingers closed as well
  // if (!checkActiveSuctionVoxelCollisions(grasp_candidate->grasp_ik_solution_, ik_thread_struct, grasp_candidate, constraint_fn))
  // {
  //   ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Unable to find IK solution where the suction voxels are not in contact with other objects");
  //   grasp_candidate->grasp_filtered_code_ = GraspFilterCode::GRASP_FILTERED_BY_IK_CLOSED;
  //   return false;
  // }

  return true;
}

// bool SuctionGraspFilter::attachActiveSuctionCupCO(GraspCandidateConstPtr& grasp_candidate, planning_scene::PlanningScenePtr& planning_scene)
// {
//   static const std::string logger_name = "grasp_filter.attachActiveSuctionCupCO";

//   const GraspDataConstPtr& grasp_data = grasp_candidate->grasp_data_;

//   Eigen::Isometry3d ik_link_to_tcp = Eigen::Isometry3d::Identity();
//   std::string ik_link;
//   if (grasp_data->tcp_name_)
//   {
//     ik_link = grasp_data->tcp_name_;
//   }
//   else
//   {
//     ROS_WARN_STREAM_NAMED(logger_name, "It is strongly encouraged to define the tcp link");
//     ik_link = grasp_data->parent_link_->getName();
//     ik_link_to_tcp = grasp_data->tcp_to_eef_mount_;
//   }

//   for (std::size_t voxel_id = 0; voxel_id < grasp_data->suction_voxel_matrix_->getNumVoxels(); ++voxel_id)
//   {
//     std::shared_ptr<const SuctionVoxel> suction_voxel;
//     if (!grasp_data->suction_voxel_matrix_->getSuctionVoxel(voxel_id, suction_voxel))
//     {
//       ROS_ERROR_NAMED(logger_name, "Invalid suction voxel id: " << voxel_id);
//       return false;
//     }

//     moveit_msgs::CollisionObject suction_cups;
//     suction_cups.id = "suction_cup_" + voxel_id;
//     suction_cups.header.frame_id = ik_link;

//     suction_cups.primitives.resize(1);
//     suction_cups.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//     suction_cups.primitives[0].dimensions.resize(3);
//     suction_cups.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = suction_voxel->x_width_;
//     suction_cups.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = suction_voxel->z_width_;
//     suction_cups.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = grasp_data->grasp_max_depth_;

//     Eigen::Isometry3d ik_link_to_voxel_center = ik_link_to_tcp * suction_voxel->center_point_ * Eigen::Isometry3d(0, 0, grasp_data->grasp_max_depth_ / 2.0);
//     suction_cups.primitive_poses.resize(1);
//     suction_cups.primitive_poses[0] = tf2::toMsg(ik_link_to_voxel_center);

//     if ()
//     suction_cups.operation = moveit_msgs::CollisionObject::ADD;
//     planning_scene->processCollisionObjectMsg(suction_cups);

//     moveit_msgs::CollisionObject co;
//     if (!planning_scene->getCollisionObjectMsg(co, suction_cups.id))
//     {
//       ROS_WARN_STREAM_NAMED(logger_name, "Failed to attach object " << suction_cups.id << " in planning scene");
//       return false;
//     }
//   }

//   // TODO: refactor into helper
//   if (params_->debug_settings().visual_debug_ && params_->debug_settings().prompts_.display_suction_cup_collision)
//   {
//     moveit_msgs::DisplayRobotState display_robot_state_msg;
//     robot_state::robotStateToRobotStateMsg(planning_scene->getCurrentState(), display_robot_state_msg.state,
//                                            true);
//     visual_tools_->publishRobotState(display_robot_state_msg);
//     visual_tools_->trigger();
//     visual_tools_->prompt("Displaying suction cups as collision box. 'next' to continue");
//   }
//   return true;
// }

void SuctionGraspFilter::setSuctionVoxelOverlapCutoff(double cutoff)
{
  suction_voxel_overlap_cutoff_ = cutoff;
}

}  // namespace
